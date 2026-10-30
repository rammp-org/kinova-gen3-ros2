"""Generalized-momentum observer for external joint torque on the Gen3.

Passive. Nothing here commands the arm. Feed it the three things /joint_states
already carries (position, velocity, measured effort) and it returns an estimate
of the external joint torque that stays valid while the arm moves.

Convention, matching core's Supervisor (supervisor.cpp, "motor + external =
gravity"): the measured joint torque tau is the torque the actuator applies to
the link, so at rest tau == g(q) and the external torque is g(q) - tau. The
dynamics in that convention are

    M(q) qdd + C(q, qd) qd + g(q) = tau + tau_ext

so the external torque is tau_ext = M qdd + C qd + g - tau. The quasi-static
estimate core publishes drops the first two terms, which is exact at rest and
wrong by the arm's own inertia while it moves.

The observer avoids qdd. With generalized momentum p = M(q) qd and the identity
dM/dt = C + C^T,

    dp/dt = tau + C^T qd - g + tau_ext

Everything on the right except tau_ext is known, so integrate it, compare with
the measured p, and the gap is the external torque:

    r = K ( p - p0 - integral( tau + C^T qd - g + r ) dt )

r follows tau_ext as a first-order filter with cutoff K rad/s. This is De Luca's
residual (De Luca, Albu-Schaffer, Haddadin, Hirzinger, IROS 2006), fed with the
torque sensors rather than the motor command, which keeps gearbox friction and
actuator tracking error out of the estimate.

Model error (the URDF's masses, issue kinova-gen3-driver#5) lands in r as a slow,
pose-dependent bias, the same bias the quasi-static residual already carries.
"""

from __future__ import annotations

import re

import numpy as np
import pinocchio as pin

_ARM_JOINT = re.compile(r"joint_([1-7])$")


class ArmModel:
    """A URDF reduced to its seven arm joints, with the q packing core uses.

    Joints whose names end in joint_1 .. joint_7 are kept, in that order; every
    other joint (the gripper linkage) is locked at zero, which is what core's
    Dynamics does. Continuous joints are packed (cos, sin), as in dynamics.cpp.
    """

    def __init__(self, urdf: str):
        full = (
            pin.buildModelFromXML(urdf)
            if urdf.lstrip().startswith("<")
            else pin.buildModelFromUrdf(urdf)
        )
        arm = {}
        for name in full.names[1:]:
            m = _ARM_JOINT.search(name)
            if m:
                arm[int(m.group(1))] = name
        if sorted(arm) != list(range(1, 8)):
            raise ValueError(
                f"expected joints named *joint_1..*joint_7, found {sorted(arm.values())}"
            )
        self.joint_names = [arm[i] for i in range(1, 8)]
        lock = [full.getJointId(n) for n in full.names[1:] if n not in arm.values()]
        self.model = pin.buildReducedModel(full, lock, pin.neutral(full))
        if self.model.nv != 7:
            raise ValueError(f"reduced model has nv={self.model.nv}, expected 7")
        self.data = self.model.createData()
        self._qcfg = pin.neutral(self.model)
        self._nqs = []
        self._idx_q = []
        for i in range(7):
            jid = self.model.getJointId(self.joint_names[i])
            self._nqs.append(self.model.nqs[jid])
            self._idx_q.append(self.model.idx_qs[jid])

    def pack(self, q: np.ndarray) -> np.ndarray:
        for i in range(7):
            k = self._idx_q[i]
            if self._nqs[i] == 2:
                self._qcfg[k] = np.cos(q[i])
                self._qcfg[k + 1] = np.sin(q[i])
            else:
                self._qcfg[k] = q[i]
        return self._qcfg

    def gravity(self, q: np.ndarray) -> np.ndarray:
        return np.array(
            pin.computeGeneralizedGravity(self.model, self.data, self.pack(q))
        )

    def mass(self, q: np.ndarray) -> np.ndarray:
        M = pin.crba(self.model, self.data, self.pack(q))
        return np.triu(M) + np.triu(M, 1).T

    def coriolis(self, q: np.ndarray, qd: np.ndarray) -> np.ndarray:
        return np.array(
            pin.computeCoriolisMatrix(self.model, self.data, self.pack(q), qd)
        )

    def inverse_dynamics(self, q, qd, qdd) -> np.ndarray:
        """Joint torque that produces qdd from (q, qd) with no external load."""
        return np.array(pin.rnea(self.model, self.data, self.pack(q), qd, qdd))


def quasi_static_residual(
    model: ArmModel, q: np.ndarray, tau: np.ndarray
) -> np.ndarray:
    """What core publishes today: g(q) - tau. Exact at rest, inertia-corrupted in motion."""
    return model.gravity(q) - tau


class MomentumObserver:
    """De Luca residual on the torque-sensor signal. One instance per run; call step in time order."""

    def __init__(self, model: ArmModel, gain: float = 10.0):
        self.model = model
        self.gain = float(gain)
        self._integral = None
        self._p0 = None
        self.r = np.zeros(7)

    def reset(self):
        self._integral = None
        self._p0 = None
        self.r = np.zeros(7)

    def step(self, q, qd, tau, dt: float) -> np.ndarray:
        q = np.asarray(q, float)
        qd = np.asarray(qd, float)
        tau = np.asarray(tau, float)
        p = self.model.mass(q) @ qd
        if self._integral is None:
            self._p0 = p.copy()
            self._integral = np.zeros(7)
            return self.r
        drive = tau + self.model.coriolis(q, qd).T @ qd - self.model.gravity(q) + self.r
        self._integral = self._integral + drive * dt
        self.r = self.gain * (p - self._p0 - self._integral)
        return self.r.copy()

    def rebase(self, model: ArmModel, q, qd, r) -> None:
        """Swap the model mid-run and carry on from residual r instead of re-converging from zero.

        For a payload picked up or put down: the caller knows how much of the
        residual the new model now explains, and passes the shifted value.
        """
        self.model = model
        self.r = np.asarray(r, float).copy()
        if self._integral is not None:
            p = model.mass(np.asarray(q, float)) @ np.asarray(qd, float)
            self._integral = p - self._p0 - self.r / self.gain


def run_observer(model: ArmModel, t, q, qd, tau, gain: float = 10.0) -> np.ndarray:
    """Vectorised convenience: arrays of shape (N,) and (N, 7) in, residual (N, 7) out."""
    obs = MomentumObserver(model, gain)
    out = np.zeros_like(q, dtype=float)
    for k in range(len(t)):
        dt = float(t[k] - t[k - 1]) if k > 0 else 0.0
        out[k] = obs.step(q[k], qd[k], tau[k], dt)
    return out
