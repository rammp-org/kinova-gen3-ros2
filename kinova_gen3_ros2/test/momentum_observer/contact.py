"""Collision detection and payload weighing on top of the momentum observer.

Passive, like observer.py: it reads q, qd and measured torque and reports. What
to do about a collision (stop, back off, go compliant) is left to the caller.

Two features, deliberately nothing more:

1. Impulse detection. The observer residual r is split into a slow band (an
   exponential average, `slow_tau_s` = 0.2 s) and a fast band, r minus the
   slow band. Only the fast band can fire. What lands in the slow band never
   does: the model's pose-dependent bias (kinova-gen3-driver#5), a hand leaning
   on the arm, hand-guiding, resting on a table, a payload the model does not
   know about yet. A collision is an abrupt change, so it lands in the fast band.
   While an event is open the slow band is frozen so the collision is not
   absorbed into it, and after `max_event_s` it is released, so sustained
   contact that follows a collision becomes the new baseline instead of keeping
   the event open or firing again.

2. Payload weighing. A held object is folded into the model as a point mass on
   link 7 (`with_payload`), so the observer stops reporting its weight and its
   inertia as external torque, and the fast band stays quiet while it is
   carried. The weight is measured as the change in residual between a tare just
   before the grasp and a still moment after the lift, at nearly the same pose,
   which cancels the model's pose-dependent bias (1 to 4 N.m on the shoulder,
   much larger than a water bottle's torque). With one pose the mass is fitted
   at an assumed centre of mass, the grasp point by default; with several poses
   the centre of mass can be fitted too (`fit_payload`).

   The grasp itself is a fast change, the very thing the detector looks for.
   Between `begin_load_change` and `end_load_change` the fast band is projected
   off the torques a load held in the gripper can produce (a vertical force and
   a horizontal moment at the gripper: three of the seven directions), so a bump
   from the side, on the forearm or on the elbow during a grasp still fires.

Thresholds are per joint: profile.scale times the calibrated floor, which is the
fast band's high percentile on a hands-off run, interpolated between the still
and the moving floor by joint speed, and never below profile.min_nm. A profile
(`PROFILES`) is the caller's choice per held object: a fork wants a fine
threshold, a bottle or an empty gripper a coarser one with fewer false alarms.
No profile can sit below the floor without false alarms; on the arm today that
floor is the mass model, and moving slower lowers it.
"""

from __future__ import annotations

import copy
import json
from collections import deque
from dataclasses import asdict, dataclass

import numpy as np
import pinocchio as pin

from observer import ArmModel, MomentumObserver

EE_FRAME = (
    "gen3_end_effector_link"  # core's Dynamics default; payload COM is expressed here
)
FINGER_TIPS = (
    "gen3_robotiq_85_left_finger_tip_link",
    "gen3_robotiq_85_right_finger_tip_link",
)
STILL_RAD_S = 0.02  # max |qd| for a sample to count as still, for tare and weighing


@dataclass(frozen=True)
class Payload:
    """A held object as a point mass: kg, and its centre of mass in the EE frame, m."""

    mass: float = 0.0
    com: tuple = (0.0, 0.0, 0.0)

    @property
    def theta(self) -> np.ndarray:
        """The four gravity parameters (m, m*cx, m*cy, m*cz) the torque is linear in."""
        return np.r_[self.mass, self.mass * np.asarray(self.com, float)]

    @staticmethod
    def from_theta(theta) -> "Payload":
        m = float(theta[0])
        if m <= 1e-6:
            return Payload()
        return Payload(m, tuple(float(v) for v in np.asarray(theta[1:4]) / m))


@dataclass(frozen=True)
class Profile:
    """How sensitive the detector is: threshold = scale * floor, per joint.

    scale: multiple of the calibrated floor. Lower is more sensitive.
    min_nm: no joint's threshold goes below this, N.m. The smallest abrupt change
      that counts as a collision at all, however quiet the floor: below it is
      ordinary handling, a hand on the arm, not something to react to.
    min_samples: consecutive samples above threshold before an event fires; one
      sample above a percentile floor is noise, two at 100 Hz is 20 ms.
    max_event_s: how long the slow band stays frozen once an event opens.
    refractory_s: no new event this soon after the previous one closed.
    """

    name: str
    scale: float
    min_nm: float = 1.0
    min_samples: int = 2
    max_event_s: float = 1.0
    refractory_s: float = 0.3


# Starting points, to be tuned on the arm with replay_contact.py. A sharp object
# near a person wants every bump caught and tolerates false alarms; an empty
# gripper or a bottle wants only the real hits.
PROFILES = {
    "empty": Profile("empty", scale=2.0, min_nm=1.0),
    "bottle": Profile("bottle", scale=2.0, min_nm=1.0),
    "fork": Profile("fork", scale=1.2, min_nm=0.4),
}


@dataclass
class Floor:
    """Fast-band magnitude, N.m per joint, that a hands-off run reaches still and moving.

    The threshold at joint speed v (max |qd|) is still + (moving - still) * min(1, v / v_ref).
    """

    still: np.ndarray
    moving: np.ndarray
    v_ref: float = 0.5
    calibrated: bool = True

    @staticmethod
    def uncalibrated() -> "Floor":
        # Placeholders, not measurements: the shoulder value follows the ~3 N.m
        # observer floor in motion measured on abra (HANDOFF.md); the rest are
        # guesses scaled to each joint's load. Calibrate before trusting a profile.
        return Floor(
            still=np.full(7, 0.3),
            moving=np.array([1.0, 3.0, 1.0, 2.0, 0.6, 0.6, 0.4]),
            calibrated=False,
        )

    def at(self, speed: float) -> np.ndarray:
        a = min(1.0, speed / self.v_ref) if self.v_ref > 0 else 1.0
        return self.still + (self.moving - self.still) * a

    def to_json(self) -> str:
        d = asdict(self)
        d["still"], d["moving"] = list(self.still), list(self.moving)
        return json.dumps(d, indent=2)

    @staticmethod
    def from_json(text: str) -> "Floor":
        d = json.loads(text)
        return Floor(
            np.array(d["still"]), np.array(d["moving"]), d["v_ref"], d["calibrated"]
        )


@dataclass
class CollisionEvent:
    """One impulse. Fields up to `torque` are fixed at onset; the rest fill in as it runs."""

    t: float  # onset, s
    joint: int  # 0-based joint with the largest threshold ratio at onset
    score: float  # largest |fast| / threshold at onset; > 1 by definition
    torque: np.ndarray  # fast-band torque at onset, N.m
    moving: bool  # the arm was moving under its own command
    during_load_change: bool
    peak_score: float = 0.0
    t_end: float | None = None


@dataclass
class PayloadEstimate:
    payload: Payload  # the payload the model carries from now on
    delta_kg: float  # change from the previous payload
    fit_rms: float  # residual the payload could not explain, N.m RMS over joints
    ok: bool
    reason: str = ""


# --- geometry ------------------------------------------------------------------


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _ee(model: ArmModel, q, ee_frame: str = EE_FRAME):
    """(J, R): the EE frame's world-aligned 6x7 Jacobian [v; w] and its rotation."""
    M, d = model.model, model.data
    qc = model.pack(np.asarray(q, float))
    fid = M.getFrameId(ee_frame)
    J = pin.computeFrameJacobian(M, d, qc, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    pin.updateFramePlacement(M, d, fid)
    return np.array(J), np.array(d.oMf[fid].rotation)


def point_jacobian(
    model: ArmModel, q, point_ee, ee_frame: str = EE_FRAME
) -> np.ndarray:
    """3x7 world-frame linear Jacobian of a point fixed in the EE frame."""
    J, R = _ee(model, q, ee_frame)
    return J[:3] - _skew(R @ np.asarray(point_ee, float)) @ J[3:]


def payload_regressor(model: ArmModel, q, ee_frame: str = EE_FRAME) -> np.ndarray:
    """Y (7x4) with payload gravity torque = Y @ Payload.theta, in the g(q) convention.

    Potential energy of a point mass m at c is -m g.p(c); its gradient is
    -J_c^T m g with J_c = Jv - [R c]x Jw, which is linear in (m, m c):
    Y = [ -Jv^T g ,  Jw^T [g]x R ].
    """
    J, R = _ee(model, q, ee_frame)
    g = np.array(model.model.gravity.linear)
    return np.column_stack([-J[:3].T @ g, J[3:].T @ _skew(g) @ R])


def grasp_point(model: ArmModel, ee_frame: str = EE_FRAME) -> np.ndarray:
    """Midpoint of the 2F-85 fingertip frames in the EE frame, or 0.13 m out on z without them."""
    M, d = model.model, model.data
    if not all(M.existFrame(n) for n in FINGER_TIPS):
        return np.array([0.0, 0.0, 0.13])
    pin.framesForwardKinematics(M, d, pin.neutral(M))
    ee_inv = d.oMf[M.getFrameId(ee_frame)].inverse()
    tips = [(ee_inv * d.oMf[M.getFrameId(n)]).translation for n in FINGER_TIPS]
    return np.mean(tips, axis=0)


def with_payload(
    base: ArmModel, payload: Payload, ee_frame: str = EE_FRAME
) -> ArmModel:
    """A copy of `base` with the payload welded to link 7 as a point mass."""
    out = copy.copy(base)
    out.model = pin.Model(base.model)
    out._qcfg = base._qcfg.copy()
    if payload.mass > 0:
        jid = out.model.getJointId(base.joint_names[6])
        frame = out.model.frames[out.model.getFrameId(ee_frame)]
        assert (
            frame.parentJoint == jid
        ), f"{ee_frame} must hang off {base.joint_names[6]}"
        c = frame.placement.act(np.asarray(payload.com, float))
        out.model.inertias[jid] = out.model.inertias[jid] + pin.Inertia(
            payload.mass, c, np.zeros((3, 3))
        )
    out.data = out.model.createData()
    return out


def fit_payload(Ys, drs, com=None) -> tuple[np.ndarray, float]:
    """Payload parameters that explain residual changes drs at regressors Ys.

    A payload makes the residual move by -Y @ theta. With `com` given, fit the
    mass alone (one pose is enough); with com=None, fit all four (needs poses
    with the wrist pointed different ways). Returns (theta, unexplained RMS).
    """
    Y = np.vstack(Ys)
    b = -np.concatenate(drs)
    if com is not None:
        y = Y @ np.r_[1.0, np.asarray(com, float)]
        m = float(y @ b / (y @ y)) if y @ y > 0 else 0.0
        theta = m * np.r_[1.0, np.asarray(com, float)]
    else:
        theta = np.linalg.lstsq(Y, b, rcond=None)[0]
    return theta, float(np.sqrt(np.mean((Y @ theta - b) ** 2)))


# --- the monitor -----------------------------------------------------------------


class ContactMonitor:
    """Feed it every sample with step(); it returns a CollisionEvent at each onset.

    Grasp and release:
        mon.begin_load_change()      # arm still, just before the gripper closes or opens
        ... close or open, lift ...
        est = mon.end_load_change()  # arm still again; weighs and loads the payload
        mon.set_profile(PROFILES["fork"])
    """

    def __init__(
        self,
        model: ArmModel,
        profile: Profile = PROFILES["empty"],
        floor: Floor | None = None,
        gain: float = 20.0,
        slow_tau_s: float = 0.2,
        settle_s: float = 0.5,
        ee_frame: str = EE_FRAME,
        grasp_com=None,
    ):
        self.bare = model
        self.ee_frame = ee_frame
        self.payload = Payload()
        self.model = model
        self.observer = MomentumObserver(model, gain)
        self.profile = profile
        self.floor = floor or Floor.uncalibrated()
        self.slow_tau_s = slow_tau_s
        self.settle_s = settle_s
        self.grasp_com = (
            grasp_point(model, ee_frame)
            if grasp_com is None
            else np.asarray(grasp_com, float)
        )
        self.events: list[CollisionEvent] = []
        self.r = np.zeros(7)
        self.slow = None
        self.fast = np.zeros(7)
        self.threshold = np.full(7, np.inf)
        self._t0 = self._t = None
        self._q = self._qd = None
        self._above = 0
        self._open: CollisionEvent | None = None
        self._quiet_until = -np.inf
        self._recent = deque(maxlen=64)  # (t, q, qd, r) for tare and weighing
        self._tare = None  # (q, r) while a load change is in progress

    # configuration

    def set_profile(self, profile: Profile) -> None:
        self.profile = profile

    def set_payload(self, payload: Payload) -> None:
        """Load a payload the caller already knows, keeping the fast band continuous."""
        if self._q is None:
            self.payload, self.model = (
                payload,
                with_payload(self.bare, payload, self.ee_frame),
            )
            self.observer.model = self.model
            return
        shift = payload_regressor(self.bare, self._q, self.ee_frame) @ (
            payload.theta - self.payload.theta
        )
        self.payload = payload
        self.model = with_payload(self.bare, payload, self.ee_frame)
        self.r = self.r + shift
        self.observer.rebase(self.model, self._q, self._qd, self.r)
        if self.slow is not None:
            self.slow = self.slow + shift
        self._recent = deque(
            ((t, q, qd, r + shift) for t, q, qd, r in self._recent),
            maxlen=self._recent.maxlen,
        )

    # grasp and release

    @property
    def in_load_change(self) -> bool:
        return self._tare is not None

    def begin_load_change(self, window_s: float = 0.3) -> bool:
        """Tare. Call with the arm still, before the gripper moves. False if it was not still."""
        q, r, still = self._still_mean(window_s)
        self._tare = (q, r)
        return still

    def end_load_change(
        self, window_s: float = 0.3, max_fit_rms: float = 1.0
    ) -> PayloadEstimate:
        """Weigh what the gripper now holds and load it. Call with the arm still.

        If the arm is not still, or the change does not look like a load in the
        gripper (fit_rms above max_fit_rms: the object is still on the table,
        someone is holding the arm), nothing changes and the load change stays
        open; call again.
        """
        if self._tare is None:
            raise RuntimeError("end_load_change without begin_load_change")
        q, r, still = self._still_mean(window_s)
        if not still:
            return PayloadEstimate(
                self.payload, 0.0, float("nan"), False, "arm not still"
            )
        q0, r0 = self._tare
        Y = payload_regressor(self.bare, q, self.ee_frame)
        dtheta, fit_rms = fit_payload([Y], [r - r0], com=self.grasp_com)
        new = Payload.from_theta(self.payload.theta + dtheta)
        delta = new.mass - self.payload.mass
        if fit_rms > max_fit_rms:
            return PayloadEstimate(
                new, delta, fit_rms, False, "change does not look like a held load"
            )
        self._tare = None
        self.set_payload(new)
        # The slow band may not have caught up with the grasp yet; what is left
        # of the step is explained now, so it must not fire once projection stops.
        self.slow = self.r.copy()
        return PayloadEstimate(new, delta, fit_rms, True)

    def _still_mean(self, window_s):
        if not self._recent:
            raise RuntimeError("no samples yet")
        t_last = self._recent[-1][0]
        rows = [s for s in self._recent if s[0] >= t_last - window_s]
        q = np.mean([s[1] for s in rows], axis=0)
        r = np.mean([s[3] for s in rows], axis=0)
        still = max(float(np.abs(s[2]).max()) for s in rows) < STILL_RAD_S
        return q, r, still

    # the loop

    def step(self, t, q, qd, tau) -> CollisionEvent | None:
        q, qd, tau = (np.asarray(v, float) for v in (q, qd, tau))
        dt = 0.0 if self._t is None else float(t - self._t)
        if self._t0 is None:
            self._t0 = t
        self._t, self._q, self._qd = t, q, qd
        self.r = self.observer.step(q, qd, tau, dt)
        self._recent.append((t, q, qd, self.r.copy()))

        if self.slow is None or t - self._t0 < self.settle_s:
            # observer start-up transient: track it, never fire
            self.slow = self.r.copy()
            self.fast = np.zeros(7)
            return None

        fast = self.r - self.slow
        if self.in_load_change:
            Y = payload_regressor(self.model, q, self.ee_frame)
            fast = fast - Y @ np.linalg.lstsq(Y, fast, rcond=None)[0]
        self.fast = fast

        speed = float(np.abs(qd).max())
        self.threshold = np.maximum(
            self.profile.scale * self.floor.at(speed), self.profile.min_nm
        )
        ratio = np.abs(fast) / self.threshold
        score = float(ratio.max())

        fired = None
        if self._open is not None:
            ev = self._open
            ev.peak_score = max(ev.peak_score, score)
            timed_out = t - ev.t >= self.profile.max_event_s
            if score < 0.5 or timed_out:
                ev.t_end = t
                self._open = None
                self._quiet_until = t + self.profile.refractory_s
                if timed_out:
                    # still pressed after max_event_s: sustained contact, the new baseline
                    self.slow = self.r.copy()
        else:
            self._above = self._above + 1 if score > 1.0 else 0
            if self._above >= self.profile.min_samples and t >= self._quiet_until:
                j = int(np.argmax(ratio))
                fired = CollisionEvent(
                    t=t,
                    joint=j,
                    score=score,
                    torque=fast.copy(),
                    moving=speed > 0.05,
                    during_load_change=self.in_load_change,
                    peak_score=score,
                )
                self.events.append(fired)
                self._open = fired
                self._above = 0

        if self._open is None:
            a = dt / (self.slow_tau_s + dt) if dt > 0 else 0.0
            self.slow = self.slow + a * (self.r - self.slow)
        return fired


def calibrate_floor(
    model: ArmModel,
    t,
    q,
    qd,
    tau,
    payload: Payload = Payload(),
    percentile: float = 99.9,
    v_ref: float = 0.5,
    **monitor_kw,
) -> Floor:
    """The fast band's high percentile over a hands-off run, still and moving separately."""
    mon = ContactMonitor(model, Profile("calibrate", scale=np.inf), **monitor_kw)
    mon.set_payload(payload)
    fast, speed = [], []
    for k in range(len(t)):
        mon.step(t[k], q[k], qd[k], tau[k])
        if t[k] - t[0] >= mon.settle_s:
            fast.append(np.abs(mon.fast))
            speed.append(float(np.abs(qd[k]).max()))
    fast, speed = np.array(fast), np.array(speed)
    still = speed < 0.05
    moving = speed >= v_ref / 2
    lo = np.percentile(fast[still], percentile, axis=0) if still.sum() >= 10 else None
    hi = np.percentile(fast[moving], percentile, axis=0) if moving.sum() >= 10 else None
    if lo is None and hi is None:
        raise ValueError("run too short to calibrate")
    lo = hi if lo is None else lo
    hi = lo if hi is None else np.maximum(hi, lo)
    return Floor(np.maximum(lo, 0.02), np.maximum(hi, 0.02), v_ref=v_ref)
