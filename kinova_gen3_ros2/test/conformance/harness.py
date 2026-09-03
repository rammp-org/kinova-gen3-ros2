"""Shared plumbing for the conformance runner.

Attaches to an ALREADY RUNNING kinova_gen3_node -- it never launches one. The
arbitration mode is read-only at launch, so the runner discovers it from
/control_status and reports any check it could not run rather than passing silently.
"""
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from kinova_gen3_interfaces.msg import ControlStatus, EStop, GripperState, StreamStatus
from kinova_gen3_interfaces.srv import (AcquireControl, CloseStream, ListControllers,
                                       OpenStream, ReleaseControl, RevokeControl)
from sensor_msgs.msg import JointState

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"
ZERO_TOKEN = [0] * 16


def tok(t):
    """Normalise a token to a list of plain Python ints.

    rclpy hands uint8[16] back as a numpy array, so list(token) yields numpy.uint8 and
    the outgoing message assertion rejects it:
        The 'token' field must be ... each value of type 'int'
    Every token that makes a round trip -- acquire -> release, acquire -> setpoint --
    has to come through here.
    """
    return [int(x) for x in t]

# Best-effort for the high-rate state topics; latched for the two status topics. A
# subscriber that does not match these gets silence, which looks exactly like a broken
# publisher -- so the runner uses the same profiles the node advertises.
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)


@dataclass
class Result:
    name: str
    status: str
    detail: str = ""


@dataclass
class Registry:
    checks: List = field(default_factory=list)

    def add(self, section: str, name: str, needs_motion: bool = False,
            needs_mode: Optional[str] = None):
        def deco(fn: Callable):
            self.checks.append((section, name, fn, needs_motion, needs_mode))
            return fn
        return deco


REGISTRY = Registry()


class Ctx:
    """One ROS node plus the helpers every check needs."""

    def __init__(self, node: Node):
        self.n = node
        self._latest = {}
        self._subs = {}
        self._spec = {}
        self.sub(ControlStatus, "control_status", LATCHED_QOS, latched=True)
        self.sub(StreamStatus, "stream_status", LATCHED_QOS, latched=True)
        self.sub(JointState, "joint_states", SENSOR_QOS)
        self.sub(GripperState, "gripper_state", SENSOR_QOS)

    # ---- topics -------------------------------------------------------------
    def sub(self, msg_type, topic, qos, latched=False):
        self._latest.setdefault(topic, None)
        self._spec[topic] = (msg_type, qos, latched)
        if topic not in self._subs:
            self._subs[topic] = self.n.create_subscription(
                msg_type, topic, lambda m, t=topic: self._latest.__setitem__(t, m), qos)

    def _resubscribe(self, topic):
        """Recreate a subscription to pull a latched topic's CURRENT value.

        /control_status and /stream_status are transient_local and publish ON CHANGE.
        Merely clearing the cache and waiting would hang forever whenever nothing
        changed -- the latched sample is delivered on match, not on a timer. A fresh
        subscription re-triggers that delivery, which is what `ros2 topic echo --once`
        relies on too."""
        msg_type, qos, _ = self._spec[topic]
        self.n.destroy_subscription(self._subs[topic])
        self._latest[topic] = None
        self._subs[topic] = self.n.create_subscription(
            msg_type, topic, lambda m, t=topic: self._latest.__setitem__(t, m), qos)

    def spin(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.02)

    def latest(self, topic, timeout=5.0, fresh=False):
        """Newest message on a topic, or None. fresh=True discards what is already
        cached first, which matters for latched topics where the cached value may
        predate the thing the check just did."""
        if fresh:
            if self._spec.get(topic, (None, None, False))[2]:
                self._resubscribe(topic)
            else:
                self._latest[topic] = None
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.02)
            if self._latest.get(topic) is not None:
                return self._latest[topic]
        return self._latest.get(topic)

    # ---- services -----------------------------------------------------------
    def call(self, srv_type, name, req, timeout=8.0):
        cli = self.n.create_client(srv_type, name)
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service {name} never appeared")
        fut = cli.call_async(req)
        end = time.time() + timeout
        while time.time() < end and not fut.done():
            rclpy.spin_once(self.n, timeout_sec=0.02)
        if not fut.done():
            raise RuntimeError(f"service {name} timed out")
        return fut.result()

    def acquire(self, owner="conformance"):
        r = self.call(AcquireControl, "acquire_control",
                      AcquireControl.Request(owner_id=owner))
        return r

    def release(self, token):
        return self.call(ReleaseControl, "release_control",
                         ReleaseControl.Request(token=tok(token)))

    def revoke(self, reason="conformance cleanup"):
        return self.call(RevokeControl, "revoke_control",
                         RevokeControl.Request(reason=reason))

    def list_controllers(self):
        return self.call(ListControllers, "list_controllers",
                         ListControllers.Request()).controllers

    def open_stream(self, controller, timeout_s=0.5, token=None):
        return self.call(OpenStream, "open_stream", OpenStream.Request(
            controller=controller, timeout_s=timeout_s,
            token=tok(token if token is not None else ZERO_TOKEN)))

    def close_stream(self, token=None):
        return self.call(CloseStream, "close_stream", CloseStream.Request(
            token=tok(token if token is not None else ZERO_TOKEN)))

    # ---- e-stop -------------------------------------------------------------
    def estop(self, engaged: bool, source="conformance", reason="", stamp=None,
              settle=1.0):
        pub = self.n.create_publisher(EStop, "/estop", QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10))
        m = EStop()
        m.header.stamp = stamp if stamp is not None else self.n.get_clock().now().to_msg()
        m.engaged = engaged
        m.source = source
        m.reason = reason
        # Publish a few times: a subscription that has only just matched can miss the
        # first message, and an e-stop that depends on one packet is not an e-stop.
        for _ in range(10):
            pub.publish(m)
            rclpy.spin_once(self.n, timeout_sec=0.02)
        self.spin(settle)

    # ---- state --------------------------------------------------------------
    def control_status(self, fresh=True):
        return self.latest("control_status", fresh=fresh)

    def stream_status(self, fresh=True):
        return self.latest("stream_status", fresh=fresh)

    def joint_positions(self, timeout=5.0):
        js = self.latest("joint_states", timeout=timeout, fresh=True)
        return list(js.position[:7]) if js else None

    def cleanup(self):
        """Leave the arm as we found it: no session, no owner, not stopped."""
        try:
            self.close_stream()
        except Exception:
            pass
        try:
            self.revoke()
        except Exception:
            pass
        try:
            self.estop(False, reason="conformance cleanup", settle=0.5)
        except Exception:
            pass
