#!/usr/bin/env python3
# kinova_arm_ros2/test/send_trajectory.py
import argparse, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTolerance
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from kinova_arm_interfaces.action import ExecuteJointTrajectory

class C(Node):
    def __init__(self, args):
        super().__init__('send_trajectory')
        self.args = args; self.code = None; self.status = None
        self.current_q = None
        self.create_subscription(JointState, 'joint_states', self._on_js, qos_profile_sensor_data)
        self.cli = ActionClient(self, ExecuteJointTrajectory, 'execute_joint_trajectory')

    def _on_js(self, m):
        if len(m.position) >= 7 and self.current_q is None:
            self.current_q = list(m.position[:7])

    def run(self):
        self.cli.wait_for_server()

        t0 = time.time()
        while self.current_q is None and (time.time() - t0) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_q is None:
            print('ERROR: no /joint_states received (is the node up? QoS best-effort?)')
            self.code = None; return

        start = list(self.current_q)
        goal = list(start); goal[self.args.joint] += self.args.delta
        print(f'measured start: joint{self.args.joint}={start[self.args.joint]:+.4f} rad -> target {goal[self.args.joint]:+.4f}')

        g = ExecuteJointTrajectory.Goal()
        g.control_mode = 1 if self.args.mode == 'impedance' else 0
        g.preemption = 1  # LATEST_WINS
        p0 = JointTrajectoryPoint(); p0.positions = start; p0.time_from_start = Duration(sec=0)
        p1 = JointTrajectoryPoint(); p1.positions = goal
        s = int(self.args.dur); p1.time_from_start = Duration(sec=s, nanosec=int((self.args.dur-s)*1e9))
        g.trajectory.points = [p0, p1]
        if self.args.path_tol > 0:
            jt = JointTolerance(); jt.position = self.args.path_tol
            g.path_tolerance = [jt]*7
        fut = self.cli.send_goal_async(g, feedback_callback=self.on_fb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            print('REJECTED'); self.code = None; return
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(self, rf)
        self.code = rf.result().result.error_code; self.status = rf.result().status
        print(f'result error_code={self.code} status={self.status}')

    def on_fb(self, fb):
        print(f'fb frac={fb.feedback.fraction_complete:.2f} measured_j{self.args.joint}={fb.feedback.actual.positions[self.args.joint]:+.4f}')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='position'); ap.add_argument('--delta', type=float, default=0.05)
    ap.add_argument('--dur', type=float, default=0.4); ap.add_argument('--path-tol', type=float, default=-1.0)
    ap.add_argument('--joint', type=int, default=6)
    ap.add_argument('--expect', type=int, required=True)
    a = ap.parse_args()
    if not (0 <= a.joint <= 6):
        print(f'ERROR: --joint must be in [0,6], got {a.joint}'); sys.exit(2)
    rclpy.init(); c = C(a); c.run(); rclpy.shutdown()
    ok = (c.code == a.expect)
    print('PASS' if ok else f'FAIL (got {c.code}, expected {a.expect})')
    sys.exit(0 if ok else 1)

if __name__ == '__main__':
    main()
