#include "kinova_arm_ros2/message_mapping.h"
namespace kinova_arm_ros2 {
using namespace kinova; using namespace kinova::interface;

static JointVec tol_to_vec(const std::vector<control_msgs::msg::JointTolerance>& t) {
  if (t.empty()) return JointVec::Constant(-1.0);           // disabled
  JointVec v = JointVec::Constant(-1.0);
  for (int i = 0; i < kNumJoints && i < static_cast<int>(t.size()); ++i) v[i] = t[i].position;
  return v;
}

TrajectoryGoal to_trajectory_goal(const ExecuteJointTrajectory::Goal& g) {
  TrajectoryGoal tg;
  for (const auto& p : g.trajectory.points) {
    JointWaypoint w{JointVec::Zero(), 0.0};
    for (int i = 0; i < kNumJoints && i < static_cast<int>(p.positions.size()); ++i) w.q[i] = p.positions[i];
    w.t_s = static_cast<double>(p.time_from_start.sec) + static_cast<double>(p.time_from_start.nanosec) * 1e-9;
    tg.trajectory.points.push_back(w);
  }
  tg.path_tolerance = tol_to_vec(g.path_tolerance);
  tg.goal_tolerance = tol_to_vec(g.goal_tolerance);
  tg.goal_time_tolerance_s = static_cast<double>(g.goal_time_tolerance.sec)
                           + static_cast<double>(g.goal_time_tolerance.nanosec) * 1e-9;
  tg.control_mode = (g.control_mode == 1) ? ControlModeKind::kImpedance : ControlModeKind::kPosition;
  tg.preemption   = (g.preemption == 1)   ? Preemption::kLatestWins    : Preemption::kQueue;
  tg.has_gains = (g.control_mode == 1);
  if (tg.has_gains) {
    for (int i = 0; i < kNumJoints; ++i) { tg.gains.kq[i] = g.gains.kq[i]; tg.gains.torque_limit[i] = g.gains.torque_limit[i]; }
    tg.gains.zeta = g.gains.zeta;
  }
  tg.sender_id = g.sender_id;
  return tg;
}

static trajectory_msgs::msg::JointTrajectoryPoint vec_to_point(const JointVec& v) {
  trajectory_msgs::msg::JointTrajectoryPoint p; p.positions.resize(kNumJoints);
  for (int i = 0; i < kNumJoints; ++i) p.positions[i] = v[i];
  return p;
}

ExecuteJointTrajectory::Feedback to_feedback_msg(const GoalId&, const TrajectoryFeedback& fb) {
  ExecuteJointTrajectory::Feedback m;
  m.desired = vec_to_point(fb.desired);
  m.actual  = vec_to_point(fb.actual);
  m.error   = vec_to_point(fb.error);
  m.fraction_complete = static_cast<float>(fb.fraction_complete);
  return m;
}

ExecuteJointTrajectory::Result to_result_msg(const TrajectoryResult& r) {
  ExecuteJointTrajectory::Result m;
  m.error_code = r.error_code; m.error_string = r.error_string;
  m.final_error = vec_to_point(r.final_error);
  return m;
}
}  // namespace kinova_arm_ros2
