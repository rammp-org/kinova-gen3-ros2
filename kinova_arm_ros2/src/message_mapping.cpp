#include "kinova_arm_ros2/message_mapping.h"
#include "kinova_arm_ros2/joint_point.h"   // shared vec_to_point
namespace kinova_arm_ros2 {
using namespace kinova; using namespace kinova::interface;

static JointVec tol_to_vec(const std::vector<control_msgs::msg::JointTolerance>& t) {
  if (t.empty()) return JointVec::Constant(-1.0);           // disabled
  JointVec v = JointVec::Constant(-1.0);
  for (int i = 0; i < kNumJoints && i < static_cast<int>(t.size()); ++i) v[i] = t[i].position;
  return v;
}

// Copy waypoints, carrying whatever velocity/acceleration profile the planner
// supplied. The driver picks its interpolation order from the has_* flags —
// linear, cubic Hermite, or quintic — so dropping the profile here is what made
// cuRobo plans jerky (see the driver's docs/deep-dive/trajectory-interpolation.md).
// A profile counts as present only when EVERY point carries a correctly sized
// one; a partial profile degrades to the next order down rather than mixing
// conventions mid-trajectory.
static void fill_trajectory(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& points, Trajectory& tr) {
  const auto n = static_cast<size_t>(kNumJoints);
  bool all_vel = !points.empty(), all_acc = !points.empty();
  for (const auto& p : points) {
    all_vel = all_vel && p.velocities.size() == n;
    all_acc = all_acc && p.accelerations.size() == n;
  }
  for (const auto& p : points) {
    JointWaypoint w{JointVec::Zero(), 0.0};
    for (int i = 0; i < kNumJoints && i < static_cast<int>(p.positions.size()); ++i) w.q[i] = p.positions[i];
    if (all_vel) for (int i = 0; i < kNumJoints; ++i) w.qd[i] = p.velocities[i];
    if (all_acc) for (int i = 0; i < kNumJoints; ++i) w.qdd[i] = p.accelerations[i];
    w.t_s = static_cast<double>(p.time_from_start.sec)
          + static_cast<double>(p.time_from_start.nanosec) * 1e-9;
    tr.points.push_back(w);
  }
  tr.has_velocities    = all_vel;
  tr.has_accelerations = all_vel && all_acc;   // accelerations only count alongside velocities
}

TrajectoryGoal to_trajectory_goal(const ExecuteJointTrajectory::Goal& g) {
  TrajectoryGoal tg;
  fill_trajectory(g.trajectory.points, tg.trajectory);
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

TrajectoryGoal to_trajectory_goal(const trajectory_msgs::msg::JointTrajectory& traj) {
  TrajectoryGoal tg;
  fill_trajectory(traj.points, tg.trajectory);   // cuRobo emits qd/qdd — keep them
  tg.control_mode = ControlModeKind::kPosition;
  tg.preemption   = Preemption::kLatestWins;
  // path_tolerance / sender_id are set by the caller (GoToEEPoseServer).
  return tg;
}

GoToEEPose::Feedback to_goto_feedback_msg(const TrajectoryFeedback& fb) {
  GoToEEPose::Feedback m;
  m.phase = "executing";
  m.fraction_complete = static_cast<float>(fb.fraction_complete);
  m.actual = vec_to_point(fb.actual);
  return m;
}

GoToEEPose::Result to_goto_result_msg(const TrajectoryResult& r) {
  GoToEEPose::Result m;
  m.error_code = r.error_code;
  m.error_string = r.error_string;
  m.final_error = vec_to_point(r.final_error);
  return m;
}
}  // namespace kinova_arm_ros2
