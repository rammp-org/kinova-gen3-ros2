#include <gtest/gtest.h>
#include "kinova_arm_ros2/message_mapping.h"
using namespace kinova_arm_ros2;
using kinova::interface::ControlModeKind; using kinova::interface::Preemption;

static trajectory_msgs::msg::JointTrajectoryPoint pt(double v, double t) {
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions.assign(7, v);
  p.time_from_start.sec = static_cast<int32_t>(t);
  p.time_from_start.nanosec = static_cast<uint32_t>((t - static_cast<int32_t>(t)) * 1e9);
  return p;
}

TEST(MessageMapping, GoalToTrajectoryGoalPosition) {
  kinova_arm_interfaces::action::ExecuteJointTrajectory::Goal g;
  g.trajectory.points = { pt(0.0, 0.0), pt(0.5, 2.0) };
  g.control_mode = 0;      // POSITION
  g.preemption   = 1;      // LATEST_WINS
  // path_tolerance empty -> guard disabled (-1)
  auto tg = to_trajectory_goal(g);
  EXPECT_EQ(tg.trajectory.points.size(), 2u);
  EXPECT_NEAR(tg.trajectory.points[1].q[0], 0.5, 1e-12);
  EXPECT_NEAR(tg.trajectory.points[1].t_s, 2.0, 1e-9);
  EXPECT_EQ(tg.control_mode, ControlModeKind::kPosition);
  EXPECT_EQ(tg.preemption, Preemption::kLatestWins);
  EXPECT_LT(tg.path_tolerance[0], 0.0);       // disabled
  EXPECT_FALSE(tg.has_gains);
}

TEST(MessageMapping, GoalImpedanceGainsAndPathTol) {
  kinova_arm_interfaces::action::ExecuteJointTrajectory::Goal g;
  g.trajectory.points = { pt(0.0, 0.0), pt(0.1, 1.0) };
  g.control_mode = 1;      // IMPEDANCE
  for (int i = 0; i < 7; ++i) g.gains.kq[i] = 60.0;
  g.gains.zeta = 0.6;
  for (int i = 0; i < 7; ++i) g.gains.torque_limit[i] = 9.0;
  control_msgs::msg::JointTolerance jt; jt.position = 0.2;
  g.path_tolerance.assign(7, jt);
  auto tg = to_trajectory_goal(g);
  EXPECT_TRUE(tg.has_gains);
  EXPECT_NEAR(tg.gains.kq[0], 60.0, 1e-12);
  EXPECT_NEAR(tg.gains.zeta, 0.6, 1e-12);
  EXPECT_NEAR(tg.path_tolerance[0], 0.2, 1e-12);
}

TEST(MessageMapping, GoalWithFewerThanSevenPositionsZeroFillsRemainder) {
  kinova_arm_interfaces::action::ExecuteJointTrajectory::Goal g;
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions = {0.1, 0.2, 0.3, 0.4, 0.5};   // 5 < 7
  p.time_from_start.sec = 1;
  g.trajectory.points = { p };
  auto tg = to_trajectory_goal(g);
  ASSERT_EQ(tg.trajectory.points.size(), 1u);
  const auto& q = tg.trajectory.points[0].q;
  EXPECT_NEAR(q[0], 0.1, 1e-12);
  EXPECT_NEAR(q[1], 0.2, 1e-12);
  EXPECT_NEAR(q[2], 0.3, 1e-12);
  EXPECT_NEAR(q[3], 0.4, 1e-12);
  EXPECT_NEAR(q[4], 0.5, 1e-12);
  EXPECT_EQ(q[5], 0.0);   // not provided -> must be defined 0.0, not garbage
  EXPECT_EQ(q[6], 0.0);
}

TEST(MessageMapping, GoalWithMoreThanSevenPositionsTakesFirstSeven) {
  kinova_arm_interfaces::action::ExecuteJointTrajectory::Goal g;
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};   // 9 > 7
  p.time_from_start.sec = 1;
  g.trajectory.points = { p };
  auto tg = to_trajectory_goal(g);
  ASSERT_EQ(tg.trajectory.points.size(), 1u);
  const auto& q = tg.trajectory.points[0].q;
  for (int i = 0; i < 7; ++i) EXPECT_NEAR(q[i], static_cast<double>(i + 1), 1e-12);
}

TEST(MessageMapping, ResultCarriesErrorCode) {
  kinova::interface::TrajectoryResult r; r.error_code = -4; r.error_string = "path tol";
  r.final_error = kinova::JointVec::Constant(0.01);
  auto m = to_result_msg(r);
  EXPECT_EQ(m.error_code, -4);
  EXPECT_EQ(m.error_string, "path tol");
  ASSERT_EQ(m.final_error.positions.size(), 7u);
  EXPECT_NEAR(m.final_error.positions[0], 0.01, 1e-12);
}

TEST(MessageMapping, JointTrajectoryToPositionGoal) {
  trajectory_msgs::msg::JointTrajectory traj;
  traj.points = { pt(0.0, 0.0), pt(0.3, 0.5) };
  auto tg = to_trajectory_goal(traj);
  ASSERT_EQ(tg.trajectory.points.size(), 2u);
  EXPECT_NEAR(tg.trajectory.points[1].q[0], 0.3, 1e-12);
  EXPECT_NEAR(tg.trajectory.points[1].t_s, 0.5, 1e-9);
  EXPECT_EQ(tg.control_mode, ControlModeKind::kPosition);
  EXPECT_EQ(tg.preemption, Preemption::kLatestWins);
  EXPECT_FALSE(tg.has_gains);
}

TEST(MessageMapping, GotoResultCarriesPlanningFailed) {
  kinova::interface::TrajectoryResult r;
  r.error_code = kinova::interface::result_code::kPlanningFailed;
  r.error_string = "no plan";
  r.final_error = kinova::JointVec::Zero();
  auto m = to_goto_result_msg(r);
  EXPECT_EQ(m.error_code, -7);
  EXPECT_EQ(m.error_string, "no plan");
  ASSERT_EQ(m.final_error.positions.size(), 7u);
}

TEST(MessageMapping, GotoExecutingFeedback) {
  kinova::interface::TrajectoryFeedback fb;
  fb.fraction_complete = 0.5;
  fb.actual = kinova::JointVec::Constant(0.2);
  auto m = to_goto_feedback_msg(fb);
  EXPECT_EQ(m.phase, "executing");
  EXPECT_NEAR(m.fraction_complete, 0.5f, 1e-6);
  ASSERT_EQ(m.actual.positions.size(), 7u);
  EXPECT_NEAR(m.actual.positions[0], 0.2, 1e-12);
}

// --- issue #13: the planner's velocity/acceleration profile must survive the
// --- mapping, since the driver picks its interpolation order from these flags.
namespace {
// A point with positions plus an optionally-sized velocity/acceleration profile.
trajectory_msgs::msg::JointTrajectoryPoint prof_pt(double v, double t, size_t n_vel, size_t n_acc) {
  trajectory_msgs::msg::JointTrajectoryPoint p = pt(v, t);
  p.velocities.assign(n_vel, v * 2.0);
  p.accelerations.assign(n_acc, v * 3.0);
  return p;
}
}  // namespace

TEST(MessageMapping, PositionsOnlyLeavesInterpolationLinear) {
  trajectory_msgs::msg::JointTrajectory traj;
  traj.points = { pt(0.0, 0.0), pt(0.5, 2.0) };      // no velocities/accelerations
  auto tg = to_trajectory_goal(traj);
  EXPECT_FALSE(tg.trajectory.has_velocities);
  EXPECT_FALSE(tg.trajectory.has_accelerations);
}

TEST(MessageMapping, CarriesFullVelocityAndAccelerationProfile) {
  trajectory_msgs::msg::JointTrajectory traj;
  traj.points = { prof_pt(0.0, 0.0, 7, 7), prof_pt(0.5, 2.0, 7, 7) };
  auto tg = to_trajectory_goal(traj);
  ASSERT_EQ(tg.trajectory.points.size(), 2u);
  EXPECT_TRUE(tg.trajectory.has_velocities);
  EXPECT_TRUE(tg.trajectory.has_accelerations);
  EXPECT_NEAR(tg.trajectory.points[1].qd[0],  1.0, 1e-12);   // 0.5 * 2
  EXPECT_NEAR(tg.trajectory.points[1].qdd[0], 1.5, 1e-12);   // 0.5 * 3
}

TEST(MessageMapping, VelocitiesWithoutAccelerationsGivesCubic) {
  trajectory_msgs::msg::JointTrajectory traj;
  traj.points = { prof_pt(0.0, 0.0, 7, 0), prof_pt(0.5, 2.0, 7, 0) };
  auto tg = to_trajectory_goal(traj);
  EXPECT_TRUE(tg.trajectory.has_velocities);
  EXPECT_FALSE(tg.trajectory.has_accelerations);
  EXPECT_NEAR(tg.trajectory.points[1].qd[0], 1.0, 1e-12);
}

TEST(MessageMapping, PartialOrMissizedProfileIsTreatedAsAbsent) {
  trajectory_msgs::msg::JointTrajectory partial;      // second point has no velocities
  partial.points = { prof_pt(0.0, 0.0, 7, 7), prof_pt(0.5, 2.0, 0, 0) };
  auto tg = to_trajectory_goal(partial);
  EXPECT_FALSE(tg.trajectory.has_velocities) << "a partial profile must not be trusted";
  EXPECT_FALSE(tg.trajectory.has_accelerations);

  trajectory_msgs::msg::JointTrajectory wrong_width;  // 6 velocities for a 7-DOF arm
  wrong_width.points = { prof_pt(0.0, 0.0, 6, 6), prof_pt(0.5, 2.0, 6, 6) };
  auto tg2 = to_trajectory_goal(wrong_width);
  EXPECT_FALSE(tg2.trajectory.has_velocities);
  EXPECT_FALSE(tg2.trajectory.has_accelerations);

  // accelerations without velocities cannot select quintic on their own
  trajectory_msgs::msg::JointTrajectory acc_only;
  acc_only.points = { prof_pt(0.0, 0.0, 0, 7), prof_pt(0.5, 2.0, 0, 7) };
  auto tg3 = to_trajectory_goal(acc_only);
  EXPECT_FALSE(tg3.trajectory.has_velocities);
  EXPECT_FALSE(tg3.trajectory.has_accelerations);
}

TEST(MessageMapping, ExecuteJointTrajectoryGoalAlsoCarriesTheProfile) {
  kinova_arm_interfaces::action::ExecuteJointTrajectory::Goal g;
  g.trajectory.points = { prof_pt(0.0, 0.0, 7, 7), prof_pt(0.5, 2.0, 7, 7) };
  g.control_mode = 0;
  auto tg = to_trajectory_goal(g);
  EXPECT_TRUE(tg.trajectory.has_velocities);
  EXPECT_TRUE(tg.trajectory.has_accelerations);
  EXPECT_NEAR(tg.trajectory.points[1].qd[0], 1.0, 1e-12);
}
