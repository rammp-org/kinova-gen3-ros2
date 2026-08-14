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

TEST(MessageMapping, ResultCarriesErrorCode) {
  kinova::interface::TrajectoryResult r; r.error_code = -4; r.error_string = "path tol";
  r.final_error = kinova::JointVec::Constant(0.01);
  auto m = to_result_msg(r);
  EXPECT_EQ(m.error_code, -4);
  EXPECT_EQ(m.error_string, "path tol");
  ASSERT_EQ(m.final_error.positions.size(), 7u);
  EXPECT_NEAR(m.final_error.positions[0], 0.01, 1e-12);
}
