#include <gtest/gtest.h>
#include "kinova_arm_ros2/goal_router.h"
using namespace kinova::interface;
namespace {
struct FakePort : ActionServerPort {
  int fb = 0, settled = 0;
  void publish_feedback(const GoalId&, const TrajectoryFeedback&) override { ++fb; }
  void settle(const GoalId&, const TrajectoryResult&) override { ++settled; }
};
GoalId mkid(uint8_t x) { GoalId id{}; id[0] = x; return id; }
}  // namespace

TEST(GoalRouter, UnregisteredFallsThroughToDefault) {
  FakePort def, overlay;
  kinova_arm_ros2::GoalRouter r(def);
  r.publish_feedback(mkid(1), {});
  r.settle(mkid(1), {});
  EXPECT_EQ(def.fb, 1);
  EXPECT_EQ(def.settled, 1);
  EXPECT_EQ(overlay.fb, 0);
  EXPECT_EQ(overlay.settled, 0);
}

TEST(GoalRouter, RegisteredOwnerReceivesFeedbackAndSettle) {
  FakePort def, overlay;
  kinova_arm_ros2::GoalRouter r(def);
  r.register_owner(mkid(2), overlay);
  r.publish_feedback(mkid(2), {});
  r.settle(mkid(2), {});
  EXPECT_EQ(overlay.fb, 1);
  EXPECT_EQ(overlay.settled, 1);
  EXPECT_EQ(def.fb, 0);
  EXPECT_EQ(def.settled, 0);
}

TEST(GoalRouter, SettleClearsOwnerSoNextRoutesToDefault) {
  FakePort def, overlay;
  kinova_arm_ros2::GoalRouter r(def);
  r.register_owner(mkid(3), overlay);
  r.settle(mkid(3), {});             // clears the override
  r.publish_feedback(mkid(3), {});   // now falls through to default
  EXPECT_EQ(overlay.settled, 1);
  EXPECT_EQ(def.fb, 1);
}
