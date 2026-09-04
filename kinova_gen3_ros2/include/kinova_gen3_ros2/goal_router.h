#pragma once
#include <map>
#include <mutex>
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_gen3_ros2 {

// ActionServerPort demux. The Supervisor holds ONE ActionServerPort; this fans
// feedback/settle out by GoalId. Unregistered ids fall through to a default
// port (the pre-existing ExecuteJointTrajectory backend); overlay owners (the
// GoToEEPose server) register their ids explicitly. rclcpp-free.
class GoalRouter : public kinova::interface::ActionServerPort {
public:
  explicit GoalRouter(kinova::interface::ActionServerPort &default_port);
  void register_owner(const kinova::interface::GoalId &id,
                      kinova::interface::ActionServerPort &owner);
  void
  publish_feedback(const kinova::interface::GoalId &id,
                   const kinova::interface::TrajectoryFeedback &fb) override;
  void settle(const kinova::interface::GoalId &id,
              const kinova::interface::TrajectoryResult &r) override;

private:
  kinova::interface::ActionServerPort &default_;
  std::mutex m_;
  std::map<kinova::interface::GoalId, kinova::interface::ActionServerPort *>
      owners_;
};
} // namespace kinova_gen3_ros2
