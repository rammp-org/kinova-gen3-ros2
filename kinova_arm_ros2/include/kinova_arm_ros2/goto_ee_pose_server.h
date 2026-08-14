#pragma once
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kinova_arm_interfaces/action/go_to_ee_pose.hpp"
#include "kinova_arm_ros2/curobo_plan_client.h"
#include "kinova_arm_ros2/goal_router.h"
#include "kinova_lowlevel/interface/ports.h"
namespace kinova_arm_ros2 {

// Hosts GoToEEPose: validate -> cuRobo plan -> feed the planned trajectory into the
// shared CommandSink seam (same path as ExecuteJointTrajectory) -> settle. Implements
// ActionServerPort for its OWN goals; the GoalRouter routes the Supervisor's
// execution feedback/settle back here by GoalId.
class GoToEEPoseServer : public kinova::interface::ActionServerPort {
 public:
  using Action = kinova_arm_interfaces::action::GoToEEPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  GoToEEPoseServer(rclcpp::Node::SharedPtr node, GoalRouter& router,
                   CuroboPlanClient& planner, rclcpp::CallbackGroup::SharedPtr cb_group);
  void set_command_sink(kinova::interface::CommandSink* sink) { sink_ = sink; }

  // ActionServerPort (called by the supervisor sampler thread via the router):
  void publish_feedback(const kinova::interface::GoalId&,
                        const kinova::interface::TrajectoryFeedback&) override;
  void settle(const kinova::interface::GoalId&,
              const kinova::interface::TrajectoryResult&) override;

 private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const Action::Goal>);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
  void handle_accepted(std::shared_ptr<GoalHandle>);
  void on_plan_done(kinova::interface::GoalId id, CuroboPlanClient::Outcome outcome);
  void settle_local(std::shared_ptr<GoalHandle> gh, int error_code, const std::string& msg);

  static constexpr double kGotoPathTolRad = 0.35;  // generous; full-speed tracking lag

  rclcpp::Node::SharedPtr node_;
  GoalRouter& router_;
  CuroboPlanClient& planner_;
  kinova::interface::CommandSink* sink_ = nullptr;
  rclcpp_action::Server<Action>::SharedPtr server_;

  struct Goal { std::shared_ptr<GoalHandle> gh; bool executing = false; };
  std::mutex m_;
  std::map<kinova::interface::GoalId, Goal> goals_;
};
}  // namespace kinova_arm_ros2
