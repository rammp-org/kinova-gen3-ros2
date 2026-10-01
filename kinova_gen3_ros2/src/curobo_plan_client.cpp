#include "kinova_gen3_ros2/curobo_plan_client.h"
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <string>
namespace kinova_gen3_ros2 {
namespace {

// Only joint-space plans carry a joint-space goal to miss.
double mismatch_of(const CuroboPlanClient::PlanToPose::Result &) { return 0.0; }
double mismatch_of(const CuroboPlanClient::PlanToJoints::Result &r) {
  return r.goal_mismatch_rad;
}

// Arm contract -> planner contract. Modes are mapped BY NAME, never by
// assigning the raw integer: the two enumerations are independent contracts
// that happen to agree today, and a silent renumbering on either side would
// otherwise turn LEVEL into FIXED without a compile error.
rammp_curobo_interfaces::msg::OrientationHold
to_planner(const rammp_arm_interfaces::msg::OrientationHold &in) {
  using Arm = rammp_arm_interfaces::msg::OrientationHold;
  using Planner = rammp_curobo_interfaces::msg::OrientationHold;
  Planner out;
  switch (in.hold) {
  case Arm::HOLD_NONE:
    out.hold = Planner::HOLD_NONE;
    break;
  case Arm::HOLD_LEVEL:
    out.hold = Planner::HOLD_LEVEL;
    break;
  case Arm::HOLD_FIXED:
    out.hold = Planner::HOLD_FIXED;
    break;
  default:
    throw std::invalid_argument("unknown orientation hold mode " +
                                std::to_string(in.hold));
  }
  return out;
}

// Shared dispatch for both plan actions. Only the goal type differs; the
// exactly-once contract, the type-erased cancel and the result mapping are
// identical, so they live here once rather than being duplicated per action.
template <typename ActionT>
void dispatch_plan(typename rclcpp_action::Client<ActionT>::SharedPtr client,
                   const typename ActionT::Goal &goal, std::mutex &m,
                   std::function<void()> &active_cancel,
                   CuroboPlanClient::FeedbackCb on_fb,
                   CuroboPlanClient::DoneCb on_done) {
  using GH = rclcpp_action::ClientGoalHandle<ActionT>;
  // Guarantee on_done fires exactly once across the goal-response / result
  // paths.
  auto once = std::make_shared<std::once_flag>();
  auto fire = [once, on_done](CuroboPlanClient::Outcome o) {
    std::call_once(*once, [&] { on_done(std::move(o)); });
  };

  if (!client->wait_for_action_server(std::chrono::milliseconds(200))) {
    fire({false, "cuRobo action server unavailable", {}, 0.0});
    return;
  }

  typename rclcpp_action::Client<ActionT>::SendGoalOptions opts;
  opts.goal_response_callback = [fire, client, &m,
                                 &active_cancel](typename GH::SharedPtr gh) {
    if (!gh) {
      fire({false, "cuRobo rejected plan goal", {}, 0.0});
      return;
    }
    std::lock_guard<std::mutex> l(m);
    active_cancel = [client, gh] { client->async_cancel_goal(gh); };
  };
  opts.feedback_callback =
      [on_fb](typename GH::SharedPtr,
              const std::shared_ptr<const typename ActionT::Feedback> fb) {
        if (on_fb)
          on_fb(fb->state);
      };
  opts.result_callback =
      [fire, &m, &active_cancel](const typename GH::WrappedResult &wr) {
        {
          std::lock_guard<std::mutex> l(m);
          active_cancel = nullptr;
        }
        CuroboPlanClient::Outcome o;
        if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && wr.result &&
            wr.result->success) {
          o.ok = true;
          o.message = wr.result->message;
          o.trajectory = wr.result->trajectory;
          o.goal_mismatch_rad = mismatch_of(*wr.result);
        } else {
          o.ok = false;
          o.message = (wr.result && !wr.result->message.empty())
                          ? wr.result->message
                          : "cuRobo plan failed";
        }
        fire(std::move(o));
      };
  client->async_send_goal(goal, opts);
}

} // namespace

CuroboPlanClient::CuroboPlanClient(rclcpp::Node::SharedPtr node,
                                   rclcpp::CallbackGroup::SharedPtr cb_group,
                                   std::string action_name,
                                   std::string joints_action_name)
    : node_(node) {
  client_ =
      rclcpp_action::create_client<PlanToPose>(node_, action_name, cb_group);
  client_joints_ = rclcpp_action::create_client<PlanToJoints>(
      node_, joints_action_name, cb_group);
}

void CuroboPlanClient::plan(const geometry_msgs::msg::Pose &target,
                            const std::vector<double> &start_joints,
                            FeedbackCb on_fb, DoneCb on_done) {
  plan(target, start_joints, rammp_arm_interfaces::msg::OrientationHold{},
       std::move(on_fb), std::move(on_done));
}

void CuroboPlanClient::plan(
    const geometry_msgs::msg::Pose &target,
    const std::vector<double> &start_joints,
    const rammp_arm_interfaces::msg::OrientationHold &hold, FeedbackCb on_fb,
    DoneCb on_done) {
  PlanToPose::Goal goal;
  goal.target = target;
  goal.start_joints = start_joints; // plan from where the arm actually is
  try {
    goal.hold = to_planner(hold);
  } catch (const std::invalid_argument &e) {
    // An unknown mode must not become a silently different constraint -- and
    // in particular must not become "no hold", which would execute an
    // unconstrained move for a caller who asked for a held one.
    on_done({false, e.what(), {}, 0.0});
    return;
  }
  dispatch_plan<PlanToPose>(client_, goal, m_, active_cancel_, std::move(on_fb),
                            std::move(on_done));
}

void CuroboPlanClient::plan_to_joints(const std::vector<double> &target_joints,
                                      const std::vector<double> &start_joints,
                                      FeedbackCb on_fb, DoneCb on_done) {
  PlanToJoints::Goal goal;
  goal.target_joints = target_joints;
  goal.start_joints = start_joints; // plan from where the arm actually is
  dispatch_plan<PlanToJoints>(client_joints_, goal, m_, active_cancel_,
                              std::move(on_fb), std::move(on_done));
}

void CuroboPlanClient::cancel() {
  std::function<void()> c;
  {
    std::lock_guard<std::mutex> l(m_);
    c = active_cancel_;
  }
  if (c)
    c();
}
} // namespace kinova_gen3_ros2
