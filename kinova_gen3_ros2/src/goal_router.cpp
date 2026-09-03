#include "kinova_gen3_ros2/goal_router.h"
namespace kinova_gen3_ros2 {
using namespace kinova::interface;

GoalRouter::GoalRouter(ActionServerPort& default_port) : default_(default_port) {}

void GoalRouter::register_owner(const GoalId& id, ActionServerPort& owner) {
  std::lock_guard<std::mutex> l(m_);
  owners_[id] = &owner;
}

void GoalRouter::publish_feedback(const GoalId& id, const TrajectoryFeedback& fb) {
  ActionServerPort* p;
  { std::lock_guard<std::mutex> l(m_);
    auto it = owners_.find(id);
    p = (it == owners_.end()) ? &default_ : it->second; }
  p->publish_feedback(id, fb);
}

void GoalRouter::settle(const GoalId& id, const TrajectoryResult& r) {
  ActionServerPort* p;
  { std::lock_guard<std::mutex> l(m_);
    auto it = owners_.find(id);
    if (it == owners_.end()) { p = &default_; }
    else { p = it->second; owners_.erase(it); } }   // clear override before settling
  p->settle(id, r);
}
}  // namespace kinova_gen3_ros2
