// The driver's model arrives on /arm/robot_description, not as a file.
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "kinova_gen3_ros2/robot_description.h"

using namespace std::chrono_literals;
using kinova_gen3_ros2::joint_state_names;
using kinova_gen3_ros2::wait_for_robot_description;
using kinova_gen3_ros2::watch_robot_description;

namespace {
// Smallest URDF urdf::Model accepts, with one named revolute joint.
std::string urdf_with(const std::string &joint) {
  return "<robot name='t'><link name='a'/><link name='b'/>"
         "<joint name='" +
         joint +
         "' type='revolute'><parent link='a'/><child link='b'/>"
         "<axis xyz='0 0 1'/><limit lower='0' upper='0.8' effort='1' "
         "velocity='1'/></joint></robot>";
}
const std::vector<std::string> kArm = {
    "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4",
    "arm_joint_5", "arm_joint_6", "arm_joint_7"};
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
latched_publisher(const rclcpp::Node::SharedPtr &n) {
  return n->create_publisher<std_msgs::msg::String>(
      "/arm/robot_description", rclcpp::QoS(1).reliable().transient_local());
}
} // namespace

class RobotDescriptionTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }
};

// Published BEFORE the driver subscribes -- arm_state_publisher started first.
// Only a transient-local subscription still gets it.
TEST_F(RobotDescriptionTest, ALatchedDescriptionPublishedEarlierArrives) {
  auto pub_node = std::make_shared<rclcpp::Node>("fake_asp_early");
  auto pub = latched_publisher(pub_node);
  std_msgs::msg::String m;
  m.data = urdf_with("arm_robotiq_85_left_knuckle_joint");
  pub->publish(m);

  auto node = std::make_shared<rclcpp::Node>("driver_early");
  const auto got = wait_for_robot_description(node, 5s);
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(*got, m.data);
}

// Published AFTER the driver starts waiting -- the driver started first.
TEST_F(RobotDescriptionTest, PublishedAfterTheDriverStartsWaitingArrives) {
  auto pub_node = std::make_shared<rclcpp::Node>("fake_asp_late");
  auto pub = latched_publisher(pub_node);
  std::thread late([&pub] {
    std::this_thread::sleep_for(500ms);
    std_msgs::msg::String m;
    m.data = "late";
    pub->publish(m);
  });
  auto node = std::make_shared<rclcpp::Node>("driver_late");
  const auto got = wait_for_robot_description(node, 5s);
  late.join();
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(*got, "late");
}

TEST_F(RobotDescriptionTest, NoPublisherTimesOut) {
  auto node = std::make_shared<rclcpp::Node>("driver_alone");
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(wait_for_robot_description(node, 300ms).has_value());
  EXPECT_GE(std::chrono::steady_clock::now() - t0, 300ms);
}

TEST(JointStateNames, ArmJointsThenTheGripperJoint) {
  const auto names =
      joint_state_names(urdf_with("arm_robotiq_85_left_knuckle_joint"), kArm);
  ASSERT_EQ(names.size(), 8u);
  EXPECT_EQ(names.front(), "arm_joint_1");
  EXPECT_EQ(names[6], "arm_joint_7");
  EXPECT_EQ(names.back(), "arm_robotiq_85_left_knuckle_joint");
}

// An unprefixed model on the topic: refuse, and say which joint.
TEST(JointStateNames, AModelWithoutTheGripperJointIsRefusedByName) {
  try {
    joint_state_names(urdf_with("robotiq_85_left_knuckle_joint"), kArm);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("arm_robotiq_85_left_knuckle_joint"),
              std::string::npos)
        << e.what();
  }
}

TEST(JointStateNames, GarbageIsRefused) {
  EXPECT_THROW(joint_state_names("not xml", kArm), std::runtime_error);
}

// After startup the driver keeps listening: the latched copy of its own model
// is not a change; a different model is, every time it is published.
TEST_F(RobotDescriptionTest, OnlyADifferentModelIsReportedAsAChange) {
  auto pub_node = std::make_shared<rclcpp::Node>("fake_asp_watch");
  auto pub = latched_publisher(pub_node);
  std_msgs::msg::String m;
  m.data = "model A";
  pub->publish(m);

  auto node = std::make_shared<rclcpp::Node>("driver_watch");
  std::atomic<int> changes{0};
  auto watch = watch_robot_description(node, "model A", [&] { ++changes; });
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  auto spin_for = [&](std::chrono::milliseconds d) {
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end)
      ex.spin_once(20ms);
  };
  spin_for(500ms);
  EXPECT_EQ(changes.load(), 0); // the latched copy of the model in use

  m.data = "model B";
  pub->publish(m);
  spin_for(500ms);
  EXPECT_EQ(changes.load(), 1);

  pub->publish(m); // still different: still reported
  spin_for(500ms);
  EXPECT_EQ(changes.load(), 2);
  ex.remove_node(node);
}
