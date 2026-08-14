// kinova_arm_ros2/src/bringup_node.cpp
#include <atomic>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "kinova_arm_ros2/ros2_backend.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/interface/supervisor.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/transport.h"
#ifndef KINOVA_NO_KORTEX
#include "kinova_lowlevel/kortex_transport.h"
#endif
using namespace kinova;

namespace { std::atomic<bool> g_stop{false}; void on_sigint(int){ g_stop.store(true); } }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::string urdf = "models/gen3_7dof_2f85.urdf", ip;
  bool use_sim = false; int cpu = -1, prio = 80; double rate = 1000.0;
  for (int i = 1; i < argc; ++i) { std::string a = argv[i];
    auto nxt = [&]{ return std::string(argv[++i]); };
    if (a == "--sim") use_sim = true; else if (a == "--ip") ip = nxt();
    else if (a == "--urdf") urdf = nxt(); else if (a == "--cpu") cpu = std::stoi(nxt());
    else if (a == "--rt-priority") prio = std::stoi(nxt()); else if (a == "--rate") rate = std::stod(nxt()); }

  Dynamics dyn(urdf), pump_dyn(urdf);
  std::unique_ptr<Transport> base;
  if (use_sim) { JointFeedback init; base = std::make_unique<SimTransport>(init); }
  else {
#ifndef KINOVA_NO_KORTEX
    if (ip.empty()) { RCLCPP_ERROR(rclcpp::get_logger("kinova_arm_node"), "real mode needs --ip"); return 2; }
    base = std::make_unique<KortexTransport>(ip);
#else
    RCLCPP_ERROR(rclcpp::get_logger("kinova_arm_node"), "built without KORTEX; use --sim"); return 2;
#endif
  }
  Seqlock<JointFeedback> snap; FeedbackTap tap(*base, snap);

  JointPositionMode pos(dyn); JointImpedanceMode imp(dyn);
  SampleRing ring(1u << 16);
  RtExecutor exec(tap, ring, {rate, Pacing::kSleepSpin, {prio, cpu, true}});

  auto node = std::make_shared<rclcpp::Node>("kinova_arm_node");
  auto backend = std::make_shared<kinova_arm_ros2::Ros2Backend>(node);
  interface::Supervisor sup(pos, imp, exec, snap, pump_dyn, *backend, *backend);
  backend->set_command_sink(&sup);

  // Handle both SIGINT (Ctrl-C) and SIGTERM (e.g. `kill`/`kill %job`, which
  // defaults to SIGTERM, not SIGINT) — rclcpp's own default handler logs and
  // begins context shutdown on either signal, but only setting g_stop actually
  // unblocks the RT loop (exec.run) blocking the main thread.
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  tap.connect(); tap.set_servoing_low_level();
  sup.start();

  std::thread ros_spin([&]{ rclcpp::executors::SingleThreadedExecutor ex; ex.add_node(node);
    while (!g_stop.load() && rclcpp::ok()) ex.spin_some(std::chrono::milliseconds(10)); });
  std::thread drain([&]{ CycleSample s; while (!g_stop.load()) { while (ring.pop(s)) {} std::this_thread::sleep_for(std::chrono::milliseconds(5)); } while (ring.pop(s)) {} });

  RCLCPP_INFO(node->get_logger(), "kinova_arm_node up (%s); action: /execute_joint_trajectory", use_sim ? "sim" : "real");
  exec.run(g_stop);            // RT loop on the main thread; returns when g_stop set

  sup.stop(); base->safe_shutdown();
  ros_spin.join(); drain.join(); rclcpp::shutdown();
  return 0;
}
