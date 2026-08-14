// kinova_arm_ros2/src/bringup_node.cpp
#include <atomic>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "kinova_arm_ros2/ros2_backend.h"
#include "kinova_arm_ros2/curobo_plan_client.h"
#include "kinova_arm_ros2/goal_router.h"
#include "kinova_arm_ros2/goto_ee_pose_server.h"
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
  double max_ref_speed = 0.0;          // <=0 => seed from the URDF velocity limits
  for (int i = 1; i < argc; ++i) { std::string a = argv[i];
    auto nxt = [&]{ return std::string(argv[++i]); };
    if (a == "--sim") use_sim = true; else if (a == "--ip") ip = nxt();
    else if (a == "--urdf") urdf = nxt(); else if (a == "--cpu") cpu = std::stoi(nxt());
    else if (a == "--rt-priority") prio = std::stoi(nxt()); else if (a == "--rate") rate = std::stod(nxt());
    else if (a == "--max-ref-speed") max_ref_speed = std::stod(nxt()); }

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

  // Seed the reference rate limit from the URDF instead of taking
  // JointPositionParams' 0.5 rad/s default. That default is a conservative
  // bring-up value (trajectory_run overrides it from a CLI flag); left in place
  // here it throttles every joint to ~0.4x of what the arm can do, so any
  // planned trajectory faster than that is tracked late and per-joint by a
  // DIFFERENT amount — joints stop arriving together. Worse, the divergence
  // guard compares measured q against the PLANNED sample while the mode
  // commands the rate-limited reference, so the throttle manufactures the very
  // divergence that aborts the goal with PATH_TOLERANCE_VIOLATED.
  // --max-ref-speed <rad/s> still forces a slower cap for cautious on-robot runs.
  JointPositionParams pos_params;
  if (max_ref_speed > 0.0) pos_params.max_ref_speed.setConstant(max_ref_speed);
  else                     dyn.velocity_limits(pos_params.max_ref_speed);
  JointPositionMode pos(dyn, pos_params); JointImpedanceMode imp(dyn);
  SampleRing ring(1u << 16);
  RtExecutor exec(tap, ring, {rate, Pacing::kSleepSpin, {prio, cpu, true}});

  auto node = std::make_shared<rclcpp::Node>("kinova_arm_node");
  auto backend = std::make_shared<kinova_arm_ros2::Ros2Backend>(node);

  // Router demuxes the Supervisor's single ActionServerPort by GoalId; the
  // pre-existing ExecuteJointTrajectory backend is the default (fall-through) port.
  kinova_arm_ros2::GoalRouter router(*backend);
  // Async cuRobo planning + the high-level server run on a reentrant group so the
  // plan round-trip never starves the ExecuteJointTrajectory server/feedback.
  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_arm_ros2::CuroboPlanClient planner(node, cb_group);
  kinova_arm_ros2::GoToEEPoseServer goto_server(node, router, planner, cb_group);

  interface::Supervisor sup(pos, imp, exec, snap, pump_dyn, *backend, router);
  backend->set_command_sink(&sup);
  goto_server.set_command_sink(&sup);

  // Handle both SIGINT (Ctrl-C) and SIGTERM (e.g. `kill`/`kill %job`, which
  // defaults to SIGTERM, not SIGINT) — rclcpp's own default handler logs and
  // begins context shutdown on either signal, but only setting g_stop actually
  // unblocks the RT loop (exec.run) blocking the main thread.
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  tap.connect(); tap.set_servoing_low_level();
  sup.start();

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread ros_spin([&]{ ex.spin(); });
  std::thread drain([&]{ CycleSample s; while (!g_stop.load()) { while (ring.pop(s)) {} std::this_thread::sleep_for(std::chrono::milliseconds(5)); } while (ring.pop(s)) {} });

  RCLCPP_INFO(node->get_logger(),
              "kinova_arm_node up (%s); actions: /execute_joint_trajectory, /go_to_ee_pose",
              use_sim ? "sim" : "real");
  RCLCPP_INFO(node->get_logger(),
              "max_ref_speed [rad/s] = %.2f %.2f %.2f %.2f %.2f %.2f %.2f (%s)",
              pos_params.max_ref_speed[0], pos_params.max_ref_speed[1],
              pos_params.max_ref_speed[2], pos_params.max_ref_speed[3],
              pos_params.max_ref_speed[4], pos_params.max_ref_speed[5],
              pos_params.max_ref_speed[6],
              max_ref_speed > 0.0 ? "--max-ref-speed" : "URDF limits");
  exec.run(g_stop);            // RT loop on the main thread; returns when g_stop set

  sup.stop(); base->safe_shutdown();
  ex.cancel();
  ros_spin.join(); drain.join(); rclcpp::shutdown();
  return 0;
}
