// kinova_gen3_ros2/src/bringup_node.cpp
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "kinova_gen3_ros2/ros2_backend.h"
#include "kinova_gen3_ros2/robot_description.h"
#include "kinova_gen3_ros2/curobo_plan_client.h"
#include "kinova_gen3_ros2/goal_router.h"
#include "kinova_gen3_ros2/goto_ee_pose_server.h"
#include "kinova_gen3_ros2/goto_ee_poses_server.h"
#include "kinova_gen3_ros2/goto_joint_config_server.h"
#include "kinova_gen3_ros2/goto_joint_configs_server.h"
#include "kinova_gen3_ros2/goto_preset_server.h"
#include "kinova_gen3_ros2/arbitration_server.h"
#include "kinova_gen3_ros2/gripper_server.h"
#include "kinova_gen3_ros2/link_server.h"
#include "kinova_gen3_ros2/stream_server.h"
#include "kinova_lowlevel/arm_link.h"
#include "kinova_lowlevel/dynamics.h"
#include "kinova_lowlevel/feedback_tap.h"
#include "kinova_lowlevel/gripper_controller.h"
#include "kinova_lowlevel/interface/arbiter.h"
#include "kinova_lowlevel/interface/supervisor.h"
#include "kinova_lowlevel/joint_impedance_mode.h"
#include "kinova_lowlevel/joint_position_mode.h"
#include "kinova_lowlevel/joint_torque_mode.h"
#include "kinova_lowlevel/joint_velocity_mode.h"
#include "kinova_lowlevel/rt_executor.h"
#include "kinova_lowlevel/sim_transport.h"
#include "kinova_lowlevel/telemetry.h"
#include "kinova_lowlevel/telemetry_consumers.h"
#include "kinova_lowlevel/transport.h"
#ifndef KINOVA_NO_KORTEX
#include "kinova_lowlevel/kortex_transport.h"
#endif
using namespace kinova;

namespace {
std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

// How long to wait for arm_state_publisher's model. sheppy starts containers
// concurrently, so it may come up a little after us; the topic is latched, so
// order does not matter within this window.
constexpr std::chrono::seconds kDescriptionTimeout{30};
} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::string ip;
  // Dynamics resolves the EE frame BY NAME and throws if it is absent. The
  // model comes from /arm/robot_description, where every link carries the arm_
  // prefix. Always passed explicitly: core's own default names its test model.
  std::string ee_frame = "arm_end_effector_link";
  bool use_sim = false;
  int cpu = -1, prio = 80;
  double rate = 1000.0;
  double max_ref_speed = 0.0; // <=0 => seed from the URDF velocity limits
  // Per-cycle RT samples to a CSV. Empty => summary line only. The samples are
  // measured either way; before this they were drained and discarded, which
  // left the node unable to answer whether a stutter was ITS fault or the
  // plan's.
  std::string rt_csv;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto nxt = [&] { return std::string(argv[++i]); };
    if (a == "--sim")
      use_sim = true;
    else if (a == "--ip")
      ip = nxt();
    else if (a == "--cpu")
      cpu = std::stoi(nxt());
    else if (a == "--ee-frame")
      ee_frame = nxt();
    else if (a == "--rt-csv")
      rt_csv = nxt();
    else if (a == "--rt-priority")
      prio = std::stoi(nxt());
    else if (a == "--rate")
      rate = std::stod(nxt());
    else if (a == "--max-ref-speed")
      max_ref_speed = std::stod(nxt());
    else if (a == "--urdf") {
      // Rejected, not ignored: a stale flag must not look like the operator
      // chose the model.
      std::fprintf(stderr,
                   "--urdf was removed: the model comes from "
                   "/arm/robot_description (published by arm_state_publisher / "
                   "description.launch.py mount_file:=<mount.yaml>)\n");
      rclcpp::shutdown();
      return 2;
    }
  }

  // Fetch the model from arm_state_publisher BEFORE anything real-time exists:
  // the RT loop must never wait on DDS. It is rendered from the robot's mount
  // file, so gravity compensation, FK and the Jacobian are all in arm_mount.
  auto node = std::make_shared<rclcpp::Node>("kinova_gen3_node");
  const auto urdf_xml =
      kinova_gen3_ros2::wait_for_robot_description(node, kDescriptionTimeout);
  if (!urdf_xml) {
    if (rclcpp::ok()) // a SIGTERM during the wait is a clean stop, not this
      RCLCPP_FATAL(node->get_logger(),
                   "no model on %s after %lld s. It is published by "
                   "arm_state_publisher: ros2 launch kinova_gen3_description "
                   "description.launch.py mount_file:=<robot>/mount.yaml",
                   kinova_gen3_ros2::kRobotDescriptionTopic,
                   static_cast<long long>(kDescriptionTimeout.count()));
    // SIGTERM (docker stop) during the wait is a requested stop: exit 0. Read
    // before shutdown(), which would make ok() false either way.
    const bool interrupted = !rclcpp::ok();
    rclcpp::shutdown();
    return interrupted ? 0 : 1;
  }
  // Handle both SIGINT (Ctrl-C) and SIGTERM (e.g. `kill`/`kill %job`, which
  // defaults to SIGTERM, not SIGINT) -- rclcpp's own default handler logs and
  // begins context shutdown on either signal, but only setting g_stop actually
  // unblocks the RT loop (link.run) blocking the main thread. Installed here,
  // right after the wait (which relies on rclcpp's handler), and not after the
  // servers below: a signal under rclcpp's handler in between would shut the
  // context down under the entities still being created, and they throw.
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);
  // Core reduces the 13-DOF model to the seven joints on the chain to
  // ee_frame, locking the gripper at q = 0, which lumps its mass into the
  // wrist.
  // Built inside the try: Dynamics throws on a model it cannot use (ee_frame
  // absent, e.g. an unprefixed model on the topic), and that must reach the
  // FATAL below rather than std::terminate. Emplaced once and never reset, so
  // every reference taken below stays valid for the process lifetime.
  std::optional<Dynamics> dyn, pump_dyn;
  std::vector<std::string> js_names;
  try {
    dyn.emplace(UrdfXml{*urdf_xml}, ee_frame);
    pump_dyn.emplace(UrdfXml{*urdf_xml}, ee_frame);
    js_names =
        kinova_gen3_ros2::joint_state_names(*urdf_xml, dyn->joint_names());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(node->get_logger(), "model on %s: %s",
                 kinova_gen3_ros2::kRobotDescriptionTopic, e.what());
    rclcpp::shutdown();
    return 1;
  }
  // The driver's own record of the model it compensates with. The embodiment
  // line is in arm_state_publisher's log; this one is in ours, and the tilt is
  // read from the model, so a wrong robot shows here before torque is on.
  {
    std::string names;
    for (const auto &n : dyn->joint_names())
      names += (names.empty() ? "" : ",") + n;
    RCLCPP_INFO(
        node->get_logger(),
        "model from %s: mount tilt %.1f deg from vertical, ee_frame %s, "
        "joints [%s]",
        kinova_gen3_ros2::kRobotDescriptionTopic, dyn->mount_tilt_deg(),
        ee_frame.c_str(), names.c_str());
  }
  // The model is read once. A different one appearing later
  // (arm_state_publisher restarted with another mount, a second publisher)
  // would move TF -- and a restarted cuRobo -- away from the gravity this
  // driver keeps using, so say so, every time, until the driver is restarted.
  auto description_watch = kinova_gen3_ros2::watch_robot_description(
      node, *urdf_xml, [logger = node->get_logger()] {
        RCLCPP_ERROR(logger,
                     "a DIFFERENT model was published on %s; this driver is "
                     "still using the one it started with (TF may now disagree "
                     "with its gravity). Restart the driver.",
                     kinova_gen3_ros2::kRobotDescriptionTopic);
      });
  std::unique_ptr<Transport> base;
  if (use_sim) {
    JointFeedback init;
    base = std::make_unique<SimTransport>(init);
  } else {
#ifndef KINOVA_NO_KORTEX
    if (ip.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("kinova_gen3_node"),
                   "real mode needs --ip");
      return 2;
    }
    // Every 1 kHz exchange() waits on the KORTEX SDK's UDP receive thread, so
    // it is created one priority below the RT loop, on the RT core. Without
    // it, it was pre-empted ~10k times in 30 min on flatbot (2026-10-22),
    // delaying replies up to 15 ms.
    base = std::make_unique<KortexTransport>(ip, "admin", "admin",
                                             prio > 1 ? prio - 1 : prio, cpu);
#else
    RCLCPP_ERROR(rclcpp::get_logger("kinova_gen3_node"),
                 "built without KORTEX; use --sim");
    return 2;
#endif
  }
  Seqlock<JointFeedback> snap;
  // Decorates Transport to stamp the gripper field into every outgoing frame.
  // NOT a ControlMode: modes are mutually exclusive, so making the gripper one
  // would mean giving up arm control to move it. Order follows core's own idiom
  // in apps/teleop_socket_server.cpp -- GripperController first, then
  // FeedbackTap.
  GripperController grip(*base);
  FeedbackTap tap(grip, snap);

  // Seed the reference rate limit from the URDF instead of taking
  // JointPositionParams' 0.5 rad/s default. That default is a conservative
  // bring-up value (trajectory_run overrides it from a CLI flag); left in place
  // here it throttles every joint to ~0.4x of what the arm can do, so any
  // planned trajectory faster than that is tracked late and per-joint by a
  // DIFFERENT amount — joints stop arriving together. Worse, the divergence
  // guard compares measured q against the PLANNED sample while the mode
  // commands the rate-limited reference, so the throttle manufactures the very
  // divergence that aborts the goal with PATH_TOLERANCE_VIOLATED.
  // --max-ref-speed <rad/s> still forces a slower cap for cautious on-robot
  // runs.
  JointPositionParams pos_params;
  if (max_ref_speed > 0.0)
    pos_params.max_ref_speed.setConstant(max_ref_speed);
  else
    dyn->velocity_limits(pos_params.max_ref_speed);
  JointPositionMode pos(*dyn, pos_params);
  JointImpedanceMode imp(*dyn);
  // The Supervisor holds all three modes because the streaming tier can open a
  // session in any of them. This node drives only trajectories (position and
  // impedance), so tau is never entered here -- but the supervisor needs it to
  // construct, and it defaults to gravity compensation with no tau_ff set.
  JointTorqueMode tau(*dyn);
  // The Supervisor holds every mode a streaming session can open in. Velocity
  // mode is STIFF by contract -- it tracks the commanded rate and does not
  // yield to contact -- and its max_qd is seeded from the URDF when left
  // non-finite, so the default here is already the hardware's rating rather
  // than a guess.
  JointVelocityMode vel(*dyn);
  SampleRing ring(1u << 16);
  ExecutorConfig exec_cfg{rate, Pacing::kSleepSpin, {prio, cpu, true}};
  // A fault confirmed in feedback ends the RT run so ArmLink can hand the arm
  // back to its base and wait for /clear_faults (ArmLink requires it).
  exec_cfg.exit_on_fault = true;
  RtExecutor exec(tap, ring, exec_cfg);

  auto backend =
      std::make_shared<kinova_gen3_ros2::Ros2Backend>(node, js_names);

  // READ-ONLY on purpose: core's ArbitrationMode is a constructor argument with
  // no setter, so a dynamic parameter would appear to work and silently do
  // nothing. Defaults to "disabled" so every existing client and script keeps
  // working; that costs no safety, because estop() latches over BOTH modes.
  rcl_interfaces::msg::ParameterDescriptor mode_desc;
  mode_desc.read_only = true;
  mode_desc.description =
      "enforced | disabled. Launch-time only; core has no setter.";
  const std::string mode_str =
      node->declare_parameter("arbitration_mode", "disabled", mode_desc);
  if (mode_str != "enforced" && mode_str != "disabled") {
    RCLCPP_FATAL(node->get_logger(),
                 "arbitration_mode must be 'enforced' or 'disabled', got '%s'",
                 mode_str.c_str());
    return 1;
  }
  const auto arb_mode = (mode_str == "enforced")
                            ? interface::ArbitrationMode::kEnforced
                            : interface::ArbitrationMode::kDisabled;
  const double estop_clear_max_age_s =
      node->declare_parameter("estop_clear_max_age_s", 1.0);
  // "Expected" cannot be inferred from the node's own model: core reduces it to
  // the seven arm joints, so it never has a gripper DOF regardless of the
  // hardware. A robot genuinely built without one sets this false -- but its
  // model must still declare arm_robotiq_85_left_knuckle_joint, which
  // joint_state_names() requires: a gripper:=none render cannot start the node
  // today (gripper is not a launch argument, so no deployment renders one).
  const bool expect_gripper = node->declare_parameter("expect_gripper", true);

  // Router demuxes the Supervisor's single ActionServerPort by GoalId; the
  // pre-existing ExecuteJointTrajectory backend is the default (fall-through)
  // port.
  kinova_gen3_ros2::GoalRouter router(*backend);
  // Async cuRobo planning + the high-level server run on a reentrant group so
  // the plan round-trip never starves the ExecuteJointTrajectory
  // server/feedback.
  auto cb_group =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  kinova_gen3_ros2::CuroboPlanClient planner(node, cb_group);
  // All the high-level servers share the router, planner and group, and must
  // be declared before `sup` so they outlive it (the router hands it their
  // ports).
  kinova_gen3_ros2::GoToEEPoseServer goto_server(node, router, planner,
                                                 cb_group);
  kinova_gen3_ros2::GoToJointConfigServer jc_server(node, router, planner,
                                                    cb_group);
  kinova_gen3_ros2::GoToPresetServer preset_server(
      node, router, planner, cb_group,
      kinova_gen3_ros2::load_preset_registry(*node));
  kinova_gen3_ros2::GoToEEPosesServer poses_server(node, router, planner,
                                                   cb_group);
  kinova_gen3_ros2::GoToJointConfigsServer jcs_server(node, router, planner,
                                                      cb_group);

  // Core takes a SupervisorDeps aggregate rather than positional arguments, so
  // the mode/port wiring reads by name.
  interface::SupervisorDeps deps;
  deps.pos = &pos;
  deps.imp = &imp;
  deps.tau = &tau;
  deps.vel = &vel;
  deps.exec = &exec;
  deps.snap = &snap;
  deps.pump_dyn = &*pump_dyn;
  deps.stream =
      backend.get();     // Ros2Backend publishes /joint_states + /ee_state
  deps.action = &router; // GoalRouter demuxes the single ActionServerPort
  deps.grip = &grip;     // was nullptr: every setpoint was silently dropped
  interface::Supervisor sup(deps);
  // Supervisor implements CommandSink, StreamSink AND GripperSink, so it is
  // passed three times -- the idiom core's own tests use (Arbiter arb{sink,
  // sink, sink, mode, seed}).
  interface::Arbiter arb(sup, sup, sup, arb_mode);
  // Declared AFTER arb so it is destroyed FIRST: it must stop accepting ROS
  // calls before arb stops delegating, and arb before sup goes away. (The
  // servers are declared before sup and so outlive all of it, as the comment
  // above requires.)
  kinova_gen3_ros2::ArbitrationServer arbitration_server(
      node, arb, use_sim ? std::string("sim") : ip, estop_clear_max_age_s);
  // Wired to the Arbiter, not the Supervisor: that is how setpoint tokens get
  // checked at all. Declared here so it is destroyed before arb stops
  // delegating.
  kinova_gen3_ros2::StreamServer stream_server(node, arb);
  // Wired to the Arbiter, exactly like StreamServer: that is what makes the
  // gripper's token load-bearing. There is no session to open --
  // Arbiter::on_gripper_setpoint gates on admit(token) alone.
  kinova_gen3_ros2::GripperServer gripper_server(node, arb, expect_gripper);
  backend->set_gripper_sink(&arb);
  // 20 Hz is enough for a gripper and keeps the pump thread free of a second
  // publisher; Ros2Backend::publish_state stamps /joint_states from node->now()
  // the same way.
  auto gripper_timer = node->create_wall_timer(
      std::chrono::milliseconds(50),
      [&gripper_server, node]() { gripper_server.publish_state(node->now()); });

  // Every command path now goes through the Arbiter rather than straight to the
  // Supervisor. Nothing else about the servers changes.
  backend->set_command_sink(&arb);
  goto_server.set_command_sink(&arb);
  jc_server.set_command_sink(&arb);
  preset_server.set_command_sink(&arb);
  poses_server.set_command_sink(&arb);
  jcs_server.set_command_sink(&arb);

  // The link to the arm: connects (retrying forever while the arm is off or
  // the link is down), runs the RT loop, and on a fault hands the arm back to
  // its base and waits for /clear_faults. The Supervisor listens, so goals halt
  // and work is refused while the arm is unavailable.
  ArmLink link(tap, exec, &sup);
  kinova_gen3_ros2::LinkServer link_server(
      node, link, use_sim ? std::string("sim") : ip, g_stop);

  // A signal during setup is a requested stop: do not connect to the arm.
  if (g_stop.load()) {
    rclcpp::shutdown();
    return 0;
  }
  sup.start();

  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  std::thread ros_spin([&] { ex.spin(); });
  // Drain thread. It used to pop samples and discard them, so the node measured
  // its own 1 kHz timing and then threw the measurement away -- the one thing
  // you need when someone reports a stutter. All formatting and file I/O
  // happens HERE, off the RT thread, which is the whole point of the ring.
  std::thread drain([&] {
    TelemetrySink sink(rt_csv);
    CycleSample s;
    auto last = std::chrono::steady_clock::now();
    while (!g_stop.load()) {
      while (ring.pop(s))
        sink.consume(s);
      const auto now = std::chrono::steady_clock::now();
      if (now - last >= std::chrono::seconds(5)) {
        // Percentiles are cumulative since startup, not per interval; the CSV
        // is the one to read for a rolling view of a specific run.
        RCLCPP_INFO(node->get_logger(), "rt: %s dropped=%llu",
                    sink.console_line().c_str(),
                    static_cast<unsigned long long>(ring.dropped()));
        last = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    while (ring.pop(s))
      sink.consume(s);
    RCLCPP_INFO(node->get_logger(), "rt final: %s dropped=%llu",
                sink.console_line().c_str(),
                static_cast<unsigned long long>(ring.dropped()));
  });

  RCLCPP_INFO(
      node->get_logger(),
      "kinova_gen3_node up (%s); arbitration=%s; actions: "
      "/execute_joint_trajectory, "
      "/go_to_ee_pose, /go_to_joint_config, /go_to_preset, /go_to_ee_poses, "
      "/go_to_joint_configs; control: "
      "/acquire_control, /release_control, /revoke_control, /estop, "
      "/control_status; "
      "streaming: /open_stream, /close_stream, /list_controllers, /setpoint/*, "
      "/stream_status; gripper: /setpoint/gripper, /gripper_state; "
      "link: /clear_faults",
      use_sim ? "sim" : "real", mode_str.c_str());
  RCLCPP_INFO(node->get_logger(),
              "max_ref_speed [rad/s] = %.2f %.2f %.2f %.2f %.2f %.2f %.2f (%s)",
              pos_params.max_ref_speed[0], pos_params.max_ref_speed[1],
              pos_params.max_ref_speed[2], pos_params.max_ref_speed[3],
              pos_params.max_ref_speed[4], pos_params.max_ref_speed[5],
              pos_params.max_ref_speed[6],
              max_ref_speed > 0.0 ? "--max-ref-speed" : "URDF limits");
  // The RT loop runs on the main thread inside the link; returns when g_stop
  // is set. Arm-side events (fault, disconnect, arm off) never get out of it:
  // only a configuration error does.
  int rc = 0;
  try {
    link.run(g_stop);
  } catch (const std::logic_error &e) {
    RCLCPP_FATAL(node->get_logger(), "this driver cannot drive this arm: %s",
                 e.what());
    rc = 1;
    g_stop.store(true); // the drain thread runs until g_stop
  }

  sup.stop();
  base->safe_shutdown();
  ex.cancel();
  ros_spin.join();
  drain.join();
  rclcpp::shutdown();
  return rc;
}
