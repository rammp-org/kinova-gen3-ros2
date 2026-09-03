# What the gripper reports. The units-bearing home for what cannot honestly go in
# sensor_msgs/JointState: see /joint_states, where gripper effort and velocity are NaN.
std_msgs/Header header
float32 position   # 0 (open) .. 1 (closed), normalized
# 0..1, |current| / 1.0 A. NOT Newtons -- MotorFeedback carries no force field.
# A SUSTAINED grasp reports a SMALL effort (~0.05): the current spikes while the fingers
# close on the object, then settles to a low holding current. Anything keying off "high
# effort means holding something" is wrong.
float32 effort
float32 current    # amps, raw, exactly as reported
# False when no gripper is attached OR no GripperController is wired. Core conflates the
# two deliberately: both mean there is nothing to report. Without this, a missing gripper
# and a fully-open one are both position 0.
bool    present
