# Cartesian state of the tool, the sibling of /joint_states.
#
# Both fields come from the SAME URDF model and the SAME feedback sample: pose is
# fk(q) and twist is J(q)*qd, computed together in the driver's pump. So they agree
# by construction, and they agree with the model cuRobo plans against.
#
# NOT the arm's own tool_pose/tool_twist feedback. KORTEX reports those in its
# configured tool frame, which carries an offset the URDF does not know about --
# mixing the two sources would give a pose and a twist that disagree by an unknown
# transform.
#
# Frame: LOCAL_WORLD_ALIGNED -- a world-aligned frame at the tool, NOT the body
# frame. Linear and angular parts are both expressed in base_link orientation.
#
# There is deliberately no wrench here. The driver has no force estimate of its own,
# and the arm's tool_external_wrench is in KORTEX's tool frame; publishing a zero or
# a frame-mismatched value would be worse than publishing nothing.
std_msgs/Header header
geometry_msgs/Pose  pose
geometry_msgs/Twist twist
