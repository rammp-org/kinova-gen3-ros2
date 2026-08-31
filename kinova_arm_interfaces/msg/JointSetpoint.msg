# One shape, three meanings -- the TOPIC says which: rad on /setpoint/joint_position,
# rad/s on /setpoint/joint_velocity, N*m on /setpoint/joint_torque. Mirrors core's
# JointSetpoint, where the METHOD disambiguates for the same reason.
#
# Absolute, never a delta. Re-sending the same value is a no-op, and dropping an
# intermediate setpoint is correct -- which is why these topics are best-effort.
float64[7] values
uint8[16]  token     # from /acquire_control; validates the sender, every message
