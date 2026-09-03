# Absolute gripper command. Rides the ARM's token -- one physical machine, one holder.
#
# NOT sticky: core's GripperController::set_target takes all three fields every call and
# does not remember the previous speed/force. A message carrying only `position` commands
# speed and force to the DEFAULTS BELOW, it does not leave them alone.
#
# There is deliberately no `active` field. GripperCommand has one, but it belongs to the
# outgoing frame (kortex_transport reads `if (cmd.gripper.active)`), not to the caller:
# set_target arms stamping unconditionally and discards it. A client sending active=false
# to stop commanding the gripper would get it commanded anyway.
#
# There is no "let go". release() does not open the gripper -- the last command stays
# latched in KORTEX's cyclic frame and the 2F-85 is effectively self-locking. To open,
# command position 0.
float32   position    # 0 (open) .. 1 (closed)
float32   speed 1.0   # fraction of max closing speed. 1.0 IS the maximum -- a client
                       # that fills in only `position` closes at full speed.
# A CEILING on motor current, NOT a force setpoint: the gripper closes at `speed` toward
# `position` and stalls when it reaches this limit. No force servo exists on this hardware.
float32   force 0.5
uint8[16] token       # from /acquire_control; validates the sender, every message
