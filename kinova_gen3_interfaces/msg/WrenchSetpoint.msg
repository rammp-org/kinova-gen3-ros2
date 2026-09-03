# Target tool wrench, base frame. NOTE: no controller can consume this yet -- core has
# no SetpointKind::kEeWrench. The topic exists so the surface is complete and clients
# can be written against it; setpoints are dropped with a throttled warning.
geometry_msgs/Wrench wrench
uint8[16]            token
