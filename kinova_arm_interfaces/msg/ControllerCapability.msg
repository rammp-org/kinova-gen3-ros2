string   name         # e.g. "joint_impedance"
string[] channels     # topics under /setpoint/ this controller reads
bool     available    # computed live from core's pair_supported(), never declared
