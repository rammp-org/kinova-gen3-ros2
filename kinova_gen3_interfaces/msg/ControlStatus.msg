# Who may command the arm right now. Published ON CHANGE, latched (transient_local),
# so a client that starts late or reconnects learns the current state immediately.
# Maps 1:1 onto kinova::interface::ArbitrationStatus.
#
# `generation` is how a client detects it was dispossessed: engaging the e-stop and
# every acquire both clear ownership, silently invalidating outstanding tokens.
std_msgs/Header header
bool   arbitration_enabled   # false = tokens are ignored (launch-time parameter)
bool   estopped              # latched; nothing is admitted
bool   owned                 # does anyone hold the arm?
string owner_id              # "" if none
uint64 generation            # bumps on every grant
uint64 rejected_count        # commands refused since start
