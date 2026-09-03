# What the streaming tier is actually doing. `open`, `timeout_s` and `rejected_count`
# come from core via StreamSink::on_query_stream(), so they are exact rather than this
# node's guess -- a session torn down on deadline expiry shows up here immediately.
#
# `rejected_count` counts setpoints the SESSION refused (wrong channel, or closed).
# Token failures are counted by the Arbiter and appear on /control_status instead.
std_msgs/Header header
bool     open
string   controller       # "" when closed; this node's label for core's (kind, mode)
string[] channels         # topics the open controller reads
float64  timeout_s
uint64   rejected_count
