# Task planner state machine status.
std_msgs/Header header
string state           # current TaskState enum name (e.g. "MOVE_TO_PREGRASP")
string previous_state
int32 attempt          # current retry attempt (0-based)
bool dry_run
string message         # human-readable status line
