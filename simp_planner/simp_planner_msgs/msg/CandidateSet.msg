# Every lateral candidate evaluated during one planning cycle. Published
# only when the planner's export_debug_candidates parameter is enabled.

std_msgs/Header header
uint64 plan_id
CandidatePath[] candidates
