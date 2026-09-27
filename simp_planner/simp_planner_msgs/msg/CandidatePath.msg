# One lateral candidate path from a single planning cycle, plus how far it
# got through the planner's screening/trajectory evaluation. See
# PathVelocityPlanner::plan's export_debug_candidates block (core.cpp) for
# the producer and CandidateSet.msg for the per-cycle envelope.

int32 candidate_id
float64 n_target
float64 total_cost

# Coarse/preview collision screening (Phase 1-2, runs for every candidate).
bool preview_collision_free

# Whether this candidate also went through full trajectory evaluation
# (Phase 3, only the shortlisted subset). dynamic_valid/collision_free/safe
# are meaningless when this is false.
bool trajectory_evaluated
bool dynamic_valid
bool collision_free
bool safe

bool selected

float64[] x
float64[] y
