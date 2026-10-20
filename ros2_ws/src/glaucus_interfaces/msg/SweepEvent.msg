# Discrete things that happened during a sweep. Logged + shown in the app timeline.
# Mirrors glaucus::Event in glaucus_core/types.hpp.

std_msgs/Header header

uint8 END_OF_LANE=0
uint8 END_REJECTED=1
uint8 STEP_DOWN_BLOCKED=2
uint8 SWEEP_COMPLETE=3
uint8 kind

int32 lane
int8 direction
float32 value                # lane length (m) or retry number
bool verified
