# Published by vision_node at camera rate (~15-30 Hz). Consumed by sweep_executor.

std_msgs/Header header
float32 cam_edge             # 0..1 confidence that the hull edge / an obstacle is just ahead
float32 stall                # 0..1 "commanded motion but the image isn't moving"
bool flow_valid              # false on featureless patches
float32 flow_ds              # m moved along the forward axis since the previous message
float32 flow_quality         # 0..1, for logging / tuning
