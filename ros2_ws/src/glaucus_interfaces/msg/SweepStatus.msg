# Published by sweep_executor at 5 Hz. This is what the web app shows.
# Same fields as Simulation.status() in sim/sweep_sim.py (used by the app's mock mode).

std_msgs/Header header
string state                 # STEP_DOWN | TURN | LANE | VERIFY | SHIFT | DONE | IDLE | PAUSED
bool paused
int32 lane_index
float32 lane_depth           # m
int8 direction               # +1 toward bow, -1 toward stern
float32 depth                # m
float32 heading              # rad
float32 x_est                # m along hull (from WallEstimator)
float32 end_score            # current end-of-lane evidence score
uint16 faults                # passthrough from the MCU
