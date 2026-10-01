# Published by the vS-Graphs wrapper each time ORB-SLAM3's LoopClosing::CorrectLoop() completes on
# the current map (Map::GetLoopCorrectionVersion() changed). The correction itself is non-rigid
# (essential-graph optimisation), so no transform is carried: consumers that buffer world-frame
# state can only discard it.
std_msgs/Header header
uint64 map_id
uint64 correction_count
