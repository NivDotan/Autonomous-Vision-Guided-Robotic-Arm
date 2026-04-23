float32[3] target_position    # (x, y, z) in robot base frame (metres)
float32[3] approach_axis      # unit vector, base frame
float32 pre_grasp_offset_m    # approach standoff distance (default 0.08 m)
bool close_gripper_after      # whether to close gripper at target pose
string target_name            # optional: object class name for logging
