std_msgs/Header header

# 2D tracking results (normalised 0..1)
float32 center_x
float32 center_y
float32 area          # object area in pixels²

# Depth / 3D information (zero if no RealSense)
float32 depth_m
float32[3] position_3d   # (x, y, z) in robot base frame (metres)

# Status
bool tracking_active
float32 confidence       # CSRT confidence 0..1
