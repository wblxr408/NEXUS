uint8 SOURCE_DIRECT_UWB=1
uint8 SOURCE_DIRECT_VISION=2
uint8 SOURCE_PLATFORM_RELATIVE=3
uint8 SOURCE_FUSED=4

std_msgs/Header header
string target_id
uint8 source_mode
geometry_msgs/Pose pose
float64[36] covariance
float32 confidence
