uint8 SOURCE_DIRECT_UWB=1
uint8 SOURCE_DIRECT_VISION=2
uint8 SOURCE_PLATFORM_RELATIVE=3
uint8 SOURCE_FUSED=4

uint8 VALIDITY_UNKNOWN=0
uint8 VALIDITY_VALID=1
uint8 VALIDITY_INVALID=2

std_msgs/Header header
uint64 receive_timestamp_ns
string target_id
uint8 source_mode
uint8 validity
string invalid_reason
uint64 last_valid_sample_timestamp_ns
string unit
geometry_msgs/Pose pose
float64[36] covariance
float32 confidence
