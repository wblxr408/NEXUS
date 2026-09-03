# A target estimate with explicit observability; unknown components are NaN.
# covariance is row-major [map position, map velocity, map fixed-axis angle].
# Fusion prediction-only output: valid=false, all observed flags=false,
# reason="prediction_only"; available predicted values/covariance stay finite.
# last_valid_sample_timestamp_ns remains the last accepted observation time.
std_msgs/Header header
uint64 receive_timestamp_ns
string target_id
bool valid
bool position_observed
bool velocity_observed
bool orientation_observed
string state
string reason
string source
string position_reference
uint64 last_valid_sample_timestamp_ns
string unit
geometry_msgs/Pose pose
geometry_msgs/Vector3 velocity
float64[81] covariance
float32 confidence
