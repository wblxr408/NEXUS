# A target estimate with explicit observability; unknown components are NaN.
# covariance is row-major [map position, map velocity, map fixed-axis angle].
# Static history: valid=false, historical=true, all observed flags=false.
# Finite pose/covariance are the last accepted estimate, never extrapolated.
# header.stamp is the status event time; last_valid_sample_timestamp_ns is
# the original observation time. Covariance is conditional on platform/mount.
# Target velocity is unobserved (NaN); static is a model, not a measured speed.
std_msgs/Header header
uint64 receive_timestamp_ns
string target_id
bool valid
bool historical
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
