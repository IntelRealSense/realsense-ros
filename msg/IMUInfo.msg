# header.frame_id is either set to "imu_accel" or "imu_gyro"
# to distinguish between "accel" and "gyro" info.
std_msgs/Header header
float64[12] data
float64[3] noise_variances
float64[3] bias_variances
