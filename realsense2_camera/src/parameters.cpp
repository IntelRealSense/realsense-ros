#include "../include/base_realsense_node.h"
#include <ros_utils.h>

using namespace realsense2_camera;

void BaseRealSenseNode::getParameters()
{
    ROS_INFO("getParameters...");
    _parameters->setParamT(std::string("align_depth"), rclcpp::ParameterValue(ALIGN_DEPTH), _align_depth);
    _camera_name = _parameters->setParam("camera_name", rclcpp::ParameterValue("camera")).get<rclcpp::PARAMETER_STRING>();

    _publish_tf = _parameters->setParam("publish_tf", rclcpp::ParameterValue(PUBLISH_TF)).get<rclcpp::PARAMETER_BOOL>();
    _tf_publish_rate = _parameters->setParam("tf_publish_rate", rclcpp::ParameterValue(TF_PUBLISH_RATE)).get<rclcpp::PARAMETER_DOUBLE>();

    _parameters->setParamT(std::string("enable_sync"), rclcpp::ParameterValue(SYNC_FRAMES || _align_depth), _sync_frames);

    _json_file_path = _parameters->setParam("json_file_path", rclcpp::ParameterValue("")).get<rclcpp::PARAMETER_STRING>();

    std::string unite_imu_method_str = _parameters->setParam("unite_imu_method", rclcpp::ParameterValue(DEFAULT_UNITE_IMU_METHOD)).get<rclcpp::PARAMETER_STRING>();
    if (unite_imu_method_str == "linear_interpolation")
        _imu_sync_method = imu_sync_method::LINEAR_INTERPOLATION;
    else if (unite_imu_method_str == "copy")
        _imu_sync_method = imu_sync_method::COPY;
    else
        _imu_sync_method = imu_sync_method::NONE;

    // _allow_no_texture_points = _node.declare_parameter("allow_no_texture_points", rclcpp::ParameterValue(ALLOW_NO_TEXTURE_POINTS)).get<rclcpp::PARAMETER_BOOL>();
    _clipping_distance = _parameters->setParam("clip_distance", rclcpp::ParameterValue(-1.0)).get<rclcpp::PARAMETER_DOUBLE>();
    _linear_accel_cov = _parameters->setParam("linear_accel_cov", rclcpp::ParameterValue(0.01)).get<rclcpp::PARAMETER_DOUBLE>();
    _angular_velocity_cov = _parameters->setParam("angular_velocity_cov", rclcpp::ParameterValue(0.01)).get<rclcpp::PARAMETER_DOUBLE>();
   
    _hold_back_imu_for_frames = _parameters->setParam("hold_back_imu_for_frames", rclcpp::ParameterValue(HOLD_BACK_IMU_FOR_FRAMES)).get<rclcpp::PARAMETER_BOOL>();
    _publish_odom_tf = _parameters->setParam("publish_odom_tf", rclcpp::ParameterValue(PUBLISH_ODOM_TF)).get<rclcpp::PARAMETER_BOOL>();
    _base_frame_id = _parameters->setParam("base_frame_id", rclcpp::ParameterValue(DEFAULT_BASE_FRAME_ID)).get<rclcpp::PARAMETER_STRING>();
}


