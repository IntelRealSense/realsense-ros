#include "../include/base_realsense_node.h"
#include <ros_utils.h>

using namespace realsense2_camera;

void BaseRealSenseNode::getParameters()
{
    ROS_INFO("getParameters...");

    std::string param_name;
    param_name = std::string("camera_name");
    _camera_name = _parameters->setParam(param_name, rclcpp::ParameterValue("camera")).get<rclcpp::PARAMETER_STRING>();
    _parameters_names.push_back(param_name);

    param_name = std::string("publish_tf");
    _publish_tf = _parameters->setParam(param_name, rclcpp::ParameterValue(PUBLISH_TF)).get<rclcpp::PARAMETER_BOOL>();
    _parameters_names.push_back(param_name);

    param_name = std::string("tf_publish_rate");
    _parameters->setParamT(param_name, rclcpp::ParameterValue(TF_PUBLISH_RATE), _tf_publish_rate, [this](const rclcpp::Parameter& )
            {
                startDynamicTf();
            });
    _parameters_names.push_back(param_name);
    startDynamicTf();

    param_name = std::string("diagnostics_period");
    _diagnostics_period = _parameters->setParam(param_name, rclcpp::ParameterValue(DIAGNOSTICS_PERIOD)).get<rclcpp::PARAMETER_DOUBLE>();
    _parameters_names.push_back(param_name);

    param_name = std::string("enable_sync");
    _parameters->setParamT(param_name, rclcpp::ParameterValue(SYNC_FRAMES), _sync_frames);
    _parameters_names.push_back(param_name);

    param_name = std::string("json_file_path");
    _json_file_path = _parameters->setParam(param_name, rclcpp::ParameterValue("")).get<rclcpp::PARAMETER_STRING>();
    _parameters_names.push_back(param_name);

    param_name = std::string("unite_imu_method");
    std::string unite_imu_method_str = _parameters->setParam(param_name, rclcpp::ParameterValue(DEFAULT_UNITE_IMU_METHOD)).get<rclcpp::PARAMETER_STRING>();
    _parameters_names.push_back(param_name);

    if (unite_imu_method_str == "linear_interpolation")
        _imu_sync_method = imu_sync_method::LINEAR_INTERPOLATION;
    else if (unite_imu_method_str == "copy")
        _imu_sync_method = imu_sync_method::COPY;
    else
        _imu_sync_method = imu_sync_method::NONE;

    param_name = std::string("clip_distance");
    _clipping_distance = _parameters->setParam(param_name, rclcpp::ParameterValue(-1.0)).get<rclcpp::PARAMETER_DOUBLE>();
    _parameters_names.push_back(param_name);

    param_name = std::string("linear_accel_cov");
    _linear_accel_cov = _parameters->setParam(param_name, rclcpp::ParameterValue(0.01)).get<rclcpp::PARAMETER_DOUBLE>();
    _parameters_names.push_back(param_name);

    param_name = std::string("angular_velocity_cov");
    _angular_velocity_cov = _parameters->setParam(param_name, rclcpp::ParameterValue(0.01)).get<rclcpp::PARAMETER_DOUBLE>();
    _parameters_names.push_back(param_name);
   
    param_name = std::string("hold_back_imu_for_frames");
    _hold_back_imu_for_frames = _parameters->setParam(param_name, rclcpp::ParameterValue(HOLD_BACK_IMU_FOR_FRAMES)).get<rclcpp::PARAMETER_BOOL>();
    _parameters_names.push_back(param_name);

    param_name = std::string("publish_odom_tf");
    _publish_odom_tf = _parameters->setParam(param_name, rclcpp::ParameterValue(PUBLISH_ODOM_TF)).get<rclcpp::PARAMETER_BOOL>();
    _parameters_names.push_back(param_name);

    param_name = std::string("base_frame_id");
    _base_frame_id = _parameters->setParam(param_name, rclcpp::ParameterValue(DEFAULT_BASE_FRAME_ID)).get<rclcpp::PARAMETER_STRING>();
    _parameters_names.push_back(param_name);
}

void BaseRealSenseNode::clearParameters()
{
    while ( !_parameters_names.empty() )
    {
        auto name = _parameters_names.back();
        _parameters->removeParam(name);
        _parameters_names.pop_back();        
    }
}

