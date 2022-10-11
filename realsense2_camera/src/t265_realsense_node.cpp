// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#include "../include/t265_realsense_node.h"
#include <fstream>

using namespace realsense2_camera;

T265RealsenseNode::T265RealsenseNode(rclcpp::Node& node,
                                     rs2::device dev,
                                     std::shared_ptr<Parameters> parameters,
                                     bool use_intra_process) : 
                                     BaseRealSenseNode(node, dev, parameters, use_intra_process),
                                     _wo_snr(dev.first<rs2::wheel_odometer>()),
                                     _use_odom_in(false) 
                                     {
                                         _monitor_options = {RS2_OPTION_ASIC_TEMPERATURE, RS2_OPTION_MOTION_MODULE_TEMPERATURE};
                                         initializeOdometryInput();
                                     }

void T265RealsenseNode::initializeOdometryInput()
{
    std::string calib_odom_file;
    std::string param_name("calib_odom_file");
    calib_odom_file = _parameters->setParam<std::string>(param_name, "");
    if (calib_odom_file.empty())
    {
        ROS_INFO("No calib_odom_file. No input odometry accepted.");
        return;
    }
    _parameters_names.push_back(param_name);

    std::ifstream calibrationFile(calib_odom_file);
    if (!calibrationFile)
    {
        ROS_FATAL_STREAM("calibration_odometry file not found. calib_odom_file = " << calib_odom_file);
        throw std::runtime_error("calibration_odometry file not found" );
    }
    const std::string json_str((std::istreambuf_iterator<char>(calibrationFile)),
        std::istreambuf_iterator<char>());
    const std::vector<uint8_t> wo_calib(json_str.begin(), json_str.end());

    if (!_wo_snr.load_wheel_odometery_config(wo_calib))
    {
        ROS_FATAL_STREAM("Format error in calibration_odometry file: " << calib_odom_file);
        throw std::runtime_error("Format error in calibration_odometry file" );
    }
    _use_odom_in = true;
}

void T265RealsenseNode::publishTopics()
{
    BaseRealSenseNode::publishTopics();
    setupSubscribers();
}

void T265RealsenseNode::setupSubscribers()
{
    if (!_use_odom_in) return;

    std::string topic_odom_in = _parameters->setParam<std::string>("topic_odom_in", DEFAULT_TOPIC_ODOM_IN);
    ROS_INFO_STREAM("Subscribing to in_odom topic: " << topic_odom_in);

    _odom_subscriber = _node.create_subscription<nav_msgs::msg::Odometry>(topic_odom_in, 1, [this](const nav_msgs::msg::Odometry::SharedPtr msg){odom_in_callback(msg);});
}

void T265RealsenseNode::odom_in_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    ROS_DEBUG("Got in_odom message");
    rs2_vector velocity {-(float)(msg->twist.twist.linear.y),
                          (float)(msg->twist.twist.linear.z),
                         -(float)(msg->twist.twist.linear.x)};

    ROS_DEBUG_STREAM("Add odom: " << velocity.x << ", " << velocity.y << ", " << velocity.z);
    _wo_snr.send_wheel_odometry(0, 0, velocity);
}

void T265RealsenseNode::calcAndPublishStaticTransform(const rs2::stream_profile& profile, const rs2::stream_profile& base_profile)
{
    // Transform base to stream
    stream_index_pair sip(profile.stream_type(), profile.stream_index());

    tf2::Quaternion quaternion_optical;
    quaternion_optical.setRPY(M_PI / 2, 0.0, -M_PI / 2);    //Pose To ROS
    float3 zero_trans{0, 0, 0};

    rclcpp::Time transform_ts_ = _node.now();

    rs2_extrinsics ex;
    try
    {
        ex = base_profile.get_extrinsics_to(profile);
    }
    catch (std::exception& e)
    {
        if (!strcmp(e.what(), "Requested extrinsics are not available!"))
        {
            ROS_WARN_STREAM("(" << rs2_stream_to_string(profile.stream_type()) << ", " << profile.stream_index() << ") -> (" << rs2_stream_to_string(base_profile.stream_type()) << ", " << base_profile.stream_index() << "): " << e.what() << " : using unity as default.");
            ex = rs2_extrinsics({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}});
        }
        else
        {
            throw e;
        }
    }

    auto Q = rotationMatrixToQuaternion(ex.rotation);
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    float3 trans{ex.translation[0], ex.translation[1], ex.translation[2]};
    
    if (sip == POSE)
    {
        Q = Q.inverse();
        publish_static_tf(transform_ts_, trans, Q, FRAME_ID(sip), _base_frame_id);
    }
    else
    {
        publish_static_tf(transform_ts_, trans, Q, _base_frame_id, FRAME_ID(sip));
        publish_static_tf(transform_ts_, zero_trans, quaternion_optical, FRAME_ID(sip), OPTICAL_FRAME_ID(sip));

        if (profile.is<rs2::video_stream_profile>() && profile.stream_type() != RS2_STREAM_DEPTH && profile.stream_index() == 1)
        {
            publish_static_tf(transform_ts_, trans, Q, _base_frame_id, ALIGNED_DEPTH_TO_FRAME_ID(sip));
            publish_static_tf(transform_ts_, zero_trans, quaternion_optical, ALIGNED_DEPTH_TO_FRAME_ID(sip), OPTICAL_FRAME_ID(sip));
        }
    }
}
