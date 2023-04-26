// Copyright 2023 Intel Corporation. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../include/base_realsense_node.h"
#include <geometry_msgs/msg/vector3_stamped.hpp>

using namespace realsense2_camera;

void BaseRealSenseNode::restartStaticTransformBroadcaster()
{
    // Since the _static_tf_broadcaster is a latched topic, the old transforms will
    // be alive even if the sensors are dynamically disabled. So, reset the
    // broadcaster everytime and publish the transforms of enabled sensors alone.
    if (_static_tf_broadcaster)
    {
        _static_tf_broadcaster.reset();
    }

    // intra-process do not support latched QoS, so we need to disable intra-process for this topic
    rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

    #ifndef DASHING
    _static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(_node, 
                                                                                    tf2_ros::StaticBroadcasterQoS(), 
                                                                                    std::move(options));
    #else
    _static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(_node, 
                                                                                    rclcpp::QoS(100), 
                                                                                    std::move(options));
    #endif
}

void BaseRealSenseNode::append_static_tf_msg(const rclcpp::Time& t,
                                          const float3& trans,
                                          const tf2::Quaternion& q,
                                          const std::string& from,
                                          const std::string& to)
{
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = from;
    msg.child_frame_id = to;

    // Convert x,y,z (taken from camera extrinsics)
    // from optical cooridnates to ros coordinates
    msg.transform.translation.x = trans.z;
    msg.transform.translation.y = -trans.x;
    msg.transform.translation.z = -trans.y;

    msg.transform.rotation.x = q.getX();
    msg.transform.rotation.y = q.getY();
    msg.transform.rotation.z = q.getZ();
    msg.transform.rotation.w = q.getW();
    _static_tf_msgs.push_back(msg);
}

struct compare_tf_ids
{
    const std::string& frame_id;
    const std::string& child_frame_id;

    // Struct Constructor
    compare_tf_ids(const std::string& frame_id, const std::string& child_frame_id) :
                            frame_id(frame_id), child_frame_id(child_frame_id) {}

    bool operator()(const geometry_msgs::msg::TransformStamped &static_tf_msg) const
    {
        return (static_tf_msg.header.frame_id == frame_id &&
                static_tf_msg.child_frame_id == child_frame_id);
    }
};

void BaseRealSenseNode::erase_static_tf_msg(const std::string& frame_id,
                                            const std::string& child_frame_id)
{
    std::vector<geometry_msgs::msg::TransformStamped>::iterator it;

    // Find whether there is any TF with given 'frame_id' and 'child_frame_id'
    it = std::find_if(_static_tf_msgs.begin(), _static_tf_msgs.end(),
                                        compare_tf_ids(frame_id, child_frame_id));

    // If found, erase that specific TF
    if (it != std::end(_static_tf_msgs))
    {
        _static_tf_msgs.erase(it);
    }
}

void BaseRealSenseNode::publishExtrinsicsTopic(const stream_index_pair& sip, const rs2_extrinsics& ex){
    // Publish extrinsics topic:
    Extrinsics msg = rsExtrinsicsToMsg(ex);
    if (_extrinsics_publishers.find(sip) != _extrinsics_publishers.end())
    {
        _extrinsics_publishers[sip]->publish(msg);
    }
}

tf2::Quaternion BaseRealSenseNode::rotationMatrixToQuaternion(const float rotation[9]) const
{
    Eigen::Matrix3f m;
    // We need to be careful about the order, as RS2 rotation matrix is
    // column-major, while Eigen::Matrix3f expects row-major.
    m << rotation[0], rotation[3], rotation[6],
         rotation[1], rotation[4], rotation[7],
         rotation[2], rotation[5], rotation[8];
    Eigen::Quaternionf q(m);
    return tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
}

void BaseRealSenseNode::calcAndAppendTransformMsgs(const rs2::stream_profile& profile, 
                                                   const rs2::stream_profile& base_profile)
{
    // Transform base to stream
    stream_index_pair sip(profile.stream_type(), profile.stream_index());
    tf2::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    float3 zero_trans{0, 0, 0};
    tf2::Quaternion zero_rot_quaternions;
    zero_rot_quaternions.setRPY(0, 0, 0);

    rclcpp::Time transform_ts_ = _node.now();

    // extrinsic from A to B is the position of A relative to B
    // TF from A to B is the transformation to be done on A to get to B
    // so, we need to calculate extrinsics in two opposite ways, one for extrinsic topic
    // and the second is for transformation topic (TF)
    rs2_extrinsics normal_ex;  // used to for extrinsics topic
    rs2_extrinsics tf_ex; // used for TF

    try
    {
        normal_ex = base_profile.get_extrinsics_to(profile);
        tf_ex = profile.get_extrinsics_to(base_profile);
    }
    catch (std::exception& e)
    {
        if (!strcmp(e.what(), "Requested extrinsics are not available!"))
        {
            ROS_WARN_STREAM("(" << rs2_stream_to_string(base_profile.stream_type()) << ", "
                                << base_profile.stream_index() << ") -> ("
                                << rs2_stream_to_string(profile.stream_type()) << ", "
                                << profile.stream_index() << "): "
                                << e.what()
                                << " : using unity as default.");
            normal_ex = rs2_extrinsics({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0,0,0}});
            tf_ex = normal_ex;
        }
        else
        {
            throw e;
        }
    }

    // publish normal extrinsics e.g. /camera/extrinsics/depth_to_color
    publishExtrinsicsTopic(sip, normal_ex);

    // publish static TF
    auto Q = rotationMatrixToQuaternion(tf_ex.rotation);
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    float3 trans{tf_ex.translation[0], tf_ex.translation[1], tf_ex.translation[2]};

    append_static_tf_msg(transform_ts_, trans, Q, _base_frame_id, FRAME_ID(sip));

    // Transform stream frame to stream optical frame and publish it
    append_static_tf_msg(transform_ts_, zero_trans, quaternion_optical, FRAME_ID(sip), OPTICAL_FRAME_ID(sip));

    if (profile.is<rs2::video_stream_profile>() &&
                profile.stream_type() != RS2_STREAM_DEPTH &&
                profile.stream_index() == 1)
    {
        append_static_tf_msg(transform_ts_, trans, Q, _base_frame_id, ALIGNED_DEPTH_TO_FRAME_ID(sip));
        append_static_tf_msg(transform_ts_, zero_trans, quaternion_optical, 
                                                ALIGNED_DEPTH_TO_FRAME_ID(sip), OPTICAL_FRAME_ID(sip));
    }

    if ((_imu_sync_method > imu_sync_method::NONE) && (profile.stream_type() == RS2_STREAM_GYRO))
    {
        append_static_tf_msg(transform_ts_, zero_trans, zero_rot_quaternions, FRAME_ID(sip), IMU_FRAME_ID);
        append_static_tf_msg(transform_ts_, zero_trans, quaternion_optical, IMU_FRAME_ID, IMU_OPTICAL_FRAME_ID);
    }
}

void BaseRealSenseNode::eraseTransformMsgs(const stream_index_pair& sip, const rs2::stream_profile& profile)
{
    std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);

    erase_static_tf_msg(_base_frame_id, FRAME_ID(sip));
    erase_static_tf_msg(FRAME_ID(sip), OPTICAL_FRAME_ID(sip));

    if (profile.is<rs2::video_stream_profile>() &&
                profile.stream_type() != RS2_STREAM_DEPTH &&
                profile.stream_index() == 1)
    {
        erase_static_tf_msg(_base_frame_id, ALIGNED_DEPTH_TO_FRAME_ID(sip));
        erase_static_tf_msg(ALIGNED_DEPTH_TO_FRAME_ID(sip), OPTICAL_FRAME_ID(sip));
    }

    if ((_imu_sync_method > imu_sync_method::NONE) && (profile.stream_type() == RS2_STREAM_GYRO))
    {
        erase_static_tf_msg(FRAME_ID(sip), IMU_FRAME_ID);
        erase_static_tf_msg(IMU_FRAME_ID, IMU_OPTICAL_FRAME_ID);
    }
}

void BaseRealSenseNode::publishStaticTransforms()
{
    restartStaticTransformBroadcaster();

    _static_tf_broadcaster->sendTransform(_static_tf_msgs);
}

void BaseRealSenseNode::publishDynamicTransforms()
{
    if (!_dynamic_tf_broadcaster)
    {
        _dynamic_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(_node);
    }

    // Publish transforms for the cameras
    std::unique_lock<std::mutex> lock(_publish_dynamic_tf_mutex);
    while (rclcpp::ok() && _is_running && _tf_publish_rate > 0)
    {
        _cv_tf.wait_for(lock, std::chrono::milliseconds((int)(1000.0/_tf_publish_rate)), 
                                        [&]{return (!(_is_running && _tf_publish_rate > 0));});
        {
            std::lock_guard<std::mutex> lock_guard(_publish_tf_mutex);
            rclcpp::Time t = _node.now();
            try
            {
                for(auto& msg : _static_tf_msgs)
                    msg.header.stamp = t;
                _dynamic_tf_broadcaster->sendTransform(_static_tf_msgs);
            }
            catch(const std::exception& e)
            {
                ROS_ERROR_STREAM("Error publishing dynamic transforms: " << e.what());
            }
        }
    }
}

void BaseRealSenseNode::startDynamicTf()
{
    if (!_publish_tf)
    {
        ROS_WARN("Since the param 'publish_tf' is set to 'false',"
                "the value set on the param 'tf_publish_rate' won't have any effect");
        return;
    }
    if (_tf_publish_rate > 0)
    {
        // Start publishing dynamic TF, if the param 'tf_publish_rate' is set to > 0.0 Hz
        ROS_WARN("Publishing dynamic camera transforms (/tf) at %g Hz", _tf_publish_rate);
        if (!_tf_t)
        {
            _tf_t = std::make_shared<std::thread>([this]()
            {
                publishDynamicTransforms();
            });
        }
    }
    else
    {
        if (_tf_t && _tf_t->joinable())
        {
            // Stop publishing dynamic TF by resetting the '_tf_t' thread and '_dynamic_tf_broadcaster'
            _tf_t->join();
            _tf_t.reset();
            _dynamic_tf_broadcaster.reset();
            ROS_WARN("Stopped publishing dynamic camera transforms (/tf)");
        }
        else
        {
            // '_tf_t' thread is not running currently. i.e, dynamic tf is not getting broadcasted.
            ROS_WARN("Currently not publishing dynamic camera transforms (/tf). "
                     "To start publishing it, set the 'tf_publish_rate' param to > 0.0 Hz ");
        }
    }
}

void BaseRealSenseNode::pose_callback(rs2::frame frame)
{
    double frame_time = frame.get_timestamp();
    bool placeholder_false(false);
    if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
    {
        _is_initialized_time_base = setBaseTime(frame_time, frame.get_frame_timestamp_domain());
    }

    ROS_DEBUG("Frame arrived: stream: %s ; index: %d ; Timestamp Domain: %s",
                rs2_stream_to_string(frame.get_profile().stream_type()),
                frame.get_profile().stream_index(),
                rs2_timestamp_domain_to_string(frame.get_frame_timestamp_domain()));
    rs2_pose pose = frame.as<rs2::pose_frame>().get_pose_data();
    rclcpp::Time t(frameSystemTimeSec(frame));

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose.position.x = -pose.translation.z;
    pose_msg.pose.position.y = -pose.translation.x;
    pose_msg.pose.position.z = pose.translation.y;
    pose_msg.pose.orientation.x = -pose.rotation.z;
    pose_msg.pose.orientation.y = -pose.rotation.x;
    pose_msg.pose.orientation.z = pose.rotation.y;
    pose_msg.pose.orientation.w = pose.rotation.w;

    static tf2_ros::TransformBroadcaster br(_node);
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = DEFAULT_ODOM_FRAME_ID;
    msg.child_frame_id = FRAME_ID(POSE);
    msg.transform.translation.x = pose_msg.pose.position.x;
    msg.transform.translation.y = pose_msg.pose.position.y;
    msg.transform.translation.z = pose_msg.pose.position.z;
    msg.transform.rotation.x = pose_msg.pose.orientation.x;
    msg.transform.rotation.y = pose_msg.pose.orientation.y;
    msg.transform.rotation.z = pose_msg.pose.orientation.z;
    msg.transform.rotation.w = pose_msg.pose.orientation.w;

    if (_publish_odom_tf) br.sendTransform(msg);

    if (0 != _odom_publisher->get_subscription_count())
    {
        double cov_pose(_linear_accel_cov * pow(10, 3-(int)pose.tracker_confidence));
        double cov_twist(_angular_velocity_cov * pow(10, 1-(int)pose.tracker_confidence));

        geometry_msgs::msg::Vector3Stamped v_msg;
        tf2::Vector3 tfv(-pose.velocity.z, -pose.velocity.x, pose.velocity.y);
        tf2::Quaternion q(-msg.transform.rotation.x,
                          -msg.transform.rotation.y,
                          -msg.transform.rotation.z,
                           msg.transform.rotation.w);
        tfv=tf2::quatRotate(q,tfv);
        v_msg.vector.x = tfv.x();
        v_msg.vector.y = tfv.y();
        v_msg.vector.z = tfv.z();
    
        tfv = tf2::Vector3(-pose.angular_velocity.z, -pose.angular_velocity.x, pose.angular_velocity.y);
        tfv=tf2::quatRotate(q,tfv);
        geometry_msgs::msg::Vector3Stamped om_msg;
        om_msg.vector.x = tfv.x();
        om_msg.vector.y = tfv.y();
        om_msg.vector.z = tfv.z();    

        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.frame_id = DEFAULT_ODOM_FRAME_ID;
        odom_msg.child_frame_id = FRAME_ID(POSE);
        odom_msg.header.stamp = t;
        odom_msg.pose.pose = pose_msg.pose;
        odom_msg.pose.covariance = {cov_pose, 0, 0, 0, 0, 0,
                                    0, cov_pose, 0, 0, 0, 0,
                                    0, 0, cov_pose, 0, 0, 0,
                                    0, 0, 0, cov_twist, 0, 0,
                                    0, 0, 0, 0, cov_twist, 0,
                                    0, 0, 0, 0, 0, cov_twist};
        odom_msg.twist.twist.linear = v_msg.vector;
        odom_msg.twist.twist.angular = om_msg.vector;
        odom_msg.twist.covariance ={cov_pose, 0, 0, 0, 0, 0,
                                    0, cov_pose, 0, 0, 0, 0,
                                    0, 0, cov_pose, 0, 0, 0,
                                    0, 0, 0, cov_twist, 0, 0,
                                    0, 0, 0, 0, cov_twist, 0,
                                    0, 0, 0, 0, 0, cov_twist};
        _odom_publisher->publish(odom_msg);
        ROS_DEBUG("Publish %s stream", rs2_stream_to_string(frame.get_profile().stream_type()));
    }
}
