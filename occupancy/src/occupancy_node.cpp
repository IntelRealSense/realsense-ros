#include <thread>
#include <chrono>
#include <memory>
#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/OccupancyGrid.h"

#include "SP_MapManager.h"

using namespace std;
using namespace ScenePerception;

std::unique_ptr<ScenePerception::SP_MapManager> mapManager;
PoseMatrix4f  gH_T265ref_refROS;
PoseMatrix4f gH_T265ref_imu;
std::mutex pose_mutex;
unsigned int framesSinceLastPublished;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  PoseMatrix4f H_refROS_T265bodyROS;  // T265 body frame w.r.t. reference frame (in ROS)
  H_refROS_T265bodyROS.rotationFromQuaternion(q);
  float position[3];
  position[0] = msg->pose.pose.position.x;
  position[1] = msg->pose.pose.position.y;
  position[2] = msg->pose.pose.position.z;
  H_refROS_T265bodyROS.SetTranslation(position);

  PoseMatrix4f H_T265bodyROS_imu = PoseMatrix4f::Zero;  // IMU w.r.t. T265 (ROS) body frame
  H_T265bodyROS_imu.m_data[2] = 1.0f;
  H_T265bodyROS_imu.m_data[4] = 1.0f;
  H_T265bodyROS_imu.m_data[9] = 1.0f;
  
  std::lock_guard<std::mutex> guard(pose_mutex);
  gH_T265ref_imu = gH_T265ref_refROS * H_refROS_T265bodyROS * H_T265bodyROS_imu;
}

void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  std::lock_guard<std::mutex> guard(pose_mutex);
  PoseMatrix4f H_T265ref_imu = gH_T265ref_imu;
  framesSinceLastPublished++;
  mapManager->TestAndPushFrame((const unsigned short *)&(msg->data[0]), H_T265ref_imu);
}

int main(int argc, char **argv)
{
    // initialize coord. frames
    // ROS reference frame w.r.t. T265 reference frame (see data sheet)
    gH_T265ref_refROS = PoseMatrix4f::Zero;
    gH_T265ref_refROS.m_data[1] = -1.0f;
    gH_T265ref_refROS.m_data[6] = -1.0f;
    gH_T265ref_refROS.m_data[8] = 1.0f;

    ros::init(argc, argv, "occupancy");
    ros::NodeHandle n;
        
    // get depth camera calibration 
    SP_CameraIntrinsics intrinsics;
    SP_CameraExtrinsics H_T265imu_depthOptical;

    // hard-coded (default), ToDo: read from ROS
    intrinsics.imageWidth = 640;
    intrinsics.imageHeight = 480;
    intrinsics.focalLengthHorizontal = 632.831420898438;
    intrinsics.focalLengthVertical = 632.831420898438;
    intrinsics.principalPointCoordU = 314.722412109375;
    intrinsics.principalPointCoordV = 242.948120117188;

    // depth optical frame w.r.t. T265 IMU: assumes "stacked" configuration with cameras mounted on top of each other
    H_T265imu_depthOptical.pose = PoseMatrix4f::Identity;
    H_T265imu_depthOptical.pose.m_data[0] = -1;
    H_T265imu_depthOptical.pose.m_data[5] = -1;
    

    ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy", 1);  // http://docs.ros.org/jade/api/nav_msgs/html/msg/OccupancyGrid.html
    nav_msgs::OccupancyGrid msg;
    msg.header.frame_id = "spatial";

    if ( !n.getParam("resolution", msg.info.resolution) )
        ROS_ERROR("Failed to get param 'resolution'");
    
    float2 heightOfInterst;
    if ( !n.getParam("HeightOfInterestMin", heightOfInterst.x) )
        ROS_ERROR("Failed to get param 'HeightOfInterestMin'");
    if ( !n.getParam("HeightOfInterestMax", heightOfInterst.y) )
        ROS_ERROR("Failed to get param 'HeightOfInterestMax'");
    
    float2 depthOfInterst;
    if ( !n.getParam("DepthOfInterestMin", depthOfInterst.x) )
        ROS_ERROR("Failed to get param 'DepthOfInterestMin'");
    if ( !n.getParam("DepthOfInterestMax", depthOfInterst.y) )
        ROS_ERROR("Failed to get param 'DepthOfInterestMax'");
    
    // create the map manager
	mapManager.reset(new ScenePerception::SP_MapManager);
	mapManager->reset();
    mapManager->configure(intrinsics, H_T265imu_depthOptical.pose);
	mapManager->setMapHeightOfInterest(heightOfInterst.x, heightOfInterst.y);
	mapManager->setDepthOfInterest(depthOfInterst.x, depthOfInterst.y);
	mapManager->setMapResolution(msg.info.resolution);

    // register callbacks
    ros::Subscriber sub_pose = n.subscribe("t265/odom/sample", 1, poseCallback, ros::TransportHints().udp());  // min. latency: queue size = 1, use UDP
    ros::Subscriber sub_depth = n.subscribe("d400/depth/image_rect_raw", 1, depthCallback, ros::TransportHints().udp());

    // publish
    unique_ptr<SP_OccupancyGridMsg> occGrid(new SP_OccupancyGridMsg);
    unsigned int allocated_width = 1;
    unsigned int allocated_height = 1;
    occGrid->data = (int8_t*) malloc(allocated_width * allocated_height);
    unsigned int frame_id = 0;

    while (ros::ok())
    {
        if (framesSinceLastPublished > 5)
        {
            framesSinceLastPublished = 0;
            auto status = SP_STATUS_WARNING;
            while (status != SP_STATUS_SUCCESS)
            {
                occGrid->mapMetaData.width = allocated_width;
                occGrid->mapMetaData.height = allocated_height;            
                status = mapManager->getOccupancyMapAsGridMsg(occGrid.get());
                if (status == SP_STATUS_WARNING)
                {
                    if (occGrid->data)
                    {
                        free(occGrid->data);
                    }
                    allocated_width = occGrid->mapMetaData.width;
                    allocated_height = occGrid->mapMetaData.height;
                    occGrid->data = (int8_t*) malloc(allocated_width * allocated_height);
                }
                else if (status != SP_STATUS_SUCCESS)
                {                    
                    break;
                }
            }
            if (status != SP_STATUS_SUCCESS)
            {
                break;
            }
            
            // fill the message
            msg.header.seq = frame_id++;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "t265_odom_frame";
            msg.info.map_load_time = ros::Time::now();
            msg.info.resolution = occGrid->mapMetaData.resolution;
            msg.info.width = occGrid->mapMetaData.width;       
            msg.info.height = occGrid->mapMetaData.height;
            msg.data.resize(allocated_width * allocated_height);
            PoseMatrix4f H_refROS_T265ref = gH_T265ref_refROS.Inverse();  // T265 ref. frame w.r.t. ROS ref. frame
            float4 center(occGrid->mapMetaData.offset[0], 0.0f, occGrid->mapMetaData.offset[1], 1.0f);
            center = H_refROS_T265ref * center;

            // trafo occ. to ref. frame
            msg.info.origin.position.x = center.x;
            msg.info.origin.position.y = center.y;
            msg.info.origin.position.z = 0.0f;
            msg.info.origin.orientation.x = 0.0f;
            msg.info.origin.orientation.y = 0.0f;
            msg.info.origin.orientation.z = M_PI_4;
            msg.info.origin.orientation.w = -M_PI_4;

            memcpy(&(msg.data[0]), occGrid->data, msg.info.width * msg.info.height);
            
            pub.publish(msg);
        }   
        
        ros::spinOnce();
    }

    return 0;
}
