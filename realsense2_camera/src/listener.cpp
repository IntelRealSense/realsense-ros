#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener &listener)
{

    geometry_msgs::PointStamped camera_ref;
    camera_ref.header.frame_id = "camera_odom_frame";

    //we'll just use the most recent transform available for our simple example
    camera_ref.header.stamp = ros::Time();

    //just an arbitrary point in space
    camera_ref.point.x = 0.0;
    camera_ref.point.y = 0.0;
    camera_ref.point.z = 0.0;

    try
    {
        geometry_msgs::PointStamped base_point;
        listener.transformPoint("base_link", camera_ref, base_point);

        ROS_INFO("camera_odom_frame: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                 camera_ref.point.x, camera_ref.point.y, camera_ref.point.z,
                 base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_tf_listener");
    ros::NodeHandle n;

    tf::TransformListener listener(ros::Duration(10));

    //we'll transform a point once every second
    ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

    ros::spin();
}