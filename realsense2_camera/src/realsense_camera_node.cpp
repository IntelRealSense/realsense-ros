#include <rclcpp/rclcpp.hpp>
#include "realsense_node_factory.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("RealSenseCameraNode", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    realsense2_camera::RealSenseNodeFactory rs_factory(node);
    rs_factory.onInit();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
