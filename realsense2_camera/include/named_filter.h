
#include <string>
#include <memory>
#include <librealsense2/rs.hpp>
#include <sensor_params.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace realsense2_camera
{
    // class FiletrInterface
    // {
    //     public:
    //         virtual bool is_enabled() = 0;
    //         virtual void setParameters() = 0;
    //         virtual rs2::frameset Process(rs2::frameset) = 0;
    // };

    class NamedFilter
    {
        public:
            NamedFilter(std::shared_ptr<rs2::filter> filter, std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled=false);
            void setParameters();
            bool is_enabled() {return _is_enabled;};
            rs2::frameset Process(rs2::frameset frameset);

        private:
            void clearParameters();

        public:
            std::shared_ptr<rs2::filter> _filter;

        private:
            bool _is_enabled;
            rclcpp::Logger _logger;
            SensorParams _params;
            std::vector<std::string> _parameters_names;

    };

    class PointcloudFilter
    {
        public:
            PointcloudFilter(std::shared_ptr<rs2::filter> filter, rclcpp::Node& node, std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled=false);
            void setParameters();
            bool is_enabled() {return _is_enabled;};
            rs2::frameset Process(rs2::frameset frameset);
        
            void set();
            void Publish(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset, const std::string& frame_id);

        private:
            void clearParameters();

        public:
            std::shared_ptr<rs2::filter> _filter;

        private:
            bool _is_enabled;
            rclcpp::Logger _logger;
            SensorParams _params;
            std::vector<std::string> _parameters_names;

            bool _is_enabled_pc;
            rclcpp::Node& _node;
            bool _allow_no_texture_points;
            bool _ordered_pc;
            std::vector< unsigned int > _valid_pc_indices;
            std::mutex _mutex_publisher;
            sensor_msgs::msg::PointCloud2 _msg_pointcloud;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_publisher;
    };
}