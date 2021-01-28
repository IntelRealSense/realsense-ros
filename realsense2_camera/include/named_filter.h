
#include <string>
#include <memory>
#include <librealsense2/rs.hpp>
#include <sensor_params.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace realsense2_camera
{
    class NamedFilter
    {
        public:
            std::string _name;
            std::shared_ptr<rs2::filter> _filter;
            bool _is_enabled;
            rclcpp::Logger _logger;

        protected:
            SensorParams _params;

        public:
            NamedFilter(std::string name, std::shared_ptr<rs2::filter> filter, rclcpp::Node& node, rclcpp::Logger logger, bool is_enabled=false):
            _name(name), _filter(filter), _is_enabled(is_enabled), _logger(logger), _params(node, logger)
            {
                setParameters();
            }
            virtual void set(const bool is_enabled) {_is_enabled = is_enabled;};

            template<class T> 
            bool is() const
            {
                return (dynamic_cast<const T*> (&(*this)));
            }
        
        private:
            void setParameters();
    };

    class PointcloudFilter : public NamedFilter
    {
        public:
            PointcloudFilter(std::string name, std::shared_ptr<rs2::filter> filter, rclcpp::Node& node, rclcpp::Logger logger, bool is_enabled=false);
        
            void set(const bool is_enabled) override;
            void Publish(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset, const std::string& frame_id);

        private:
            rclcpp::Node& _node;
            bool _allow_no_texture_points;
            std::vector< unsigned int > _valid_pc_indices;
            std::mutex _mutex_publisher;
            sensor_msgs::msg::PointCloud2 _msg_pointcloud;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_publisher;
    };
}