#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "constants.h"
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <ros_utils.h>
#include <sensor_params.h>

#define STREAM_NAME(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << create_graph_resource_name(rs2_stream_to_string(sip.first)) << ((sip.second>0) ? std::to_string(sip.second) : ""))).str()

namespace realsense2_camera
{
    typedef std::pair<rs2_stream, int> stream_index_pair;

    class RosSensor : public rs2::sensor
    {
        public:
            RosSensor(rs2::sensor sensor,
                      rclcpp::Node& node, 
                      std::function<void(rs2::frame)> frame_callback,
                      std::function<void()> update_sensor_func): 
                rs2::sensor(sensor),
                _node(node),
                _logger(rclcpp::get_logger("RealSenseCameraNode")),
                _frame_callback(frame_callback),
                _params(node, _logger),
                _update_sensor_func(update_sensor_func)
                {
                    setParameters();
                };
            virtual void getUpdatedSensorParameters() = 0;
            virtual void getUpdatedProfileParameters(const rs2::stream_profile& profile) = 0;
            virtual bool isWantedProfile(const rs2::stream_profile& profile) = 0;
            virtual void registerProfileParameters() = 0;
            
            ////////////////////////////////////////////////////////////
            // updateProfiles():
            // return: a vector of profiles matching the updated parameters of an empty one if active profiles matches parameters.
            //
            bool getUpdatedProfiles(std::vector<rs2::stream_profile>& wanted_profiles);
            
            bool start(const std::vector<rs2::stream_profile>& profiles);
            void stop();
            template<class T>
            void registerSensorUpdateParam(std::string template_name, T value);

            template<class T> 
            bool is() const
            {
                return (dynamic_cast<const T*> (&(*this)));
            }

        private:
            void setupErrorCallback();
            void setParameters();

        protected:
            rclcpp::Node& _node;
            rclcpp::Logger _logger;
            std::function<void(rs2::frame)> _frame_callback;
            SensorParams _params;
            std::function<void()> _update_sensor_func;
            // std::condition_variable _cv_update;
            // std::mutex _m_update;
            std::shared_ptr<std::thread> _check_update_t;

            std::map<stream_index_pair, bool> _enabled_profiles;
    };

    class VideoSensor : public RosSensor
    {
        public:
            VideoSensor(rs2::sensor sensor, rclcpp::Node& node,
                        std::function<void(rs2::frame)> frame_callback,
                        std::function<void()> update_sensor_func);
            void getUpdatedSensorParameters() override;
            void getUpdatedProfileParameters(const rs2::stream_profile& profile) override;        
            bool isWantedProfile(const rs2::stream_profile& profile) override;
            void registerProfileParameters() override;
            // template<class T>
            // rcl_interfaces::msg::SetParametersResult set_sensor_general_param(std::string option_name, const std::vector<rclcpp::Parameter> & parameters);

        private:
            std::map<rs2_stream, rs2_format>  _allowed_formats;
            double      _fps;
            int _width, _height;
    };

    class ImuSensor : public RosSensor
    {
        public:
            ImuSensor(rs2::sensor sensor, rclcpp::Node& node,
                      std::function<void(rs2::frame)> frame_callback,
                      std::function<void()> update_sensor_func);
            void getUpdatedSensorParameters() override;
            void getUpdatedProfileParameters(const rs2::stream_profile& profile) override;        
            bool isWantedProfile(const rs2::stream_profile& profile) override;
            void registerProfileParameters() override;

        private:
            std::map<stream_index_pair, double> _fps;

    };

    // class PoseSensor : public RosSensor
    // {
    //     public:
    //         PoseSensor(rs2::sensor sensor, rclcpp::Node& node,
    //                   std::function<void(rs2::frame)> frame_callback,
    //                   std::function<void()> update_sensor_func): RosSensor(node, sensor, frame_callback, update_sensor_func) 
    //         {
    //             _stream_name[RS2_STREAM_POSE] = "pose";
    //         };
    //         void getUpdatedSensorParameters() override;
    //         void getUpdatedProfileParameters(const rs2::stream_profile& profile) override;        
    //         rs2::stream_profile getWantedProfile(const stream_index_pair& sip) override;
    //         bool isWantedProfile(const rs2::stream_profile& profile) override;
    //         void registerSensorParameters();
    //         void registerProfileParameters();
// };
}