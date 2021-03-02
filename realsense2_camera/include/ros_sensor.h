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
                      std::function<void()> update_sensor_func);
            virtual void getUpdatedProfileParameters(const rs2::stream_profile& profile) = 0;
            virtual bool isWantedProfile(const rs2::stream_profile& profile) = 0;
            virtual void registerSensorParameters() = 0;
            
            ////////////////////////////////////////////////////////////
            // updateProfiles():
            // return: a vector of profiles matching the updated parameters of an empty one if active profiles matches parameters.
            //
            bool getUpdatedProfiles(std::vector<rs2::stream_profile>& wanted_profiles);
            void runFirstFrameInitialization();
            virtual bool start(const std::vector<rs2::stream_profile>& profiles);
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
            std::function<void(rs2::frame)> _origin_frame_callback;
            std::function<void(rs2::frame)> _frame_callback;
            SensorParams _params;
            std::function<void()> _update_sensor_func;
            // std::condition_variable _cv_update;
            // std::mutex _m_update;
            std::shared_ptr<std::thread> _check_update_t;
            std::map<stream_index_pair, bool> _enabled_profiles;
            bool _is_first_frame;
            std::vector<std::function<void()> > _first_frame_functions_stack;
    };

    class VideoSensor : public RosSensor
    {
        public:
            VideoSensor(rs2::sensor sensor, rclcpp::Node& node,
                        std::function<void(rs2::frame)> frame_callback,
                        std::function<void()> update_sensor_func);
            bool start(const std::vector<rs2::stream_profile>& profiles) override;
            void getUpdatedProfileParameters(const rs2::stream_profile& profile) override;        
            bool isWantedProfile(const rs2::stream_profile& profile) override;
            void registerSensorParameters() override;
            // template<class T>
            // rcl_interfaces::msg::SetParametersResult set_sensor_general_param(std::string option_name, const std::vector<rclcpp::Parameter> & parameters);

        private:
            void set_sensor_auto_exposure_roi();
            void registerAutoExposureROIOptions();

        private:
            std::map<rs2_stream, rs2_format>  _allowed_formats;
            double      _fps;
            int _width, _height;
            rs2::region_of_interest _auto_exposure_roi;
    };

    class ImuSensor : public RosSensor
    {
        public:
            ImuSensor(rs2::sensor sensor, rclcpp::Node& node,
                      std::function<void(rs2::frame)> frame_callback,
                      std::function<void()> update_sensor_func);
            void getUpdatedProfileParameters(const rs2::stream_profile& profile) override;        
            bool isWantedProfile(const rs2::stream_profile& profile) override;
            void registerSensorParameters() override;

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
    //         void getUpdatedProfileParameters(const rs2::stream_profile& profile) override;        
    //         rs2::stream_profile getWantedProfile(const stream_index_pair& sip) override;
    //         bool isWantedProfile(const rs2::stream_profile& profile) override;
    //         void registerSensorParameters();
    //         void registerSensorParameters();
// };
}