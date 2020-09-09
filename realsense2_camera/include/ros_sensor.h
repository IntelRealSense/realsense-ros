#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "constants.h"
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <ros_utils.h>

#define STREAM_NAME(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << create_graph_resource_name(rs2_stream_to_string(sip.first)) << ((sip.second>0) ? std::to_string(sip.second) : ""))).str()

namespace realsense2_camera
{
    typedef std::pair<rs2_stream, int> stream_index_pair;

    class RosSensor : public rs2::sensor
    {
        public:
            RosSensor(rs2::sensor sensor,
                      rclcpp::Node& node, 
                      std::function<void(rs2::frame)> frame_callback): rs2::sensor(sensor),
                _node(node),
                _logger(rclcpp::get_logger("RealSenseCameraNode")),
                _frame_callback(frame_callback)
                {
                };
            virtual void getUpdatedSensorParameters() = 0;
            virtual void getUpdatedProfileParameters(const rs2::stream_profile& profile) = 0;
            virtual bool isWantedProfile(const rs2::stream_profile& profile) = 0;
            
            ////////////////////////////////////////////////////////////
            // updateProfiles():
            // return: a vector of profiles matching the updated parameters of an empty one if active profiles matches parameters.
            //
            std::vector<rs2::stream_profile> getUpdatedProfiles();   
            
            bool start(const std::vector<rs2::stream_profile>& profiles);
            void stop();

            template<class T> 
            bool is() const
            {
                return (dynamic_cast<const T*> (&(*this)));
            }

        private:
            void setupErrorCallback();

        protected:
            rs2::sensor _sensor;
            rclcpp::Node& _node;
            rclcpp::Logger _logger;
            std::function<void(rs2::frame)> _frame_callback;

            std::map<stream_index_pair, bool> _enable;
    };

    class VideoSensor : public RosSensor
    {
        public:
            VideoSensor(rs2::sensor sensor, rclcpp::Node& node,
                        std::function<void(rs2::frame)> frame_callback);
            void getUpdatedSensorParameters() override;
            void getUpdatedProfileParameters(const rs2::stream_profile& profile) override;        
            bool isWantedProfile(const rs2::stream_profile& profile) override;

        private:
            std::map<rs2_stream, rs2_format>  _allowed_formats;
            double      _fps;
            int _width, _height;
    };

    class ImuSensor : public RosSensor
    {
        public:
            ImuSensor(rs2::sensor sensor, rclcpp::Node& node,
                      std::function<void(rs2::frame)> frame_callback);
            void getUpdatedSensorParameters() override;
            void getUpdatedProfileParameters(const rs2::stream_profile& profile) override;        
            bool isWantedProfile(const rs2::stream_profile& profile) override;

        private:
            std::map<stream_index_pair, double> _fps;

    };

    // class PoseSensor : public RosSensor
    // {
    //     public:
    //         PoseSensor(rs2::sensor sensor, rclcpp::Node& node,
    //                   std::function<void(rs2::frame)> frame_callback): RosSensor(node, sensor, frame_callback) 
    //         {
    //             _stream_name[RS2_STREAM_POSE] = "pose";
    //         };
    //         void getUpdatedSensorParameters() override;
    //         void getUpdatedProfileParameters(const rs2::stream_profile& profile) override;        
    //         rs2::stream_profile getWantedProfile(const stream_index_pair& sip) override;
    //         bool isWantedProfile(const rs2::stream_profile& profile) override;
    // };
}