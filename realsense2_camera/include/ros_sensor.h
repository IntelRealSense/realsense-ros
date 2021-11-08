#ifndef ___ROS_SENSOR_HEADER_FILE___
#define ___ROS_SENSOR_HEADER_FILE___

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include "constants.h"
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <ros_utils.h>
#include <sensor_params.h>
#include <profile_manager.h>

namespace realsense2_camera
{
    typedef std::pair<rs2_stream, int> stream_index_pair;

    class RosSensor : public rs2::sensor
    {
        
        public:
            RosSensor(rs2::sensor sensor,
                      std::shared_ptr<Parameters> parameters, 
                      std::function<void(rs2::frame)> frame_callback,
                      std::function<void()> update_sensor_func,
                      std::function<void()> hardware_reset_func);
            ~RosSensor();
            void registerSensorParameters();
            bool getUpdatedProfiles(std::vector<rs2::stream_profile>& wanted_profiles);
            void runFirstFrameInitialization();
            virtual bool start(const std::vector<rs2::stream_profile>& profiles);
            void stop();
            rmw_qos_profile_t getQOS(const stream_index_pair& sip) const;
            rmw_qos_profile_t getInfoQOS(const stream_index_pair& sip) const;

            template<class T> 
            bool is() const
            {
                return (dynamic_cast<const T*> (&(*this)));
            }

        private:
            void setupErrorCallback();
            void setParameters();
            void clearParameters();
            void set_sensor_auto_exposure_roi();
            void registerAutoExposureROIOptions();
            void UpdateSequenceIdCallback();
            void set_sensor_parameter_to_ros(rs2_option option);

        private:
            rclcpp::Logger _logger;
            std::function<void(rs2::frame)> _origin_frame_callback;
            std::function<void(rs2::frame)> _frame_callback;
            SensorParams _params;
            std::function<void()> _update_sensor_func, _hardware_reset_func;
            bool _is_first_frame;
            std::vector<std::function<void()> > _first_frame_functions_stack;
            std::vector<std::shared_ptr<ProfilesManager> > _profile_managers;
            rs2::region_of_interest _auto_exposure_roi;
            std::vector<std::string> _parameters_names;
    };
}
#endif //___ROS_SENSOR_HEADER_FILE___