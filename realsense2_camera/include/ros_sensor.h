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
                      std::function<void()> update_sensor_func);
            ~RosSensor();
            void registerSensorParameters();
            bool getUpdatedProfiles(std::vector<rs2::stream_profile>& wanted_profiles);
            void runFirstFrameInitialization();
            virtual bool start(const std::vector<rs2::stream_profile>& profiles);
            void stop();

            template<class T> 
            bool is() const
            {
                return (dynamic_cast<const T*> (&(*this)));
            }

        private:
            void setupErrorCallback();
            void setParameters();
            void set_sensor_auto_exposure_roi();
            void registerAutoExposureROIOptions();

        private:
            rclcpp::Logger _logger;
            std::function<void(rs2::frame)> _origin_frame_callback;
            std::function<void(rs2::frame)> _frame_callback;
            SensorParams _params;
            std::function<void()> _update_sensor_func;
            std::shared_ptr<std::thread> _check_update_t;
            std::map<stream_index_pair, bool> _enabled_profiles;
            bool _is_first_frame;
            std::vector<std::function<void()> > _first_frame_functions_stack;
            std::vector<std::shared_ptr<ProfilesManager> > _profile_managers;
            rs2::region_of_interest _auto_exposure_roi;
    };
}