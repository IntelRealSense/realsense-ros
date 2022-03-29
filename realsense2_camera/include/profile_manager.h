// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2022 Intel Corporation. All Rights Reserved.

#pragma once

#include <librealsense2/rs.hpp>
#include <sensor_params.h>

#define STREAM_NAME(sip) (static_cast<std::ostringstream&&>(std::ostringstream() << create_graph_resource_name(ros_stream_to_string(sip.first)) << ((sip.second>0) ? std::to_string(sip.second) : ""))).str()

using namespace rs2;
namespace realsense2_camera
{
    typedef std::pair<rs2_stream, int> stream_index_pair;

    class ProfilesManager
    {
        public:
            ProfilesManager(std::shared_ptr<Parameters> parameters, rclcpp::Logger logger);
            virtual bool isWantedProfile(const rs2::stream_profile& profile) = 0;
            virtual void registerProfileParameters(std::vector<stream_profile> all_profiles, std::function<void()> update_sensor_func) = 0;
            bool isTypeExist();
            static std::string profile_string(const rs2::stream_profile& profile);
            
            void registerSensorQOSParam(std::string template_name, 
                                        std::set<stream_index_pair> unique_sips, 
                                        std::map<stream_index_pair, std::shared_ptr<std::string> >& params, 
                                        std::string value);

            template<class T>
            void registerSensorUpdateParam(std::string template_name, 
                                           std::set<stream_index_pair> unique_sips, 
                                           std::map<stream_index_pair, std::shared_ptr<T> >& params, 
                                           T value, 
                                           std::function<void()> update_sensor_func);
            void addWantedProfiles(std::vector<rs2::stream_profile>& wanted_profiles);
            void clearParameters();
            bool hasSIP(const stream_index_pair& sip) const;
            rmw_qos_profile_t getQOS(const stream_index_pair& sip) const;
            rmw_qos_profile_t getInfoQOS(const stream_index_pair& sip) const;

        protected:
            rs2::stream_profile getDefaultProfile();

        protected:
            rclcpp::Logger _logger;
            SensorParams _params;
            std::map<stream_index_pair, std::shared_ptr<bool>> _enabled_profiles;
            std::map<stream_index_pair, std::shared_ptr<std::string>> _profiles_image_qos_str, _profiles_info_qos_str;
            std::vector<rs2::stream_profile> _all_profiles;
            std::vector<std::string> _parameters_names;
    };

    class VideoProfilesManager : public ProfilesManager
    {
        public:
            VideoProfilesManager(std::shared_ptr<Parameters> parameters, const std::string& module_name, rclcpp::Logger logger, bool force_image_default_qos = false);
            bool isWantedProfile(const rs2::stream_profile& profile) override;
            void registerProfileParameters(std::vector<stream_profile> all_profiles, std::function<void()> update_sensor_func) override;
            int getHeight() {return _height;};
            int getWidth() {return _width;};
            int getFPS() {return _fps;};

        private:
            bool isSameProfileValues(const rs2::stream_profile& profile, const int width, const int height, const int fps);
            void registerVideoSensorParams();
            std::string get_profiles_descriptions();

        private:
            std::string _module_name;
            std::map<rs2_stream, rs2_format>  _allowed_formats;
            int      _fps;
            int _width, _height;
            bool _is_profile_exist;
            bool _force_image_default_qos;
    };

    class MotionProfilesManager : public ProfilesManager
    {
        public:
            using ProfilesManager::ProfilesManager;
            bool isWantedProfile(const rs2::stream_profile& profile) override;
            void registerProfileParameters(std::vector<stream_profile> all_profiles, std::function<void()> update_sensor_func) override;

        private:
            void registerFPSParams();
            bool isSameProfileValues(const rs2::stream_profile& profile, const rs2_stream stype, const int fps);
            std::map<stream_index_pair, std::vector<int>> getAvailableFPSValues();

        protected:
            std::map<stream_index_pair, std::shared_ptr<int> > _fps;
    };

    class PoseProfilesManager : public MotionProfilesManager
    {
        public:
            using MotionProfilesManager::MotionProfilesManager;
            void registerProfileParameters(std::vector<stream_profile> all_profiles, std::function<void()> update_sensor_func) override;
    };

}
