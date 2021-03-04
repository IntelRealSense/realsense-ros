#include <ros_sensor.h>
#include <ros_utils.h>
#include <stdio.h>

using namespace realsense2_camera;
using namespace rs2;

ImuSensor::ImuSensor(rs2::sensor sensor, std::shared_ptr<Parameters> parameters,
                     std::function<void(rs2::frame)> frame_callback,
                     std::function<void()> update_sensor_func): 
    RosSensor(sensor, parameters, frame_callback, update_sensor_func) 
{
    registerSensorParameters();
}

bool ImuSensor::isWantedProfile(const rs2::stream_profile& profile)
{
    stream_index_pair stream(profile.stream_type(), profile.stream_index());
    return (profile.fps() == _fps[stream]);
}

void ImuSensor::registerSensorParameters()
{
    registerSensorUpdateParam("enable_%s", _enabled_profiles, true);
    registerSensorUpdateParam("%s_fps", _fps, 0.0);
}
