#pragma once
#include <realsense_srvs/TrackingConfig.h>

class PersonTrackingConfigurator
{
public:
    virtual void configureTracking(realsense_srvs::TrackingConfig::Request& request) = 0;
};
