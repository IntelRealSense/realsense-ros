#pragma once
#include <algorithm>
#include <stdexcept>
#include <string>
#include <sstream>

struct PersonTrackingConfig
{
    enum StreamingMode {STREAMING_MODE_PLAYBACK, STREAMING_MODE_LIVE, STREAMING_MODE_RECORD};
    enum Resolution {RESOLUTION_QVGA, RESOLUTION_VGA, RESOLUTION_HD, RESOLUTION_FULLHD};
    bool gesturesEnabled;
    bool sceletonEnabled;
    bool recognitionEnabled;
    bool trackingEnabled;
    int trackingMode;
    bool loadDb;
    bool saveDb;
    std::string dbPath;
    StreamingMode streamingMode;
    std::string clipPath;
    bool segmentationEnabled;
    bool orientationEnabled;
    bool headPositionEnabled;
    bool headBoundingBoxEnabled;
    bool landmarksEnabled;
    Resolution colorResolution;
    Resolution depthResolution;

public:
    PersonTrackingConfig():
        gesturesEnabled(false),
        sceletonEnabled(false),
        recognitionEnabled(false),
        trackingEnabled(false),
        trackingMode(0),
        loadDb(false),
        saveDb(false),
        dbPath("bad path"),
        streamingMode(STREAMING_MODE_LIVE),
        clipPath("bad path"),
        segmentationEnabled(false),
        orientationEnabled(false),
        headPositionEnabled(false),
        headBoundingBoxEnabled(false),
        landmarksEnabled(false),
        colorResolution(RESOLUTION_VGA),
        depthResolution(RESOLUTION_QVGA)
        {}

    static int GetResolutionWidth(Resolution res)
    {
        switch(res)
        {
            case RESOLUTION_QVGA:
                return 320;
            case RESOLUTION_VGA:
                return 640;
            case RESOLUTION_HD:
                return 1280;
            case RESOLUTION_FULLHD:
                return 1920;
            default:
                throw std::runtime_error("unknown value for resolution");
        }
    }

    static int GetResolutionHeight(Resolution res)
    {
        switch(res)
        {
            case RESOLUTION_QVGA:
                return 240;
            case RESOLUTION_VGA:
                return 480;
            case RESOLUTION_HD:
                return 720;
            case RESOLUTION_FULLHD:
                return 1080;
            default:
                throw std::runtime_error("unknown value for resolution");
        }
    }
};
