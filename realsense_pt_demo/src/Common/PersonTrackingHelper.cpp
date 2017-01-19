#include "PersonTrackingHelper.h"
#include <iostream>
#include <fstream>
#include <vector>
void ConfigurePersonTracking(PersonTrackingConfig config, Intel::RealSense::PersonTracking::PersonTrackingConfiguration* ptConfiguration)
{

    if(config.recognitionEnabled)
    {
        std::cout << "recognition enabled" << std::endl;
        ptConfiguration->QueryRecognition()->Enable();
    }
    else
    {
        std::cout << "recognition disabled" << std::endl;
        ptConfiguration->QueryRecognition()->Disable();
    }

    if(config.gesturesEnabled)
    {
        std::cout << "gestures enabled" << std::endl;
        ptConfiguration->QueryGestures()->Enable();
        ptConfiguration->QueryGestures()->EnableAllGestures();
    }
    else
    {
        std::cout << "gestures disabled" << std::endl;
        ptConfiguration->QueryGestures()->Disable();
        ptConfiguration->QueryGestures()->DisableAllGestures();
    }


    if(config.sceletonEnabled)
    {
        std::cout << "skeleton enabled" << std::endl;
        ptConfiguration->QueryGestures()->Enable();
        ptConfiguration->QuerySkeletonJoints()->Enable();
    }
    else
    {
        std::cout << "skeleton disabled" << std::endl;
        ptConfiguration->QuerySkeletonJoints()->Disable();
    }


    if(config.trackingEnabled)
    {
        ptConfiguration->QueryTracking()->Enable();
    }
    else
    {
        ptConfiguration->QueryTracking()->Disable();
    }

    if (config.segmentationEnabled)
    {
        std::cout << "segmentation enabled" << std::endl;
        ptConfiguration->QueryTracking()->Enable();
        ptConfiguration->QueryTracking()->EnableSegmentation();
	ptConfiguration->QueryTracking()->EnableBlob();
    }
    else
    {
        std::cout << "segmentation disabled" << std::endl;
        ptConfiguration->QueryTracking()->DisableSegmentation();
	ptConfiguration->QueryTracking()->DisableBlob();

    }


    if (config.headPositionEnabled)
    {
        std::cout << "head head position enabled" << std::endl;
        ptConfiguration->QueryTracking()->Enable();
        ptConfiguration->QueryFace()->EnableHeadPose();//enableHeadPose
    }
    else
    {
        std::cout << "head head position disabled" << std::endl;
        ptConfiguration->QueryFace()->DisableHeadPose();
    }


    if (config.headBoundingBoxEnabled)
    {
        std::cout << "head bounding box enabled" << std::endl;
        ptConfiguration->QueryTracking()->Enable();
        ptConfiguration->QueryTracking()->EnableHeadBoundingBox();
    }
    else
    {
        std::cout << "head bounding box disabled" << std::endl;
        ptConfiguration->QueryTracking()->DisableHeadBoundingBox();
    }


    if (config.landmarksEnabled)
    {
        std::cout << "landmarks enabled" << std::endl;
        ptConfiguration->QueryTracking()->Enable();
        ptConfiguration->QueryFace()->EnableFaceLandmarks();
    }
    else
    {
        std::cout << "landmarks disabled" << std::endl;
        ptConfiguration->QueryFace()->DisableFaceLandmarks();
    }

    if(config.loadDb)
    {
        try{
            LoadDatabase(config.dbPath, ptConfiguration);
	}catch(const std::runtime_error& error){
		std::cerr<<"load data failed"<<std::endl;
	}
    }

}

void LoadDatabase(std::string dbPath, Intel::RealSense::PersonTracking::PersonTrackingConfiguration* ptConfiguration)
{
    std::cout << "loading database from: " + dbPath << std::endl;
    auto database = ptConfiguration->QueryRecognition()->QueryDatabase();
    auto dbUtils = ptConfiguration->QueryRecognition()->QueryDatabaseUtilities();

    std::filebuf fb;
    if (!fb.open (dbPath, std::ios::in | std::ios::binary))
    {
        throw  std::runtime_error("failed to open database file");
    }
    std::istream is(&fb);
    is.seekg(0, std::ios::end);
    int size = is.tellg();
    is.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    is.read(buffer.data(), buffer.size());
    fb.close();
    if(!dbUtils->DeserializeDatabase(database, (unsigned char*)buffer.data(), (int)buffer.size()))
    {
        throw std::runtime_error("fail to load database");
    }
}

void SaveDatabase(std::string dbPath, Intel::RealSense::PersonTracking::PersonTrackingConfiguration* ptConfiguration)
{
    std::cout << "save database to: " + dbPath << std::endl;
    auto db = ptConfiguration->QueryRecognition()->QueryDatabase();
    auto dbUtils = ptConfiguration->QueryRecognition()->QueryDatabaseUtilities();

    const size_t dbSize = dbUtils->GetDatabaseMemorySize(db);

    std::vector<uint8_t> buffer(dbSize, 0);
    int32_t writtenSize;
    dbUtils->SerializeDatabase(db, buffer.data(), (int32_t)buffer.size(), &writtenSize);

    std::ofstream out(dbPath, std::ios::binary);
    if(out){
    out.write((char*)buffer.data(), buffer.size());
    }
    else{
        throw std::runtime_error("fail to save database");
    }
}

void SetTracking(const PersonTrackingConfig& config, Intel::RealSense::PersonTracking::PersonTrackingData* trackingData)
{
    if (config.trackingEnabled && config.trackingMode == 1 &&
        trackingData->GetTrackingState() == Intel::RealSense::PersonTracking::PersonTrackingData::TrackingState::TRACKING_STATE_DETECTING &&
        trackingData->QueryNumberOfPeople() > 0)
    {
        auto personData = trackingData->QueryPersonData(Intel::RealSense::PersonTracking::PersonTrackingData::ACCESS_ORDER_BY_INDEX, 0);
        if (personData)
        {
            trackingData->StartTracking(personData->QueryTracking()->QueryId());
        }
    }
}

