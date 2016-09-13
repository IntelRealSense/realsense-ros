#include "PersonTrackingServer.h"
#include "RecognitionUtils.h"
#include <vector>
#include <fstream>

using namespace PT;

PersonTrackingServer::PersonTrackingServer()
    : mConfigurator(nullptr), mPersonTracking(nullptr) {}

void PersonTrackingServer::onInit(ros::NodeHandle& nodeHandle, PtOpencvAdapter* personTracking, PersonTrackingConfigurator* configurator)
{
    mPersonTracking = personTracking;
    mConfigurator = configurator;

    mTrackingConfigService = nodeHandle.advertiseService("person_tracking/tracking_config", &PersonTrackingServer::trackingConfigRequestCallback, this);
    mRecognitionRequestService = nodeHandle.advertiseService("person_tracking/recognition_request", &PersonTrackingServer::recognitionRequestCallback, this);
    mTrackingRequestService = nodeHandle.advertiseService("person_tracking/tracking_request", &PersonTrackingServer::trackingRequestCallback, this);
    mSaveRecognitionDbService = nodeHandle.advertiseService("person_tracking/save_recognition", &PersonTrackingServer::saveRecognitionDbCallback, this);
    mLoadRecognitionDbService = nodeHandle.advertiseService("person_tracking/load_recognition", &PersonTrackingServer::loadRecognitionDbCallback, this);
    mResetTrackingRequestService = nodeHandle.advertiseService("person_tracking/reset_tracking", &PersonTrackingServer::resetTrackingCallback, this);
}

bool PersonTrackingServer::trackingConfigRequestCallback(   realsense_srvs::TrackingConfig::Request& request,
                                                            realsense_srvs::TrackingConfig::Response& response)
{
    mConfigurator->configureTracking(request);
    return true;
}

bool PersonTrackingServer::recognitionRequestCallback(  realsense_srvs::RecognitionRequest::Request& request,
                                                        realsense_srvs::RecognitionRequest::Response& response)
{
    if (!mPersonTracking->QueryConfiguration()->QueryRecognition()->IsEnabled())
    {
        ROS_INFO("Recognition is not enabled");
        return false;
    }

    ROS_INFO_STREAM("Received recognition request for person: " << request.personId);

    PersonTrackingData* trackingData = mPersonTracking->QueryOutput();
    PersonTrackingData::Person* personData = trackingData->QueryPersonDataById(request.personId);

    if (!personData)
    {
        ROS_ERROR_STREAM("Couldn't find recognition request target");
        return false;
    }

    PersonTrackingData::PersonRecognition* recognition = personData->QueryRecognition();

    if (request.recognitionId <= 0)
    {
        registerPerson(recognition, response);
    }
    else
    {
        reinforceRecognition(recognition, request.recognitionId);
    }

    return true;
}

void PersonTrackingServer::registerPerson(PersonTrackingData::PersonRecognition* recognition, realsense_srvs::RecognitionRequest::Response& response)
{
    ROS_INFO_STREAM("Found recognition request target");

    int32_t outputRecognitionId;
    auto status = recognition->RegisterUser(&outputRecognitionId);

    response.returnCode = status;
    response.description = toString(status, outputRecognitionId);

    if (status == PersonTrackingData::PersonRecognition::RegistrationSuccessful ||
        status == PersonTrackingData::PersonRecognition::RegistrationFailedAlreadyRegistered)
    {
        ROS_INFO_STREAM("Registered ID: " << outputRecognitionId);
        response.recognitionId = outputRecognitionId;
    }
    else
    {
        ROS_WARN("Failed to register person: %d", status);
        response.recognitionId = -1;
    }
}

void PersonTrackingServer::reinforceRecognition(PersonTrackingData::PersonRecognition* recognition, int rid)
{
    ROS_INFO("Found reinforce recognition request target - rid = %d", rid);

    auto result = recognition->ReinforceUserRegistration(rid, PersonTrackingData::PersonRecognition::RegistrationPolicy::RegisterPolicyManualAdd);

    ROS_INFO("Reinforce registration finished with status %d", result);
}

bool PersonTrackingServer::trackingRequestCallback( realsense_srvs::TrackingRequest::Request& request,
                                                    realsense_srvs::TrackingRequest::Response& response)
{
    ROS_INFO_STREAM("Received tracking request for person: " << request.personId);

    PersonTrackingData* trackingData = mPersonTracking->QueryOutput();
    PersonTrackingData::Person* personData = trackingData->QueryPersonDataById(request.personId);

    if (!personData)
    {
        ROS_ERROR_STREAM("Couldn't find tracking request target");
        return false;
    }

    ROS_INFO_STREAM("Found tracknig request target");

    PersonTrackingData::TrackingState trackingState = trackingData->GetTrackingState();
    if (trackingState == PersonTrackingData::TRACKING_STATE_DETECTING)
    {
        trackingData->StartTracking(request.personId);
    }
    else
    {
        trackingData->StopTracking(request.personId);
    }

    response.status = 1;
    return true;
}

bool PersonTrackingServer::saveRecognitionDbCallback(   realsense_srvs::SaveRecognitionDB::Request& request,
                                                        realsense_srvs::SaveRecognitionDB::Response& response)
{
    PersonTrackingData* personTrackingData = mPersonTracking->QueryOutput();
    if (!personTrackingData)
    {
        ROS_WARN("personTrackingData == null");
        return false;
    }

    auto config = mPersonTracking->QueryConfiguration();
    auto db = config->QueryRecognition()->QueryDatabase();
    auto dbUtils = config->QueryRecognition()->QueryDatabaseUtilities();

    auto dbSize = dbUtils->GetDatabaseMemorySize(db);
    std::vector<uint8_t> buffer(dbSize, 0);
    int32_t writtenSize;
    dbUtils->SerializeDatabase(db, buffer.data(), (int32_t)buffer.size(), &writtenSize);

    std::ofstream out(request.saveToPath, std::ios::binary);
    out.write((char*)buffer.data(), buffer.size());

    ROS_INFO("Successfully saved recognition database to %s", request.saveToPath.c_str());

    return true;
}

bool PersonTrackingServer::loadRecognitionDbCallback(   realsense_srvs::LoadRecognitionDB::Request& request,
                                                        realsense_srvs::LoadRecognitionDB::Response& response)
{
    std::ifstream file(request.loadFromPath, std::ios::binary | std::ios::ate);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    if (!file.read(buffer.data(), size))
    {
        ROS_ERROR("Failed to read recognition database from %s", request.loadFromPath.c_str());
        return false;
    }

    auto database = mPersonTracking->QueryConfiguration()->QueryRecognition()->QueryDatabase();
    auto dbUtils = mPersonTracking->QueryConfiguration()->QueryRecognition()->QueryDatabaseUtilities();

    dbUtils->DeserializeDatabase(database, (unsigned char*)buffer.data(), (int)buffer.size());

    ROS_INFO("Successfully loaded recognition database from %s", request.loadFromPath.c_str());

    return true;
}

bool PersonTrackingServer::resetTrackingCallback(   realsense_srvs::ResetTrackingRequest::Request& request,
                                                    realsense_srvs::ResetTrackingRequest::Response& response)
{
    ROS_INFO("Received reset tracking request");
    mPersonTracking->QueryOutput()->ResetTracking();
    return true;
}
