#pragma once

#include "RealSense/PersonTracking/PersonTrackingData.h"
#include <string>

namespace PT = Intel::RealSense::PersonTracking;

std::string toString(PT::PersonTrackingData::PersonRecognition::RegistrationStatus status, int32_t recognitionId)
{
    switch (status)
    {
        case PT::PersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationSuccessful:
            return "Person registered with id: " + std::to_string(recognitionId);
        case PT::PersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailed:
            return "Person registration failed!";
        case PT::PersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedAlreadyRegistered:
            return "Person already registered with id: " + std::to_string(recognitionId);
        case PT::PersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedFaceNotDetected:
            return "Person registration error: Face not detected";
        case PT::PersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedFaceNotClear:
            return "Person registration error: Face not clear";
        case PT::PersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedPersonTooFar:
            return "Person registration error: Person too far";
        case PT::PersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedPersonTooClose:
            return "Person registration error: Person too close";
        default:
            return ": Person registration failed!";
    }
}