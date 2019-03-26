#ifndef SP_TYPE_H
#define SP_TYPE_H

#include <stdint.h>
#include "MathUtil.h"

using namespace ScenePerception;

/// status of API function calls.
typedef enum SP_STATUS : int
{
    SP_STATUS_SUCCESS,
    SP_STATUS_ERROR,
    SP_STATUS_WARNING,
    SP_STATUS_INVALIDARG,
    SP_STATUS_NOT_CONFIGURED,
    SP_STATUS_PLATFORM_NOT_SUPPORTED
} SP_STATUS;

/// A struct to represent camera extrinsic parameters that include rotation and translation
/// extrinsic parameters as well as their variances.
typedef struct
{
    PoseMatrix4f pose; /*!< translation and rotation extrinsic parameters. */
    PoseMatrix4f variance; /*!< variance of translation and rotation extrinsic parameters. */
}
SP_CameraExtrinsics;

/// A struct to represent intrinsic parameters of camera sensor.
typedef struct
{
    unsigned int imageWidth; /*!< imageWidth : width of camera images. */
    unsigned int imageHeight; /*!< imageHeight : height of camera images. */
    float focalLengthHorizontal; /*!< focalLengthHorizontal : focal length along x axis of Camera Sensor in pixels. */
    float focalLengthVertical; /*!< focalLengthVertical : focal length along y axis of Camera Sensor in pixels. */
    float principalPointCoordU; /*!< principalPointCoordU : x-component of principle point of Camera Sensor in pixels. */
    float principalPointCoordV; /*!< principalPointCoordV : y-component of principle point of Camera Sensor in pixels. */
}
SP_CameraIntrinsics;


#endif //SP_TYPE_H
