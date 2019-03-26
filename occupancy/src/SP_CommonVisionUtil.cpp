#include "SP_CommonVisionUtil.h"

using namespace ScenePerception;

float4 ScenePerception::imgPosToPoint(const SP_CameraIntrinsics* intrinsics, const float3& f3Imgpos)
{
    const float& rfx = intrinsics->focalLengthHorizontal;
    const float& rfy = intrinsics->focalLengthVertical;
    const float& fpx = intrinsics->principalPointCoordU;
    const float& fpy = intrinsics->principalPointCoordV;

    return float4(
               f3Imgpos.z*(f3Imgpos.x - fpx) / rfx,
               f3Imgpos.z*(f3Imgpos.y - fpy) / rfy,
               f3Imgpos.z, 1.0f);
}

float3 ScenePerception::pointToImgPos(const SP_CameraIntrinsics* intrinsics, const float3& f3Cam3d)
{
    const float& rfx = intrinsics->focalLengthHorizontal;
    const float& rfy = intrinsics->focalLengthVertical;
    const float& fpx = intrinsics->principalPointCoordU;
    const float& fpy = intrinsics->principalPointCoordV;

    return float3(
               rfx*f3Cam3d.x / f3Cam3d.z + fpx,
               rfy*f3Cam3d.y / f3Cam3d.z + fpy,
               1.0f);
}

float3 ScenePerception::pointToImgPos(const SP_CameraIntrinsics* intrinsics, const float4& f3Cam3d)
{
    const float& rfx = intrinsics->focalLengthHorizontal;
    const float& rfy = intrinsics->focalLengthVertical;
    const float& fpx = intrinsics->principalPointCoordU;
    const float& fpy = intrinsics->principalPointCoordV;

    return float3(
               rfx*f3Cam3d.x / f3Cam3d.z + fpx,
               rfy*f3Cam3d.y / f3Cam3d.z + fpy,
               1.0f);
}
