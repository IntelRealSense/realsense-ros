#include "MathUtil.h"
#include "SP_Types.h"

namespace ScenePerception {

	inline SP_CameraIntrinsics arrayToIntrinsics(const float* cameraParam)
	{
		SP_CameraIntrinsics intrinsics;
		intrinsics.focalLengthHorizontal = cameraParam[0];
		intrinsics.focalLengthVertical = cameraParam[1];
		intrinsics.principalPointCoordU = cameraParam[2];
		intrinsics.principalPointCoordV = cameraParam[3];
		intrinsics.imageWidth = (unsigned int)lrintf(cameraParam[4]);
		intrinsics.imageHeight = (unsigned int)lrintf(cameraParam[5]);
		return intrinsics;
	}

	SP_CameraIntrinsics arrayToIntrinsics(const float* cameraParam);
	float4 imgPosToPoint(const SP_CameraIntrinsics* intrinsics, const float3& f3Imgpos);
	float3 pointToImgPos(const SP_CameraIntrinsics* intrinsics, const float3& f3Cam3d);
	float3 pointToImgPos(const SP_CameraIntrinsics* intrinsics, const float4& f3Cam3d);
}
