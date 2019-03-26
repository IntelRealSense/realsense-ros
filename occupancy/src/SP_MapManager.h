#ifndef SP_MAPMANAGER_H
#define SP_MAPMANAGER_H

#include <fstream>
#include <list>
#include <map>
#include <set>
#include <time.h>
#include "SP_MappingTypes.h"
#include "MathUtil.h"
#include "SP_CrossPlatformsFunctions.h"
#include <mutex>
#include <memory>
#include <thread>
#include <condition_variable>

namespace ScenePerception
{
#define maxFrameRejections 3
#define mapUpdateProblemSize 2

	struct TileInfo
	{
		int32_t x, z;
		uint8_t occupancy;
		uint8_t hasHeight;
		int32_t height;
	};

	class frameData
	{
	public:
		frameData() {}
		frameData(const frameData& o)
		{
			// do not copy lists
			frameId = o.frameId;
		}

		~frameData()
		{
			depthImage.reset(nullptr);
		}

		uint64_t frameId;
		PoseMatrix4f pose;
		std::unique_ptr<unsigned short[]> depthImage;
	};

	struct OccupancyCell
	{
		int weight;
		std::map<int, int> height2pointCount;

		SP_STATUS Export(std::ostream& out)
		{
			out.write(reinterpret_cast<const char*>(&weight),
				std::streamsize(sizeof(weight)));

			int height2pointCountSize = static_cast<int>(height2pointCount.size());
			out.write(reinterpret_cast<const char*>(&height2pointCountSize),
				std::streamsize(sizeof(height2pointCountSize)));
			for (auto it = height2pointCount.begin(); it != height2pointCount.end(); it++)
			{
				out.write(reinterpret_cast<const char*>(&(it->first)),
					std::streamsize(sizeof(it->first)));
				out.write(reinterpret_cast<const char*>(&(it->second)),
					std::streamsize(sizeof(it->second)));
			}

			return SP_STATUS_SUCCESS;
		}

		SP_STATUS Import(std::istream& in)
		{
			in.read(reinterpret_cast<char*>(&weight),
				std::streamsize(sizeof(weight)));

			height2pointCount.clear();
			int height2pointCountSize;
			in.read(reinterpret_cast<char*>(&height2pointCountSize),
				std::streamsize(sizeof(height2pointCountSize)));
			for (int i = 0; i < height2pointCountSize; i++)
			{
				int first, second;
				in.read(reinterpret_cast<char*>(&first),
					std::streamsize(sizeof(first)));
				in.read(reinterpret_cast<char*>(&second),
					std::streamsize(sizeof(second)));
				height2pointCount[first] = second;
			}

			return SP_STATUS_SUCCESS;
		}
	};

	class SP_MapManager
	{
	public:
		SP_MapManager(const SP_MapManager&) = delete;
		SP_MapManager& operator=(const SP_MapManager&) = delete;
		SP_MapManager(SP_MapManager&&) = delete;
		SP_MapManager& operator=(SP_MapManager&&) = delete;
		SP_MapManager();
		~SP_MapManager();

		// resetting and configuring
		void reset();
		SP_STATUS configure(const SP_CameraIntrinsics &depthInreinsics, const PoseMatrix4f &depthToFisheye);

		// setters and getters
		SP_STATUS setMapHeightOfInterest(float minHeight, float maxHeight);
		SP_STATUS getMapHeightOfInterest(float* minHeight, float* maxHeight);
		SP_STATUS setDepthOfInterest(float minDepth, float maxDepth);
		SP_STATUS getDepthOfInterest(float* minDepth, float* maxDepth);
		SP_STATUS setMapResolution(float resolution);
		SP_STATUS getMapResolution(float* resolution);
		SP_STATUS getMapBounds(int* minX, int* minZ, int* maxX, int* maxZ);
		void setCellPointCountThreshold(unsigned int threshold);
		unsigned int getCellPointCountThreshold();
		void setPointInCellNoiseThreshold(unsigned int threshold);
		unsigned int getPointInCellNoiseThreshold();

		// managing the frame and feature lists
		void setOccupancyMapBuilding(bool mapBuildingOn);
		bool getOccupancyMapBuilding();
		void TestAndPushFrame(const unsigned short *depthPtr, const PoseMatrix4f& cameraPose);
		void PushFrame(const unsigned short *depthPtr, const PoseMatrix4f& pose_);
		void PopFrame();

		// saves class to a stream
		SP_STATUS ExportMap(const char* pFileName);

		// importas a class from a stream
		SP_STATUS ImportMap(const char* pFileName);

		// saves the image as a PPM file, camera trajectory not drawn by default
		SP_STATUS ExportMapAsPPM(const char* pFileName, int drawCameraTrajectory);

		// APIs to get and same the occupancy map
		SP_STATUS GetMap(int* maxSize, TileInfo* tileList, bool getHeight, bool getAll, const int* regionOfInterest);
		SP_STATUS getOccupancyMapAsGridMsg(SP_OccupancyGridMsg* occGrid);
		SP_STATUS SaveTrajectoryViewAsRGBA(unsigned char **arr, unsigned int *pWidth, unsigned int* pHeight,
			bool drawCameraTrajectory, bool drawOccMap);

		// wait for map building to complete
		void WaitForMapBuilding();

		// only exposed for the testing
		bool IsCellFullyVisible(const PoseMatrix4f& poseInv, float4& deltaXPt, float4& deltaYPt, float4& deltaZPt, const int3& cell);
		bool IsCellPartiallyVisible(const PoseMatrix4f& poseInv, const int3& cell);

	private:

		SP_CameraIntrinsics m_depthIntrinsics;
		PoseMatrix4f m_depthToFisheye;
		unsigned int m_frameRejections;
		unsigned int m_frameCounter;
		unsigned int m_framesPending;
		float2 m_heightInterestRange;
		float2 m_depthInterestRange;
		std::list<frameData*> m_frames;
		std::list<frameData*> m_framesSafe;
		bool m_mapBuilding;
		unsigned int m_pointInCellNoiseThreshold;
		unsigned int m_cellPointCountThreshold;

		// Managing the occupancy grid and trajectory
		float m_occupancyGridResolution;
		int m_occupancyGridBounds[4];
		std::map< std::pair<int, int>, OccupancyCell> m_occupancyGrid;
		std::map< std::pair<int, int>, OccupancyCell>::iterator m_occupancyGridIterator;
        std::mutex m_occupancyGridLock;
		std::set< std::pair<int, int>> m_occupancyGridChangeSet;
		std::set< std::pair<int, int>>::iterator m_occupancyGridChangeSetIterator;
		std::mutex m_changeSetMutex;
		std::vector<std::pair<int, int>> m_trajectory;
		std::vector<unsigned char> m_occupancyMapRGB;

		// main functionality
		unsigned int RunMapUpdate();
		bool ExecuteMapBuilding(int status);
		void BuildMap();

		// map building threading
		std::thread m_mapUpdateThread;
		std::mutex m_mapUpdateMutex;
		std::condition_variable m_mapUpdateConditionVariable;
		bool m_mapUpdateWorkDone;
		bool m_mapUpdateQuit;
		void StopMappingThread();
		void RestartMappingThread();
		std::mutex m_waitMutex;
		std::condition_variable m_waitConditionVariable;

		// helper functions
		void finalize();
		void ComputeMapSize(unsigned int &mapW, unsigned int &mapH, int &minX, int &minZ, bool withTrajectory);
	};
}

#endif //SP_MAPMANAGER_H
