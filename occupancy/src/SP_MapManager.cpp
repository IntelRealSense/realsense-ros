#include "SP_MapManager.h"
#include "SP_MappingTypes.h"
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include "SP_CommonVisionUtil.h"

using namespace std;
using namespace ScenePerception;

SP_MapManager::SP_MapManager() : m_depthToFisheye(PoseMatrix4f::Identity), m_pointInCellNoiseThreshold(5), m_cellPointCountThreshold(40)
{
	//m_imageManager = imageManager;
	memset(&m_depthIntrinsics, 0, sizeof(SP_CameraIntrinsics));

	m_heightInterestRange.x = -0.5f;
	m_heightInterestRange.y = 1.0f;
	m_occupancyGridResolution = 0.1f;

	m_depthInterestRange.x = 0.7f;
	m_depthInterestRange.y = 3.0f;

	m_occupancyGridBounds[0] = 0;
	m_occupancyGridBounds[1] = 0;
	m_occupancyGridBounds[2] = 0;
	m_occupancyGridBounds[3] = 0;

	m_framesPending = 0;
	m_mapBuilding = true;
}

SP_MapManager::~SP_MapManager()
{
	finalize();
}

SP_STATUS SP_MapManager::configure(const SP_CameraIntrinsics &depthInreinsics, const PoseMatrix4f &depthToFisheye)
{
	SP_memcpy(&m_depthIntrinsics, sizeof(SP_CameraIntrinsics), &depthInreinsics, sizeof(SP_CameraIntrinsics));
	m_depthToFisheye = depthToFisheye;
	reset();

	return SP_STATUS_SUCCESS;
}

void SP_MapManager::finalize()
{
	StopMappingThread();

	// who ever is waiting should leave
	m_mapUpdateWorkDone = true;
	m_waitConditionVariable.notify_all();

	// delete remaining frames
	{
		unique_lock<mutex> lk(m_mapUpdateMutex);
		while (!m_framesSafe.empty())
		{
			auto& frame = m_framesSafe.front();
			delete frame;
			m_framesSafe.pop_front();
		}
		m_framesPending = 0;
	}

	// delete remaining frames
	while (!m_frames.empty())
	{
		auto& frame = m_frames.front();
		delete frame;
		m_frames.pop_front();
	}
}

void SP_MapManager::reset()
{
	finalize();
    m_occupancyGridLock.lock();

	m_frameCounter = 0;
	m_frameRejections = maxFrameRejections;
	m_occupancyGridBounds[0] = 0;
	m_occupancyGridBounds[1] = 0;
	m_occupancyGridBounds[2] = 0;
	m_occupancyGridBounds[3] = 0;
	m_occupancyGrid.clear();
	m_occupancyGridChangeSet.clear();
	m_occupancyGridIterator = m_occupancyGrid.begin();
	m_occupancyGridChangeSetIterator = m_occupancyGridChangeSet.begin();
	m_trajectory.clear();
	m_mapUpdateWorkDone = true;

    m_occupancyGridLock.unlock();
	RestartMappingThread();
}

void SP_MapManager::setOccupancyMapBuilding(bool mapBuildingOn)
{
	m_mapBuilding = mapBuildingOn;
}

bool SP_MapManager::getOccupancyMapBuilding()
{
	return m_mapBuilding;
}

void SP_MapManager::TestAndPushFrame(const unsigned short *depthPtr, const PoseMatrix4f& cameraPose)
{
	// if no depth leave and check mapping is enabled
	if (!depthPtr || !m_mapBuilding)
	{
		return;
	}

	// check skipping some frames
	if (m_frameRejections >= maxFrameRejections
		&& m_framesPending < 3 * mapUpdateProblemSize)
	{
		const PoseMatrix4f& pose_ = cameraPose * m_depthToFisheye;
		PushFrame(depthPtr, pose_);
		m_frameRejections = 0;
	}
	else
	{
		m_frameRejections++;
	}
}

void SP_MapManager::PushFrame(const unsigned short *depthPtr, const PoseMatrix4f& pose_)
{
	const int depthSize = m_depthIntrinsics.imageWidth * m_depthIntrinsics.imageHeight;
	frameData* newFrame = nullptr;

	// input validation
	if (nullptr == depthPtr || 0 == PoseMatrix4f::validatePoseMatrix(pose_))
	{
		return;
	}

	// new frame and depth copy
	try
	{
		newFrame = new frameData();
		newFrame->depthImage.reset(new unsigned short[depthSize]);
		SP_memcpy(newFrame->depthImage.get(), depthSize * sizeof(unsigned short), depthPtr, depthSize * sizeof(unsigned short));
	}
	catch (...)
	{
		return;
	}

	// populate frame data
	newFrame->frameId = m_frameCounter++;
	newFrame->pose = pose_;

	// add frame to safe list
	{
		unique_lock<mutex> lk(m_mapUpdateMutex);
		m_framesSafe.push_back(newFrame);
		m_framesPending++;
	}

	// launch the thread
	if (m_mapUpdateThread.joinable())
	{
		m_mapUpdateWorkDone = false;
		m_mapUpdateConditionVariable.notify_all();
	}
}

void SP_MapManager::PopFrame()
{
	if (m_frames.size() > 0)
	{
		auto& frame = m_frames.front();
		delete frame;
		m_frames.pop_front();
	}
}

void SP_MapManager::StopMappingThread()
{
	{
		// wait for work in hand to complete
		unique_lock<mutex> lk(m_mapUpdateMutex);

		// set the exit flag and notify
		m_mapUpdateQuit = true;
		m_mapUpdateWorkDone = false;
	}

	// notify thread of exit
	m_mapUpdateConditionVariable.notify_all();

	// wait for exit
	if (m_mapUpdateThread.joinable())
	{
		m_mapUpdateThread.join();
	}
}

void SP_MapManager::RestartMappingThread()
{
	// re-start mapping thread
	m_mapUpdateQuit = false;
	if (!m_mapUpdateThread.joinable())
	{
		m_mapUpdateThread = thread(&SP_MapManager::ExecuteMapBuilding, this, 0);
	}
}

bool SP_MapManager::IsCellFullyVisible(const PoseMatrix4f& poseInv, float4& deltaXPt, float4& deltaYPt, float4& deltaZPt, const int3& cell)
{
	// find the corner points
	float4 basePt(m_occupancyGridResolution * cell.x, m_occupancyGridResolution * cell.y, m_occupancyGridResolution * cell.z, 1.0f);
	basePt = poseInv * basePt;

	for (int k = 0; k < 8; k++)
	{
		float4 pt = basePt;
		if (k & 1)
		{
			pt += deltaXPt;
		}
		if ((k >> 1) & 1)
		{
			pt += deltaYPt;
		}
		if (k >> 2)
		{
			pt += deltaZPt;
		}

		if (pt.z < m_depthInterestRange.x || pt.z > m_depthInterestRange.y)
		{
			return false;
		}
		float px = m_depthIntrinsics.focalLengthHorizontal * pt.x / pt.z + m_depthIntrinsics.principalPointCoordU;
		if (px < 0 || px >= float(m_depthIntrinsics.imageWidth - 1))
		{
			return false;
		}
		float py = m_depthIntrinsics.focalLengthVertical * pt.y / pt.z + m_depthIntrinsics.principalPointCoordV;
		if (py < 0 || py >= float(m_depthIntrinsics.imageHeight - 1))
		{
			return false;
		}
	}

	return true;
}

bool SP_MapManager::IsCellPartiallyVisible(const PoseMatrix4f& poseInv, const int3& cell)
{
	//const SP_CameraIntrinsics* outIntrinsics = m_imageManager->getDepthIntrinsics();

	// find the corner points
	for (int k = 0; k < 8; k++)
	{
		float3 pt(m_occupancyGridResolution * (cell.x + (k & 1)),
			m_occupancyGridResolution * (cell.y + ((k >> 1) & 1)),
			m_occupancyGridResolution * (cell.z + ((k >> 2) & 1)));
		pt = poseInv * pt;
		if (pt.z < m_depthInterestRange.x || pt.z > m_depthInterestRange.y)
		{
			continue;
		}
		float px = m_depthIntrinsics.focalLengthHorizontal * pt.x / pt.z + m_depthIntrinsics.principalPointCoordU;
		if (px < 0 || px >= float(m_depthIntrinsics.imageWidth - 1))
		{
			continue;
		}
		float py = m_depthIntrinsics.focalLengthVertical * pt.y / pt.z + m_depthIntrinsics.principalPointCoordV;
		if (py < 0 || py >= float(m_depthIntrinsics.imageHeight - 1))
		{
			continue;
		}
		return true;
	}

	return false;
}

unsigned int SP_MapManager::RunMapUpdate()
{
	std::set< std::pair<int, int> > modifiedSet;
	float occupancyGridResolutionInverse = 1.0f / m_occupancyGridResolution;

	unsigned int frame_count = 0;
	for (auto it = m_frames.begin(); it != m_frames.end() && frame_count < mapUpdateProblemSize; it++, frame_count++)
	{
		auto& frame = (*it);
		auto poseInv = frame->pose.Inverse();

		// compute the box of possibly visible occupancy cells
		int3 minR(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
		int3 maxR(std::numeric_limits<int>::min(), std::numeric_limits<int>::min(), std::numeric_limits<int>::min());
		for (int j = 0; j < 2; j++)
		{
			for (int i = 0; i < 4; i++)
			{
				int x = (i & 1) * (m_depthIntrinsics.imageWidth - 1);
				int y = ((i >> 1) & 1) * (m_depthIntrinsics.imageHeight - 1);
				const float fDepth = m_depthInterestRange.x + j * (m_depthInterestRange.y - m_depthInterestRange.x);
				const float3 camearaCorner((x - m_depthIntrinsics.principalPointCoordU) * fDepth / m_depthIntrinsics.focalLengthHorizontal,
					(y - m_depthIntrinsics.principalPointCoordV) * fDepth / m_depthIntrinsics.focalLengthVertical,
					fDepth);
				float3 corner = frame->pose * camearaCorner;
				corner *= occupancyGridResolutionInverse;

				minR.x = std::min(minR.x, int(floor(corner.x)));
				minR.y = std::min(minR.y, int(floor(corner.y)));
				minR.z = std::min(minR.z, int(floor(corner.z)));
				maxR.x = std::max(maxR.x, int(ceil(corner.x)));
				maxR.y = std::max(maxR.y, int(ceil(corner.y)));
				maxR.z = std::max(maxR.z, int(ceil(corner.z)));
			}
		}
		uint3 visibleRange((maxR.x - minR.x + 1), (maxR.y - minR.y + 1), (maxR.z - minR.z + 1));

		// allocate grids to keep track of visible cells
		unsigned int gridSize = visibleRange.x * visibleRange.y * visibleRange.z;
		std::unique_ptr<unsigned char[]> cellCount;
		std::unique_ptr<unsigned char[]> tileCount;
		try
		{
			cellCount.reset(new unsigned char[gridSize]);
			tileCount.reset(new unsigned char[visibleRange.x * visibleRange.z]);
		}
		catch (...)
		{
			break;
		}
		unsigned char* cellCountPtr = cellCount.get();
		memset(cellCountPtr, 0, gridSize);
		unsigned char* tileCountPtr = tileCount.get();
		memset(tileCountPtr, 0, visibleRange.x * visibleRange.z);

		// loop on image updating count
		int idx = 0;
		unsigned short* ptr = frame->depthImage.get();
		for (unsigned int y = 0; y < m_depthIntrinsics.imageHeight; ++y)
		{
			for (unsigned int x = 0; x < m_depthIntrinsics.imageWidth; ++x, idx++)
			{
				const float d = 0.001f * (float)ptr[idx];
				if ((d == 0.0f) || d < m_depthInterestRange.x || d > m_depthInterestRange.y)
				{
					continue;
				}

				// get point in world coordinates
				float3 pt((x - m_depthIntrinsics.principalPointCoordU) * d / m_depthIntrinsics.focalLengthHorizontal,
					(y - m_depthIntrinsics.principalPointCoordV) * d / m_depthIntrinsics.focalLengthVertical, d);
				pt = frame->pose * pt;

				// check height of interest (height points up oposite to y axis direction)
				if ((pt.y - frame->pose.m_data[7]) > m_heightInterestRange.y || (pt.y - frame->pose.m_data[7]) < m_heightInterestRange.x)
				{
					continue;
				}

				// get tile location and update occupancy grid
				int3 tile(int(floor(pt.x * occupancyGridResolutionInverse)),
					int(floor(pt.y * occupancyGridResolutionInverse)),
					int(floor(pt.z * occupancyGridResolutionInverse)));

				unsigned char* ref = &cellCountPtr[(tile.y - minR.y) + visibleRange.y * ((tile.z - minR.z) + visibleRange.z * (tile.x - minR.x))];
				*ref = *ref >= 200 ? 200 : *ref + 1;
				tileCountPtr[(tile.z - minR.z) + visibleRange.z * (tile.x - minR.x)] |= 1;
			}
		}

		// save pose to trajectory
        m_occupancyGridLock.lock();
		m_trajectory.push_back(std::make_pair((int)(frame->pose.m_data[3] / m_occupancyGridResolution), (int)(frame->pose.m_data[11] / m_occupancyGridResolution)));

		// now loop on the entire grid
		float4 deltaXPt(m_occupancyGridResolution, 0.0f, 0.0f, 0.0f);
		deltaXPt = poseInv * deltaXPt;
		float4 deltaYPt(0.0f, m_occupancyGridResolution, 0.0f, 0.0f);
		deltaYPt = poseInv * deltaYPt;
		float4 deltaZPt(0.0f, 0.0f, m_occupancyGridResolution, 0.0f);
		deltaZPt = poseInv * deltaZPt;
		unsigned char* cellCountRef = cellCountPtr;
		unsigned char* tileCountRef = tileCountPtr;
		for (int x = minR.x; x <= maxR.x; x++)
		{
			for (int z = minR.z; z <= maxR.z; z++, tileCountRef++)
			{
				auto it = m_occupancyGrid.find(std::make_pair(x, z));

				// tile has no data
				if (*tileCountRef == 0 && it == m_occupancyGrid.end())
				{
					cellCountRef += visibleRange.y;

					// check it is not missing or occluded
					int y = (int)(frame->pose.m_data[7] * occupancyGridResolutionInverse);
					if (y < minR.y || y > maxR.y)
					{
						continue;
					}
					float3 pt((float)x + 0.5f, (float)y + 0.5f, (float)z + 0.5f);
					pt = m_occupancyGridResolution * pt;
					pt = poseInv * pt;
					float3 pix = pointToImgPos(&m_depthIntrinsics, pt);
					if (pix.x < 0.0f || pix.y < 0.0f)
					{
						continue;
					}
					unsigned int px = (unsigned int)(pix.x);
					unsigned int py = (unsigned int)(pix.y);
					if (px >= m_depthIntrinsics.imageWidth || py >= m_depthIntrinsics.imageHeight)
					{
						continue;
					}
					float d = 0.001f * (float)ptr[px + m_depthIntrinsics.imageWidth * py];
					if (d == 0.0f || d < m_depthInterestRange.x || d > m_depthInterestRange.y || d < pt.z + 0.5f * m_occupancyGridResolution)
					{
						continue;
					}

					// check visibility
					if (!IsCellFullyVisible(poseInv, deltaXPt, deltaYPt, deltaZPt, int3(x, y, z)))
					{
						continue;
					}

					m_occupancyGrid[std::make_pair(x, z)].weight = 0;
					m_occupancyGridBounds[0] = std::min(m_occupancyGridBounds[0], x);
					m_occupancyGridBounds[1] = std::max(m_occupancyGridBounds[1], x);
					m_occupancyGridBounds[2] = std::min(m_occupancyGridBounds[2], z);
					m_occupancyGridBounds[3] = std::max(m_occupancyGridBounds[3], z);
					modifiedSet.insert(std::make_pair(x, z));
				}
				else // tile has data
				{
					for (int y = minR.y; y <= maxR.y; y++, cellCountRef++)
					{
						// (1) image points are in the cell
						if (*cellCountRef > m_pointInCellNoiseThreshold)
						{
							// if cell not fully visible ignore it
							if (!IsCellFullyVisible(poseInv, deltaXPt, deltaYPt, deltaZPt, int3(x, y, z)))
							{
								continue;
							}

							// is tile new?
							if (it == m_occupancyGrid.end())
							{
								OccupancyCell occCell;
								occCell.weight = 0;
								it = m_occupancyGrid.insert(it, std::pair<std::pair<int, int>, OccupancyCell>(std::make_pair(x, z), occCell));
								m_occupancyGridBounds[0] = std::min(m_occupancyGridBounds[0], x);
								m_occupancyGridBounds[1] = std::max(m_occupancyGridBounds[1], x);
								m_occupancyGridBounds[2] = std::min(m_occupancyGridBounds[2], z);
								m_occupancyGridBounds[3] = std::max(m_occupancyGridBounds[3], z);
							}

							// modify cell
							it->second.height2pointCount[y] += *cellCountRef;
							modifiedSet.insert(std::make_pair(x, z));
						}
						// (2) cell has values and needs down graded
						else if (it != m_occupancyGrid.end() && it->second.height2pointCount.find(y) != it->second.height2pointCount.end())
						{
							// check it is not missing or occluded
							float3 pt((float)x + 0.5f, (float)y + 0.5f, (float)z + 0.5f);
							pt = m_occupancyGridResolution * pt;
							pt = poseInv * pt;
							float3 pix = pointToImgPos(&m_depthIntrinsics, pt);
							if (pix.x < 0.0f || pix.y < 0.0f)
							{
								continue;
							}
							unsigned int px = (unsigned int)(pix.x);
							unsigned int py = (unsigned int)(pix.y);
							if (px >= m_depthIntrinsics.imageWidth || py >= m_depthIntrinsics.imageHeight)
							{
								continue;
							}
							float d = 0.001f * (float)ptr[px + m_depthIntrinsics.imageWidth * py];
							if (d != 0.0f && d < pt.z + 0.5f * m_occupancyGridResolution)
							{
								continue;
							}

							// if cell not fully visible ignore it
							if (!IsCellFullyVisible(poseInv, deltaXPt, deltaYPt, deltaZPt, int3(x, y, z)))
							{
								continue;
							}

							// ok have to decrease weight
							it->second.height2pointCount[y] = (int)(it->second.height2pointCount[y] * 0.75f);
							modifiedSet.insert(std::make_pair(x, z));
						}
					}
				}
			}
		}
        m_occupancyGridLock.unlock();
	}

	{
        m_occupancyGridLock.lock();
		std::lock_guard<std::mutex> guard(m_changeSetMutex);

		// loop on the modified list updating weights
		for (auto it = modifiedSet.begin(); it != modifiedSet.end(); it++)
		{
			auto oit = m_occupancyGrid.find(*it);
			if (oit == m_occupancyGrid.end())
			{
				continue;
			}
			int total = 0;
			auto& oc = oit->second;
			for (auto bit = oc.height2pointCount.begin(); bit != oc.height2pointCount.end(); bit++)
			{
				total += bit->second;
			}
			auto new_weight = total > 200 ? 100 : total / 2;
			if (new_weight != oc.weight || new_weight != 100)
			{
				oc.weight = new_weight;
				m_occupancyGridChangeSet.insert(oit->first);
			}
		}

        m_occupancyGridLock.unlock();
	}

	return frame_count;
}

void SP_MapManager::BuildMap()
{
	// run map update
	unsigned int processedCount = RunMapUpdate();

	// delete frames processed
	while (processedCount > 0)
	{
		PopFrame();
		processedCount--;
	}
}

bool SP_MapManager::ExecuteMapBuilding(int status)
{
	while (!m_mapUpdateQuit)
	{
		// wait for the mapping request
		{
			unique_lock<mutex> lk(m_mapUpdateMutex);
			m_mapUpdateConditionVariable.wait(lk, [&] {return m_mapUpdateWorkDone == false; });
			// copy new data
			m_frames.insert(m_frames.end(), m_framesSafe.begin(), m_framesSafe.end());
			m_framesSafe.clear();
			m_framesPending = (unsigned int)m_frames.size();

			// indicate work is done
			if (m_frames.empty())
			{
				m_mapUpdateWorkDone = true;
				m_waitConditionVariable.notify_all();
			}
		}

		// build the map
		BuildMap();
	}

	return true;
}

void ScenePerception::SP_MapManager::WaitForMapBuilding()
{
	// wait for the meshing request
	unique_lock<mutex> lk(m_waitMutex);
	m_waitConditionVariable.wait(lk, [&] {return m_mapUpdateWorkDone == true; });
}

SP_STATUS SP_MapManager::GetMap(int* maxSize, TileInfo* tileList, bool getHeight, bool getAll, const int* regionOfInterest)
{
	if (NULL == maxSize || NULL == tileList)
	{
		return SP_STATUS_INVALIDARG;
	}

	bool overflow = false;
	int maxCount = *maxSize;
	*maxSize = 0;

    m_occupancyGridLock.lock();
	if (getAll == false)
	{
		std::lock_guard<std::mutex> guard(m_changeSetMutex);

		// restart iterator
		m_occupancyGridChangeSetIterator = m_occupancyGridChangeSet.begin();

		// fill list
		while (m_occupancyGridChangeSetIterator != m_occupancyGridChangeSet.end() && *maxSize < maxCount)
		{
			auto oit = m_occupancyGrid.find(*m_occupancyGridChangeSetIterator);
			if (oit == m_occupancyGrid.end())
			{
				continue;
			}
			int x = std::get<0>(oit->first);
			int z = std::get<1>(oit->first);
			if (NULL != regionOfInterest && (x < regionOfInterest[0] || x > regionOfInterest[1] || z < regionOfInterest[2] || z > regionOfInterest[3]))
			{
				m_occupancyGridChangeSetIterator++;
				continue;
			}
			tileList[(*maxSize)].x = x;
			tileList[(*maxSize)].z = z;
			tileList[(*maxSize)].occupancy = oit->second.weight;
			tileList[(*maxSize)].hasHeight = 0;
			if (getHeight)
			{
				auto bestit = oit->second.height2pointCount.end();
				for (auto cellit = oit->second.height2pointCount.begin(); cellit != oit->second.height2pointCount.end(); cellit++)
				{
					if (cellit->second > m_cellPointCountThreshold)
					{
						bestit = cellit;
						break;
					}
				}
				if (bestit != oit->second.height2pointCount.end())
				{
					tileList[(*maxSize)].height = bestit->first;
					tileList[(*maxSize)].hasHeight = 1;
				}
			}			
			(*maxSize)++;

			m_occupancyGridChangeSetIterator = m_occupancyGridChangeSet.erase(m_occupancyGridChangeSetIterator);
		}

		// check overflow
		if (m_occupancyGridChangeSetIterator != m_occupancyGridChangeSet.end())
		{
			overflow = true;
		}
		else
		{
			m_occupancyGridChangeSet.clear();
		}
	}
	else
	{
		// restart iterator
		if (m_occupancyGridIterator == m_occupancyGrid.end())
		{
			m_occupancyGridIterator = m_occupancyGrid.begin();
		}

		// fill list
		while (m_occupancyGridIterator != m_occupancyGrid.end() && *maxSize < maxCount)
		{
			int x = std::get<0>(m_occupancyGridIterator->first);
			int z = std::get<1>(m_occupancyGridIterator->first);
			if (NULL != regionOfInterest && (x < regionOfInterest[0] || x > regionOfInterest[1] || z < regionOfInterest[2] || z > regionOfInterest[3]))
			{
				m_occupancyGridIterator++;
				continue;
			}
			tileList[(*maxSize)].x = x;
			tileList[(*maxSize)].z = z;
			tileList[(*maxSize)].occupancy = m_occupancyGridIterator->second.weight;
			tileList[(*maxSize)].hasHeight = 0;
			if (getHeight)
			{
				auto bestit = m_occupancyGridIterator->second.height2pointCount.end();
				for (auto cellit = m_occupancyGridIterator->second.height2pointCount.begin(); cellit != m_occupancyGridIterator->second.height2pointCount.end(); cellit++)
				{
					if (cellit->second > m_cellPointCountThreshold)
					{
						bestit = cellit;
						break;
					}
				}
				if (bestit != m_occupancyGridIterator->second.height2pointCount.end())
				{
					tileList[(*maxSize)].height = bestit->first;
					tileList[(*maxSize)].hasHeight = 1;
				}
			}
			(*maxSize)++;

			m_occupancyGridIterator++;
		}

		// check overflow
		if (m_occupancyGridIterator != m_occupancyGrid.end())
		{
			overflow = true;
		}
	}
    m_occupancyGridLock.unlock();

	return (overflow) ? SP_STATUS_WARNING : SP_STATUS_SUCCESS;
}

SP_STATUS SP_MapManager::setMapHeightOfInterest(float minHeight, float maxHeight)
{
	if (minHeight >= maxHeight)
	{
		return SP_STATUS_INVALIDARG;
	}
	m_heightInterestRange.x = minHeight;
	m_heightInterestRange.y = maxHeight;
	return SP_STATUS_SUCCESS;
}

SP_STATUS SP_MapManager::getMapHeightOfInterest(float* minHeight, float* maxHeight)
{
	if (minHeight == NULL || maxHeight == NULL)
	{
		return SP_STATUS_INVALIDARG;
	}
	*minHeight = m_heightInterestRange.x;
	*maxHeight = m_heightInterestRange.y;
	return SP_STATUS_SUCCESS;
}

SP_STATUS SP_MapManager::setDepthOfInterest(float minDepth, float maxDepth)
{
	if (minDepth >= maxDepth)
	{
		return SP_STATUS_INVALIDARG;
	}
	m_depthInterestRange.x = minDepth;
	m_depthInterestRange.y = maxDepth;
	return SP_STATUS_SUCCESS;
}

SP_STATUS SP_MapManager::getDepthOfInterest(float* minDepth, float* maxDepth)
{
	if (minDepth == NULL || maxDepth == NULL)
	{
		return SP_STATUS_INVALIDARG;
	}
	*minDepth = m_depthInterestRange.x;
	*maxDepth = m_depthInterestRange.y;
	return SP_STATUS_SUCCESS;
}

SP_STATUS SP_MapManager::setMapResolution(float resolution)
{
	if (resolution <= 0)
	{
		return SP_STATUS_INVALIDARG;
	}
	m_occupancyGridResolution = resolution;
	return SP_STATUS_SUCCESS;
}

SP_STATUS SP_MapManager::getMapResolution(float* resolution)
{
	if (resolution == NULL)
	{
		return SP_STATUS_INVALIDARG;
	}
	*resolution = m_occupancyGridResolution;
	return SP_STATUS_SUCCESS;
}

SP_STATUS SP_MapManager::getMapBounds(int* minX, int* minZ, int* maxX, int* maxZ)
{
	if (NULL == minX && NULL == minZ && NULL == maxX && NULL == maxZ)
	{
		return SP_STATUS_INVALIDARG;
	}

	if (NULL != minX)
	{
		*minX = m_occupancyGridBounds[0];
	}

	if (NULL != minZ)
	{
		*minZ = m_occupancyGridBounds[2];
	}

	if (NULL != maxX)
	{
		*maxX = m_occupancyGridBounds[1];
	}

	if (NULL != maxZ)
	{
		*maxZ = m_occupancyGridBounds[3];
	}
	return SP_STATUS_SUCCESS;
}

void SP_MapManager::setCellPointCountThreshold(unsigned int threshold)
{
	m_cellPointCountThreshold = threshold;
}

unsigned int SP_MapManager::getCellPointCountThreshold()
{
	return m_cellPointCountThreshold;
}

void SP_MapManager::setPointInCellNoiseThreshold(unsigned int threshold)
{
	m_pointInCellNoiseThreshold = threshold;
}

unsigned int SP_MapManager::getPointInCellNoiseThreshold()
{
	return m_pointInCellNoiseThreshold;
}

SP_STATUS SP_MapManager::getOccupancyMapAsGridMsg(SP_OccupancyGridMsg* occGrid)
{
	if (NULL == occGrid)
	{
		return SP_STATUS_INVALIDARG;
	}

	if (occGrid->data == NULL)  // memory has not been allocated for storing the occupancy data
	{
		return SP_STATUS_INVALIDARG;
	}

    m_occupancyGridLock.lock();

	// get the size of the map
	unsigned int width, height;
	int minX = 0, minZ = 0;
	ComputeMapSize(width, height, minX, minZ, false);

	// check memory allocated
	if (occGrid->mapMetaData.width < width || occGrid->mapMetaData.height < height)
	{
		occGrid->mapMetaData.width = width;
		occGrid->mapMetaData.height = height;

        m_occupancyGridLock.unlock();
		return  SP_STATUS_WARNING;  // overflown, allocate a bigger buffer
	}

	// write Image MetaData
	occGrid->mapMetaData.width = width;
	occGrid->mapMetaData.height = height;
	occGrid->mapMetaData.resolution = m_occupancyGridResolution;

	// start point
	memset(occGrid->mapMetaData.offset, 0, 3 * sizeof(float));
	occGrid->mapMetaData.offset[0] = (float)minX * m_occupancyGridResolution;
	occGrid->mapMetaData.offset[1] = (float)minZ * m_occupancyGridResolution;

	// write image pixels
	memset(occGrid->data, -1, width * height);
	for (auto it = m_occupancyGrid.begin(); it != m_occupancyGrid.end(); it++)
	{
		int x = std::get<0>(it->first) - minX;
		int z = std::get<1>(it->first) - minZ;
		int arrIdx = (x + z * width);

		occGrid->data[arrIdx] = (char)(it->second.weight);
	}

    m_occupancyGridLock.unlock();

	return SP_STATUS_SUCCESS;
}

SP_STATUS SP_MapManager::ExportMapAsPPM(const char* pFileName, int drawCameraTrajectory)
{
	if (pFileName == NULL)
	{
		return SP_STATUS_INVALIDARG;
	}
	std::ofstream out(pFileName, std::ios::binary);
	if (!out.is_open())
	{
		return SP_STATUS_INVALIDARG;
	}
	unsigned char *inputArr = nullptr;
	unsigned int width = 0, height = 0;
	SP_STATUS saveStatus = SaveTrajectoryViewAsRGBA(&inputArr, &width, &height, (drawCameraTrajectory == 1), true);
	if (saveStatus == SP_STATUS_SUCCESS)
	{
		// write the image to file
		out << "P6" << std::endl;                 // P5 is pgm, P6 is ppm
		out << width << " " << height << std::endl;  // Height and Width of the image
		out << 255 << std::endl;
		for (unsigned int idx = 0; idx < width * height; idx++)
		{
			out << inputArr[idx * 4] << inputArr[idx * 4 + 1] << inputArr[idx * 4 + 2];
		}
		if (!out.good())
		{
			saveStatus = SP_STATUS_ERROR;
		}
		out.flush();
		out.close();
		if (out.fail())
		{
			saveStatus = SP_STATUS_ERROR;
		}
	}
	return saveStatus;
}

SP_STATUS SP_MapManager::ExportMap(const char* pFileName)
{
	if (pFileName == NULL)
	{
		return SP_STATUS_INVALIDARG;
	}
	std::ofstream out(pFileName, std::ios::binary);
	if (!out.is_open())
	{
		return SP_STATUS_INVALIDARG;
	}

	// stop the current running thread
	StopMappingThread();
    m_occupancyGridLock.lock();

	// write the version
	float version = 0.01f;
	out.write(reinterpret_cast<const char*>(&version),
		std::streamsize(sizeof(version)));

	// height and resolution
	out.write(reinterpret_cast<const char*>(&m_occupancyGridResolution),
		std::streamsize(sizeof(m_occupancyGridResolution)));
	out.write(reinterpret_cast<const char*>(&m_heightInterestRange),
		std::streamsize(sizeof(m_heightInterestRange)));

	// write the occupancy grid bounds
	out.write(reinterpret_cast<const char*>(m_occupancyGridBounds),
		std::streamsize(sizeof(m_occupancyGridBounds)));

	// write the occupancy grid data
	int occupancyGridSize = static_cast<int>(m_occupancyGrid.size());
	out.write(reinterpret_cast<const char*>(&occupancyGridSize),
		std::streamsize(sizeof(occupancyGridSize)));
	for (auto it = m_occupancyGrid.begin(); it != m_occupancyGrid.end(); it++)
	{
		int x = std::get<0>(it->first);
		int z = std::get<1>(it->first);
		out.write(reinterpret_cast<const char*>(&x),
			std::streamsize(sizeof(x)));
		out.write(reinterpret_cast<const char*>(&z),
			std::streamsize(sizeof(z)));
		it->second.Export(out);
	}

	// restart the thread for map building
    m_occupancyGridLock.unlock();
	RestartMappingThread();

	out.flush();
	out.close();
	if (out.fail())
	{
		return SP_STATUS_ERROR;
	}
	return SP_STATUS_SUCCESS;
}

SP_STATUS SP_MapManager::ImportMap(const char* pFileName)
{
	if (pFileName == NULL)
	{
		return SP_STATUS_INVALIDARG;
	}

	std::ifstream in(pFileName, std::ios::binary);
	if (!in.is_open())
	{
		return SP_STATUS_INVALIDARG;
	}
	// stop the current running thread
	StopMappingThread();
    m_occupancyGridLock.lock();

	// write the version
	float version;
	in.read(reinterpret_cast<char*>(&version),
		std::streamsize(sizeof(version)));
	if (version != 0.01f)
	{
		// restart the thread for map building
        m_occupancyGridLock.unlock();
		RestartMappingThread();
		return SP_STATUS_ERROR;
	}

	// initialize some structures
	m_frameCounter = 0;
	m_frameRejections = maxFrameRejections;
	m_occupancyGridBounds[0] = 0;
	m_occupancyGridBounds[1] = 0;
	m_occupancyGridBounds[2] = 0;
	m_occupancyGridBounds[3] = 0;
	m_occupancyGrid.clear();
	m_occupancyGridChangeSet.clear();
	m_occupancyGridIterator = m_occupancyGrid.begin();
	m_occupancyGridChangeSetIterator = m_occupancyGridChangeSet.begin();
	m_frames.clear();

	// height and resolution
	in.read(reinterpret_cast<char*>(&m_occupancyGridResolution),
		std::streamsize(sizeof(m_occupancyGridResolution)));
	in.read(reinterpret_cast<char*>(&m_heightInterestRange),
		std::streamsize(sizeof(m_heightInterestRange)));

	// write the occupancy grid bounds
	in.read(reinterpret_cast<char*>(m_occupancyGridBounds),
		std::streamsize(sizeof(m_occupancyGridBounds)));

	// write the occupancy grid data
	int occupancyGridSize;
	in.read(reinterpret_cast<char*>(&occupancyGridSize),
		std::streamsize(sizeof(occupancyGridSize)));
	for (int i = 0; i < occupancyGridSize; i++)
	{
		int x, z;
		in.read(reinterpret_cast<char*>(&x),
			std::streamsize(sizeof(x)));
		in.read(reinterpret_cast<char*>(&z),
			std::streamsize(sizeof(z)));

		m_occupancyGrid[std::make_pair(x, z)].weight = 1;
		m_occupancyGrid[std::make_pair(x, z)].Import(in);

		m_occupancyGridChangeSet.insert(std::make_pair(x, z));
	}

	// restart the thread for map building
    m_occupancyGridLock.unlock();
	RestartMappingThread();
	in.close();
	if (in.fail())
	{
		return SP_STATUS_ERROR;
	}
	return SP_STATUS_SUCCESS;
}

SP_STATUS SP_MapManager::SaveTrajectoryViewAsRGBA(unsigned char **inputArr, unsigned int *pWidth, unsigned int *pHeight,
	bool drawCameraTrajectory, bool drawOccMap)
{
	if (inputArr == NULL || pWidth == NULL || pHeight == NULL)
	{
		return SP_STATUS_INVALIDARG;
	}

	// acquire read lock
    m_occupancyGridLock.lock();

	// get the size of the map
	int minX = 0, minZ = 0;
	ComputeMapSize(*pWidth, *pHeight, minX, minZ, true);

	// allocate size
	size_t imgByteSize = (*pWidth * *pHeight) << 2;
	if (imgByteSize > m_occupancyMapRGB.size())
	{
		try
		{
			m_occupancyMapRGB.resize(imgByteSize);
		}
		catch (...)
		{
			// unlock occupancy map
            m_occupancyGridLock.unlock();

			return SP_STATUS_ERROR;
		}
	}
	memset(m_occupancyMapRGB.data(), 200, imgByteSize);
	unsigned char *arr = m_occupancyMapRGB.data();

	// loop on the occupancy map
	if (drawOccMap)
	{
		for (auto it = m_occupancyGrid.begin(); it != m_occupancyGrid.end(); it++)
		{
			int x = std::get<0>(it->first) - minX;
			int z = *pHeight - 1 - (std::get<1>(it->first) - minZ);
			int arrIdx = (x + z * *pWidth) << 2;

			char v = (char)(it->second.weight * 2.55f);
			arr[arrIdx++] = (unsigned char)(255);
			arr[arrIdx++] = (unsigned char)(255 - v);
			arr[arrIdx++] = (unsigned char)(255 - v);
			arr[arrIdx++] = (unsigned char)(255);
		}
	}

	// loop on the trajectory
	if (drawCameraTrajectory)
	{
		const unsigned char POSE_ALPHA_CHANNEL = 254; //use alpha channel value to identify if the pixel is pose drawing
		float ratioUdpate = 1 / (float)(m_trajectory.size());
		float ratio = 0.0f;
		for (auto it = m_trajectory.begin(); it != m_trajectory.end(); it++, ratio += ratioUdpate)
		{
			int x = std::get<0>(*it) - minX;
			int z = *pHeight - 1 - (std::get<1>(*it) - minZ);
			unsigned char r = 64;
			unsigned char g = (unsigned char)(180 * std::min((1 - ratio) / 0.75f, 1.0f));
			unsigned char b = (unsigned char)(180 * std::max((0.75f - ratio) / 0.75f, 0.0f));
			int arrIdx = (x + z * (*pWidth)) << 2;
			arr[arrIdx++] = r;
			arr[arrIdx++] = g;
			arr[arrIdx++] = b;
			arr[arrIdx++] = POSE_ALPHA_CHANNEL;
		}
	}

	// returned array
	*inputArr = m_occupancyMapRGB.data();

	// unlock occupancy map
    m_occupancyGridLock.unlock();

	return SP_STATUS_SUCCESS;
}

void SP_MapManager::ComputeMapSize(unsigned int &mapW, unsigned int &mapH, int &minX, int &minZ, bool withTrajectory)
{
	// Compute the boundary of the occupancyMap
	minX = m_occupancyGridBounds[0];
	int maxX = m_occupancyGridBounds[1];
	minZ = m_occupancyGridBounds[2];
	int maxZ = m_occupancyGridBounds[3];

	if (withTrajectory)
	{
		// sort trajectory on image traversal order
		for (unsigned int i = 0; i < m_trajectory.size(); i++)
		{
			int x = std::get<0>(m_trajectory[i]);
			int z = std::get<1>(m_trajectory[i]);
			maxX = std::max(maxX, x);
			minX = std::min(minX, x);
			maxZ = std::max(maxZ, z);
			minZ = std::min(minZ, z);
		}
	}

	mapW = maxX - minX + 1;
	mapH = maxZ - minZ + 1;
}
