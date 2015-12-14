#pragma once

#include <ros/ros.h>
#include "lookup/LookupInterface.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"
#include "argus_utils/PoseSE3.h"
#include "argus_utils/WorkerPool.h"
#include "argus_utils/SynchronizationUtils.h"

#include "camera_array/SystemStates.h"

#include <unordered_map>
#include <memory>

namespace camera_array
{

// TODO Documentation
/*! \brief The base camera array manager class. Contains basic functionality for
 * registering member cameras and managing streaming. Broadcasts the status. */
class CameraArrayManager
{
public:
	
	typedef std::shared_ptr<CameraArrayManager> Ptr;
	
	CameraArrayManager( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	virtual ~CameraArrayManager();
	
	/*! \brief Sets the desired active camera set. Controller takes the necessary 
	 * actions to make the specified cameras the active set.
	 * Disables non-specified active cameras and enables specified cameras. */
	bool SetActiveCameras( const CameraSet& ref );
	
	/*! \brief Returns the upper bound on the number of active cameras. */
	unsigned int MaxActiveCameras() const;

	CameraArrayState GetState() const;
	
protected:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	ros::Publisher statusPub;
	std::shared_ptr<ros::Timer> controllerTimer;
	
	std::string referenceFrame;
	std::string bodyFrame;
	
	lookup::LookupInterface lookupInterface;

	mutable argus_utils::Mutex mutex;
	
	enum CameraStatus
	{
		CAMERA_INACTIVE = 0,
		CAMERA_ACTIVE,
		CAMERA_DEACTIVATING,
		CAMERA_ACTIVATING
	};
	
	struct CameraRegistration
	{
		std::string name;
		CameraStatus status;
		ros::ServiceClient setStreaming;
	};
	typedef std::unordered_map <std::string, CameraRegistration> CameraRegistry;
	CameraRegistry cameraRegistry;
	unsigned int numActiveCameras;
	
	CameraSet referenceActiveCameras;
	
	unsigned int maxNumActive;
	argus_utils::WorkerPool cameraWorkers;
	
	void TimerCallback( const ros::TimerEvent& event );
	
	bool RequestSetStreaming( const std::string& name, bool mode,
							  const argus_utils::WriteLock& lock );
	
	/*! \brief Jobs for the threadpool. */
	void SetStreamingJob( const std::string& name, bool mode );
	
	static std::string StatusToString( CameraStatus status );
	
};
	
}
