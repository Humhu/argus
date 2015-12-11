#pragma once

#include <ros/ros.h>
#include "lookup/LookupInterface.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"
#include "argus_utils/PoseSE3.h"
#include "argus_utils/WorkerPool.h"
#include "argus_utils/SynchronizationUtils.h"

#include <unordered_map>
#include <set>
#include <memory>

namespace camera_array
{
	
typedef std::set <std::string> CameraSet;

/*! \brief Describes the state of a camera array. Active and inactive cameras
 * are given as sets for easy querying and iterating. */
struct CameraArrayState
{
	CameraSet activeCameras;
	CameraSet inactiveCameras;
	std::unordered_map<std::string, argus_utils::PoseSE3> extrinsics;
};

/*! \brief The base camera array manager class. Contains basic functionality for
 * registering member cameras and getting pose updates. */
class CameraArrayManager
{
public:
	
	typedef std::shared_ptr<CameraArrayManager> Ptr;
	
	CameraArrayManager( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	virtual ~CameraArrayManager();
	
	/*! \brief Takes the necessary actions to make the specified cameras the active set.
	 * Disables non-specified active cameras and enables specified cameras. */
	bool SetActiveCameras( const CameraSet& ref );
	
	/*! \brief Returns the upper bound on the number of active cameras. */
	unsigned int MaxActiveCameras() const;
	
	void RequestSetStreaming( const std::string& name, bool mode );
	void RequestSwitchStreaming( const std::string& toDisable, const std::string& toEnable );

	const CameraArrayState& GetState() const;
	
protected:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
		
	std::string referenceFrame;
	std::string bodyFrame;
	
	lookup::LookupInterface lookupInterface;
	extrinsics_array::ExtrinsicsInfoManager extrinsicsManager;

	mutable argus_utils::Mutex mutex;
	
	struct CameraRegistration
	{
		std::string name;
		ros::ServiceClient setStreaming;
	};
	typedef std::unordered_map <std::string, CameraRegistration> CameraRegistry;
	CameraRegistry cameraRegistry;
	
	CameraArrayState currentState;
	
	unsigned int maxNumActive;
	argus_utils::WorkerPool cameraWorkers;
	
	bool IsActive( const std::string& name ) const;
	
	/*! \brief Set the streaming state of a camera. */
	bool SetStreaming( const std::string& name, bool mode, bool force = false );
	bool SwitchStreaming( const std::string& toDisable, const std::string& toEnable );
	
	void SetStreamingJob( const std::string& name, bool mode );
	void SwitchStreamingJob( const std::string& toDisable, const std::string& toEnable );
	
	
	
};
	
}
