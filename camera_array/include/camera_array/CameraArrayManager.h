#pragma once

#include <ros/ros.h>
#include "lookup/LookupInterface.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"
#include "argus_utils/PoseSE3.h"

#include <unordered_map>
#include <set>
#include <memory>

namespace camera_array
{
	
typedef std::set <std::string> CameraSet;

// TODO Threadify streaming calls to allow parallel calls
/*! \brief The base camera array manager class. Contains basic functionality for
 * registering member cameras and getting pose updates. */
class CameraArrayManager
{
public:
	
	typedef std::shared_ptr<CameraArrayManager> Ptr;
	
	CameraArrayManager( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	
	// TODO Make dispatch a thread?
	bool SetStreaming( const std::string& name, bool mode );
	
	/*! \brief Disable a camera and enable another, in that order. */
	bool SwitchStreaming( const std::string& toDisable, const std::string& toEnable );
	
	/*! \brief Takes the necessary actions to make the specified cameras the active set.
	 * Disables non-specified active cameras and enables specified cameras. */
	bool SetActiveCameras( const CameraSet& ref );
	
	/*! \brief Returns the upper bound on the number of active cameras. */
	unsigned int MaxActiveCameras() const;
	
	bool IsActive( const std::string& name ) const;
	
	/*! \brief Returns the set of currently active cameras. */
	const CameraSet& ActiveCameras() const;
	
protected:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
		
	std::string referenceFrame;
	std::string bodyFrame;
	
	lookup::LookupInterface lookupInterface;
	extrinsics_array::ExtrinsicsInfoManager extrinsicsManager;

	struct CameraRegistration
	{
		std::string name;
		ros::ServiceClient setStreaming;
	};
	typedef std::unordered_map <std::string, CameraRegistration> CameraRegistry;
	CameraRegistry cameraRegistry;
	CameraSet activeCameras;
	unsigned int maxNumActive;
	
	
};
	
}
