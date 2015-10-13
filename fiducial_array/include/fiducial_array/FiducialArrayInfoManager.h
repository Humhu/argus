#pragma once

#include <ros/ros.h>

// Messages
#include "fiducial_array/FiducialArrayInfo.h"

// Services
#include "fiducial_array/GetFiducialArrayInfo.h"
#include "fiducial_array/SetFiducialArrayInfo.h"

namespace fiducial_array
{
	
/*! \brief Manages extrinsics information for a set of frames relative to a reference.
 * Closely mirrors interface to CameraInfoManager. 
 * NOTE This class is not synchronized and thus not safe for threading */
class FiducialArrayInfoManager
{
public:
	
	FiducialArrayInfoManager( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	/*! \brief Set the reference frame name for this manager. */
	void SetReferenceName( const std::string& name );
	
	/*! \brief Load extrinsics information from a YAML file. */
	bool LoadFiducialArrayInfo( const std::string& filePath );
	
	/*! \brief Returns whether or not info has been loaded. */
	bool IsCalibrated() const;
	
	/*! \brief Get the current extrinsics info. */
	FiducialArrayInfo GetFiducialArrayInfo() const;
	
	/*! \brief Set the current extrinsics info. */
	void SetFiducialArrayInfo( const FiducialArrayInfo& info );

	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	ros::ServiceServer getInfoServer;
	ros::ServiceServer setInfoServer;
	
	std::string referenceName;
	bool initialized;
	
	bool GetFiducialArrayInfoService( GetFiducialArrayInfo::Request& req,
	                                  GetFiducialArrayInfo::Response& res );
	bool SetFiducialArrayInfoService( SetFiducialArrayInfo::Request& req,
	                                  SetFiducialArrayInfo::Response& res );
	
	FiducialArrayInfo extrinsics;
	
};
	
} // end namespace fiducial_array
