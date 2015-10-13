#pragma once

#include <ros/ros.h>

// Messages
#include "camera_array/CameraArrayInfo.h"

// Services
#include "camera_array/GetCameraArrayInfo.h"
#include "camera_array/SetCameraArrayInfo.h"

namespace camera_array
{
	
/*! \brief Manages extrinsics information for a set of frames relative to a reference.
 * Closely mirrors interface to CameraInfoManager. 
 * NOTE This class is not synchronized and thus not safe for threading
 */
class CameraArrayInfoManager
{
public:
	
	CameraArrayInfoManager( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	/*! \brief Set the reference frame name for this manager. */
	void SetReferenceName( const std::string& name );
		
	/*! \brief Load extrinsics information from a YAML file. */
	bool LoadCameraArrayInfo( const std::string& filePath );
	
	/*! \brief Returns whether or not info has been loaded. */
	bool IsCalibrated() const;
	
	/*! \brief Get the current extrinsics info. */
	CameraArrayInfo GetCameraArrayInfo() const;
	
	/*! \brief Set the current extrinsics info. */
	void SetCameraArrayInfo( const CameraArrayInfo& info );

	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	ros::ServiceServer getInfoServer;
	ros::ServiceServer setInfoServer;
	
	std::string referenceName;
	bool initialized;
	CameraArrayInfo extrinsics;
	
	bool GetCameraArrayInfoService( GetCameraArrayInfo::Request& req,
	                                GetCameraArrayInfo::Response& res );
	bool SetCameraArrayInfoService( SetCameraArrayInfo::Request& req,
	                                SetCameraArrayInfo::Response& res );
};
	
} // end namespace camera_array
