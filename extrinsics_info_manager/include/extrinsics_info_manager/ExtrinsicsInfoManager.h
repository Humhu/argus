#pragma once

#include <ros/ros.h>

// Messages
#include "extrinsics_info_manager/ExtrinsicsInfo.h"

// Services
#include "extrinsics_info_manager/GetExtrinsicsInfo.h"
#include "extrinsics_info_manager/SetExtrinsicsInfo.h"

namespace extrinsics_info_manager
{
	
/*! \brief Manages extrinsics information for a set of frames relative to a reference.
 * Closely mirrors interface to CameraInfoManager. 
 * NOTE This class is not synchronized and thus not safe for threading
 */
class ExtrinsicsInfoManager
{
public:
	
	ExtrinsicsInfoManager( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	/*! \brief Set the reference frame name for this manager. */
	void SetReferenceName( const std::string& name );
		
	/*! \brief Load extrinsics information from a YAML file. */
	bool LoadExtrinsicsInfo( const std::string& filePath );
	
	/*! \brief Returns whether or not info has been loaded. */
	bool IsCalibrated() const;
	
	/*! \brief Get the current extrinsics info. */
	ExtrinsicsInfo GetExtrinsicsInfo() const;
	
	/*! \brief Set the current extrinsics info. */
	void SetExtrinsicsInfo( const ExtrinsicsInfo& info );

	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	ros::ServiceServer getInfoServer;
	ros::ServiceServer setInfoServer;
	
	std::string referenceName;
	bool initialized;
	
	bool GetExtrinsicsInfoService( GetExtrinsicsInfo::Request& req,
								   GetExtrinsicsInfo::Response& res );
	bool SetExtrinsicsInfoService( SetExtrinsicsInfo::Request& req,
								   SetExtrinsicsInfo::Response& res );
	
	ExtrinsicsInfo extrinsics;
	
};
	
} // end namespace camera_array
