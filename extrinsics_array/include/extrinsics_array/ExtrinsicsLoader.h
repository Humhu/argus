#pragma once

#include "extrinsics_array/ExtrinsicsLookup.h"

#include <ros/ros.h>

namespace extrinsics_array
{
	
/*! \brief Pushes a YAML calibration file to the parameter server and sets up a
 * lookup table. */
class ExtrinsicsLoader
{
public:
	
	ExtrinsicsLoader( ros::NodeHandle& nh );
	
	/*! \brief Sets the lookup table param path prefix. */
	void SetLookupNamespace( const std::string& paramPath );
	
	/*! \brief Sets the parent for lookup entries. */
	void SetLookupParent( const std::string& parent );
	
	/*! \brief Load a YAML file to the specified parameter path. */
	bool LoadYAML( const std::string& filePath, const std::string& paramPath = "" );
	
private:
	
	ros::NodeHandle nodeHandle;
	
	ExtrinsicsLookupManager lookupManager;
	
	std::string lookupPath;
	std::string parentPath;
};
	
} // end namespace extrinsics_array
