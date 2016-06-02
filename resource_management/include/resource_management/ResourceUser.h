#pragma once

#include <ros/ros.h>
#include "resource_management/ResourceGrant.h"
#include "resource_management/ResourceRequest.h"

#include "resource_management/RequestResources.h"
#include "resource_management/ReleaseResources.h"

namespace argus
{

// NOTE Unsynchronized
class ResourceUser
{
public:
	
	ResourceUser( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	bool HasResources() const;
	
	/*! \brief Attempts to acquire the necessary resources. Returns true if
		* resources have been acquired or false if the resources are unavailable. */
	bool AcquireResources();
	
	/*! \brief Returns the currently held resources. */
	void RelinquishResources();
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	ros::ServiceClient requestClient;
	ros::ServiceClient releaseClient;
	
	std::vector<resource_management::ResourceRequest> resourceRequirements;
	bool hasResources;
	unsigned int currentGrantID;
	
};
	
} // end namespace resource_management
