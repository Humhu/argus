#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>

#include <unordered_map>

#include "resource_management/ResourceGrant.h"
#include "resource_management/RequestResources.h"
#include "resource_management/ReleaseResources.h"

namespace argus
{
	
class ResourceManager
{
public:
	
	ResourceManager( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	bool RequestResourcesService( resource_management::RequestResources::Request& req,
	                              resource_management::RequestResources::Response& res );
	
	bool ReleaseResourcesService( resource_management::ReleaseResources::Request& req,
	                              resource_management::ReleaseResources::Response& res );
	
private:
	
	typedef boost::unique_lock< boost::mutex > Lock;
	
	boost::mutex mutex;
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	ros::ServiceServer requestServer;
	ros::ServiceServer releaseServer;
	
	struct ResourceEntry
	{
		unsigned int available;
	};
	
	// TODO Heartbeat on old grants?
	struct GrantEntry
	{
		ros::Time startTime;
		std::vector<resource_management::ResourceGrant> grants;
	};
	
	std::unordered_map< std::string, ResourceEntry > registry;
	std::unordered_map< unsigned int, GrantEntry > grants;
	
	unsigned int grantCounter;
	
	bool ValidateRequest( const resource_management::RequestResources::Request& req ) const;
	bool TestSetResources( const resource_management::RequestResources::Request& req,
	                       resource_management::RequestResources::Response& res );
	
};
	
} // end namespace resource_management
