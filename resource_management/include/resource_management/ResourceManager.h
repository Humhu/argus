#ifndef _V4L2D_RESOURCE_MANAGER_H_
#define _V4L2D_RESOURCE_MANAGER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>

#include <unordered_map>

#include "resource_management/ResourceGrant.h"
#include "resource_management/RequestResources.h"
#include "resource_management/ReleaseResources.h"

namespace resource_management
{
	
	class ResourceManager
	{
	public:
		
		ResourceManager( ros::NodeHandle& nh, ros::NodeHandle& ph );
		~ResourceManager();
		
		bool RequestResourcesService( RequestResources::Request& req,
									  RequestResources::Response& res );
		
		bool ReleaseResourcesService( ReleaseResources::Request& req,
									  ReleaseResources::Response& res );
		
	private:
		
		typedef boost::unique_lock< boost::shared_mutex > Lock;
		
		boost::shared_mutex mutex;
		boost::condition_variable_any blockedCondition;
		
		std::shared_ptr<ros::AsyncSpinner> requestSpinner;
		ros::CallbackQueue requestQueue;
		ros::AsyncSpinner releaseSpinner;
		ros::CallbackQueue releaseQueue;
		
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privHandle;
		
		ros::ServiceServer requestServer;
		ros::ServiceServer releaseServer;
		
		struct ResourceEntry
		{
			unsigned int available;
			unsigned int max;
			unsigned int min;
		};
		
		// TODO Heartbeat on old grants?
		struct GrantEntry
		{
			ros::Time startTime;
			std::vector<ResourceGrant> grants;
		};
		
		std::unordered_map< std::string, ResourceEntry > registry;
		std::unordered_map< unsigned int, GrantEntry > grants;
		
		unsigned int grantCounter;
		
		bool ValidateRequest( const RequestResources::Request& req ) const;
		bool TestSetResources( const RequestResources::Request& req,
							   RequestResources::Response& res,
							   Lock& lock );
		
	};
	
}

#endif
