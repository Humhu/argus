#include "resource_management/ResourceUser.h"

#include "resource_management/RequestResources.h"
#include "resource_management/ReleaseResources.h"

#include <boost/foreach.hpp>

using namespace resource_management;

namespace argus
{
	
	ResourceUser::ResourceUser( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ),
		hasResources( false )
	{
		if( !privHandle.hasParam( "resources" ) )
		{
			return;
		}
	
		std::string resourceManager;
		privHandle.getParam("resource_manager", resourceManager );
		requestClient = 
			nodeHandle.serviceClient<resource_management::RequestResources>( resourceManager + "/request_resources", true );
		releaseClient = 
			nodeHandle.serviceClient<resource_management::ReleaseResources>( resourceManager + "/release_resources", true );
			
		typedef std::map< std::string, int > ResourceMap;
		ResourceMap resources;
		privHandle.getParam( "resources", resources );
		BOOST_FOREACH( const ResourceMap::value_type& item, resources )
		{
			ResourceRequest req;
			req.resourceName = item.first;
			req.requestedQuantity = item.second;
			resourceRequirements.push_back( req );
		}
	}
	
	bool ResourceUser::HasResources() const
	{
		return hasResources;
	}
	
	bool ResourceUser::AcquireResources()
	{
		if( resourceRequirements.empty() || hasResources ) { return true; }
		
		RequestResources req;
		req.request.requests = resourceRequirements;
		if( !requestClient.call( req ) )
		{
			ROS_ERROR_STREAM( "Failed to acquire resources." );
			return false;
		}
		
		ROS_INFO_STREAM( "Acquired resource grant " << req.response.grantID );

		currentGrantID = req.response.grantID;
		hasResources = true;
		return true;
	}
	
	void ResourceUser::RelinquishResources()
	{
		if( resourceRequirements.empty() || !hasResources ) { return; }
		
		ROS_INFO_STREAM( "Relinquishing resource grant " << currentGrantID );
		
		ReleaseResources rel;
		rel.request.grantID = currentGrantID;
		if( !releaseClient.call( rel ) )
		{
			ROS_ERROR_STREAM( "Failed to release resources." );
			return;
		}
		
		hasResources = false;
	}
	
}
