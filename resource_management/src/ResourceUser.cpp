#include "resource_management/ResourceUser.h"

#include "resource_management/RequestResources.h"
#include "resource_management/ReleaseResources.h"

namespace resource_management
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
			nodeHandle.serviceClient<RequestResources>( resourceManager + "/request_resources", true );
		releaseClient = 
			nodeHandle.serviceClient<ReleaseResources>( resourceManager + "/release_resources", true );
			
		XmlRpc::XmlRpcValue resources;
		privHandle.getParam( "resources", resources );
		
		XmlRpc::XmlRpcValue::iterator iter = resources.begin();
		while( iter != resources.end() )
		{
			XmlRpc::XmlRpcValue::ValueStruct::value_type item = *iter;
			
			int nominal = item.second["nominal"];
			int minimum = item.second["minimum"];
			
			ResourceRequest req;
			req.resourceName = item.first;
			req.nominalQuantity = nominal;
			req.minimumQuantity = minimum;
			resourceRequirements.push_back( req );
			
			iter++;
		}
		
	}
	
	bool ResourceUser::HasResources() const
	{
		return hasResources;
	}
	
	void ResourceUser::AcquireResources()
	{
		if( resourceRequirements.empty() || hasResources ) { return; }
		
		RequestResources req;
		req.request.requests = resourceRequirements;
		if( !requestClient.call( req ) )
		{
			ROS_ERROR_STREAM( "Failed to acquire resources." );
			return;
		}
		
		ROS_INFO_STREAM( "Acquired resource grant " << req.response.grantID );

		currentGrantID = req.response.grantID;
		hasResources = true;
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
