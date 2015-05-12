#include "resource_management/ResourceManager.h"

#include <boost/foreach.hpp>

namespace resource_management
{
	
	ResourceManager::ResourceManager( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ),
		grantCounter( 0 )
	{
		
		XmlRpc::XmlRpcValue resources;
		privHandle.getParam( "resources", resources);
		
		XmlRpc::XmlRpcValue::iterator iter = resources.begin();
		while( iter != resources.end() )
		{
			XmlRpc::XmlRpcValue::ValueStruct::value_type item = *iter;
			iter++;
			std::string name = item.first;
			
			if( registry.count( name ) > 0 )
			{
				throw std::runtime_error( "Duplicate resource entry for " + name );
			}
			
			int avail = item.second["available"];
			
			ResourceEntry entry;
			entry.available = avail;
			registry[name] = entry;
		}
		
		requestServer = privHandle.advertiseService( "request_resources", 
			&ResourceManager::RequestResourcesService, this );
		releaseServer = privHandle.advertiseService( "release_resources",
			&ResourceManager::ReleaseResourcesService, this );
	}
	
	bool ResourceManager::RequestResourcesService( RequestResources::Request& req,
												   RequestResources::Response& res )
	{
		// First validate the request contents
		if( !ValidateRequest( req ) )
		{
			ROS_ERROR_STREAM( "Resource request failed validation." );
			return false;
		}

		if( !TestSetResources( req, res ) )
		{
			return false;
		}
		
		return true;
	}
	
	bool ResourceManager::ReleaseResourcesService( ReleaseResources::Request& req,
												   ReleaseResources::Response& res )
	{
		Lock lock( mutex );
		
		if( grants.count( req.grantID ) == 0 )
		{
			ROS_ERROR_STREAM( "No existing grant with ID " << req.grantID );
			return false;
		}
		
		GrantEntry entry = grants[req.grantID];
		BOOST_FOREACH( const ResourceGrant& grant, entry.grants )
		{
			registry[grant.resourceName].available += grant.amountGranted;
		}
		
		return true;
	}
	
	bool ResourceManager::ValidateRequest( const RequestResources::Request& req ) const
	{
		BOOST_FOREACH( const ResourceRequest& r, req.requests )
		{
			if( registry.count( r.resourceName ) == 0 )
			{
				ROS_ERROR_STREAM( "No resource of type " << r.resourceName );
				return false;
			}
		}
		return true;
	}
	
	bool ResourceManager::TestSetResources( const RequestResources::Request& req,
											RequestResources::Response& res )
	{
		Lock lock( mutex );

		BOOST_FOREACH( const ResourceRequest& r, req.requests )
		{			
			if( registry[r.resourceName].available < r.requestedQuantity )
			{
				return false;
			}
		}
		
		res.grants.clear();
		res.grants.reserve( req.requests.size() );
		BOOST_FOREACH( const ResourceRequest& r, req.requests )
		{
			ResourceGrant g;
			g.resourceName = r.resourceName;
			
			g.amountGranted = r.requestedQuantity;
			registry[r.resourceName].available -= r.requestedQuantity;
			res.grants.push_back( g );
		}
		res.grantID = grantCounter;
		grantCounter++;
		
		GrantEntry entry;
		entry.startTime = ros::Time::now();
		entry.grants = res.grants;
		grants[res.grantID] = entry;
		
		ROS_INFO_STREAM( "Granted resources with ID " << res.grantID );
		
		return true;
	}
}
