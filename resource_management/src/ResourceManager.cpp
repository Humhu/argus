#include "resource_management/ResourceManager.h"

#include <boost/foreach.hpp>

namespace resource_management
{
	
	ResourceManager::ResourceManager( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ),
		releaseSpinner( 1, &releaseQueue ),
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
			int max = item.second["max"];
			int min = item.second["min"];
			
			ResourceEntry entry;
			entry.available = avail;
			entry.max = max;
			entry.min = min;
			registry[name] = entry;
		}
		
		int numRequestSpinners;
		privHandle.param( "num_request_threads", numRequestSpinners, 4 );
		requestSpinner = std::make_shared<ros::AsyncSpinner>( numRequestSpinners, &requestQueue );
		requestSpinner->start();
		
		ros::AdvertiseServiceOptions requestOptions;
		requestOptions.init<RequestResources>( "request_resources", 
			boost::bind( &ResourceManager::RequestResourcesService, this, _1, _2 ) );
		requestOptions.callback_queue = &requestQueue;
		requestServer = privHandle.advertiseService( requestOptions );
		
		ros::AdvertiseServiceOptions releaseOptions;
		releaseOptions.init<ReleaseResources>( "release_resources",
			boost::bind( &ResourceManager::ReleaseResourcesService, this, _1, _2 ) );
		releaseOptions.callback_queue = &releaseQueue;
		releaseServer = privHandle.advertiseService( releaseOptions );
		
		releaseSpinner.start();
	}
	
	ResourceManager::~ResourceManager()
	{}
	
	bool ResourceManager::RequestResourcesService( RequestResources::Request& req,
												   RequestResources::Response& res )
	{
		// First validate the request contents
		if( !ValidateRequest( req ) )
		{
			ROS_ERROR_STREAM( "Resource request failed validation." );
			return false;
		}

		Lock lock( mutex );

		while( !TestSetResources( req, res, lock ) )
		{
			blockedCondition.wait( lock );
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
		
		blockedCondition.notify_all();
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
											RequestResources::Response& res,
											Lock& lock )
	{
		// TODO Check lock state
		BOOST_FOREACH( const ResourceRequest& r, req.requests )
		{			
			if( registry[r.resourceName].available < r.minimumQuantity )
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
			
			unsigned int quant = std::min( registry[r.resourceName].available,
										   r.nominalQuantity );
			g.amountGranted = quant;
			registry[r.resourceName].available -= quant;
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
