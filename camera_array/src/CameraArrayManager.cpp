#include "camera_array/CameraArrayManager.h"
#include "camera_array/CameraArrayStatus.h"
#include "camera_array/CameraStatus.h"

#include "argus_utils/ParamUtils.h"
#include "camplex/SetStreaming.h" // TODO Move into argus_msgs?
#include <boost/foreach.hpp>

using namespace argus;

namespace camera_array
{
	
CameraArrayManager::CameraArrayManager( const ros::NodeHandle& nh,
                                        const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), numActiveCameras( 0 )
{
	std::vector<std::string> members;
	if( !ph.getParam( "cameras", members ) )
	{
		ROS_ERROR_STREAM( "Must specify member cameras." );
		exit( -1 );
	}
	
	if( !ph.getParam( "body_frame", bodyFrame ) ||
		!ph.getParam( "reference_frame", referenceFrame ) )
	{
		ROS_ERROR_STREAM( "Must specify reference and body frame IDs." );
		exit( -1 );
	}
	
	GetParamDefault<unsigned int>( privHandle, "max_active_cameras", maxNumActive, 1 );
	cameraWorkers.SetNumWorkers( maxNumActive );
	cameraWorkers.StartWorkers();
	
	WriteLock lock( mutex );
	BOOST_FOREACH( const std::string& name, members )
	{
		CameraRegistration registration;
		
		std::string ns;
		if( !lookupInterface.ReadNamespace( name, ns ) )
		{
			ROS_ERROR_STREAM( "Could not read member namespace." );
			exit( -1 );
		}
		
		std::string setStreamingName = ns + "set_streaming";
		if( !ros::service::waitForService( setStreamingName, ros::Duration( 5.0 ) ) )
		{
			ROS_ERROR_STREAM( "Could not find service " << setStreamingName );
			exit( -1 );
		}
		
		ROS_INFO_STREAM( "Registering camera " << name );
		// HACK to allow forcing streaming off
		registration.name = name;
		registration.status = CAMERA_ACTIVE;
		registration.setStreaming = 
			nodeHandle.serviceClient<camplex::SetStreaming>( setStreamingName );
		cameraRegistry[name] = registration;
		
		numActiveCameras++;
		RequestSetStreaming( name, false, lock );
	}
	
	statusPub = nodeHandle.advertise<CameraArrayStatus>( "array_status", 5 );
	
	double controlRate;
	ph.param<double>( "manager_update_rate", controlRate, 5.0 );
	controllerTimer = std::make_shared<ros::Timer>
	    ( nodeHandle.createTimer( ros::Duration( 1.0/controlRate ),
		                          &CameraArrayManager::TimerCallback,
		                          this ) );
}

CameraArrayManager::~CameraArrayManager()
{
	cameraWorkers.StopWorkers();
	cameraWorkers.WaitOnJobs();
}

unsigned int CameraArrayManager::MaxActiveCameras() const
{
	return maxNumActive;
}

CameraArrayState CameraArrayManager::GetState() const
{
	WriteLock lock( mutex );
	
	CameraArrayState state;
	BOOST_FOREACH( const CameraRegistry::value_type& item, cameraRegistry )
	{
		switch( item.second.status )
		{
			case CAMERA_INACTIVE:
			case CAMERA_DEACTIVATING:
				state.inactiveCameras.insert( item.first );
				break;
			case CAMERA_ACTIVE:
			case CAMERA_ACTIVATING:
				state.activeCameras.insert( item.first );
				break;
			default:
				ROS_ERROR_STREAM( "Invalid status for " << item.first );
		}
	}
	if( state.activeCameras.size() > maxNumActive )
	{
		ROS_ERROR_STREAM( "Invalid state!" );
	}
	return state;
}

void CameraArrayManager::TimerCallback( const ros::TimerEvent& event )
{
	
	WriteLock lock( mutex );
	
	CameraArrayStatus arrayStatus;
	arrayStatus.header.stamp = event.current_real;
	BOOST_FOREACH( const CameraRegistry::value_type& item, cameraRegistry )
	{
		const std::string& cameraName = item.first;
		
		camera_array::CameraStatus camStatus;
		camStatus.name = cameraName;
		camStatus.status = StatusToString( item.second.status );
		arrayStatus.status.push_back( camStatus );
		
		if( referenceActiveCameras.count( cameraName ) > 0 )
		{
			if( numActiveCameras < maxNumActive )
			{
				RequestSetStreaming( cameraName, true, lock );
			}
		}
		else
		{
			RequestSetStreaming( cameraName, false, lock );
		}
	}
	// TODO Check return values
	
	lock.unlock();
	statusPub.publish( arrayStatus );
}

bool CameraArrayManager::SetActiveCameras( const CameraSet& ref )
{
	if( ref.size() > maxNumActive )
	{
		ROS_ERROR_STREAM( "Cannot set " << ref.size() << " active cameras for limit "
		    << maxNumActive );
		return false;
	}
	
	WriteLock lock( mutex );
	referenceActiveCameras = ref;
	
	return true;
}

bool CameraArrayManager::RequestSetStreaming( const std::string& name, 
                                              bool mode,
                                              const WriteLock& lock )
{
	
	// TODO Make sure owned mutex is same
	// TODO Move into argus_utils/SynchronizationUtils.h
	if( !lock.owns_lock() )
	{
		throw std::runtime_error( "Invalid external lock." );
	}

	if( cameraRegistry.count( name ) == 0 ) { return false; }
	
	// If the current status already matches, return
	// If busy, return
	CameraRegistration& registration = cameraRegistry[ name ];
	switch( registration.status )
	{
		case CAMERA_ACTIVE:
			if( mode ) { return true; }
			break;
		case CAMERA_INACTIVE:
			if( !mode ) { return true; }
			break;
		case CAMERA_ACTIVATING:
			return mode;
		case CAMERA_DEACTIVATING:
			return !mode;
		default:
			ROS_ERROR_STREAM( "Invalid status for camera " << name );
			exit( -1 );
	}
	
	// If we're at capacity, return
	if( mode && numActiveCameras >= maxNumActive )
	{
		ROS_WARN_STREAM( "Cannot activate camera " << name << ". Already max num active." );
		return false;
	}
	
	// Set status to busy
	registration.status = mode ? CAMERA_ACTIVATING : CAMERA_DEACTIVATING;
	if( mode ) { numActiveCameras++; }
	
	WorkerPool::Job job = boost::bind( &CameraArrayManager::SetStreamingJob,
	                                   this,
	                                   name,
	                                   mode );
	cameraWorkers.EnqueueJob( job );
	return true;
}

void CameraArrayManager::SetStreamingJob( const std::string& name, 
                                          bool mode )
{
	camplex::SetStreaming srv;
	srv.request.enableStreaming = mode;

	//	ros::Time start = ros::Time::now();
	CameraRegistration& registration = cameraRegistry.at( name );
	if( !registration.setStreaming.call( srv ) )
	{
		ROS_WARN_STREAM( "Could not set streaming for camera " << name );
		return;
	}
	//	double dt = (ros::Time::now() - start).toSec();
	//	ROS_INFO_STREAM( "Turning on streaming took " << dt );
	
	WriteLock lock( mutex );
	registration.status = mode ? CAMERA_ACTIVE : CAMERA_INACTIVE;
	if( !mode ) { numActiveCameras--; }
}

std::string CameraArrayManager::StatusToString( CameraStatus status )
{
	switch( status )
	{
		case CAMERA_INACTIVE:
			return "inactive";
		case CAMERA_ACTIVE:
			return "active";
		case CAMERA_DEACTIVATING:
			return "deactivating";
		case CAMERA_ACTIVATING:
			return "activating";
		default:
			return "error";
	}
}

} // end namespace camera_array
