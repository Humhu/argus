#include "camera_array/RandomArrayManager.h"
#include "argus_utils/random/RandomUtils.hpp"
#include <boost/foreach.hpp>

namespace argus
{
	
RandomArrayManager::RandomArrayManager( const ros::NodeHandle& nh,
                                        const ros::NodeHandle& ph )
: CameraArrayManager( nh, ph )
{

	boost::random::random_device rng;
	generator.seed( rng );
		
	cameraNames.reserve( cameraRegistry.size() );
	BOOST_FOREACH( const CameraRegistry::value_type& item, cameraRegistry )
	{
		cameraNames.push_back( item.first );
	}

  double updateRate;
  ph.param<double>( "update_rate", updateRate, 1.0 );
  timer = std::make_shared<ros::Timer>
    ( nodeHandle.createTimer( ros::Duration( 1.0/updateRate ),
			      &RandomArrayManager::TimerCallback,
			      this ) );

}

void RandomArrayManager::TimerCallback( const ros::TimerEvent& event )
{
	std::vector<unsigned int> cameraInds;
	BitmapSampling( cameraNames.size(), maxNumActive, cameraInds, generator );
	
	CameraSet active;
	for( unsigned int i = 0; i < cameraInds.size(); i++ )
	{
		active.insert( cameraNames[ cameraInds[i] ] );
		ROS_INFO_STREAM( "Activating " << cameraNames[ cameraInds[i] ] );
	}
	SetActiveCameras( active );
}

}
