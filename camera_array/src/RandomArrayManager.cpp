#include "camera_array/RandomArrayManager.h"
#include "argus_utils/RandomUtils.hpp"
#include <boost/foreach.hpp>

using namespace argus_utils;

namespace camera_array
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
}

void RandomArrayManager::TimerCallback( const ros::TimerEvent& event )
{
	std::vector<unsigned int> cameraInds;
	BitmapSampling( cameraNames.size(), maxNumActive, cameraInds, generator );
	
	CameraSet active;
	for( unsigned int i = 0; i < cameraInds.size(); i++ )
	{
		active.insert( cameraNames[ cameraInds[i] ] );
	}
	SetActiveCameras( active );
}

}
