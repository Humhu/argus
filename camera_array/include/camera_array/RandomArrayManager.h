#pragma once

#include "camera_array/CameraArrayManager.h"

#include <boost/random/random_device.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

namespace argus
{
	
class RandomArrayManager
: public CameraArrayManager
{
public:
	
	typedef std::shared_ptr<RandomArrayManager> Ptr;
	
	RandomArrayManager( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	
protected: 
	
	boost::random::mt19937 generator;
	std::vector<std::string> cameraNames;
	std::shared_ptr<ros::Timer> timer;
	
	virtual void TimerCallback( const ros::TimerEvent& event );
	
};
	
}
