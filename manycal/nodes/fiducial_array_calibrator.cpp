#include <ros/ros.h>

#include "lookup/LookupInterface.h"
#include "manycal/FiducialArrayCalibrator.h"
#include "argus_msgs/ImageFiducialDetections.h"
#include "argus_utils/utils/ParamUtils.h"

class FiducialArrayCalibratorNode
{
public:

	FiducialArrayCalibratorNode( ros::NodeHandle& nh,
	                             ros::NodeHandle& ph )
		: _lookupInterface( nh )
	{
		_calibrator.Initialize( ph, _lookupInterface );

		unsigned int detBufLen;
		GetParam( ph, "detections_buffer_length", detBuffLen, 10 );
		_detSub = nh.subscribe( "detections", detBufLen,
		                        &FiducialArrayCalibratorNode::DetectionCallback,
		                        this );

		double spinRate;
		GetParam( ph, "spin_rate", spinRate, 0.1 );
		_spinTimer = nh.createTimer( ros::Duration( 1.0 / spinRate ),
		                             &FiducialArrayCalibratorNode::TimerCallback,
		                             this );
	}

	void DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg )
	{
		// Convert to C++
		std::vector<FiducialDetection> detections;
		detections.reserve( msg->detections.size() );
		BOOST_FOREACH( const argus_msgs::FiducialDetection & d, msg->detections )
		{
			detections.emplace_back( d );
		}

		_calibrator.BufferDetections( detections );
	}

	void TimerCallback( const ros::TimerEvent& event )
	{
		_calibrator.Spin();
		// TODO Publish results?
	}

private:

	ros::Subscriber _detSub;
	ros::Timer _spinTimer;

	LookupInterface _lookupInterface;
	FiducialArrayCalibrator _calibrator;
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "fiducial_array_calibrator" );

	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );

	argus::FiducialArrayCalibrator calibrator( nh, ph );

	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();

	return 0;
}
