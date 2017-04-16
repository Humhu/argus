#include <ros/ros.h>

#include <opencv2/calib3d/calib3d.hpp>

#include "camplex/FiducialCommon.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/synchronization/ThreadsafeQueue.hpp"
#include "argus_msgs/ImageFiducialDetections.h"
#include "argus_utils/synchronization/WorkerPool.h"

using namespace argus;

class CameraCalibrator
{
public:

	CameraCalibrator( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: _imagePort( nh )
	{

		_detSub = nh.subscribe( "detections",
		                        10,
		                        CameraCalibrator::DetectionCallback,
		                        this );
	}

	void DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg )
	{
		
	}

private:

    ros::Subscriber _detSub;
    std::deque<FiducialDetection>
};

int main( int argc, char**argv )
{
	ros::init( argc, argv, "camera_calibrator" );

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle( "~" );
	CameraCalibrator calibrator( nodeHandle, privHandle );

	ros::spin();

	return 0;
}
