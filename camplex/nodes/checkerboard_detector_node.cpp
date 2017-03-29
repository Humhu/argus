#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/calib3d/calib3d.hpp>

#include "camplex/FiducialCommon.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/synchronization/ThreadsafeQueue.hpp"
#include "argus_msgs/ImageFiducialDetections.h"
#include "argus_utils/synchronization/WorkerPool.h"

using namespace argus;

class CheckerboardDetector
{
public:

	CheckerboardDetector( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: _imagePort( nh )
	{

		unsigned int width, height;
		GetParamRequired<unsigned int>( ph, "board_width", width );
		GetParamRequired<unsigned int>( ph, "board_height", height );
		_boardSize = cv::Size( width, height );
        std::stringstream ss;
        ss << "checkerboard_" << width << "_" << height;
        _boardName = ss.str();

		GetParam( ph, "enable_refinement", _enableRefinement, true );
		if( _enableRefinement )
		{
			unsigned int maxIters;
			double epsilon;
			GetParam<unsigned int>( ph, "refine_max_iters", maxIters, 30 );
			GetParam( ph, "refine_epsilon", epsilon );
			_refineCriteria = cv::TermCriteria( cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
			                                    maxIters,
			                                    epsilon );
		}

		_detPub = ph.advertise<argus_msgs::ImageFiducialDetections>( "detections", 20 );

		unsigned int buffLen;
		GetParam<unsigned int>( ph, "buffer_length", buffLen, 5 );
		_imageSub = _imagePort.subscribe( "image",
		                                  buffLen,
		                                  &CheckerboardDetector::ImageCallback, this );

		unsigned int numDetectorThreads;
		GetParam<unsigned int>( ph, "num_detector_threads", numDetectorThreads, 1 );
		_detectorWorkers.SetNumWorkers( numDetectorThreads );
		for( unsigned int i = 0; i < numDetectorThreads; ++i )
		{
			WorkerPool::Job job = boost::bind( &CheckerboardDetector::DetectionSpin, this );
			_detectorWorkers.EnqueueJob( job );
		}
		_detectorWorkers.StartWorkers();
	}

	void ImageCallback( const sensor_msgs::Image::ConstPtr& msg )
	{
        _imageBuffer.PushBack(msg);
	}

	void DetectionSpin()
	{
		sensor_msgs::Image::ConstPtr msg;
		std::vector<cv::Point2f> corners;

		while( ros::ok() )
		{
			_imageBuffer.WaitPopBack( msg );
			cv::Mat msgFrame = cv_bridge::toCvShare( msg )->image;
			cv::Mat frame;
			if( msgFrame.channels() > 1 )
			{
				cv::cvtColor( msgFrame, frame, CV_BGR2GRAY );
			}
			else
			{
				frame = msgFrame;
			}

			if( !cv::findChessboardCorners( frame,
			                                _boardSize,
			                                corners,
			                                cv::CALIB_CB_ADAPTIVE_THRESH |
			                                cv::CALIB_CB_NORMALIZE_IMAGE |
			                                cv::CALIB_CB_FAST_CHECK ) )
			{
				continue;
			}

			if( _enableRefinement )
			{
				// TODO Parameterize the search window size?
				cv::cornerSubPix( frame,
				                  corners,
				                  cv::Size( 11, 11 ),
				                  cv::Size( -1, -1 ),
				                  _refineCriteria );
			}
			
            FiducialDetection det;
            det.name = _boardName;
            det.undistorted = false;
            det.normalized = false;
            det.points = CvToPoints( corners );

			ImageFiducialDetections detections;
			detections.sourceName = msg->header.frame_id;
			detections.timestamp = msg->header.stamp;
            detections.detections.push_back( det );
			_detPub.publish( detections.ToMsg() );
		}
	}

private:

	image_transport::ImageTransport _imagePort;
	image_transport::Subscriber _imageSub;

	ros::Publisher _detPub;
	WorkerPool _detectorWorkers;
	ThreadsafeQueue<sensor_msgs::Image::ConstPtr> _imageBuffer;

    std::string _boardName;
	cv::Size _boardSize;
	bool _enableRefinement;
	cv::TermCriteria _refineCriteria;
};

int main( int argc, char**argv )
{
	ros::init( argc, argv, "checkerboard_detector" );

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle( "~" );
	CheckerboardDetector recorder( nodeHandle, privHandle );

	ros::spin();

	return 0;
}
