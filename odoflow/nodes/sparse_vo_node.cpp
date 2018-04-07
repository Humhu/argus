
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TwistStamped.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "odoflow/CornerPointDetector.h"
#include "odoflow/FASTPointDetector.h"
#include "odoflow/FixedPointDetector.h"
#include "odoflow/LKPointTracker.h"
#include "odoflow/RigidEstimator.h"
#include "odoflow/MotionPredictor.h"
#include "odoflow/VelocityPublisher.h"

#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/geometry/VelocityIntegrator.hpp"

#include "camplex/CameraCalibration.h"
#include "camplex/FiducialCommon.h"
#include "paraset/ParameterManager.hpp"

using namespace argus;

class SparseVONode
{
public:

	SparseVONode( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: _imageTrans( nh ), _predictor( nh, ph ), _velPub( nh, ph )
	{
		// Initialize all runtime parameters
		_redetectThresh.InitializeAndRead( ph, 0.75, "redetection_threshold",
		                                   "Pipeline feature redetection min threshold" );
		_redetectThresh.AddCheck<GreaterThanOrEqual>( 0.0 );
		_redetectThresh.AddCheck<LessThanOrEqual>( 1.0 );

		_minInlierRatio.InitializeAndRead( ph, 0.5, "min_inlier_ratio",
		                                   "Pipeline min feature inlier threshold" );
		_minInlierRatio.AddCheck<GreaterThan>( 0 );
		_minInlierRatio.AddCheck<LessThanOrEqual>( 1.0 );

		_maxFrameDt.Initialize( ph, 1.0, "max_frame_dt",
		                        "Maximum frame time difference" );

		_minNumKeypoints.Initialize( ph, 30, "min_num_keypoints",
		                             "Minimum number of keyframe points." );
		_minInlierRatio.AddCheck<GreaterThanOrEqual>( 0 );
		_minNumKeypoints.AddCheck<IntegerValued>( ROUND_CLOSEST );

		GetParamRequired( ph, "scale", _scale );

		InitializeDetector( nh, ph );
		InitializeTracker( nh, ph );
		InitializeEstimator( nh, ph );

		GetParam( ph, "debug", _debug, false );
		if( _debug )
		{
			std::string debugTopicName = ph.resolveName( "image_debug" );
			ROS_INFO_STREAM( "Displaying debug output on topic" << debugTopicName );
			_debugPub = _imageTrans.advertise( "image_debug", 1 );
		}
	}

private:

	image_transport::ImageTransport _imageTrans;
	image_transport::CameraSubscriber _imageSub;

	bool _debug;
	image_transport::Publisher _debugPub;

	double _scale;
	InterestPointDetector::Ptr _detector;
	InterestPointTracker::Ptr _tracker;
	RigidEstimator::Ptr _estimator;
	MotionPredictor _predictor;
	VelocityPublisher _velPub;

	struct FrameInfo
	{
		ros::Time time;
		std::string frameId;
		cv::Mat frame;
		InterestPoints points;
		CameraCalibration calibration;
	};

	FrameInfo _keyFrame;
	FrameInfo _lastFrame;
	PoseSE2 _lastToKey;
	size_t _originalNumKeypoints; // Number of keypoints on detection

	NumericParam _minNumKeypoints;
	NumericParam _redetectThresh;
	NumericParam _minInlierRatio;
	NumericParam _maxFrameDt;

	void InitializeDetector( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		std::string detectorType;
		GetParamRequired( ph, "detector/type", detectorType );

		ros::NodeHandle detectorHandle( "~detector" );
		if( detectorType == "corner" )
		{
			ROS_INFO_STREAM( "Initializing corner-based point detector" );
			_detector = std::make_shared<CornerPointDetector>( nh, detectorHandle );
		}
		else if( detectorType == "fixed" )
		{
			ROS_INFO_STREAM( "Initializing fixed-grid point detector" );
			_detector = std::make_shared<FixedPointDetector>( nh, detectorHandle );
		}
		else if( detectorType == "FAST" )
		{
			ROS_INFO_STREAM( "Initializing FAST point detector" );
			_detector = std::make_shared<FASTPointDetector>( nh, detectorHandle );
		}
		else
		{
			ROS_ERROR_STREAM( "Invalid point detector type: " + detectorType );
		}
	}

	void InitializeTracker( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		std::string trackerType;
		GetParamRequired( ph, "tracker/type", trackerType );
		ros::NodeHandle trackerHandle( "~tracker" );
		if( trackerType == "lucas_kanade" )
		{
			ROS_INFO_STREAM( "Initializing Lucas-Kanade point tracker" );
			_tracker = std::make_shared<LKPointTracker>( nh, trackerHandle );
		}
		else
		{
			ROS_ERROR_STREAM( "Invalid point tracker type." );
		}
	}

	void InitializeEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		ros::NodeHandle estimatorHandle( "~estimator" );
		ROS_INFO_STREAM( "Initializing robust rigid motion estimator" );
		_estimator = std::make_shared<RigidEstimator>( nh, estimatorHandle );
	}

	void ImageCallback( const sensor_msgs::ImageConstPtr& msg,
	                    const sensor_msgs::CameraInfoConstPtr& info_msg )
	{
		cv_bridge::CvImageConstPtr frame;
		try
		{
			frame = cv_bridge::toCvShare( msg, "mono8" );
		}
		catch( cv_bridge::Exception& e )
		{
			ROS_ERROR( "cv_bridge exception: %s", e.what() );
			return;
		}

		FrameInfo current;
		current.time = msg->header.stamp;
		current.frameId = msg->header.frame_id;
		current.frame = frame->image;
		current.calibration = CameraCalibration( info_msg->header.frame_id, *info_msg );

		// TODO A more complete check of intrinsics validity
		if( current.calibration.GetFx() == 0.0 || std::isnan( current.calibration.GetFx() ) )
		{
			ROS_WARN_STREAM( "Camera " << msg->header.frame_id << " has invalid intrinsics." );
			return;
		}
		current.calibration.SetScale( current.frame.size() );
		ProcessFrame( current );
	}

	void ProcessFrame( FrameInfo& current )
	{
		PoseSE2 currToKey;
		if( !CheckInitialization( current ) ||
		    !PredictAndTrack( current ) ||
		    !EstimateMotion( current, currToKey ) )
		{
			if( !_lastFrame.frame.empty() )
			{
				ROS_INFO_STREAM( "Retrying tracking with previous frame as keyframe" );
				SetKeyFrame( _lastFrame );
				_lastFrame.frame = cv::Mat();
				ProcessFrame( current );
			}
			else
			{
				ROS_INFO_STREAM( "No previous frame, setting current to keyframe and bailing" );
				SetKeyFrame( current );
			}
		}
		else
		{
			PoseSE3 currToKey3;
			CameraToStandard( currToKey, currToKey3 );
			double imgWidth = (double) current.frame.size().width;
			double scale = _scale / imgWidth;
			_velPub.ReportPose( current.time, current.frameId, currToKey3, scale );
			_lastToKey = currToKey;
		}

		if( _debug )
		{
			const PoseSE3::TangentVector& vel3 = _velPub.GetLastVelocity();
			PoseSE2::TangentVector vel;
			vel << vel3( 0 ), vel3( 1 ), vel3( 0 );
			Visualize( current, currToKey, vel );
		}
	}

	void Visualize( FrameInfo& current, PoseSE2& pose, PoseSE2::TangentVector& vel )
	{
		if( _keyFrame.frame.empty() ) { return; }

		unsigned int width = current.frame.cols;
		unsigned int height = current.frame.rows;
		cv::Mat visImage( height, 2 * width, CV_8UC3 );
		cv::Mat visLeft( visImage, cv::Rect( 0, 0, width, height ) );
		cv::Mat visRight( visImage, cv::Rect( width, 0, width, height ) );
		cv::cvtColor( _keyFrame.frame, visLeft, cv::COLOR_GRAY2BGR );
		cv::cvtColor( current.frame, visRight, cv::COLOR_GRAY2BGR );

		// Show current frame extents on keyframe
		FixedMatrixType<3, 4> corners;
		corners << 1, 1, width - 1, width - 1,
		1, height - 1, height - 1, 1,
		1, 1, 1, 1;
		FixedMatrixType<2, 4> warped = (pose.ToMatrix() * corners).colwise().hnormalized();

		std::vector<cv::Point2f> cornerPoints;
		for( unsigned int i = 0; i < 4; ++i )
		{
			cornerPoints.emplace_back( warped( 0, i ), warped( 1, i ) );
		}
		cv::line( visLeft, cornerPoints[0], cornerPoints[1], cv::Scalar( 0, 255, 0 ) );
		cv::line( visLeft, cornerPoints[1], cornerPoints[2], cv::Scalar( 0, 255, 0 ) );
		cv::line( visLeft, cornerPoints[2], cornerPoints[3], cv::Scalar( 0, 255, 0 ) );
		cv::line( visLeft, cornerPoints[3], cornerPoints[0], cv::Scalar( 0, 255, 0 ) );

		// Display keypoints
		cv::Point2d offset( width, 0 );
		for( unsigned int i = 0; i < _keyFrame.points.size(); i++ )
		{
			cv::circle( visLeft, _keyFrame.points[i], 2, cv::Scalar( 0, 255, 0 ), -1, 8 );
		}
		for( unsigned int i = 0; i < _lastFrame.points.size(); i++ )
		{
			cv::circle( visRight, _lastFrame.points[i], 2, cv::Scalar( 0, 255, 0 ), -1, 8 );
		}

		std_msgs::Header header;
		header.stamp = _lastFrame.time;
		header.frame_id = current.frameId;
		cv_bridge::CvImage vimg( header, "bgr8", visImage );
		_debugPub.publish( vimg.toImageMsg() );
	}

	bool CheckInitialization( FrameInfo& current )
	{
		// Initialization catch
		double timeSinceLastFrame = ( current.time - _lastFrame.time ).toSec();
		if( _keyFrame.frame.empty() ||
		    timeSinceLastFrame > _maxFrameDt ||
		    _keyFrame.points.size() < _minNumKeypoints )
		{
			return false;
		}
		return true;
	}

	bool PredictAndTrack( FrameInfo& current )
	{
		// Track interest points into current frame
		// TODO Scale
		PoseSE2 disp = _predictor.PredictMotion( _lastFrame.time,
		                                         current.time,
		                                         current.frameId, // TODO
		                                         1.0 );
		PoseSE2 guessPose = _lastToKey * disp;

		InterestPoints normPoints = UndistortAndNormalizePoints( current.points,
		                                                         current.calibration );
		InterestPoints transNormPoints = TransformPoints( normPoints, guessPose );
		current.points = DistortAndUnnormalizePoints( transNormPoints,
		                                              current.calibration );

		size_t numCurrentPoints = current.points.size();
		if( !_tracker->TrackInterestPoints( _keyFrame.frame, _keyFrame.points,
		                                    current.frame, current.points ) )
		{
			ROS_INFO_STREAM( "Tracking failed!" );
			return false;
		}

		// Failure if not enough inliers in tracking
		size_t numTrackingInliers = current.points.size();
		size_t minTrackingInliers = std::max( std::round( numCurrentPoints * _minInlierRatio ),
		                                      (double) _minNumKeypoints );
		if( numTrackingInliers < minTrackingInliers )
		{
			ROS_INFO_STREAM( numTrackingInliers << " inliers after tracking less than min " <<
			                 minTrackingInliers );
			return false;
		}
		return true;
	}

	bool EstimateMotion( FrameInfo& current, PoseSE2& pose )
	{
		// Estimate motion between frames
		InterestPoints normKey = UndistortAndNormalizePoints( _keyFrame.points,
		                                                      _keyFrame.calibration );
		InterestPoints currKey = UndistortAndNormalizePoints( current.points,
		                                                      current.calibration );
		std::vector<unsigned int> inlierInds;
		if( !_estimator->EstimateMotion( _keyFrame.points,
		                                 current.points,
		                                 inlierInds,
		                                 pose ) )
		{
			ROS_WARN_STREAM( "Could not estimate motion between frames." );
			return false;
		}

		InterestPoints keyInlier;
		InterestPoints currInlier;
		BOOST_FOREACH( unsigned int i, inlierInds )
		{
			keyInlier.push_back( _keyFrame.points[i] );
			currInlier.push_back( current.points[i] );
		}
		_keyFrame.points = keyInlier;
		current.points = currInlier;

		// Check number of inliers
		size_t numMotionInliers = current.points.size();
		size_t minMotionInliers = std::max( std::round( inlierInds.size() * _minInlierRatio ),
		                                    (double) _minNumKeypoints );
		if( numMotionInliers <= minMotionInliers )
		{
			ROS_INFO_STREAM( numMotionInliers << " inliers after motion estimation less than "
			                                  << minMotionInliers << ". Resetting keyframe." );
			SetKeyFrame( current );
			return false;
		}

		// TODO Do we still want this logic?
		// // Check if we need to redetect
		// double inlierRatio = numMotionInliers / (double) _originalNumKeypoints;
		// if( inlierRatio <= _redetectThresh )
		// {
		//  ROS_INFO_STREAM( _keyFrame.points.size() << " inliers less than "
		//                                          << _redetectThresh * _originalNumKeypoints
		//                                          << ". Resetting keyframe." );
		// }

		return true;
	}

	void SetKeyFrame( const FrameInfo& current )
	{
		_keyFrame = current;
		_keyFrame.points = _detector->FindInterestPoints( _keyFrame.frame );
		_originalNumKeypoints = _keyFrame.points.size();
		if( _originalNumKeypoints < _minNumKeypoints )
		{
			ROS_INFO_STREAM( "Found " << _originalNumKeypoints << " keypoints, less than min: " <<
			                 (unsigned int) _minNumKeypoints );
			_keyFrame.frame = cv::Mat();
			_keyFrame.points.clear();
			return;
		}
		// ROS_INFO_STREAM( "Found " << _originalNumKeypoints << " keypoints in new keyframe" );

		_lastFrame = _keyFrame;
		_lastToKey = PoseSE2();
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "visual_odometry_node" );
	ros::NodeHandle nh, ph( "~" );
	SparseVONode vo( nh, ph );
	ros::spin();
	return 0;
}
