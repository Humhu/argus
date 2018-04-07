#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include "odoflow/ECCDenseTracker.h"
#include "odoflow/MotionPredictor.h"
#include "odoflow/VelocityPublisher.h"

#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "paraset/ParameterManager.hpp"
#include "camplex/FiducialCommon.h"

using namespace argus;

class DenseVONode
{
public:

	DenseVONode( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: _imageTrans( nh ), _tracker( nh, ph ),
		_predictor( nh, ph ), _velPub( nh, ph )
	{
		// Initialize all runtime parameters
		_pyramidDepth.InitializeAndRead( ph, 0, "pyramid_depth",
		                                 "Number of image pyramids" );
		_pyramidDepth.AddCheck<IntegerValued>();
		_pyramidDepth.AddCheck<GreaterThanOrEqual>( 0 );

		_maxDisplacement.InitializeAndRead( ph, 0.2, "max_displacement",
		                                    "Max keyframe displacement as ratio of image dimension" );
		_maxDisplacement.AddCheck<GreaterThanOrEqual>( 0.0 );
		_maxDisplacement.AddCheck<LessThanOrEqual>( 1.0 );


		_minPixelVariance.InitializeAndRead( ph, 10.0, "min_pixel_variance",
		                                     "Minimum raw intensity variance across image" );
		_minPixelVariance.AddCheck<GreaterThanOrEqual>( 0.0 );

		GetParamRequired( ph, "scale", _scale );

		unsigned int buffSize;
		GetParam<unsigned int>( ph, "buffer_size", buffSize, 2 );
		_imageSub = _imageTrans.subscribe( "image",
		                                   buffSize,
		                                   &DenseVONode::ImageCallback,
		                                   this );

		GetParam( ph, "debug", _debug, false );
		if( _debug )
		{
			std::string debugTopicName = ph.resolveName( "image_debug" );
			ROS_INFO_STREAM( "Displaying debug output on topic" << debugTopicName );
			_debugPub = _imageTrans.advertise( "image_debug", 1 );
			GetParam( ph, "vis_arrow_scale", _visArrowScale, 1.0 );
		}
	}

private:

	image_transport::ImageTransport _imageTrans;
	image_transport::Subscriber _imageSub;

	bool _debug;
	image_transport::Publisher _debugPub;
	double _visArrowScale;

	struct FrameInfo
	{
		std::vector<cv::Mat> pyramid;
		ros::Time time;
		std::string frameId;
	};


	// Keyframe
	FrameInfo _keyFrame;
	FrameInfo _lastFrame;
	PoseSE2 _lastToKey;

	double _scale;
	NumericParam _pyramidDepth;
	NumericParam _maxDisplacement;
	NumericParam _minPixelVariance;

	ECCDenseTracker _tracker;
	MotionPredictor _predictor;
	VelocityPublisher _velPub;


	void ImageCallback( const sensor_msgs::ImageConstPtr& msg )
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

		// Check for image having too little variation
		if( !HasEnoughVariance( frame->image ) ) { return; }

		FrameInfo current;
		current.time = msg->header.stamp;
		current.frameId = msg->header.frame_id;
		CreatePyramid( frame->image, current.pyramid, (unsigned int) _pyramidDepth );
		ProcessFrame( current );
	}

	void ProcessFrame( FrameInfo& current )
	{
		// If initialization and validation fails, reset keyframe
		PoseSE2 currToKey;
		if( !CheckInitialization( current ) ||
		    !PredictMotion( current, currToKey ) ||
		    !PerformTracking( current, currToKey ) )
		{
			if( !_lastFrame.pyramid.empty() )
			{
				ROS_INFO_STREAM( "Retrying tracking with previous frame as keyframe" );
				SetKeyframe( _lastFrame );
				_lastFrame.pyramid.clear();
				ProcessFrame( current );
			}
			else
			{
				ROS_INFO_STREAM( "No previous frame, setting current to keyframe and bailing" );
				SetKeyframe( current );
			}
		}
		// Else publish velocity
		else
		{
			PoseSE3 currToKey3;
			CameraToStandard( currToKey, currToKey3 );
			double imgWidth = (double) current.pyramid[0].size().width;
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

	bool HasEnoughVariance( const cv::Mat& img )
	{
		cv::Scalar mean, stddev;
		cv::meanStdDev( img, mean, stddev );
		bool hasEnough = stddev[0] >= _minPixelVariance;
		if( !hasEnough )
		{
			ROS_WARN_STREAM( "Pixel variance " << stddev[0] << " less than min " << _minPixelVariance );
			return false;
		}
		return true;
	}

	void CreatePyramid( const cv::Mat& img, std::vector<cv::Mat>& pyr,
	                    unsigned int depth )
	{
		pyr.clear();
		pyr.resize( depth + 1 );
		pyr[0] = img;
		for( unsigned int i = 0; i < depth; ++i )
		{
			cv::pyrDown( pyr[i], pyr[i + 1] );
		}
	}

	void SetKeyframe( FrameInfo& current )
	{
		_keyFrame = current;
		_lastFrame.pyramid.clear();
		_velPub.Reset( current.time );
		_lastToKey = PoseSE2();
	}

	void Visualize( FrameInfo& current, PoseSE2& pose,
	                PoseSE2::TangentVector& vel )
	{
		if( _keyFrame.pyramid.empty() ) { return; }

		const cv::Mat& key = _keyFrame.pyramid[0];
		unsigned int width = key.cols;
		unsigned int height = key.rows;

		// Show keyframe and current frame side by side
		cv::Mat visImage( height, 2 * width, CV_8UC3 );
		cv::Mat visLeft( visImage, cv::Rect( 0, 0, width, height ) );
		cv::Mat visRight( visImage, cv::Rect( width, 0, width, height ) );
		cv::cvtColor( key, visLeft, cv::COLOR_GRAY2BGR );
		cv::cvtColor( current.pyramid[0], visRight, cv::COLOR_GRAY2BGR );

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

		// Show velocity arrow
		cv::Point2f center( height / 2, width / 2 );
		cv::Point2f arrowEnd = cv::Point2f( vel[0], vel[1] ) * _visArrowScale;
		cv::arrowedLine( visRight, center, arrowEnd + center, cv::Scalar( 0, 255, 0 ) );

		std_msgs::Header header;
		header.stamp = current.time;
		header.frame_id = current.frameId;
		cv_bridge::CvImage vimg( header, "bgr8", visImage );
		_debugPub.publish( vimg.toImageMsg() );
	}

	bool CheckInitialization( FrameInfo& current )
	{
		if( _keyFrame.pyramid.empty() ||
		    _keyFrame.pyramid[0].size() != current.pyramid[0].size() ||
		    _keyFrame.frameId != current.frameId )
		{
			return false;
		}

		// If time negative (due to sim) reset motion predictor
		double dt = (current.time - _lastFrame.time).toSec();
		if( dt < 0 )
		{
			ROS_WARN_STREAM( "Negative dt detected" );
			_predictor.Reset();
			return false;
		}

		// If depth has changed, add more depths
		unsigned int depth = current.pyramid.size() - 1;
		if( _keyFrame.pyramid.size() <= (depth + 1) )
		{
			_keyFrame.pyramid.resize( depth + 1 );
			for( unsigned int i = _keyFrame.pyramid.size() - 1; i < depth; ++i )
			{
				cv::pyrDown( _keyFrame.pyramid[i], _keyFrame.pyramid[i + 1] );
			}
		}
		return true;
	}

	bool PredictMotion( FrameInfo& current, PoseSE2& currToKey )
	{
		double predScale = _keyFrame.pyramid[0].size().width / _scale;
		PoseSE2 disp = _predictor.PredictMotion( _lastFrame.time,
		                                         current.time,
		                                         current.frameId,
		                                         predScale );
		currToKey = _lastToKey * disp;

		FixedVectorType<3> ctkVec = currToKey.ToVector();
		double x = ctkVec( 0 );
		double y = ctkVec( 1 );
		double r = std::sqrt( x * x + y * y );
		// NOTE We use the image width to adjust for resizing
		double maxR = _maxDisplacement * _keyFrame.pyramid[0].size().width;
		if( r > maxR )
		{
			ROS_INFO_STREAM( "Predicted displacement " << r <<
			                 " larger than allowed " << maxR );
			return false;
		}
		return true;
	}

	bool PerformTracking( FrameInfo& current, PoseSE2& currToKey )
	{
		PoseSE2::TangentVector logPose = PoseSE2::Log( currToKey );
		unsigned int depth = current.pyramid.size() - 1;
		logPose.head<2>() = logPose.head<2>() / std::pow( 2, depth );

		for( int i = depth; i >= 0; --i )
		{
			currToKey = PoseSE2::Exp( logPose );

			const cv::Mat& currImg = current.pyramid[i];
			const cv::Mat& prevImg = _keyFrame.pyramid[i];

			if( !_tracker.TrackImages( prevImg, currImg, currToKey ) )
			{
				return false;
			}
			// ROS_INFO_STREAM( "Depth: " << i << " found disp: " << currToKey );
			// Next level will be twice as much resolution, so we double the translation prediction
			if( i > 0 )
			{
				logPose = PoseSE2::Log( currToKey );
				logPose.head<2>() *= 2;
			}
		}
		return true;
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "image_viewer" );

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle( "~" );
	DenseVONode ecc( nodeHandle, privHandle );

	ros::spin();

	return 0;
}
