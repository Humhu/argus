#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include "odoflow/ECCDenseTracker.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "paraset/ParameterManager.hpp"
#include "camplex/FiducialCommon.h"
#include "extrinsics_array/ExtrinsicsInterface.h"
#include "argus_utils/geometry/VelocityIntegrator.hpp"
#include "argus_utils/random/MultivariateGaussian.hpp"

using namespace argus;

class DenseVONode
{
EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:

	DenseVONode( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: _imageTrans( nh ), _extrinsics( nh, ph ), _tracker( nh, ph )
	{
		_imageSub = _imageTrans.subscribe( "image",
		                                   2,
		                                   boost::bind( &DenseVONode::ImageCallback, this, _1 ) );
		_twistPub = ph.advertise<geometry_msgs::TwistStamped>( "velocity_raw", 10 );

		_pyramidDepth.InitializeAndRead( ph, 0, "pyramid_depth", "Number of image pyramids" );
		_pyramidDepth.AddCheck<IntegerValued>();
		_pyramidDepth.AddCheck<GreaterThanOrEqual>( 0 );

		_maxDisplacement.InitializeAndRead( ph, 0.2, "max_displacement", "Max keyframe displacement as ratio of image dimension" );
		_maxDisplacement.AddCheck<GreaterThanOrEqual>( 0.0 );
		_maxDisplacement.AddCheck<LessThanOrEqual>( 1.0 );

		_maxPredictEntropy.InitializeAndRead( ph, 1.0, "max_predict_entropy", "Max predict uncertainty entropy" );

		_minTimeDelta.InitializeAndRead( ph, 0.0, "min_time_delta", "Min time between frames to estimate velocity" );
		_minTimeDelta.AddCheck<GreaterThanOrEqual>( 0.0 );

		GetParamRequired( ph, "scale", _scale );
		GetParam( ph, "min_pixel_variance", _minPixelVariance, 10.0 );

		GetParam( ph, "enable_prediction", _enablePrediction, false );
		if( _enablePrediction )
		{
			std::string predictionMode;
			GetParam<std::string>( ph, "prediction_mode", predictionMode, "odom" );
			if( predictionMode == "odometry" )
			{
				_trueSub = nh.subscribe( "odom", 10, &DenseVONode::OdomCallback, this );			
			}
			else if( predictionMode == "twist_stamped" )
			{
				_trueSub = nh.subscribe( "odom", 10, &DenseVONode::TwistStampedCallback, this );			
			}
			else
			{
				throw std::invalid_argument("Unknown prediction mode");
			}
		}

		GetParam( ph, "debug", _debug, false );
		if( _debug )
		{
			_debugPub = _imageTrans.advertise( "image_debug", 1 );
			GetParam( ph, "vis_arrow_scale", _visArrowScale, 1.0 );
		}
	}

	void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
	{
		PoseSE3::TangentVector vel = MsgToTangent( msg->twist.twist );
		PoseSE3::CovarianceMatrix cov;
		ParseMatrix( msg->twist.covariance, cov );
		_velIntegrator.BufferInfo( msg->header.stamp.toSec(), vel, cov );
		_odomFrame = msg->child_frame_id;
	}

	void TwistStampedCallback( const geometry_msgs::TwistStamped::ConstPtr& msg )
	{
		PoseSE3::TangentVector vel = MsgToTangent( msg->twist );
		PoseSE3::CovarianceMatrix cov = PoseSE3::CovarianceMatrix::Zero();
		_velIntegrator.BufferInfo( msg->header.stamp.toSec(), vel, cov );
		_odomFrame = msg->header.frame_id;		
	}

	void ImageCallback( const sensor_msgs::ImageConstPtr& msg )
	{
		cv_bridge::CvImageConstPtr frame;
		try
		{
			frame = cv_bridge::toCvShare( msg, "mono8" );
		}
		catch( cv_bridge::Exception& e )
		{
			ROS_ERROR( "VisualOdometryNode cv_bridge exception: %s", e.what() );
			return;
		}

		ProcessImage( frame->image, msg->header );
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

	bool ValidateKeyPyramid( const std::vector<cv::Mat>& pyramid )
	{
		// If image size has changed, fail
		if( _keyPyramid[0].size() != pyramid[0].size() )
		{
			return false;
		}

		// If depth has changed, add more depths
		unsigned int depth = pyramid.size();

		if( _keyPyramid.size() <= (depth + 1) )
		{
			_keyPyramid.resize( depth + 1 );
			for( unsigned int i = _keyPyramid.size() - 1; i < depth; ++i )
			{
				cv::pyrDown( _keyPyramid[i], _keyPyramid[i + 1] );
			}
		}
		return true;
	}

	void SetKeyframe( const std::vector<cv::Mat>& pyramid,
	                  const ros::Time& time )
	{
		_keyPyramid = pyramid;
		_keyTime = time;
		_lastPose = PoseSE2();
		_lastTime = time;
		_lastVelPose = PoseSE2();
		_lastVelTime = time;
	}

	PoseSE2 PredictMotion( const ros::Time& currTime, const std::string& camFrame )
	{
		// Can't predict motion if we haven't received any odom messages
		if( !_enablePrediction || _odomFrame.empty() )
		{
			// ROS_WARN_STREAM( "No odometry messages received" );
			return PoseSE2();
		}

		PoseSE3 odomDisp;
		PoseSE3::CovarianceMatrix odomCov;
		if( !_velIntegrator.Integrate( _lastTime.toSec(), currTime.toSec(),
		                               odomDisp, odomCov ) )
		{
			ROS_WARN_STREAM( "Could not predict motion from " << _keyTime << " to " << currTime );
			odomDisp = PoseSE3(); // Unnecessary
			odomCov = PoseSE3::CovarianceMatrix::Identity(); // TODO
		}

		PoseSE3 odomToCam;
		PoseSE3 camDisp;
		PoseSE3::CovarianceMatrix guessCov; // TODO Use the covariance?
		try
		{
			odomToCam = _extrinsics.GetExtrinsics( _odomFrame,
			                                       camFrame,
			                                       currTime );
		}
		catch( ExtrinsicsException& e )
		{
			ROS_WARN_STREAM( "Could not get extrinsics: " << e.what() );
			return PoseSE2();
			// guessCov = odomCov;
		}

		camDisp = odomToCam * odomDisp * odomToCam.Inverse();
		guessCov = TransformCovariance( odomCov, odomToCam );
		// HACK
		MatrixType subCov( 3, 3 );
		std::vector<unsigned int> inds = {1, 2, 3};
		GetSubmatrix( guessCov, subCov, inds, inds );
		double entropy = GaussianEntropy( subCov );
		if( entropy > _maxPredictEntropy )
		{
			ROS_WARN_STREAM( "Motion prediction entropy exceeds limit! Using default prior" );
			return PoseSE2();
		}

		PoseSE2 ret;
		StandardToCamera( camDisp, ret );

		unsigned int imgWidth = _keyPyramid[0].size().width; // NOTE Hack?
		PoseSE2::TangentVector logRet = PoseSE2::Log( ret );
		logRet.head<2>() *= imgWidth / _scale;
		ret = PoseSE2::Exp( logRet );

		return ret;
	}

	void Visualize( const cv::Mat& curr, const std_msgs::Header& header )
	{
		if( !_debug || _keyPyramid.empty() ) { return; }

		const cv::Mat& key = _keyPyramid[0];
		unsigned int width = key.cols;
		unsigned int height = key.rows;

		cv::Mat visImage( height, 2 * width, CV_8UC3 );
		cv::Mat visLeft( visImage, cv::Rect( 0, 0, width, height ) );
		cv::Mat visRight( visImage, cv::Rect( width, 0, width, height ) );
		//key.copyTo( visLeft );
		//curr.copyTo( visRight );
		cv::cvtColor( key, visLeft, cv::COLOR_GRAY2BGR );
		cv::cvtColor( curr, visRight, cv::COLOR_GRAY2BGR );

		FixedMatrixType<3, 4> corners;
		corners << 1, 1, width-1, width-1,
		1, height-1, height-1, 1,
		1, 1, 1, 1;
		FixedMatrixType<2, 4> warped = (_lastPose.ToMatrix() * corners).colwise().hnormalized();

		std::vector<cv::Point2f> cornerPoints;
		for( unsigned int i = 0; i < 4; ++i )
		{
			cornerPoints.emplace_back( warped( 0, i ), warped( 1, i ) );
		}
		cv::line( visLeft, cornerPoints[0], cornerPoints[1], cv::Scalar( 0, 255, 0 ) );
		cv::line( visLeft, cornerPoints[1], cornerPoints[2], cv::Scalar( 0, 255, 0 ) );
		cv::line( visLeft, cornerPoints[2], cornerPoints[3], cv::Scalar( 0, 255, 0 ) );
		cv::line( visLeft, cornerPoints[3], cornerPoints[0], cv::Scalar( 0, 255, 0 ) );
		
		cv::Point2f center( height/2, width/2 );
		cv::Point2f arrowEnd = cv::Point2f( _lastVel[0], _lastVel[1] ) * _visArrowScale;
		cv::arrowedLine( visRight, center, arrowEnd + center, cv::Scalar( 0, 255, 0 ) );

		cv_bridge::CvImage vimg( header, "bgr8", visImage );
		_debugPub.publish( vimg.toImageMsg() );
	}

	void ProcessImage( const cv::Mat& img, const std_msgs::Header& header )
	{
		// Check for image having too little variation
		cv::Scalar mean, stddev;
		cv::meanStdDev( img, mean, stddev );
                if( stddev[0] < _minPixelVariance )
		  {
		    ROS_WARN_STREAM( "Pixel variance " << stddev[0] << " less than min " << _minPixelVariance );
		    return;
		  }


		std::vector<cv::Mat> currPyramid;
		unsigned int depth = _pyramidDepth;
		CreatePyramid( img, currPyramid, depth );
		const ros::Time& currTime = header.stamp;

		// If initialization and validation fails, reset keyframe
		if( _keyPyramid.empty() || !ValidateKeyPyramid( currPyramid ) )
		{
			SetKeyframe( currPyramid, currTime );
			Visualize( currPyramid[0], header );
			return;
		}

		// Check for negative dt from clock reset
		double dt = (header.stamp - _lastTime).toSec();
		if( dt < 0 )
		{
			ROS_WARN_STREAM( "Negative dt detected. Resetting keyframe..." );
			_velIntegrator.Reset(); // Need to dump integration data
			SetKeyframe( currPyramid, currTime );
			Visualize( currPyramid[0], header );
			return;
		}

		PoseSE2 disp = PredictMotion( currTime, header.frame_id );
		PoseSE2 currToKey = _lastPose * disp;

		// Check for field-of-view moving too far from keyframe
		FixedVectorType<3> dispVec = currToKey.ToVector();
		double dispX = dispVec( 0 );
		double dispY = dispVec( 1 );
		double r = std::sqrt( dispX * dispX + dispY * dispY );
		double maxR = _maxDisplacement * _keyPyramid[0].size().width;
		if( r > maxR )
		{
			ROS_INFO_STREAM( "Predicted displacement " << r << " larger than allowed "
			                                           << r << ". Resetting keyframe..." );
			SetKeyframe( currPyramid, currTime );
			Visualize( currPyramid[0], header );
			return;
		}

		PoseSE2::TangentVector logPose = PoseSE2::Log( currToKey );
		logPose.head<2>() = logPose.head<2>() / std::pow( 2, _pyramidDepth );
		for( int i = _pyramidDepth; i >= 0; --i )
		{
			currToKey = PoseSE2::Exp( logPose );

			const cv::Mat& currImg = currPyramid[i];
			const cv::Mat& prevImg = _keyPyramid[i];

			// ROS_INFO_STREAM( "Depth: " << i << " predicted disp: " << currToKey );
			if( !_tracker.TrackImages( prevImg, currImg, currToKey ) )
			{
				ROS_INFO_STREAM( "Tracking failed! Resetting keyframe..." );
				SetKeyframe( currPyramid, currTime );
				Visualize( currPyramid[0], header );
				return;
			}
			// ROS_INFO_STREAM( "Depth: " << i << " found disp: " << currToKey );

			// Next level will be twice as much resolution, so we double the translation prediction
			logPose = PoseSE2::Log( currToKey );
			logPose.head<2>() *= 2;
		}

		Visualize( currPyramid[0], header );

		_lastPose = currToKey;
		_lastTime = currTime;

		// Don't compute velocities if too little time passed since last differentiation
		double velDt = (currTime - _lastVelTime).toSec();
		if( velDt < _minTimeDelta ) { return; }

		// Compute displacement relative to last image, not keyframe
		disp = _lastVelPose.Inverse() * currToKey;
		_lastVelTime = currTime;
		_lastVelPose = currToKey;
		_lastVel = PoseSE2::Log( disp ) / velDt;

		geometry_msgs::TwistStamped tmsg;
		tmsg.header = header;
		double imgWidth = (double) img.size().width;
		PoseSE3 disp3;
		CameraToStandard( disp, disp3 );

		PoseSE3::TangentVector dvel = PoseSE3::Log( disp3 ) / velDt;
		// NOTE Don't scale rotations by image scale!
		dvel.head<3>() = dvel.head<3>() * _scale / imgWidth;
		tmsg.twist = TangentToMsg( dvel );
		_twistPub.publish( tmsg );
	}

private:

	image_transport::ImageTransport _imageTrans;
	image_transport::Subscriber _imageSub;

	bool _debug;
	image_transport::Publisher _debugPub;
  double _visArrowScale;

	ros::Publisher _twistPub;

	bool _enablePrediction;
	ros::Subscriber _trueSub;
	std::string _odomFrame;
	VelocityIntegratorSE3 _velIntegrator;
  double _minPixelVariance;

	// Keyframe
	std::vector<cv::Mat> _keyPyramid;
	ros::Time _keyTime;
	PoseSE2 _lastPose;
	ros::Time _lastTime;
	ros::Time _lastVelTime;
	PoseSE2 _lastVelPose;
	PoseSE2::TangentVector _lastVel;

	double _scale;
	NumericParam _pyramidDepth;
	NumericParam _maxDisplacement;
	NumericParam _maxPredictEntropy;
	NumericParam _minTimeDelta;

	ExtrinsicsInterface _extrinsics;

	ECCDenseTracker _tracker;
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
