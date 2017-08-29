#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

#include <dynamic_reconfigure/server.h>
#include "dispair/DisparityVOConfig.h"
#include "dispair/ImageSync.h"

#include "bpvo/vo.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/random/MultivariateGaussian.hpp"
#include "argus_utils/geometry/VelocityIntegrator.hpp"
#include "argus_utils/synchronization/SynchronizationTypes.h"
#include "extrinsics_array/ExtrinsicsInterface.h"
#include "argus_utils/geometry/PoseSE3.h"

#include "camplex/FiducialCommon.h"
#include "camplex/CameraCalibration.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace argus;

// A node that runs BPVO on monocular image + disparity inputs
class DisparityVO
	: public DisparityImageSync
{
public:

	DisparityVO( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: DisparityImageSync( nh, ph ), _extrinsics( nh, ph )
	{
		_posePub = ph.advertise<geometry_msgs::PoseStamped>( "pose", 10 );
		_twistPub = ph.advertise<geometry_msgs::TwistStamped>( "twist", 10 );

		GetParam( ph, "camera_x_forward", _camXForward, true );

		GetParam( ph, "max_predict_entropy", _maxPredictEntropy, 3.0 );
		GetParam( ph, "enable_prediction", _enablePrediction, false );
		if( _enablePrediction )
		{
			std::string predictionMode;
			GetParam<std::string>( ph, "prediction_mode", predictionMode );
			if( predictionMode == "odometry" )
			{
				_trueSub = nh.subscribe( "truth", 10, &DisparityVO::OdomCallback, this );			
			}
			else if( predictionMode == "twist_stamped" )
			{
				_trueSub = nh.subscribe( "truth", 10, &DisparityVO::TwistStampedCallback, this );			
			}
			else
			{
				throw std::invalid_argument("Unknown prediction mode");
			}
		}

		dynamic_reconfigure::Server<dispair::DisparityVOConfig>::CallbackType cb;
		cb = boost::bind( &DisparityVO::ReconfigureCallback, this, _1, _2 );
		_voConfigServer.setCallback( cb );
	}

	// TODO Group the velocity prediction functionality into an abstract superclass?
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

	PoseSE3 PredictMotion( const ros::Time& currTime, const std::string& camFrame )
	{
		// Can't predict motion if we haven't received any odom messages
		if( !_enablePrediction || _odomFrame.empty() )
		{
			// ROS_WARN_STREAM( "No odometry messages received" );
			return PoseSE3();
		}

		PoseSE3 odomDisp;
		PoseSE3::CovarianceMatrix odomCov;
		if( !_velIntegrator.Integrate( _lastTime.toSec(), currTime.toSec(),
		                               odomDisp, odomCov ) )
		{
			ROS_WARN_STREAM( "Could not predict motion from " << _lastTime << " to " << currTime );
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
			return PoseSE3();
		}

		camDisp = odomToCam * odomDisp * odomToCam.Inverse();
		guessCov = TransformCovariance( odomCov, odomToCam );
		double entropy = GaussianEntropy( guessCov );
		if( entropy > _maxPredictEntropy )
		{
			ROS_WARN_STREAM( "Motion prediction entropy exceeds limit! Using default prior" );
			return PoseSE3();
		}

		static PoseSE3 xToZ( 0, 0, 0, -0.5, -0.5, 0.5, -0.5 );
		if( _camXForward )
		{
			camDisp = xToZ * camDisp * xToZ.Inverse();
		}

		return camDisp;
	}

private:

	struct ImageData
	{
		cv_bridge::CvImageConstPtr image;
		cv_bridge::CvImageConstPtr disparity;
		boost::shared_ptr<void const> disparityObject;
		CameraCalibration model;
		double baseline;
	};

	std::shared_ptr<bpvo::VisualOdometry> _vo;
	bpvo::AlgorithmParameters _voParams;
	dynamic_reconfigure::Server<dispair::DisparityVOConfig> _voConfigServer;

	PoseSE3 _integratedPose;

	ros::Publisher _posePub;
	ros::Publisher _twistPub;
	ros::Subscriber _trueSub;

	bool _camXForward;
	ExtrinsicsInterface _extrinsics;

	bool _enablePrediction;
	std::string _odomFrame;
	double _maxPredictEntropy;
	VelocityIntegratorSE3 _velIntegrator;
	ros::Time _lastTime;

	void ReconfigureCallback( dispair::DisparityVOConfig& config,
	                          unsigned int level )
	{
		_vo.reset();
		_voParams.numPyramidLevels = config.num_pyramid_levels;
		_voParams.minImageDimensionForPyramid = config.min_pyramid_img_dim;

		// TODO Give feedback when changing descriptor-specific parameters but not using that descriptor
		_voParams.sigmaPriorToCensusTransform = config.bitplanes_sigma_census;
		_voParams.sigmaBitPlanes = config.bitplanes_sigma_smooth;

		_voParams.dfSigma1 = config.descfield_sigma_1;
		_voParams.dfSigma2 = config.descfield_sigma_2;

		_voParams.latchNumBytes = config.latch_num_bytes;
		_voParams.latchRotationInvariance = config.latch_rotation_invariance;
		_voParams.latchHalfSsdSize = config.latch_half_ssd_size;

		_voParams.centralDifferenceRadius = config.centdiff_radius;
		_voParams.centralDifferenceSigmaBefore = config.centdiff_sigma_before;
		_voParams.centralDifferenceSigmaAfter = config.centdiff_sigma_after;

		_voParams.laplacianKernelSize = config.laplacian_kernel_size;

		_voParams.maxIterations = config.opt_max_iters;
		_voParams.parameterTolerance = std::pow( 10.0, config.opt_log_param_tol );
		_voParams.functionTolerance = std::pow( 10.0, config.opt_log_func_tol );
		_voParams.gradientTolerance = std::pow( 10.0, config.opt_log_grad_tol );
		_voParams.relaxTolerancesForCoarseLevels = config.opt_relax_coarse_tols;

		_voParams.gradientEstimation = bpvo::GradientEstimationType( config.gradient_estimation );
		_voParams.interp = bpvo::InterpolationType( config.interpolation );
		_voParams.lossFunction = bpvo::LossFunctionType( config.loss_function );
		_voParams.descriptor = bpvo::DescriptorType( config.descriptor );
		_voParams.verbosity = bpvo::VerbosityType( config.verbosity );

		_voParams.minTranslationMagToKeyFrame = config.keyframe_min_translation;
		_voParams.minRotationMagToKeyFrame = config.keyframe_min_rotation;
		_voParams.maxFractionOfGoodPointsToKeyFrame = config.keyframe_min_inlier_ratio;
		_voParams.goodPointThreshold = config.keyframe_inlier_threshold;
		_voParams.maxSolutionError = config.max_solution_error;

		_voParams.minNumPixelsForNonMaximaSuppression = config.min_pix_nms;
		_voParams.nonMaxSuppRadius = config.radius_nms;
		_voParams.minNumPixelsToWork = config.min_num_pix;
		_voParams.minSaliency = config.min_saliency;

		if( config.min_valid_disparity > config.max_valid_disparity )
		{
			ROS_WARN_STREAM( "Requested min valid disparity " << config.min_valid_disparity <<
			" greater than max valid disparity " << config.max_valid_disparity << "! Increasing max.");
			config.max_valid_disparity = config.min_valid_disparity;
		}
		_voParams.minValidDisparity = config.min_valid_disparity;
		_voParams.maxValidDisparity = config.max_valid_disparity;

		// NOTE test level cannot be lower than num_pyramid_levels
		// NOTE In the case of auto pyramid, we can't tell what the depth will be
		if( config.num_pyramid_levels > 0 && (config.max_test_level >= config.num_pyramid_levels) )
		{
			ROS_WARN_STREAM( "Requested test level " << config.max_test_level << " with " <<
			                 config.num_pyramid_levels << " pyramid levels! Shrinking test level appropriately." );
			config.max_test_level = config.num_pyramid_levels - 1;
		}
		_voParams.maxTestLevel = config.max_test_level;
		_voParams.withNormalization = config.normalize_linsys;
	}

	void InitializeVO( const ImageData& data )
	{
		// TODO Parameters?
		bpvo::Matrix33 K;
		const cv::Matx33d& Km = data.model.GetIntrinsicMatrix();
		for( unsigned int i = 0; i < K.rows(); ++i )
		{
			for( unsigned int j = 0; j < K.cols(); ++j )
			{
				K( i, j ) = Km( i, j );
			}
		}

		bpvo::ImageSize imgSize;
		imgSize.rows = data.image->image.size().height;
		imgSize.cols = data.image->image.size().width;

		_vo = std::make_shared<bpvo::VisualOdometry>( K, data.baseline, imgSize, _voParams );
	}

	virtual void DataCallback( const sensor_msgs::ImageConstPtr& image,
	                           const stereo_msgs::DisparityImageConstPtr& disparity,
	                           const sensor_msgs::CameraInfoConstPtr& info )
	{
		ImageData data;
		try
		{
			data.image = cv_bridge::toCvShare( image, "mono8" );
			data.disparity = cv_bridge::toCvShare( disparity->image, data.disparityObject, "32FC1" );
		}
		catch( cv_bridge::Exception& e )
		{
			ROS_ERROR( "Error in conversion: %s", e.what() );
			return;
		}

		data.baseline = disparity->T;
		data.model = CameraCalibration( info->header.frame_id, *info );
		data.model.SetScale( data.image->image.size() );

		if( !_vo ) { InitializeVO( data ); }

		const ros::Time& now = image->header.stamp;
		if( _lastTime.isZero() ) { _lastTime = now; }

		double dt = (now - _lastTime).toSec();
		PoseSE3 guessDisp = PredictMotion( now, image->header.frame_id ).Inverse();
		_lastTime = now;
		bpvo::Matrix44 guessH = guessDisp.ToMatrix().cast<float>();

		bpvo::Result res = _vo->addFrame( data.image->image,
		                                  data.disparity->image,
										  guessH );
		if( !res.success )
		{ 
		  //ROS_WARN_STREAM( "VO Failed!" );
			return; 
		}

		FixedMatrixType<4, 4> delta = res.displacement.cast<double>();
		delta(3,3) = 1.0; // NOTE Sometimes bad conditioning inside BPVO's optimization
		PoseSE3 camDisplacement = PoseSE3( delta ).Inverse();

		static PoseSE3 zToX( 0, 0, 0, -0.5, 0.5, -0.5, 0.5 );
		if( _camXForward )
		{
			camDisplacement = zToX * camDisplacement * zToX.Inverse();
		}

		_integratedPose = _integratedPose * camDisplacement;

		geometry_msgs::PoseStamped poseMsg;
		poseMsg.header.frame_id = image->header.frame_id;
		poseMsg.header.stamp = image->header.stamp;
		poseMsg.pose = PoseToMsg( _integratedPose );
		_posePub.publish( poseMsg );

		if( dt > 0 )
		{
			geometry_msgs::TwistStamped twistMsg;
			twistMsg.header.frame_id = image->header.frame_id;
			twistMsg.header.stamp = image->header.stamp;
			PoseSE3::TangentVector tang = PoseSE3::Log( camDisplacement ) / dt;
			twistMsg.twist = TangentToMsg( tang );
			_twistPub.publish( twistMsg );
		}
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "disparity_vo" );

	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	DisparityVO vo( nh, ph );

	ros::spin();

	return 0;
}
