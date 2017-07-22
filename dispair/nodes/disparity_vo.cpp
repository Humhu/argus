#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <dynamic_reconfigure/server.h>
#include "dispair/DisparityVOConfig.h"
#include "dispair/ImageSync.h"

#include "bpvo/vo.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/synchronization/SynchronizationTypes.h"

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
		: DisparityImageSync( nh, ph )
	{
		_posePub = ph.advertise<geometry_msgs::PoseStamped>( "pose", 10 );
		_twistPub = ph.advertise<geometry_msgs::TwistStamped>( "twist", 10 );

		dynamic_reconfigure::Server<dispair::DisparityVOConfig>::CallbackType cb;
		cb = boost::bind( &DisparityVO::ReconfigureCallback, this, _1, _2 );
		_voConfigServer.setCallback( cb );
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

	void ReconfigureCallback( dispair::DisparityVOConfig& config,
	                          unsigned int level )
	{
		_vo.reset();
		_voParams.numPyramidLevels = config.num_pyramid_levels;
		_voParams.minImageDimensionForPyramid = config.min_pyramid_img_dim;

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

		_voParams.minNumPixelsForNonMaximaSuppression = config.min_pix_nms;
		_voParams.nonMaxSuppRadius = config.radius_nms;
		_voParams.minNumPixelsToWork = config.min_num_pix;
		_voParams.minSaliency = config.min_saliency;
		_voParams.minValidDisparity = config.min_valid_disparity;
		_voParams.maxValidDisparity = config.max_valid_disparity;

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

		bpvo::Result res = _vo->addFrame( data.image->image,
		                                  data.disparity->image );

		FixedMatrixType<4, 4> H = res.pose.cast<double>();
		PoseSE3 camDisplacement = PoseSE3( H ).Inverse();
        
        static PoseSE3 zToX( 0, 0, 0, -0.5, 0.5, -0.5, 0.5 );
        PoseSE3 standardDisplacement = zToX * camDisplacement * zToX.Inverse();
        
        _integratedPose = _integratedPose * standardDisplacement;
                
        geometry_msgs::PoseStamped poseMsg;
		poseMsg.header.frame_id = image->header.frame_id;
        poseMsg.header.stamp = image->header.stamp;
		poseMsg.pose = PoseToMsg( _integratedPose );
		_posePub.publish( poseMsg );

        geometry_msgs::TwistStamped twistMsg;
        twistMsg.header.frame_id = image->header.frame_id;
        twistMsg.header.stamp = image->header.stamp;
        twistMsg.twist = TangentToMsg( PoseSE3::Log( standardDisplacement ) );
        _twistPub.publish( twistMsg );
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