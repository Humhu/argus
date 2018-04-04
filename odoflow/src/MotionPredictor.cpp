#include "odoflow/MotionPredictor.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/random/MultivariateGaussian.hpp"
#include "camplex/FiducialCommon.h"

namespace argus
{
MotionPredictor::MotionPredictor( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: _extrinsics( nh, ph )
{
	_maxPredictEntropy.InitializeAndRead( ph, 1.0, "max_predict_entropy",
	                                      "Max predict uncertainty entropy" );

	GetParam( ph, "enable_prediction", _enablePrediction, false );
	if( _enablePrediction )
	{
		std::string predictionMode;
		GetParam<std::string>( ph, "prediction_mode", predictionMode, "odom" );
		if( predictionMode == "odometry" )
		{
			_motionSub = nh.subscribe( "odom", 10, &MotionPredictor::OdometryCallback, this );
		}
		else if( predictionMode == "twist_stamped" )
		{
			_motionSub = nh.subscribe( "odom", 10, &MotionPredictor::TwistStampedCallback, this );
		}
		else
		{
			throw std::invalid_argument( "Unknown prediction mode" );
		}
	}
}

void MotionPredictor::Reset()
{
    _velIntegrator.Reset();
}

PoseSE2 MotionPredictor::PredictMotion( const ros::Time& fromTime,
                                           const ros::Time& currTime,
                                           const std::string& camFrame,
                                           double scale )
{
	// Can't predict motion if we haven't received any odom messages
	if( !_enablePrediction || _odomFrame.empty() )
	{
		if( _odomFrame.empty() )
		{
			ROS_WARN_STREAM( "Prediction enabled but no odometry messages received" );
		}
		return PoseSE2();
	}

	PoseSE3 odomDisp;
	PoseSE3::CovarianceMatrix odomCov;
	if( !_velIntegrator.Integrate( fromTime.toSec(), currTime.toSec(),
	                               odomDisp, odomCov ) )
	{
		ROS_WARN_STREAM( "Could not predict motion from " << fromTime << " to " << currTime );
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

	//unsigned int imgWidth = _keyPyramid[0].size().width; // NOTE Hack?
	PoseSE2::TangentVector logRet = PoseSE2::Log( ret );
	logRet.head<2>() *= scale; //imgWidth / _scale;
	ret = PoseSE2::Exp( logRet );

	return ret;
}

void MotionPredictor::OdometryCallback( const nav_msgs::Odometry::ConstPtr& msg )
{
	PoseSE3::TangentVector vel = MsgToTangent( msg->twist.twist );
	PoseSE3::CovarianceMatrix cov;
	ParseMatrix( msg->twist.covariance, cov );
	_velIntegrator.BufferInfo( msg->header.stamp.toSec(), vel, cov );
	_odomFrame = msg->child_frame_id;
}

void MotionPredictor::TwistStampedCallback( const geometry_msgs::TwistStamped::ConstPtr& msg )
{
	PoseSE3::TangentVector vel = MsgToTangent( msg->twist );
	PoseSE3::CovarianceMatrix cov = PoseSE3::CovarianceMatrix::Zero();
	_velIntegrator.BufferInfo( msg->header.stamp.toSec(), vel, cov );
	_odomFrame = msg->header.frame_id;
}
}