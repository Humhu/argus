#include "fieldtrack/SimpleStateEstimator.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/filters/FilterUtils.h"
#include "argus_utils/utils/YamlUtils.h"

#include "lookup/LookupUtils.hpp"

using namespace argus_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;

#define MAX_DT_THRESH (1E3) // The max prediction dt to allow in seconds

namespace argus
{

SimpleStateEstimator::SimpleStateEstimator( ros::NodeHandle& nodeHandle, 
                                            ros::NodeHandle& privHandle )
: _filter( PoseType(), 
           FilterType::DerivsType::Zero(), 
           FilterType::FullCovType::Zero() )
{
	GetParamRequired( privHandle, "reference_frame", _referenceFrame );
	GetParamRequired( privHandle, "body_frame", _bodyFrame );
	
	// Parse all update sources
	XmlRpc::XmlRpcValue updateSources;
	GetParam( privHandle, "update_sources", updateSources );
	YAML::Node updatesYaml = XmlToYaml( updateSources );
	YAML::Node::const_iterator iter;
	for( iter = updatesYaml.begin(); iter != updatesYaml.end(); iter++ )
	{
		const std::string& sourceName = iter->first.as<std::string>();
		const std::string& topic = iter->second.as<std::string>();
		ROS_INFO_STREAM( "Subscribing to updates from " << sourceName
		                 << " at " << topic );
		_updateSubs[sourceName] = nodeHandle.subscribe( topic, 
		                                                10, 
		                                                &SimpleStateEstimator::UpdateCallback, 
		                                                this );
	}

	// Parse covariance rate estimator
	if( privHandle.hasParam( "transition_cov_estimator" ) )
	{
		_Qestimator.Initialize( "transition", privHandle, 
		                            "transition_cov_estimator" );
		_Qestimator.SetUpdateTopic( "param_updates" );
	}
	else
	{
		ROS_WARN_STREAM( "No transition covariance estimator specified. Using fixed rate." );
	}

	// Parse covariance rate to use as a fallback
	if( !GetMatrixParam<double>( privHandle, "covariance_rate", _Qrate ) )
	{
		if( !GetDiagonalParam<double>( privHandle, "covariance_rate", _Qrate ) )
		{
			ROS_WARN_STREAM( "No covariance rate given. Using identity." );
			_Qrate = FilterType::FullCovType::Identity();
		}
		ROS_INFO_STREAM( "Using covariance rate: " << std::endl << _Qrate );
	}

	// Initialize
	FilterType::FullCovType initCov;
	if( !GetMatrixParam<double>( privHandle, "initial_covariance", initCov ) )
	{
		if( !GetDiagonalParam<double>( privHandle, "initial_covariance", initCov ) )
		{
			ROS_WARN_STREAM( "No initial covariance rate given. Using 10*identity." );
			initCov = 10 * FilterType::FullCovType::Identity();
		}
	}
	ROS_INFO_STREAM( "Using initial covariance: " << std::endl << initCov );

	// TODO Initial state?
	_filter.FullCov() = initCov;

	// NOTE This causes problems when using rosbag play because the sim time doesn't
	// start playing early enough!
	_filterTime = ros::Time::now();

	privHandle.param<bool>( "two_dimensional", twoDimensional, false );
	ROS_INFO_STREAM( "Two dimensional mode: " << twoDimensional );

	_odomPub = nodeHandle.advertise<Odometry>( "odometry", 10 );
	_stepPub = nodeHandle.advertise<argus_msgs::FilterStepInfo>( "filter_info", 10 );
	
	double timerRate;
	privHandle.param<double>( "timer_rate", timerRate, 10.0 );
	ROS_INFO_STREAM( "Publishing at " << timerRate << " Hz" );
	_updateTimer = nodeHandle.createTimer( ros::Duration( 1.0 / timerRate ),
	                                      &SimpleStateEstimator::TimerCallback,
	                                      this );
}

void SimpleStateEstimator::PredictUntil( const ros::Time& until )
{
	double dt = (until - _filterTime).toSec();
	_filterTime = until;
	if( dt > MAX_DT_THRESH )
	{
		ROS_WARN_STREAM( "dt of " << dt << " exceeds max prediction threshold." );
		return;
	}

	MatrixType Qrate;
	if( _Qestimator.IsReady() )
	{
		Qrate = _Qestimator.EstimateCovariance( until );
	}
	else
	{
		ROS_WARN_STREAM( "Transition covariance estimator not ready. Using fixed rate." );
		Qrate = _Qrate;
	}

	PredictInfo info = _filter.Predict( Qrate*dt, dt );
	argus_msgs::FilterStepInfo pmsg = PredictToMsg( info );
	pmsg.header.frame_id = "transition";
	pmsg.header.stamp = until;
	_stepPub.publish( pmsg );
}

void SimpleStateEstimator::UpdateCallback( const FilterUpdate::ConstPtr& msg )
{
 	VectorType z( msg->observation.size() );
	ParseMatrix( msg->observation, z );
	MatrixType C = MsgToMatrix( msg->observation_matrix );
	MatrixType R = MsgToMatrix( msg->observation_cov );

	if( C.cols() != FilterType::CovarianceDim )
	{
		ROS_WARN_STREAM( "Update C has wrong dimension." );
	}

	// TODO Case where there are no derivs in the _filter?
	//bool hasPosition = !( C.block( 0, 0, C.rows(), 3 ).array() == 0 ).all();
	//bool hasOrientation = !( C.block( 0, 3, C.rows(), 3 ).array() == 0 ).all();
	//bool hasDerivs = !( C.rightCols( C.cols() - FilterType::TangentDim ).array() == 0 ).all();
	bool hasPosition = !( C.block( 0, 0, C.rows(), 2 ).array() == 0 ).all();
	bool hasOrientation = !( C.col(2).array() == 0 ).all();
	bool hasDerivs = !( C.rightCols( C.cols() - FilterType::TangentDim ).array() == 0 ).all();

	// First predict up to the update time
	ros::Time updateTime = msg->header.stamp;
	if( updateTime > _filterTime )
	{
		PredictUntil( updateTime );
	}
	else if( updateTime < _filterTime )
	{
		// TODO Implement _filter reversing
		ROS_WARN_STREAM( "Update received from before _filter time." );
	}

	FilterStepInfo info;
	if( hasPosition && !hasOrientation && !hasDerivs )
	{
		ROS_WARN_STREAM( "Position only updates are not supported yet." );
		return;
	}
	else if( !hasPosition && hasOrientation && !hasDerivs )
	{
		ROS_WARN_STREAM( "Orientation only updates are not supported yet." );
		return;
	}
	else if( hasPosition && hasOrientation && !hasDerivs )
	{
		info = PoseUpdate( PoseType( z ), R );
	}
	else if( !hasPosition && !hasOrientation && hasDerivs )
	{
		info = DerivsUpdate( z, C, R );
	}
	else if( hasPosition && hasOrientation && hasDerivs )
	{
		// PoseType pose;
		// pose.FromVector( z.head(7) );
		// VectorType derivs = z.tail( z.size() - 7 );
		// info = JointUpdate( pose, derivs, C, R );
		ROS_WARN_STREAM( "Joint update not supported yet." );
		return;
	}
	else
	{
		ROS_WARN_STREAM( "Unsupported update combination." );
		return;
	}
	
	info.header = msg->header;
	_stepPub.publish( info );

}

argus_msgs::FilterStepInfo
SimpleStateEstimator::PoseUpdate( const PoseType& pose, 
                                  const MatrixType& R )
{
	UpdateInfo info = _filter.UpdatePose( pose, R );
	return UpdateToMsg( info );
	
}

argus_msgs::FilterStepInfo 
SimpleStateEstimator::DerivsUpdate( const VectorType& derivs, 
                                    const MatrixType& C,
                                    const MatrixType& R )
{
	FilterType::DerivObsMatrix Cderivs = C.rightCols( C.cols() - FilterType::TangentDim );
	UpdateInfo info = _filter.UpdateDerivs( derivs, Cderivs, R );
	return UpdateToMsg( info );
}

// argus_msgs::FilterStepInfo PositionUpdate( const Translation3Type& pos, 
//                                            const MatrixType& R )
// {}

// argus_msgs::FilterStepInfo OrientationUpdate( const QuaternionType& ori, 
//                                               const MatrixType& R )
// {}

// void SimpleStateEstimator::JointUpdate( const PoseSE2& pose,
//                                         const VectorType& derivs, 
//                                         const MatrixType& C,
//                                         const MatrixType& R )
// {}

void SimpleStateEstimator::TimerCallback( const ros::TimerEvent& event )
{
	ros::Time now = event.current_real;
	if( now > _filterTime )
	{
		PredictUntil( now );
	}

	Odometry msg;
	msg.header.frame_id = _referenceFrame;
	msg.header.stamp = event.current_real;
	msg.child_frame_id = _bodyFrame;
	
	msg.pose.pose = PoseToMsg( _filter.Pose() );
	SerializeMatrix( _filter.PoseCov(), msg.pose.covariance );
	
	PoseType::TangentVector tangents = _filter.Derivs().head<FilterType::TangentDim>();
	msg.twist.twist = TangentToMsg( tangents );
	SerializeMatrix( _filter.DerivsCov().topLeftCorner<FilterType::TangentDim,
	                                                  FilterType::TangentDim>(), msg.twist.covariance );
	
	_odomPub.publish( msg );
}

void SimpleStateEstimator::EnforceTwoDimensionality()
{
	FixedVectorType<7> poseVector = _filter.Pose().ToVector();
	PoseSE3 mean( poseVector(0), poseVector(1), 0, poseVector(3), 0, 0, poseVector(6) );
	_filter.Pose() = mean;

	PoseSE3::TangentVector velocity = _filter.Derivs().head<6>();
	velocity(2) = 0;
	velocity(3) = 0;
	velocity(4) = 0;
	_filter.Derivs().head<6>() = velocity;
}

}
