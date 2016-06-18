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
           FilterType::FullCovType::Zero() ),
  _infoNumber( 0 )
{
	GetParamRequired( privHandle, "reference_frame", _referenceFrame );
	GetParamRequired( privHandle, "body_frame", _bodyFrame );
	double upLag;
	GetParamRequired( privHandle, "update_lag", upLag );
	_updateLag = ros::Duration( upLag );

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

	privHandle.param<bool>( "velocity_only", velocityOnly, false );
	ROS_INFO_STREAM( "Velocity only mode: " << velocityOnly );

	_odomPub = nodeHandle.advertise<Odometry>( "odometry", 10 );
	_stepPub = nodeHandle.advertise<argus_msgs::FilterStepInfo>( "filter_info", 100 );
	
	double timerRate;
	privHandle.param<double>( "update_rate", timerRate, 10.0 );
	ROS_INFO_STREAM( "Publishing at " << timerRate << " Hz" );
	_updateTimer = nodeHandle.createTimer( ros::Duration( 1.0 / timerRate ),
	                                      &SimpleStateEstimator::TimerCallback,
	                                      this );
}

void SimpleStateEstimator::UpdateCallback( const FilterUpdate::ConstPtr& msg )
{
	// ros::Time now = ros::Time::now();
	// ros::Duration delay = now - msg->header.stamp;
	// ROS_INFO_STREAM( "Received update from: " << msg->header.frame_id <<
	//                  " with delay: " << delay );

	WriteLock lock( _bufferMutex );
	_updateBuffer[msg->header.stamp] = *msg;
}

void SimpleStateEstimator::ProcessUpdateBuffer( const ros::Time& until )
{
	WriteLock lock( _bufferMutex );
	// ROS_INFO_STREAM( "Buffer size: " << _updateBuffer.size() );
	while( !_updateBuffer.empty() )
	{
		UpdateBuffer::const_iterator firstItem = _updateBuffer.begin();
		const ros::Time& earliestBufferTime = firstItem->first;
		if( earliestBufferTime > until ) { break; }
		ProcessUpdate( firstItem->second );
		_updateBuffer.erase( firstItem );
	}
}

MatrixType SimpleStateEstimator::GetCovarianceRate( const ros::Time& time )
{
	if( _Qestimator.IsReady() )
	{
		return _Qestimator.EstimateCovariance( time );
	}
	else
	{
		ROS_WARN_STREAM( "Transition covariance estimator not ready. Using fixed rate." );
		return _Qrate;
	}
}

void SimpleStateEstimator::PredictUntil( const ros::Time& until )
{
	// ROS_INFO_STREAM( "Predicting until: " << until );
	double dt = (until - _filterTime).toSec();
	_filterTime = until;
	if( dt > MAX_DT_THRESH )
	{
		ROS_WARN_STREAM( "dt of " << dt << " exceeds max prediction threshold." );
		return;
	}

	PredictInfo info = _filter.Predict( GetCovarianceRate( _filterTime )*dt, dt );
	argus_msgs::FilterStepInfo pmsg = PredictToMsg( info );
	pmsg.header.frame_id = "transition";
	pmsg.header.stamp = until;
	pmsg.header.seq = _infoNumber++;
	_stepPub.publish( pmsg );
}

void SimpleStateEstimator::ProcessUpdate( const FilterUpdate& msg )
{
	VectorType z( msg.observation.size() );
	ParseMatrix( msg.observation, z );
	MatrixType C = MsgToMatrix( msg.observation_matrix );
	MatrixType R = MsgToMatrix( msg.observation_cov );

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
	ros::Time updateTime = msg.header.stamp;
	if( updateTime > _filterTime )
	{
		PredictUntil( updateTime );
	}
	else if( updateTime < _filterTime )
	{
		ROS_WARN_STREAM( "Update received from before filter time." );
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
	
	info.header = msg.header;
	info.header.seq = _infoNumber++;
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
	VectorType diagonals = _filter.FullCov().diagonal();
	if( ( diagonals.array() < 0 ).any() )
	{
		std::cout << "Full cov: " << std::endl << _filter.FullCov() << std::endl;
		std::cout << "R: " << std::endl << R << std::endl;
		throw std::runtime_error( "Negatives in covariance diagonal." );
	}
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
	if( now < _filterTime )
	{
		ROS_WARN_STREAM( "Filter time is ahead of timer callback." );
	}
	
	if( velocityOnly ) { SquashPoseUncertainty(); }
	if( twoDimensional ) { EnforceTwoDimensionality(); }

	ros::Time laggedHead = now - _updateLag;
	ProcessUpdateBuffer( laggedHead );

	// Publish the forward-predicted state
	Odometry msg;
	msg.header.frame_id = _referenceFrame;
	msg.header.stamp = event.current_real;
	msg.child_frame_id = _bodyFrame;
	
	FilterType estFilter( _filter );
	double dt = (now - _filterTime).toSec();
	PredictInfo info = estFilter.Predict( GetCovarianceRate( _filterTime )*dt, dt );


	msg.pose.pose = PoseToMsg( estFilter.Pose() );
	SerializeMatrix( estFilter.PoseCov(), msg.pose.covariance );
	
	PoseType::TangentVector tangents = estFilter.Derivs().head<FilterType::TangentDim>();
	msg.twist.twist = TangentToMsg( tangents );
	SerializeMatrix( estFilter.DerivsCov().topLeftCorner<FilterType::TangentDim,
	                                                     FilterType::TangentDim>(), 
	                 msg.twist.covariance );
	
	_odomPub.publish( msg );
}

void SimpleStateEstimator::EnforceTwoDimensionality()
{
	FixedVectorType<7> poseVector = _filter.Pose().ToVector();
	PoseSE3 mean( poseVector(0), poseVector(1), 0, poseVector(3), 0, 0, poseVector(6) );
	_filter.Pose() = mean;

	FilterType::DerivsType derivs = _filter.Derivs();
	for( unsigned int i = 0; i < FilterType::CovarianceDim/6; ++i )
	{
		_filter.FullCov().block( 6*i+2, 0, 3, FilterType::CovarianceDim ).setZero();
		_filter.FullCov().block( 0, 6*i+2, FilterType::CovarianceDim, 3 ).setZero();
		derivs.segment( 6*i+2, 3 ).setZero();
	}
	_filter.Derivs() = derivs;
}

void SimpleStateEstimator::SquashPoseUncertainty()
{
	_filter.FullCov().block(0,0,FilterType::TangentDim,FilterType::CovarianceDim).setZero();
	_filter.FullCov().block(0,0,FilterType::CovarianceDim,FilterType::TangentDim).setZero();
}

}
