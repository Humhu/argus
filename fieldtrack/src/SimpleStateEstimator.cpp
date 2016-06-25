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

StampedFilter::StampedFilter( SimpleStateEstimator& p ) 
: parent( p ), 
  filter( PoseType(), 
          FilterType::DerivsType::Zero(), 
          FilterType::FullCovType::Zero() ),
  infoNumber( 0 ) {}

FilterStepInfo StampedFilter::PredictUntil( const ros::Time& until )
{
	// ROS_INFO_STREAM( "Predicting until: " << until );
	double dt = (until - filterTime).toSec();
	filterTime = until;
	if( dt > MAX_DT_THRESH )
	{
		throw std::runtime_error( "dt of " + std::to_string(dt) + " exceeds max prediction threshold." );
	}

	PredictInfo info = filter.Predict( parent.GetCovarianceRate( filterTime )*dt, dt );
	argus_msgs::FilterStepInfo pmsg = PredictToMsg( info );
	pmsg.header.frame_id = "transition";
	pmsg.header.stamp = until;
	pmsg.header.seq = infoNumber++;
	return pmsg;
}

std::pair<FilterStepInfo,FilterStepInfo> 
StampedFilter::ProcessUpdate( const FilterUpdate& msg )
{
	std::pair<FilterStepInfo,FilterStepInfo> retPair;

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
	if( updateTime > filterTime )
	{
		retPair.first = PredictUntil( updateTime );
	}
	else if( updateTime < filterTime )
	{
		ROS_WARN_STREAM( "Update received from before filter time." );
	}

	FilterStepInfo info;
	if( hasPosition && !hasOrientation && !hasDerivs )
	{
		throw std::runtime_error( "Position only updates are not supported yet." );
	}
	else if( !hasPosition && hasOrientation && !hasDerivs )
	{
		throw std::runtime_error( "Orientation only updates are not supported yet." );
	}
	else if( hasPosition && hasOrientation && !hasDerivs )
	{
		retPair.second = info = PoseUpdate( PoseType( z ), R );
	}
	else if( !hasPosition && !hasOrientation && hasDerivs )
	{
		retPair.second = info = DerivsUpdate( z, C, R );
	}
	else if( hasPosition && hasOrientation && hasDerivs )
	{
		// PoseType pose;
		// pose.FromVector( z.head(7) );
		// VectorType derivs = z.tail( z.size() - 7 );
		// info = JointUpdate( pose, derivs, C, R );
		throw std::runtime_error( "Joint update not supported yet." );
	}
	else
	{
		throw std::runtime_error( "Unsupported update combination." );
	}
	
	retPair.second.header = msg.header;
	retPair.second.header.seq = infoNumber++;
	return retPair;
}

FilterStepInfo StampedFilter::PoseUpdate( const PoseType& pose, 
                                          const MatrixType& R )
{
	UpdateInfo info = filter.UpdatePose( pose, R );
	return UpdateToMsg( info );
	
}

FilterStepInfo StampedFilter::DerivsUpdate( const VectorType& derivs, 
                                            const MatrixType& C,
                                            const MatrixType& R )
{
	FilterType::DerivObsMatrix Cderivs = C.rightCols( C.cols() - FilterType::TangentDim );
	UpdateInfo info = filter.UpdateDerivs( derivs, Cderivs, R );
	VectorType diagonals = filter.FullCov().diagonal();
	if( ( diagonals.array() < 0 ).any() )
	{
		std::cout << "Full cov: " << std::endl << filter.FullCov() << std::endl;
		std::cout << "R: " << std::endl << R << std::endl;
		throw std::runtime_error( "Negatives in covariance diagonal." );
	}
	return UpdateToMsg( info );
}

void StampedFilter::EnforceTwoDimensionality()
{
	FixedVectorType<7> poseVector = filter.Pose().ToVector();
	PoseSE3 mean( poseVector(0), poseVector(1), 0, poseVector(3), 0, 0, poseVector(6) );
	filter.Pose() = mean;

	FilterType::DerivsType derivs = filter.Derivs();
	for( unsigned int i = 0; i < FilterType::CovarianceDim/6; ++i )
	{
		filter.FullCov().block( 6*i+2, 0, 3, FilterType::CovarianceDim ).setZero();
		filter.FullCov().block( 0, 6*i+2, FilterType::CovarianceDim, 3 ).setZero();
		derivs.segment( 6*i+2, 3 ).setZero();
	}
	filter.Derivs() = derivs;
}

void StampedFilter::SquashPoseUncertainty()
{
	filter.FullCov().block(0,0,FilterType::TangentDim,FilterType::CovarianceDim).setZero();
	filter.FullCov().block(0,0,FilterType::CovarianceDim,FilterType::TangentDim).setZero();
}

SimpleStateEstimator::SimpleStateEstimator( ros::NodeHandle& nodeHandle, 
                                            ros::NodeHandle& privHandle )
: _filter( *this ),
  _infoNumber( 0 ),
  _xlTx( "xl_features", 6, {"xl_lin_x", "xl_lin_y", "xl_lin_z",
                            "xl_ang_x", "xl_ang_y", "xl_ang_z" } )
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
	_filter.filter.FullCov() = initCov;

	// NOTE This causes problems when using rosbag play because the sim time doesn't
	// start playing early enough!
	_filter.filterTime = ros::Time::now();

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
		
		const FilterUpdate& msg = firstItem->second;
		std::pair<FilterStepInfo,FilterStepInfo> infoPair = _filter.ProcessUpdate( msg );
		_stepPub.publish( infoPair.first );
		_stepPub.publish( infoPair.second );
		_updateBuffer.erase( firstItem );

		if( velocityOnly ) { _filter.SquashPoseUncertainty(); }
		if( twoDimensional ) { _filter.EnforceTwoDimensionality(); }
	}
}

MatrixType SimpleStateEstimator::GetCovarianceRate( const ros::Time& time )
{
	if( _Qestimator.IsReady() )
	{
		MatrixType q = _Qestimator.EstimateCovariance( time );
		// ROS_INFO_STREAM( "Got Q: " << std::endl << q );
		return q;
	}
	else
	{
		ROS_WARN_STREAM( "Transition covariance estimator not ready. Using fixed rate." );
		return _Qrate;
	}
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
	if( now < _filter.filterTime )
	{
		ROS_WARN_STREAM( "Filter time is ahead of timer callback." );
	}

	ros::Time laggedHead = now - _updateLag;
	ProcessUpdateBuffer( laggedHead );

	// Publish the forward-predicted state
	Odometry msg;
	msg.header.frame_id = _referenceFrame;
	msg.header.stamp = event.current_real;
	msg.child_frame_id = _bodyFrame;
	
	StampedFilter estFilter( _filter );
	typedef UpdateBuffer::value_type Item;
	BOOST_FOREACH( const Item& item, _updateBuffer )
	{
		estFilter.ProcessUpdate( item.second );
		if( velocityOnly ) { estFilter.SquashPoseUncertainty(); }
		if( twoDimensional ) { estFilter.EnforceTwoDimensionality(); }
	}
	estFilter.PredictUntil( now );
	if( velocityOnly ) { estFilter.SquashPoseUncertainty(); }
	if( twoDimensional ) { estFilter.EnforceTwoDimensionality(); }

	msg.pose.pose = PoseToMsg( estFilter.filter.Pose() );
	SerializeMatrix( estFilter.filter.PoseCov(), msg.pose.covariance );
	
	PoseType::TangentVector tangents = estFilter.filter.Derivs().head<FilterType::TangentDim>();
	msg.twist.twist = TangentToMsg( tangents );
	SerializeMatrix( estFilter.filter.DerivsCov().topLeftCorner<FilterType::TangentDim,
	                                                     FilterType::TangentDim>(), 
	                 msg.twist.covariance );
	
	PoseType::TangentVector tanXls = estFilter.filter.Derivs().segment<FilterType::TangentDim>( FilterType::TangentDim );
	_xlTx.Publish( event.current_real, tanXls );

	_odomPub.publish( msg );
}

}
