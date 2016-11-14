#include "fieldtrack/SimpleStateEstimator.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/filters/FilterUtils.h"
#include "argus_utils/utils/YamlUtils.h"

#include "lookup/LookupUtils.hpp"

#include <boost/foreach.hpp>

using namespace argus_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace fieldtrack;

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
	pmsg.step_num = infoNumber++;
	return pmsg;
}

bool StampedFilter::ProcessUpdate( const FilterUpdate& msg, InfoPair& info )
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

	MatrixType prevCov = filter.DerivsCov();
	MatrixType Q = parent.GetCovarianceRate( filterTime );

	// First predict up to the update time
	ros::Time updateTime = msg.header.stamp;
	if( updateTime < filterTime )
	{
		ROS_WARN_STREAM( "Update received with time " << updateTime << " before filter time " << filterTime );
		return false;
	}

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
		PoseType pos( z );
		if( !CheckPoseUpdate( pos, R ) ) { return false; }
		info.first = PredictUntil( updateTime );
		info.second  = PoseUpdate( pos, R );
	}
	else if( !hasPosition && !hasOrientation && hasDerivs )
	{
		if( !CheckDerivsUpdate( z, C, R ) ) { return false; }
		info.first = PredictUntil( updateTime );
		info.second = DerivsUpdate( z, C, R );
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
	
	info.second.header = msg.header;
	info.second.step_num = infoNumber++;
	return true;
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

bool StampedFilter::CheckPoseUpdate( const PoseType& pose, const MatrixType& R )
{
	return true; // TODO
}

bool StampedFilter::CheckDerivsUpdate( const VectorType& derivs,
                                       const MatrixType& C,
                                       const MatrixType& R )
{
	FilterType::DerivObsMatrix Cderivs = C.rightCols( C.cols() - FilterType::TangentDim );
	double prob = filter.DerivsLikelihood( derivs, Cderivs, R );
	bool valid = prob > parent._likelihoodThreshold;
	if( !valid )
	{
		ROS_WARN_STREAM( "Rejecting outlier with likelihood: " << prob );
	}
	return valid;
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
	}
	for( unsigned int i = 0; i < FilterType::CovarianceDim/6 - 1; ++i )
	{
		derivs.segment( (6*i) + 2, 3 ).setZero();
	}
	filter.Derivs() = derivs;
}

void StampedFilter::SquashPoseUncertainty()
{
	filter.FullCov().block(0,0,FilterType::TangentDim,FilterType::CovarianceDim).setZero();
	filter.FullCov().block(0,0,FilterType::CovarianceDim,FilterType::TangentDim).setZero();
}

Odometry StampedFilter::GetOdomMsg() const
{
	Odometry msg;
	msg.header.stamp = filterTime;

	msg.pose.pose = PoseToMsg( filter.Pose() );
	SerializeMatrix( filter.PoseCov(), msg.pose.covariance );
	
	PoseType::TangentVector tangents = filter.Derivs().head<FilterType::TangentDim>();
	msg.twist.twist = TangentToMsg( tangents );
	SerializeMatrix( filter.DerivsCov().topLeftCorner<FilterType::TangentDim,
	                                                  FilterType::TangentDim>(), 
	                 msg.twist.covariance );
	return msg;
}

SimpleStateEstimator::SimpleStateEstimator( ros::NodeHandle& nodeHandle, 
                                            ros::NodeHandle& privHandle )
: _filter( *this )
  // _xlTx( "xl_features", 6, {"xl_lin_x", "xl_lin_y", "xl_lin_z",
  //                           "xl_ang_x", "xl_ang_y", "xl_ang_z" } )
{
	if( privHandle.hasParam( "startup_delay" ) )
	{
		double delay;
		GetParam( privHandle, "startup_delay", delay );
		ros::Duration( delay ).sleep();
	}

	GetParamRequired( privHandle, "reference_frame", _referenceFrame );
	GetParamRequired( privHandle, "body_frame", _bodyFrame );
	double upLag;
	GetParamRequired( privHandle, "update_lag", upLag );
	_updateLag = ros::Duration( upLag );
	GetParam( privHandle, "outlier_likelihood_threshold", _likelihoodThreshold, 
	          -std::numeric_limits<double>::infinity() );

	// Parse all update sources
	XmlRpc::XmlRpcValue updateSources;
	GetParam( privHandle, "update_sources", updateSources );
	YAML::Node updatesYaml = XmlToYaml( updateSources );
	YAML::Node::const_iterator iter;
	for( iter = updatesYaml.begin(); iter != updatesYaml.end(); iter++ )
	{
		const std::string& sourceName = iter->first.as<std::string>();
		const std::string& topic = iter->second["topic"].as<std::string>();
		ROS_INFO_STREAM( "Subscribing to updates from " << sourceName
		                 << " at " << topic );
		_updateSubs[sourceName].sub = nodeHandle.subscribe( topic, 
		                                                    100, 
		                                                    &SimpleStateEstimator::UpdateCallback, 
		                                                    this );

		_updateSubs[sourceName].usingAdaptive = iter->second["adaptive"].as<bool>();
		if( _updateSubs[sourceName].usingAdaptive )
		{
			ros::NodeHandle subh( privHandle.resolveName( "update_sources/" + sourceName ) );
			_updateSubs[sourceName].Radapter.Initialize( subh );
		}
	}

	// Parse covariance rate estimator
	if( privHandle.hasParam( "transition_cov_estimator" ) )
	{
		ros::NodeHandle subh( privHandle.resolveName( "transition_cov_estimator" ) );
		_Qestimator.Initialize( "transition", subh );
		_Qestimator.SetUpdateTopic( "param_updates" );
		_usingAdaptiveTrans = false;
	}
	else if( privHandle.hasParam( "transition_cov_adapter" ) )
	{
		ros::NodeHandle subh( privHandle.resolveName( "transition_cov_adapter" ) );
		_Qadapter.Initialize( subh );
		_usingAdaptiveTrans = true;
	}
	else
	{
		ROS_WARN_STREAM( "No transition covariance estimator specified. Using fixed rate." );
	}

	// Parse covariance rate to use as a fallback
	GetParam<double>( privHandle, "covariance_rate", _Qrate, FilterType::FullCovType::Identity() );

	// Initialize
	GetParam<double>( privHandle, "initial_covariance", _initCov, FilterType::FullCovType::Identity() );

	ROS_INFO_STREAM( "Using initial covariance: " << std::endl << _initCov );
	ROS_INFO_STREAM( "Using covariance rate: " << std::endl << _Qrate );

	// TODO Initial state?
	WriteLock lock( _mutex );
	_filter.filter.FullCov() = _initCov;

	// NOTE This causes problems when using rosbag play because the sim time doesn't
	// start playing early enough!
	_filter.filterTime = ros::Time::now();

	privHandle.param<bool>( "two_dimensional", twoDimensional, false );
	ROS_INFO_STREAM( "Two dimensional mode: " << twoDimensional );

	privHandle.param<bool>( "velocity_only", velocityOnly, false );
	ROS_INFO_STREAM( "Velocity only mode: " << velocityOnly );

	GetParam( privHandle, "max_entropy", _maxEntropy, std::numeric_limits<double>::infinity() );

	_resetHandler = privHandle.advertiseService( "reset", 
	                                             &SimpleStateEstimator::ResetFilterCallback,
	                                             this );

	_latestOdomPub = nodeHandle.advertise<Odometry>( "odom", 10 );
	
	bool publishInfo;
	GetParam( privHandle, "publish_info", false );
	if( publishInfo )
	{
		unsigned int outBufferSize;
		GetParam<unsigned int>( privHandle, "info_pub_buffer_size", outBufferSize, (unsigned int) 100 );
		_stepPub = nodeHandle.advertise<argus_msgs::FilterStepInfo>( "filter_info", outBufferSize );
	}
	
	double timerRate;
	privHandle.param<double>( "update_rate", timerRate, 10.0 );
	ROS_INFO_STREAM( "Publishing at " << timerRate << " Hz" );
	_updateTimer = nodeHandle.createTimer( ros::Duration( 1.0 / timerRate ),
	                                      &SimpleStateEstimator::TimerCallback,
	                                      this );
}

void SimpleStateEstimator::Reset( double waitTime, const ros::Time& time )
{
	ros::Duration( waitTime ).sleep();

	WriteLock lock( _mutex );

	_updateBuffer.clear();

	_filter.filter.Pose() = FilterType::PoseType();
	_filter.filter.Derivs() = FilterType::DerivsType::Zero();
	_filter.filter.FullCov() = _initCov;
	_filter.filterTime = time;
	_filter.infoNumber++;

	if( _usingAdaptiveTrans )
	{
		_Qadapter.Reset();
	}
	typedef UpdateRegistry::value_type Item;
	BOOST_FOREACH( Item& item, _updateSubs )
	{
		if( item.second.usingAdaptive )
		{
			item.second.Radapter.Reset();
		}
	}

}

bool SimpleStateEstimator::ResetFilterCallback( ResetFilter::Request& req,
                                                ResetFilter::Response& res )
{
	Reset( req.time_to_wait, req.filter_time );
	return true;
}

void SimpleStateEstimator::UpdateCallback( const FilterUpdate::ConstPtr& msg )
{
	const std::string& source = msg->header.frame_id;
	if( _updateSubs.count( source ) == 0 )
	{
		ROS_WARN_STREAM( "Received unregistered observation from: " << source );
		return;
	}

	WriteLock lock( _mutex );
	FilterUpdate up = *msg;
	while( _updateBuffer.count(up.header.stamp) > 0 )
	{
		// For some reason the resolution on the timestamp compare is limited
		up.header.stamp.nsec += 2; 
	}

	_updateBuffer[up.header.stamp] = up;
}

void SimpleStateEstimator::ProcessUpdateBuffer( const ros::Time& until,
                                                bool publishInfo,
                                                bool clearUsed )
{
	while( !_updateBuffer.empty() )
	{
		// TODO Have external locking semantics here to make this explicit
		// Filter mutex is acquired by calling method
		//WriteLock flock( _mutex );
		UpdateBuffer::const_iterator firstItem = _updateBuffer.begin();
		const ros::Time& earliestBufferTime = firstItem->first;
		if( earliestBufferTime > until ) { break; }
		
		FilterUpdate msg = firstItem->second;
		if( _updateSubs[msg.header.frame_id].usingAdaptive )
		{
			msg.observation_cov = MatrixToMsg( _updateSubs[msg.header.frame_id].Radapter.GetR() );
		}

		// ROS_INFO_STREAM( "Updating with: " << msg.header.frame_id );
		std::pair<FilterStepInfo,FilterStepInfo> infoPair;
		if( publishInfo_filter.ProcessUpdate( msg, infoPair ) )
		{
			_stepPub.publish( infoPair.first );
			_transCovPub.publish( infoPair.first.noiseCov );
			_stepPub.publish( infoPair.second );
			

			if( _usingAdaptiveTrans )
			{
				_Qadapter.ProcessInfo( infoPair.first );
				_Qadapter.ProcessInfo( infoPair.second );
			}

			if( _updateSubs[msg.header.frame_id].usingAdaptive )
			{
				_updateSubs[msg.header.frame_id].Radapter.ProcessInfo( infoPair.second );
			}
		}
		_updateBuffer.erase( firstItem );

		if( velocityOnly ) { _filter.SquashPoseUncertainty(); }
		if( twoDimensional ) { _filter.EnforceTwoDimensionality(); }
	}
}

MatrixType SimpleStateEstimator::GetCovarianceRate( const ros::Time& time )
{
	if( !_usingAdaptiveTrans && _Qestimator.IsReady() )
	{
		MatrixType q = _Qestimator.EstimateCovariance( time );
		return q;
	}
	else if( _usingAdaptiveTrans )
	{
		MatrixType q = _Qadapter.GetQ();
		return q;
	}
	else
	{
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

void SimpleStateEstimator::CheckForDivergence( const ros::Time& now )
{
	// Entropy of our state estimate is log-determinant of covariances
	WriteLock lock( _mutex );

	Eigen::LDLT<FilterType::FullCovType> ldlt( _filter.filter.FullCov() );
	VectorType Dvec = ldlt.transpositionsP().transpose() * ldlt.vectorD();

	if( velocityOnly )
	{
		Dvec.head<6>() = FixedVectorType<6>::Constant( 1.0 );
	}

	if( twoDimensional )
	{
		for( unsigned int i = 0; i < FilterType::CovarianceDim/6; ++i )
		{
			Dvec.segment( 6*i + 2, 3 ) = FixedVectorType<3>::Ones();
		}
	}

	double entropy = std::log( Dvec.array().prod() );
	// ROS_INFO_STREAM( "Dvec: " << Dvec.transpose() );
	// ROS_INFO_STREAM( "Entropy: " << entropy );

	lock.unlock();

	if( entropy > _maxEntropy )
	{
		ROS_WARN_STREAM( "Filter entropy: " << entropy << " greater than max: " << _maxEntropy <<
		                 " Resetting filter..." );
		Reset( 0, now );
	}
}

void SimpleStateEstimator::TimerCallback( const ros::TimerEvent& event )
{
	ros::Time now = event.current_real;
	CheckForDivergence( now );

	WriteLock lock( _mutex );
	
	// This should never happen, but check anyways
	if( now < _filter.filterTime )
	{
		ROS_WARN_STREAM( "Filter time is ahead of timer callback!" );
		return;
	}

	// Use all observations to our lagged timepoint
	ros::Time laggedHead = now - _updateLag;
	ProcessUpdateBuffer( laggedHead );

	// If we haven't quite reached the lagged timepoint, predict up to it
	if( laggedHead > _filter.filterTime )
	{
		// TODO Move this predict code into a function
		FilterStepInfo predInfo = _filter.PredictUntil( laggedHead );
		_stepPub.publish( predInfo );
		if( _usingAdaptiveTrans )
		{
			_Qadapter.ProcessInfo( predInfo );
		}
	}
	
	// Roll forward another filter on all buffered observations
	StampedFilter estFilter( _filter );
	lock.unlock();

	typedef UpdateBuffer::value_type Item;
	StampedFilter::InfoPair info;
	BOOST_FOREACH( const Item& item, _updateBuffer )
	{
		estFilter.ProcessUpdate( item.second, info );
		if( velocityOnly ) { estFilter.SquashPoseUncertainty(); }
		if( twoDimensional ) { estFilter.EnforceTwoDimensionality(); }
	}

	if( now > estFilter.filterTime ) 
	{
		estFilter.PredictUntil( now );
	}
	if( velocityOnly ) { estFilter.SquashPoseUncertainty(); }
	if( twoDimensional ) { estFilter.EnforceTwoDimensionality(); }

	// Publish the forward-predicted state
	msg = estFilter.GetOdomMsg();
	msg.header.frame_id = _referenceFrame;
	msg.child_frame_id = _bodyFrame;
	_latestOdomPub.publish( msg );
}

}
