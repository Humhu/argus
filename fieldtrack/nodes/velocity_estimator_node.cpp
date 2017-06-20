#include <ros/ros.h>

#include "fieldtrack/FieldtrackCommon.h"
#include "fieldtrack/VelocityEstimator.h"
#include "fieldtrack/NoiseLearner.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <nav_msgs/Odometry.h>
#include "fieldtrack/ResetFilter.h"
#include "fieldtrack/TargetState.h"

#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>

using namespace argus;

class VelocityEstimatorNode
{
public:

	VelocityEstimatorNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		_extrinsicsManager = std::make_shared<ExtrinsicsInterface>( nh, ph );
		_estimator.Initialize( ph, _extrinsicsManager );

		double headLag;
		GetParamRequired( ph, "update_lag", headLag );
		_headLag = ros::Duration( headLag );

		_resetServer = ph.advertiseService( "reset",
		                                    &VelocityEstimatorNode::ResetCallback,
		                                    this );

		// Initialize covariance learner
		_enableLearning = ph.hasParam( "learner" );
		if( _enableLearning )
		{
			ROS_INFO_STREAM( "Initializing learner..." );
			ros::NodeHandle lh( ph.resolveName( "learner" ) );
			GetParamRequired( lh, "rate", _learnRate );
			_learner.Initialize( lh );

			_transModel = _estimator.InitTransCovModel();
			_learner.RegisterTransModel( _transModel );

			_obsModels = _estimator.InitObsCovModels();
			typedef ModelRegistry::value_type Item;
			BOOST_FOREACH( Item & item, _obsModels )
			{
				const std::string& name = item.first;
				const CovarianceModel::Ptr& model = item.second;
				_learner.RegisterObsModel( name, model );
			}
			_learner.StartNewChain();
		}

		// Subscribe to all update topics
		YAML::Node updateSources;
		GetParamRequired( ph, "update_sources", updateSources );
		YAML::Node::const_iterator iter;
		for( iter = updateSources.begin(); iter != updateSources.end(); iter++ )
		{
			const std::string& sourceName = iter->first.as<std::string>();
			const YAML::Node& info = iter->second;

			std::vector<std::string> topics;
			std::string topic, type;
			unsigned int buffSize;
			if( !GetParam( info, "topics", topics ) )
			  {
			    GetParamRequired( info, "topic", topic );
			    topics.push_back( topic );
			  }
			GetParamRequired( info, "type", type );
			GetParam( info, "buffer_size", buffSize, (unsigned int) 10 );

			if( type == "deriv_stamped" )
			{
				SubscribeToUpdates<geometry_msgs::TwistStamped>( nh, topics, buffSize, sourceName );
			}
			else if( type == "deriv_cov_stamped" )
			{
				SubscribeToUpdates<geometry_msgs::TwistWithCovarianceStamped>( nh, topics, buffSize, sourceName );
			}
			else if( type == "imu" )
			{
				SubscribeToUpdates<sensor_msgs::Imu>( nh, topics, buffSize, sourceName );
			}
			else
			{
				throw std::invalid_argument( "Unsupported topic type: " + type );
			}
		}

		// Parse output parameters
		GetParam( ph, "publish_odom", _publishOdom, false );
		if( _publishOdom )
		{
			ROS_INFO_STREAM( "Publishing odometry output" );
			unsigned int buffLen;
			GetParam( ph, "odom_buff_len", buffLen, (unsigned int) 10 );
			_odomPub = nh.advertise<nav_msgs::Odometry>( "odom", buffLen );
		}

		GetParam( ph, "publish_twist", _publishTwist, false );
		if( _publishTwist )
		{
			ROS_INFO_STREAM( "Publishing twist output" );
			unsigned int buffLen;
			GetParam( ph, "twist_buff_len", buffLen, (unsigned int) 10 );
			_twistPub = nh.advertise<geometry_msgs::TwistStamped>( "twist", buffLen );
		}

		GetParam( ph, "publish_twist_with_cov", _publishTwistCov, false );
		if( _publishTwistCov )
		{
			ROS_INFO_STREAM( "Publishing twist covariance output" );
			unsigned int buffLen;
			GetParam( ph, "twist_cov_buff_len", buffLen, (unsigned int) 10 );
			_twistCovPub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>( "twist_with_cov", buffLen );
		}

		GetParam( ph, "publish_info", _publishInfo, false );
		if( _publishInfo )
		{
			ROS_INFO_STREAM( "Publishing info output" );
			unsigned int buffLen;
			GetParam( ph, "info_buff_len", buffLen, (unsigned int) 100 );
			_infoPub = nh.advertise<argus_msgs::FilterStepInfo>( "info", buffLen );
		}

		// NOTE We have to reset the filter to compensate for the lag, otherwise the
		// first timer call will request it predict to the past
		// NOTE This conditional catches simulated time startup
		_initialized = false;
		if( ros::Time::now() != ros::Time( 0 ) )
		{
			_estimator.Reset( ros::Time::now() - _headLag );
			_initialized = true;
		}

		GetParamRequired( ph, "update_rate", _updateRate );
		_updateTimer = nh.createTimer( ros::Duration( 1.0 / _updateRate ),
		                               &VelocityEstimatorNode::TimerCallback,
		                               this );
	}

	template <typename M>
	void SubscribeToUpdates( ros::NodeHandle& nh, const std::vector<std::string>& topics,
	                         unsigned int buffSize, const std::string& sourceName )
	{
	  BOOST_FOREACH( const std::string& topic, topics )
	    {
		ROS_INFO_STREAM( "Subscribing to source " << sourceName << " on topic " << topic );
		_updateSubs.emplace_back();
		_updateSubs.back() =
		    nh.subscribe<M>( topic, buffSize,
		                     boost::bind( &VelocityEstimatorNode::ObservationCallback<M>,
		                                  this,
		                                  _1,
		                                  sourceName ) );
	    }
	}

	template <typename M>
	void ObservationCallback( const typename M::ConstPtr& msg, const std::string& sourceName )
	{
		if( _initialized )
		{
			WriteLock lock( _estimatorMutex );
			_estimator.BufferObservation<M>( sourceName, *msg );
		}
		else
		{
			ROS_WARN_STREAM( "Filter is unitialized. Dropping measurement from: " << sourceName );
		}
	}

	void TimerCallback( const ros::TimerEvent& event )
	{
		if( !_initialized )
		{
			_estimator.Reset( event.current_real - _headLag );
			_initialized = true;
		}

		ros::Time lagged = event.current_real - _headLag;

		// NOTE We want to make sure we don't try and update the estimator models
		// while they're in use
		// NOTE Don't want to put the lock in the estimator itself since it complicates
		// copy construction
		WriteLock lock( _estimatorMutex );
		std::vector<FilterInfo> info = _estimator.Process( lagged );
		VelocityEstimator rollOutEstimator( _estimator );
		lock.unlock();
		rollOutEstimator.Process( event.current_real );

		// TODO Publish Twist, TwistStamped, TwistWithCovarianceStamped modes as well
		if( _publishOdom )
		{
			_odomPub.publish( rollOutEstimator.GetOdom() );
		}
		if( _publishTwist )
		{
			_twistPub.publish( rollOutEstimator.GetTwist() );
		}
		if( _publishTwistCov )
		{
			_twistCovPub.publish( rollOutEstimator.GetTwistWithCovariance() );
		}
		if( _publishInfo )
		{
			FilterInfoMessageVisitor vis;
			BOOST_FOREACH( const FilterInfo &fi, info )
			{
				_infoPub.publish( boost::apply_visitor( vis, fi ) );
			}
		}
		if( _enableLearning )
		{
			FilterInfoMessageVisitor vis;
			BOOST_FOREACH( const FilterInfo &fi, info )
			{
				_learner.BufferInfo( fi );
			}
		}
	}

	bool ResetCallback( fieldtrack::ResetFilter::Request& req,
	                    fieldtrack::ResetFilter::Response& res )
	{
		// NOTE May not want to use ros sleep in sim mode?
		ros::Duration( req.time_to_wait ).sleep();

		VectorType state;
		if( req.state.size() == 0 )
		{
			// Do nothing
		}
		else if( req.state.size() == _estimator.StateDim() )
		{
			Eigen::Map<VectorType> stateMap( req.state.data(), req.state.size() );
			state = stateMap;
		}
		else if( req.state.size() == _estimator.FullDim() )
		{
			Eigen::Map<VectorType> stateMap( req.state.data(), req.state.size() );
			state = stateMap;
		}
		else
		{
			ROS_ERROR_STREAM( "Expected " << _estimator.FullDim() << " or " <<
			                  _estimator.StateDim() << " dim state but got " << req.state.size() );
			return false;
		}

		MatrixType cov;
		if( req.covariance.size() == 0 )
		{
			// Do nothing
		}
		else if( req.covariance.size() == _estimator.StateDim() * _estimator.StateDim() )
		{
			Eigen::Map<MatrixType> covMap( req.covariance.data(),
			                               _estimator.StateDim(),
			                               _estimator.StateDim() );
			cov = covMap;
		}
		else if( req.covariance.size() == _estimator.FullDim() * _estimator.FullDim() )
		{
			Eigen::Map<MatrixType> covMap( req.covariance.data(),
			                               _estimator.FullDim(),
			                               _estimator.FullDim() );
			cov = covMap;
		}
		else
		{
			ROS_ERROR_STREAM( "Expected " << _estimator.FullDim() * _estimator.FullDim() <<
			                  " or " << _estimator.StateDim() * _estimator.StateDim() <<
			                  " dim state but got " << req.covariance.size() );
		}

		_estimator.Reset( req.filter_time, state, cov );
		_initialized = !req.filter_time.isZero();
		_updateTimer.stop();
		_updateTimer.start();
		if( _enableLearning )
		{
			_learner.StartNewChain();
		}
		return true;
	}

	void LearnSpin()
	{
		if( !_enableLearning )
		{
			ros::waitForShutdown();
			return;
		}

		ros::Rate spinRate( _learnRate );
		while( !ros::isShuttingDown() )
		{
			spinRate.sleep();
			ROS_INFO_STREAM( "Spinning..." );
			// TODO Check for chain time length and cap it
			_learner.LearnSpin();

			WriteLock lock( _estimatorMutex );
			ROS_INFO_STREAM( "Updating models..." );
			_estimator.SetTransCovModel( *_transModel );
			typedef ModelRegistry::value_type Item;
			BOOST_FOREACH( const Item &item, _obsModels )
			{
				const std::string& name = item.first;
				const CovarianceModel::Ptr& model = item.second;
				_estimator.SetObsCovModel( name, *model );
			}
		}
	}

private:

	bool _initialized;

	ExtrinsicsInterface::Ptr _extrinsicsManager;
	std::vector<ros::Subscriber> _updateSubs;

	bool _publishOdom;
	ros::Publisher _odomPub;
	bool _publishTwist;
	ros::Publisher _twistPub;
	bool _publishTwistCov;
	ros::Publisher _twistCovPub;
	bool _publishInfo;
	ros::Publisher _infoPub;
	bool _enableLearning;

	ros::ServiceServer _resetServer;
	ros::Timer _updateTimer;
	double _updateRate;

	ros::Duration _headLag;
	VelocityEstimator _estimator;
	Mutex _estimatorMutex;

	// ros::Timer _learnTimer;
	double _learnRate;
	NoiseLearner _learner;
	Mutex _learnerMutex;
	CovarianceModel::Ptr _transModel;
	typedef std::unordered_map<std::string, CovarianceModel::Ptr> ModelRegistry;
	ModelRegistry _obsModels;
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "velocity_estimator_node" );

	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	VelocityEstimatorNode estimator( nh, ph );

	unsigned int numThreads;
	GetParam( ph, "num_threads", numThreads, (unsigned int) 1 );

	ros::AsyncSpinner spinner( numThreads );
	spinner.start();
	estimator.LearnSpin();

	return 0;
}
