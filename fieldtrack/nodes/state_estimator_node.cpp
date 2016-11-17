#include <ros/ros.h>

#include "fieldtrack/FieldtrackCommon.h"
#include "fieldtrack/StateEstimator.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"

#include <nav_msgs/Odometry.h>
#include "fieldtrack/ResetFilter.h"
#include "fieldtrack/TargetState.h"

using namespace argus;

class StateEstimatorNode
{
public:

	StateEstimatorNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		_extrinsicsManager = std::make_shared<ExtrinsicsInterface>( nh, ph );
		_estimator.Initialize( ph, _extrinsicsManager );

		double headLag;
		GetParamRequired( ph, "update_lag", headLag );
		_headLag = ros::Duration( headLag );

		_resetServer = ph.advertiseService( "reset",
		                                    &StateEstimatorNode::ResetCallback,
		                                    this );

		// Subscribe to all update topics
		YAML::Node updateSources;
		GetParamRequired( ph, "update_sources", updateSources );
		YAML::Node::const_iterator iter;
		for( iter = updateSources.begin(); iter != updateSources.end(); iter++ )
		{
			const std::string& sourceName = iter->first.as<std::string>();
			const YAML::Node& info = iter->second;

			std::string topic, type;
			unsigned int buffSize;
			GetParamRequired( info, "topic", topic );
			GetParamRequired( info, "type", type );
			GetParam( info, "buffer_size", buffSize, (unsigned int) 0 );

			_updateSubs.emplace_back();
			if( type == "pose_stamped" )
			{
				SubscribeToUpdates<geometry_msgs::PoseStamped>( nh, topic, buffSize, sourceName );
			}
			else if( type == "pose_cov_stamped" )
			{
				SubscribeToUpdates<geometry_msgs::PoseWithCovarianceStamped>( nh, topic, buffSize, sourceName );
			}
			else if( type == "deriv_stamped" )
			{
				SubscribeToUpdates<geometry_msgs::TwistStamped>( nh, topic, buffSize, sourceName );
			}
			else if( type == "deriv_cov_stamped" )
			{
				SubscribeToUpdates<geometry_msgs::TwistWithCovarianceStamped>( nh, topic, buffSize, sourceName );
			}
			else if( type == "transform_stamped" )
			{
				SubscribeToUpdates<geometry_msgs::TransformStamped>( nh, topic, buffSize, sourceName );
			}
			else if( type == "imu" )
			{
				SubscribeToUpdates<sensor_msgs::Imu>( nh, topic, buffSize, sourceName );
			}
			else
			{
				throw std::invalid_argument( "Unknown topic type: " + type );
			}
		}

		GetParam( ph, "publish_odom", _publishOdom, false );
		if( _publishOdom )
		{
			ROS_INFO_STREAM( "Publishing odometry output" );
			unsigned int odomBuffLen;
			GetParam( ph, "odom_buff_len", odomBuffLen, (unsigned int) 10 );
			_odomPub = nh.advertise<nav_msgs::Odometry>( "odom", odomBuffLen );
		}

		GetParam( ph, "publish_state", _publishState, false );
		if( _publishState )
		{
			ROS_INFO_STREAM( "Publishing state output" );
			unsigned int stateBuffLen;
			GetParam( ph, "state_buff_len", stateBuffLen, (unsigned int) 10 );
			_statePub = nh.advertise<fieldtrack::TargetState>( "state", stateBuffLen );
		}

		// NOTE We have to reset the filter to compensate for the lag, otherwise the
		// first timer call will request it predict to the past
		_estimator.Reset( ros::Time::now() - _headLag );

		double updateRate;
		GetParamRequired( ph, "update_rate", updateRate );
		_updateTimer = nh.createTimer( ros::Duration( 1.0/updateRate ),
		                               &StateEstimatorNode::TimerCallback,
		                               this );
	}

	template <typename M>
	void SubscribeToUpdates( ros::NodeHandle& nh, const std::string& topic,
	                         unsigned int buffSize, const std::string& sourceName )
	{
		ROS_INFO_STREAM( "Subscribing to " << sourceName << " at " << topic );
		_updateSubs.emplace_back();
		_updateSubs.back() = 
		    nh.subscribe<M>( topic, buffSize,
                   boost::bind( &StateEstimatorNode::ObservationCallback<M>,
                                this,
                                _1,
                                sourceName ) );
	}

	template <typename M>
	void ObservationCallback( const typename M::ConstPtr& msg, const std::string& sourceName )
	{
		_estimator.BufferObservation<M>( sourceName, *msg );
	}

	void TimerCallback( const ros::TimerEvent& event )
	{
		ros::Time lagged = event.current_real - _headLag;
		_estimator.Process( lagged );
		
		StateEstimator rollOutEstimator( _estimator );
		rollOutEstimator.Process( event.current_real );

		TargetState state = rollOutEstimator.GetState();

		if( _publishOdom )
		{
			_odomPub.publish( state.ToOdometryMsg() );
		}
		if( _publishState )
		{
			_statePub.publish( state.ToStateMsg() );
		}
	}

	bool ResetCallback( fieldtrack::ResetFilter::Request& req,
	                    fieldtrack::ResetFilter::Response& res )
	{
		ros::Duration( req.time_to_wait ).sleep();
		_estimator.Reset( req.filter_time );
		return true;
	}

public:

	ExtrinsicsInterface::Ptr _extrinsicsManager;
	std::vector<ros::Subscriber> _updateSubs;
	
	bool _publishOdom;
	ros::Publisher _odomPub;

	bool _publishState;
	ros::Publisher _statePub;

	ros::ServiceServer _resetServer;
	ros::Timer _updateTimer;

	ros::Duration _headLag;
	StateEstimator _estimator;
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "state_estimator_node" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	StateEstimatorNode estimator( nh, ph );
	ros::spin();
	return 0;
}
