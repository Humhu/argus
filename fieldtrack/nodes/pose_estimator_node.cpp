#include <ros/ros.h>

#include "fieldtrack/FieldtrackCommon.h"
#include "fieldtrack/PoseEstimator.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <nav_msgs/Odometry.h>
#include "fieldtrack/ResetFilter.h"
#include "fieldtrack/TargetState.h"

#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>

using namespace argus;

class PoseEstimatorNode
{
public:

	PoseEstimatorNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		_extrinsicsManager = std::make_shared<ExtrinsicsInterface>( nh, ph );
		_estimator.Initialize( ph, _extrinsicsManager );

		double headLag;
		GetParamRequired( ph, "update_lag", headLag );
		_headLag = ros::Duration( headLag );

		_resetServer = ph.advertiseService( "reset",
		                                    &PoseEstimatorNode::ResetCallback,
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
			GetParam( info, "buffer_size", buffSize, (unsigned int) 10 );

			if( type == "pose_stamped" )
			{
				SubscribeToUpdates<geometry_msgs::PoseStamped>( nh, topic, buffSize, sourceName );
			}
			else if( type == "pose_cov_stamped" )
			{
				SubscribeToUpdates<geometry_msgs::PoseWithCovarianceStamped>( nh, topic, buffSize, sourceName );
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
				throw std::invalid_argument( "Unsupported topic type: " + type );
			}
		}

		std::string velTopic, velMode;
		if( GetParam( ph, "velocity_mode", velMode ) &&
		    GetParam( ph, "velocity_topic", velTopic ) )
		{
			unsigned int buffSize;
			GetParam( ph, "velocity_buff_len", buffSize, (unsigned int) 10 );

			if( velMode == "twist" )
			{
				_velSub = nh.subscribe( velTopic, buffSize, &PoseEstimatorNode::VelocityCallback, this );
				GetParamRequired( ph, "velocity_covariance", _velCov );
			}
			else if( velMode == "twist_with_cov" )
			{
				_velSub = nh.subscribe( velTopic, buffSize, &PoseEstimatorNode::VelocityCovCallback, this );
			}
			else if( velMode == "odom" )
			{
				_velSub = nh.subscribe( velTopic, buffSize, &PoseEstimatorNode::OdomCallback, this );
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

		GetParam( ph, "publish_pose", _publishPose, false );
		if( _publishPose )
		{
			ROS_INFO_STREAM( "Publishing pose output" );
			unsigned int buffLen;
			GetParam( ph, "pose_buff_len", buffLen, (unsigned int) 10 );
			_posePub = nh.advertise<geometry_msgs::PoseStamped>( "pose", buffLen );
		}

		GetParam( ph, "publish_pose_with_cov", _publishPoseCov, false );
		if( _publishPoseCov )
		{
			ROS_INFO_STREAM( "Publishing pose with covariance output" );
			unsigned int buffLen;
			GetParam( ph, "pose_cov_buff_len", buffLen, (unsigned int) 10 );
			_poseCovPub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>( "pose_with_cov", buffLen );
		}

		GetParam( ph, "publish_info", _publishInfo, false );
		if( _publishInfo )
		{
			ROS_INFO_STREAM( "Publishing info output" );
			unsigned int infoBuffLen;
			GetParam( ph, "info_buff_len", infoBuffLen, (unsigned int) 100 );
			_infoPub = nh.advertise<argus_msgs::FilterStepInfo>( "info", infoBuffLen );
		}

		GetParam( ph, "publish_tf", _publishTf, false );

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
		                               &PoseEstimatorNode::TimerCallback,
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
		                     boost::bind( &PoseEstimatorNode::ObservationCallback<M>,
		                                  this,
		                                  _1,
		                                  sourceName ) );
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

	// TODO Different velocity input types
	void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
	{
		PoseSE3::TangentVector vel = MsgToTangent( msg->twist.twist );
		PoseSE3::CovarianceMatrix cov;
		ParseMatrix( msg->twist.covariance, cov );
		WriteLock lock( _estimatorMutex );
		_estimator.BufferVelocity( msg->header.stamp, vel, cov );
	}

	// TODO Different velocity input types
	void VelocityCallback( const geometry_msgs::TwistStamped::ConstPtr& msg )
	{
		PoseSE3::TangentVector vel = MsgToTangent( msg->twist );
		WriteLock lock( _estimatorMutex );
		_estimator.BufferVelocity( msg->header.stamp, vel, _velCov );
	}

	// TODO Different velocity input types
	void VelocityCovCallback( const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg )
	{
		PoseSE3::TangentVector vel = MsgToTangent( msg->twist.twist );
		PoseSE3::CovarianceMatrix cov;
		ParseMatrix( msg->twist.covariance, cov );
		WriteLock lock( _estimatorMutex );
		_estimator.BufferVelocity( msg->header.stamp, vel, cov );
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
		PoseEstimator rollOutEstimator( _estimator );
		lock.unlock();

		rollOutEstimator.Process( event.current_real );

		// TODO Publish Twist, TwistStamped, TwistWithCovarianceStamped modes as well
		if( _publishOdom )
		{
			_odomPub.publish( rollOutEstimator.GetOdom() );
		}
		if( _publishPose )
		{
			_posePub.publish( rollOutEstimator.GetPose() );
		}
		if( _publishPoseCov )
		{
			_poseCovPub.publish( rollOutEstimator.GetPoseWithCovariance() );
		}
		if( _publishInfo )
		{
			FilterInfoMessageVisitor vis;
			BOOST_FOREACH ( const FilterInfo &fi, info )
			{
				_infoPub.publish( boost::apply_visitor( vis, fi ) );
			}
		}
		if( _publishTf )
		{
			// TODO Have a get relative pose method
			nav_msgs::Odometry odom = rollOutEstimator.GetOdom();
			_extrinsicsManager->SetExtrinsics( odom.child_frame_id,
			                                   odom.header.frame_id,
			                                   event.current_real,
			                                   MsgToPose( odom.pose.pose ) );
		}
	}

	bool ResetCallback( fieldtrack::ResetFilter::Request& req,
	                    fieldtrack::ResetFilter::Response& res )
	{
		// NOTE May not want to use ros sleep in sim mode?
		ros::Duration( req.time_to_wait ).sleep();

		VectorType state;
		if( req.state.size() != 0 )
		{
			if( req.state.size() != PoseSE3::VectorDimension )
			{
				ROS_ERROR_STREAM( "Expected " << PoseSE3::VectorDimension << " dim state but got " << req.state.size() );
				return false;
			}
			Eigen::Map<VectorType> stateMap( req.state.data(), req.state.size() );
			state = stateMap;
		}

		MatrixType cov;
		if( req.covariance.size() != 0 )
		{
			if( req.covariance.size() != PoseSE3::TangentDimension * PoseSE3::TangentDimension )
			{
				ROS_ERROR_STREAM( "Expected " << PoseSE3::TangentDimension * PoseSE3::TangentDimension <<
				                  " dim state but got " << req.covariance.size() );
			}
			Eigen::Map<MatrixType> covMap( req.covariance.data(),
			                               PoseSE3::TangentDimension,
			                               PoseSE3::TangentDimension );
			cov = covMap;
		}

		_estimator.Reset( req.filter_time, state, cov );

		_initialized = !req.filter_time.isZero();
		_updateTimer.stop();
		_updateTimer.start();
		return true;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:

	bool _initialized;

	ExtrinsicsInterface::Ptr _extrinsicsManager;
	std::vector<ros::Subscriber> _updateSubs;

	bool _publishOdom;
	ros::Publisher _odomPub;
	bool _publishPose;
	ros::Publisher _posePub;
	bool _publishPoseCov;
	ros::Publisher _poseCovPub;
	bool _publishInfo;
	ros::Publisher _infoPub;
	bool _publishTf;

	ros::Subscriber _velSub;
	PoseSE3::CovarianceMatrix _velCov;

	ros::ServiceServer _resetServer;
	ros::Timer _updateTimer;
	double _updateRate;

	ros::Duration _headLag;
	PoseEstimator _estimator;
	Mutex _estimatorMutex;
};

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "pose_estimator_node" );

	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	PoseEstimatorNode estimator( nh, ph );

	unsigned int numThreads;
	GetParam( ph, "num_threads", numThreads, (unsigned int) 1 );
	ros::AsyncSpinner spinner( numThreads );
	spinner.start();
	ros::waitForShutdown();

	return 0;
}
