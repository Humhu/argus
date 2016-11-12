#include <ros/ros.h>

#include "fieldtrack/StateEstimator.h"
#include "argus_utils/utils/ParamUtils.h"

#include "argus_msgs/FilterUpdate.h"
#include <nav_msgs/Odometry.h>
#include "fieldtrack/ResetFilter.h"

using namespace argus;

class StateEstimatorNode
{
public:

	StateEstimatorNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		_estimator.Initialize( ph );
		_estimator.Reset( ros::Time::now() );

		double headLag;
		GetParamRequired( ph, "update_lag", headLag );
		_headLag = ros::Duration( headLag );

		_resetServer = ph.advertiseService( "reset",
		                                    &StateEstimatorNode::ResetCallback,
		                                    this );

		double updateRate;
		GetParamRequired( ph, "update_rate", updateRate );
		_updateTimer = nh.createTimer( ros::Duration( 1.0/updateRate ),
		                               &StateEstimatorNode::TimerCallback,
		                               this );

		unsigned int inputBuffLen;
		GetParam( ph, "input_buff_len", inputBuffLen, (unsigned int) 100 );
		_updateSub = nh.subscribe( "updates", 
		                           inputBuffLen, 
		                           &StateEstimatorNode::UpdateCallback,
		                           this );

		unsigned int outputBuffLen;
		GetParam( ph, "output_buff_len", outputBuffLen, (unsigned int) 10 );
		_odomPub = nh.advertise<nav_msgs::Odometry>( "odom", outputBuffLen );
	}

	void UpdateCallback( const argus_msgs::FilterUpdate::ConstPtr& msg )
	{
		_estimator.BufferUpdate( FilterUpdate( *msg ) );
	}

	void TimerCallback( const ros::TimerEvent& event )
	{
		ros::Time lagged = event.current_real - _headLag;
		_estimator.Process( lagged );
		
		StateEstimator rollOutEstimator( _estimator );
		rollOutEstimator.Process( event.current_real );
		_odomPub.publish( rollOutEstimator.GetState().ToMsg() );
	}

	bool ResetCallback( fieldtrack::ResetFilter::Request& req,
	                    fieldtrack::ResetFilter::Response& res )
	{
		ros::Duration( req.time_to_wait ).sleep();
		_estimator.Reset( req.filter_time );
		return true;
	}

public:

	ros::Subscriber _updateSub;
	ros::Publisher _odomPub;
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
