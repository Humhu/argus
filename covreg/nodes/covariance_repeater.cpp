#include <ros/ros.h>

#include "covreg/CovarianceManager.h"

#include "broadcast/BroadcastReceiver.h"

#include "argus_msgs/FilterUpdate.h"

#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;
using namespace argus_msgs;
using namespace covreg;

class Repeater
{
public:

	Repeater( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		GetParamRequired( ph, "source_name", _sourceName );

		ros::NodeHandle subh( ph.resolveName( "estimator" ) );
		_estimator.Initialize( _sourceName, subh );

		_updatePublisher = nh.advertise<FilterUpdate>( "repeats",
		                                               10 );

		_updateListener = nh.subscribe( "updates", 
		                                100, 
		                                &Repeater::UpdateCallback, 
		                                this );

		_estimator.SetUpdateTopic( "param_updates" );
	}

	void UpdateCallback( const FilterUpdate::ConstPtr& msg )
	{
		if( !_estimator.IsReady() )
		{
			ROS_WARN_STREAM( "Estimator is not ready." );
			return;
		}

		FilterUpdate repeat( *msg );

		if( msg->observation.size() != _estimator.OutputDim() )
		{
			ROS_ERROR_STREAM( "Estimator has output dim: " << _estimator.OutputDim() <<
			                  " but observation has dim: " << msg->observation.size() );
			exit( -1 );
		}

		try
		{
			MatrixType Rest = _estimator.EstimateCovariance( msg->header.stamp );
			repeat.observation_cov = MatrixToMsg( Rest );
			_updatePublisher.publish( repeat );
		}
		catch( std::runtime_error e )
		{
			return;
		}
	}

private:

	std::string _sourceName;

	CovarianceManager _estimator;
	
	ros::Subscriber _updateListener;
	ros::Publisher _updatePublisher;
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "covariance_repeater" );

	ros::NodeHandle nh, ph("~");

	Repeater repeater( nh, ph );
	ros::spin();

	return 0;
}