#include <ros/ros.h>
#include <boost/foreach.hpp>

#include "argus_msgs/FilterStepInfo.h"
#include "argus_msgs/FilterUpdate.h"

#include "argus_utils/filters/FilterUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;
using namespace argus_msgs;

class AdaptiveRepeater
{
public:

	AdaptiveRepeater( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		GetParamRequired( ph, "source_name", _sourceName );
		GetParamRequired( ph, "window_length", _windowLength );
		
		_updatePublisher = nh.advertise<FilterUpdate>( "repeats",
		                                               10 );

		if( !GetMatrixParam<double>( ph, "initial_cov", _initCov ) )
		{
			if( !GetDiagonalParam<double>( ph, "initial_cov", _initCov ) )
			{
				ROS_WARN_STREAM( "No initial covariance given. Using identity." );
			}
		}

		_infoListener = nh.subscribe( "filter_info", 
		                              100, 
		                              &AdaptiveRepeater::InfoCallback, 
		                              this );
		_updateListener = nh.subscribe( "updates", 
		                                100, 
		                                &AdaptiveRepeater::UpdateCallback, 
		                                this );
	}

	void InfoCallback( const FilterStepInfo::ConstPtr& msg )
	{
		if( msg->header.frame_id != _sourceName ) { return; }

		// Update is R = Cv+ + H * P+ * H^
		UpdateInfo info = MsgToUpdate( *msg );
		_lastHPHT = info.H * info.Spost * info.H.transpose();

		MatrixType op = info.post_innovation * info.post_innovation.transpose();
		_innoOuterProds.push_back( op );
		while( _innoOuterProds.size() > _windowLength )
		{
			_innoOuterProds.pop_front();
		}
	}

	void UpdateCallback( const FilterUpdate::ConstPtr& msg )
	{
		FilterUpdate repeat( *msg );

		MatrixType Rest;
		if( !IsReady() )
		{
			if( _initCov.size() == 0 )
			{
				_initCov = MatrixType::Identity( msg->observation.size(), msg->observation.size() );
			}
			Rest = _initCov;
		}
		else
		{
			if( msg->observation.size() != _lastHPHT.rows() )
			{
				ROS_ERROR_STREAM( "Estimator has output dim: " << _lastHPHT.rows() <<
				                  " but observation has dim: " << msg->observation.size() );
				exit( -1 );
			}
			Rest = ComputeR();
		}
		repeat.observation_cov = MatrixToMsg( Rest );
		_updatePublisher.publish( repeat );
	}

	bool IsReady() const
	{
		return _innoOuterProds.size() >= _windowLength;
	}

	// Always recompute the sum for long-term stability
	// TODO Periodically recompute it instead and use an efficient buffer in/out
	MatrixType ComputeR()
	{
		MatrixType acc = MatrixType::Zero( _lastHPHT.rows(), _lastHPHT.cols() );
		BOOST_FOREACH( const MatrixType& op, _innoOuterProds )
		{
			acc += op;
		}
		return acc/_innoOuterProds.size() + _lastHPHT;
	}

private:

	std::string _sourceName;

	ros::Subscriber _infoListener;
	ros::Subscriber _updateListener;
	ros::Publisher _updatePublisher;

	unsigned int _windowLength;
	std::deque<MatrixType> _innoOuterProds;
	MatrixType _lastHPHT;
	MatrixType _initCov;
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "covariance_AdaptiveRepeater" );

	ros::NodeHandle nh, ph("~");

	AdaptiveRepeater AdaptiveRepeater( nh, ph );
	ros::spin();

	return 0;
}