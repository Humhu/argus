#include "paraset/ContinuousPolicyManager.h"
#include <ros/ros.h>
#include <sstream>

using namespace argus;

void SquashOutput( VectorType& in )
{
	for( unsigned int i = 0; i < in.size(); ++i )
	{
		if( in(i) < 0.0 ) { in(i) = 0.0; }
		if( in(i) > 1.0 ) { in(i) = 1.0; }
	}
}

class ContinuousPolicyWrapper
{
public:

	ContinuousPolicyWrapper( ros::NodeHandle& nh,
	                         ros::NodeHandle& ph )
	{
		ros::NodeHandle lh( ph.resolveName( "policy" ) );
		_manager.Initialize( nh, lh );
		_paramNames = _manager.GetPolicyInterface().GetParameterNames();

		// Advertise action message topic
		_actionPub = ph.advertise<ContinuousParamAction::MsgType>( "actions", 0 );

		// Check for RNG seed
		int seed;
		if( HasParam( ph, "seed" ) )
		{
			GetParam( ph, "seed", seed );
		}
		else
		{
			boost::random::random_device rng;
			seed = rng();
		}
		_dist = MultivariateGaussian<>( _manager.GetNumOutputs(), seed );

		GetParam( ph, "sample_devs_bound", _sampleDevs, 3.0 );

		double updateRate;
		GetParamRequired( ph, "update_rate", updateRate );
		_timer = ph.createTimer( ros::Duration( 1.0/updateRate ),
		                         &ContinuousPolicyWrapper::UpdateCallback,
		                         this );
	}

private:

	ContinuousPolicyManager _manager;
	std::vector<std::string> _paramNames;

	double _sampleDevs;

	MultivariateGaussian<> _dist;
	ros::Timer _timer;

	ros::Publisher _actionPub;

	void UpdateCallback( const ros::TimerEvent& event )
	{

		ContinuousPolicyManager::DistributionParameters distParams;
		StampedFeatures policyInputs;

		try
		{
			distParams = _manager.GetDistributionParams( event.current_real );
			policyInputs = _manager.GetInput( event.current_real );
		}
		catch( std::out_of_range e )
		{
			ROS_WARN_STREAM( e.what() );
			return;
		}

		// Sample from mean and covariance
		_dist.SetMean( distParams.mean );
		_dist.SetInformation( distParams.info );
		VectorType normalizedOutput = _dist.Sample( _sampleDevs );
		SquashOutput( normalizedOutput );

		VectorType policyOutput = ( _manager.GetScales() * normalizedOutput.array() ).matrix()
		                          + _manager.GetOffsets();

		ROS_INFO_STREAM( "input: " << policyInputs.features.transpose() << std::endl <<
		                 "raw mean: " << distParams.mean.transpose() << std::endl <<
		                 "raw info: " << std::endl << distParams.info << std::endl <<
		                 "raw sample: " << normalizedOutput.transpose() << std::endl <<
		                 "scales: " << _manager.GetScales().transpose() << std::endl <<
		                 "offsets: " << _manager.GetOffsets().transpose() << std::endl <<
		                 "output: " << policyOutput.transpose() );

		std::stringstream ss;
		ss << "Setting outputs:" << std::endl;
		for( unsigned int i = 0; i < policyOutput.size(); ++i )
		{
			ss << "\t" << _paramNames[i] << ": " << policyOutput(i) << std::endl;
		}
		ROS_INFO_STREAM( ss.str() );

		if( !policyOutput.allFinite() )
		{
			ROS_WARN_STREAM( "Outputs are not all finite!" );
		}
		else
		{
			// Set policyOutput
			_manager.SetOutput( policyOutput );
		}

		// TODO Name policy
		ContinuousParamAction action( event.current_real,
		                              policyInputs.features,
		                              policyOutput );
		_actionPub.publish( action.ToMsg() );
	}

};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "continuous_policy_node" );

	ros::NodeHandle nh, ph( "~" );
	ContinuousPolicyWrapper policy( nh, ph );
	ros::spin();
	return 0;
}