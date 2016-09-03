#include "covreg/CovarianceModelLearner.h"
#include "argus_utils/ParamUtils.h"

#include "argus_msgs/FloatVectorStamped.h"

using namespace argus_msgs;

namespace argus
{

CovarianceModelLearner::CovarianceModelLearner() {}

void CovarianceModelLearner::Initialize( ros::NodeHandle& nh,
                                         ros::NodeHandle& ph )
{
	YAML::Node models;
	GetParamRequired( ph, "models", models );
	YAML::const_iterator iter;
	for( iter = models.begin(); iter != models.end(); ++iter )
	{
		const std::string& name = iter->first.as<std::string>();
		ros::NodeHandle mh( ph.resolveName( name ) );

		EstimatorRegistration& reg = _estRegistry[ name ];
		reg.paramPublisher = mh.advertise<FloatVectorStamped>( name + "params", 1 );
		reg.manager.Initialize( mh );
		GetParamRequired( mh, "param_output_path", reg.paramOutputPath );
		GetParam( mh, "weight", reg.weight, 1.0 );
	}

	if( _estRegistry.count( "transition" ) == 0 )
	{
		throw std::runtime_error( "No transition model specified." );
	}

	_optimizer = std::make_shared<ClipOptimizer>( _estRegistry.count(  ) )

	unsigned int infoBuffSize;
	GetParam( ph, "info_buffer_size", infoBuffSize, 0 );
	_infoSubscriber = nh.subscribe( "filter_steps", infoBuffSize,
	                                 &CovarianceModelLearner::InfoMsgCallback,
	                                 this );

	double optimizeRate;
	GetParamRequired( ph, "process_rate", optimizeRate );
	_optimizerTimer = nh.createTimer( ros::Duration( 1.0 / optimizeRate ),
	                                  &CovarianceModelLearner::OptimizerTimerCallback,
	                                  this );
}

void CovarianceModelLearner::InfoMsgCallback( const FilterStepInfo::ConstPtr& msg )
{

}

void CovarianceModelLearner::OptimizerTimerCallback( const ros::TimerEvent& event )
{

}

}