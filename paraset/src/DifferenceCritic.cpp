#include "paraset/DifferenceCritic.h"
#include "argus_utils/utils/ParamUtils.h"

#include "paraset/MonteCarloValue.h"
#include "paraset/RunningAverageBaseline.h"

namespace argus
{

DifferenceCritic::DifferenceCritic()
{}

void DifferenceCritic::Initialize( ros::NodeHandle& nh,
                                   ros::NodeHandle& ph )
{
	ros::NodeHandle vh( ph.resolveName( "value" ) );
	ros::NodeHandle bh( ph.resolveName( "baseline" ) );

	std::string valueType;
	GetParamRequired( vh, "type", valueType );
	if( valueType == "monte_carlo" )
	{
		_valueFunction = std::make_shared<MonteCarloValue>();
		_valueFunction->Initialize( nh, vh );
	}
	else
	{
		throw std::invalid_argument( "DifferenceCritic: Unsupported value function type: " + valueType );
	}

	std::string baselineType;
	GetParamRequired( bh, "type", baselineType );
	if( baselineType == "running_average" )
	{
		_baselineFunction = std::make_shared<RunningAverageBaseline>( _valueFunction );
		_baselineFunction->Initialize( nh, bh );
	}
	else
	{
		throw std::invalid_argument( "DifferenceCritic: Unsupported baseline type: " + baselineType );
	}
}

double DifferenceCritic::Evaluate( const ParamAction& act ) const
{
	return _valueFunction->Evaluate( act ) - _baselineFunction->Evaluate( act );
}

}