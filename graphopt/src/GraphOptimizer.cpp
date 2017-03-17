#include "graphopt/GraphOptimizer.h"
#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

isam::Method StringToMethod( const std::string& method )
{
	if( method == "gauss_newton" )
	{
		return isam::GAUSS_NEWTON;
	}
	if( method == "levenberg_marquardt" )
	{
		return isam::LEVENBERG_MARQUARDT;
	}
	if( method == "dog_leg" )
	{
		return isam::DOG_LEG;
	}
	throw std::invalid_argument( "Invalid method: " + method );
}

GraphOptimizer::GraphOptimizer() 
{
	_optimizer = std::make_shared<isam::Slam>();
}

GraphOptimizer::GraphOptimizer( ros::NodeHandle& ph )
{
	_optimizer = std::make_shared<isam::Slam>();

	isam::Properties props;
	GetParam( ph, "verbose", props.verbose );
	GetParam( ph, "quiet", props.quiet );
	GetParam( ph, "force_numerical_jacobian", props.force_numerical_jacobian );

	std::string method;
	if( GetParam( ph, "method", method ) )
	{
		props.method = StringToMethod( method );
	}

	if( props.method == isam::GAUSS_NEWTON )
	{
		// No GN-specific parameters to parse
	}
	else if( props.method == isam::LEVENBERG_MARQUARDT )
	{
		GetParam( ph, "lambda_0", props.lm_lambda0 );
		GetParam( ph, "lambda_factor", props.lm_lambda_factor );
	}
	else if( props.method == isam::DOG_LEG )
	{
		GetParam( ph, "epsilon_1", props.epsilon1 );
		GetParam( ph, "epsilon_2", props.epsilon2 );
		GetParam( ph, "epsilon_3", props.epsilon3 );
		GetParam( ph, "epsilon_abs", props.epsilon_abs );
		GetParam( ph, "epsilon_rel", props.epsilon_rel );
	}

	GetParam( ph, "max_iters", props.max_iterations );
	GetParam( ph, "update_period", props.mod_update );
	GetParam( ph, "batch_period", props.mod_batch );
	GetParam( ph, "solve_period", props.mod_solve );

	_optimizer->set_properties( props );
}

isam::Slam& GraphOptimizer::GetOptimizer()
{
	return *_optimizer;
}

const isam::Slam& GraphOptimizer::GetOptimizer() const
{
	return *_optimizer;
}

}