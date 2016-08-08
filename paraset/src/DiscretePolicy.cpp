#include "paraset/DiscretePolicy.h"
#include "paraset/SetRuntimeParameter.h"
#include "paraset/DiscreteParamAction.h"

#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>
#include <ros/service.h>

using namespace paraset;

namespace argus
{

DiscretePolicy::DiscretePolicy() {}

void DiscretePolicy::Initialize( ros::NodeHandle& ph )
{
	YAML::Node policyParams;
	GetParamRequired( ph, "parameters", policyParams );

	// Read all policy parameters to control from ROS parameters
	YAML::const_iterator iter;
	for( iter = policyParams.begin(); iter != policyParams.end(); ++iter )
	{
		_parameters.emplace_back();
		ParameterRegistration& registration = _parameters.back();

		// Get name of parameter
		registration.name = iter->first.as<std::string>();
		
		// Get service to set parameter
		const YAML::Node& sub = iter->second;
		std::string servicePath;
		GetParamRequired( sub, "set_service", servicePath );
		ros::service::waitForService( servicePath );
		registration.setService = ph.serviceClient<SetRuntimeParameter>( servicePath );

		// Get type of parameter
		registration.type = StringToParamType( TryYamlField<std::string>( sub, "type" ) );

		// Get values of parameter
		switch( registration.type )
		{
			case PARAM_NUMERIC:
			{
				std::vector<double> params = TryYamlField<std::vector<double>>( sub, "values" );
				registration.values = ConvertToParamVariants( params );
				break;
			}
			case PARAM_STRING:
			{
				std::vector<std::string> params = TryYamlField<std::vector<std::string>>( sub, "values" );
				registration.values = ConvertToParamVariants( params );
				break;
			}
			case PARAM_BOOLEAN:
			{
				std::vector<bool> params = TryYamlField<std::vector<bool>>( sub, "values" );
				registration.values = ConvertToParamVariants( params );
				break;
			}
			default:
			{
				throw std::runtime_error( "DiscretePolicy: Invalid parameter type for: " + registration.name );
			}
		}

		_numSettings.push_back( registration.values.size() );
	}
}

unsigned int DiscretePolicy::GetNumOutputs() const
{
	return _parameters.size();
}

void DiscretePolicy::SetOutputIndices( const std::vector<unsigned int>& inds )
{
	if( inds.size() != _parameters.size() )
	{
		throw std::runtime_error( "DiscretePolicy: Incorrect number of output indices." );
	}

	SetRuntimeParameter::Request req;
	SetRuntimeParameter::Response res;
	for( unsigned int i = 0; i < _parameters.size(); ++i )
	{
		RuntimeParam outputParam = _parameters[i].values[ inds[i] ];
		req.param = ParamVariantToMsg( outputParam );

		ROS_INFO_STREAM( "Setting " << _parameters[i].name << " to: " << ParamVariantToString( outputParam ) );

		if( !_parameters[i].setService.call( req, res ) )
		{
			ROS_WARN_STREAM( "DiscretePolicy: Could not set parameter: " << _parameters[i].name );
		}
		if( MsgToParamVariant( res.actual ) != outputParam )
		{
			ROS_WARN_STREAM( "DiscretePolicy: Actual differs from request." );
		}
	}
}

const std::vector<unsigned int>& DiscretePolicy::GetNumSettings() const
{
	return _numSettings;
}

unsigned int DiscretePolicy::GetNumCombinations() const
{
	unsigned int prod = 1;
	BOOST_FOREACH( unsigned int p, _numSettings )
	{
		prod *= p;
	}
	return prod;
}

std::ostream& operator<<( std::ostream& os, const DiscretePolicy& policy )
{
	os << "Discrete policy:" << std::endl;
	BOOST_FOREACH( const DiscretePolicy::ParameterRegistration& reg, policy._parameters )
	{
		os << "\t" << reg.name << ":" << std::endl;
		os << "\t\ttype:" << ParamTypeToString( reg.type ) << std::endl;
		os << "\t\tvalues:";
		BOOST_FOREACH( const RuntimeParam& val, reg.values )
		{
			os << val << " ";
		}
		os << std::endl;
	}
	return os;
}

}