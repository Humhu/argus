#include "paraset/ContinuousParamPolicy.h"
#include "paraset/SetRuntimeParameter.h"
#include "argus_utils/utils/ParamUtils.h"
#include "paraset/ParasetCommon.h"

#include <ros/service.h>
#include <boost/foreach.hpp>

namespace argus
{

ContinuousParamPolicy::ContinuousParamPolicy() {}

void ContinuousParamPolicy::Initialize( ros::NodeHandle& nh,
                                        ros::NodeHandle& ph )
{
	YAML::Node policyParams;
	GetParamRequired( ph, "parameters", policyParams );

	// Read all parameters
	YAML::const_iterator iter;
	for( iter = policyParams.begin(); iter != policyParams.end(); ++iter )
	{
		_parameters.emplace_back();
		ParameterRegistration& registration = _parameters.back();

		// Get name
		registration.name = iter->first.as<std::string>();
		ROS_INFO_STREAM( "Parsing parameter: " << registration.name );

		const YAML::Node& info = iter->second;
		GetParam( info, 
		          "lower_limit", 
		          registration.lowerLimit, 
		          -std::numeric_limits<double>::infinity() );
		GetParam( info, 
		          "upper_limit", 
		          registration.upperLimit, 
		          std::numeric_limits<double>::infinity() );

		// Get service path
		std::string servicePath;
		GetParamRequired( info, "set_service", servicePath );
		ros::service::waitForService( servicePath );
		registration.setService = nh.serviceClient<paraset::SetRuntimeParameter>( servicePath );
	}

	_upperLimits = VectorType( _parameters.size() );
	_lowerLimits = VectorType( _parameters.size() );
	for( unsigned int i = 0; i < _parameters.size(); ++i )
	{
		_lowerLimits(i) = _parameters[i].lowerLimit;
		_upperLimits(i) = _parameters[i].upperLimit;
	}
}

unsigned int ContinuousParamPolicy::GetNumOutputs() const
{
	return _parameters.size();
}

std::vector<std::string> ContinuousParamPolicy::GetParameterNames() const
{
	std::vector<std::string> names;
	names.reserve( _parameters.size() );
	for( unsigned int i = 0; i < _parameters.size(); ++i )
	{
		names.push_back( _parameters[i].name );
	}
	return names;
}

const VectorType& ContinuousParamPolicy::GetLowerLimits() const
{
	return _lowerLimits;
}

const VectorType& ContinuousParamPolicy::GetUpperLimits() const
{
	return _upperLimits;
}

void ContinuousParamPolicy::SetOutput( const VectorType& output )
{
	if( output.size() != _parameters.size() )
	{
		throw std::runtime_error( "ContinuousParamPolicy: Invalid number of outputs." );
	}

	std::vector<RuntimeParam> params;
	params.reserve( output.size() );
	for( unsigned int i = 0; i < output.size(); ++i )
	{
		double out = output(i);
		const ParameterRegistration& reg = _parameters[i];
		out = std::max( out, reg.lowerLimit );
		out = std::min( out, reg.upperLimit );
		params.emplace_back( out );
	}

	paraset::SetRuntimeParameter::Request req;
	paraset::SetRuntimeParameter::Response res;
	for( unsigned int i = 0; i < _parameters.size(); ++i )
	{
		req.param = ParamVariantToMsg( params[i] );
		if( !_parameters[i].setService.call( req, res ) )
		{
			ROS_WARN_STREAM( "ContinuousParamPolicy: Could not set parameter: " << _parameters[i].name );
		}
		else if( MsgToParamVariant( res.actual ) != params[i] )
		{
			// ROS_WARN_STREAM( "ContinuousParamPolicy: Actual differs from request." );
		}
	}
}

std::ostream& operator<<( std::ostream& os, const ContinuousParamPolicy& policy )
{
	os << "Continuous policy:" << std::endl;
	BOOST_FOREACH( const ContinuousParamPolicy::ParameterRegistration& reg, policy._parameters )
	{
		os << "\t" << reg.name << std::endl;
	}
	return os;
}

}