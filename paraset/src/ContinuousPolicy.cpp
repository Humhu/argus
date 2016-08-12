#include "paraset/ContinuousPolicy.h"
#include "paraset/SetRuntimeParameter.h"
#include "argus_utils/utils/ParamUtils.h"

#include <ros/service.h>
#include <boost/foreach.hpp>

using namespace paraset;

namespace argus
{

ContinuousPolicy::ContinuousPolicy() {}

void ContinuousPolicy::Initialize( ros::NodeHandle& nh,
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

		// Get service path
		const YAML::Node& sub = iter->second;
		std::string servicePath;
		GetParamRequired( sub, "set_service", servicePath );
		ros::service::waitForService( servicePath );
		registration.setService = nh.serviceClient<SetRuntimeParameter>( servicePath );
	}
}

unsigned int ContinuousPolicy::GetNumOutputs() const
{
	return _parameters.size();
}

std::vector<std::string> ContinuousPolicy::GetParameterNames() const
{
	std::vector<std::string> names;
	names.reserve( _parameters.size() );
	for( unsigned int i = 0; i < _parameters.size(); ++i )
	{
		names.push_back( _parameters[i].name );
	}
	return names;
}

void ContinuousPolicy::SetOutput( const VectorType& output )
{
	if( output.size() != _parameters.size() )
	{
		throw std::runtime_error( "ContinuousPolicy: Invalid number of outputs." );
	}

	std::vector<RuntimeParam> params;
	params.reserve( output.size() );
	for( unsigned int i = 0; i < output.size(); ++i )
	{
		params.emplace_back( output(i) );
	}

	SetRuntimeParameter::Request req;
	SetRuntimeParameter::Response res;
	for( unsigned int i = 0; i < _parameters.size(); ++i )
	{
		req.param = ParamVariantToMsg( params[i] );
		if( !_parameters[i].setService.call( req, res ) )
		{
			ROS_WARN_STREAM( "ContinuousPolicy: Could not set parameter: " << _parameters[i].name );
		}
		else if( MsgToParamVariant( res.actual ) != params[i] )
		{
			// ROS_WARN_STREAM( "ContinuousPolicy: Actual differs from request." );
		}
	}
}

std::ostream& operator<<( std::ostream& os, const ContinuousPolicy& policy )
{
	os << "Continuous policy:" << std::endl;
	BOOST_FOREACH( const ContinuousPolicy::ParameterRegistration& reg, policy._parameters )
	{
		os << "\t" << reg.name << std::endl;
	}
	return os;
}

}