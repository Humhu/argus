#include "covreg/EstimatorInfoParsers.h"
#include "argus_utils/utils/YamlUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include <fstream>

using namespace covreg;

namespace argus
{

template <>
bool ParseInfo<CovarianceEstimatorInfo>( const YAML::Node& yaml,
                                         CovarianceEstimatorInfo& info )
{
	bool ret = GetYamlField( yaml, "parameters", info.parameters )
	        && GetYamlField( yaml, "source_name", info.source_name );
	return ret;
}

template <>
void PopulateInfo<CovarianceEstimatorInfo>( const CovarianceEstimatorInfo& info,
                                            YAML::Node& yaml )
{
	yaml["source_name"] = info.source_name;
	yaml["parameters"] = info.parameters;
}
/*
bool ParseEstimatorInfo( const YAML::Node& yaml,
                         covreg::CovarianceEstimatorInfo& info )
{
	bool ret = GetYamlField( yaml, "source_name", info.source_name )
	        && GetYamlField( yaml, "correlation_parameters", info.correlation_parameters )
	        && ParseNeuralNetInfo( yaml["variance_info"], info.variance_info );
	MatrixType offset;
	ret = ret && GetMatrixYaml( yaml["offset"], offset );
	if( ret ) { info.offset = MatrixToMsg( offset ); }
	return ret;
}

void PopulateEstimatorInfo( const covreg::CovarianceEstimatorInfo& info,
                            YAML::Node& yaml )
{
	yaml["source_name"] = info.source_name;
	yaml["correlation_parameters"] = info.correlation_parameters;
	YAML::Node varNode;
	PopulateNeuralNetInfo( info.variance_info, varNode );
	yaml["variance_info"] = varNode;
	yaml["offset"] = SetMatrixYaml( MsgToMatrix( info.offset ) );
}

bool ReadEstimatorInfo( const std::string& path, 
                        covreg::CovarianceEstimatorInfo& info )
{
	YAML::Node yaml;
	try 
	{
		yaml = YAML::LoadFile( path );
	}
	catch( YAML::BadFile e ) { return false; }
	return ParseEstimatorInfo( yaml, info );
}

bool WriteEstimatorInfo( const std::string& path, 
                         const covreg::CovarianceEstimatorInfo& info )
{
	std::ofstream output( path );
	if( !output.is_open() ) { return false; }
	YAML::Node yaml;
	PopulateEstimatorInfo( info, yaml );
	output << yaml;
	return true;
}

bool ParseNeuralNetInfo( const YAML::Node& yaml,
                         covreg::NeuralNetworkInfo& info )
{
	return GetYamlField( yaml, "activation_type", info.activation_type )
	    && GetYamlField( yaml, "activation_params", info.activation_params )
	    && GetYamlField( yaml, "output_dim", info.output_dim )
	    && GetYamlField( yaml, "input_dim", info.input_dim )
	    && GetYamlField( yaml, "num_hidden_layers", info.num_hidden_layers )
	    && GetYamlField( yaml, "parameters", info.parameters );
}

void PopulateNeuralNetInfo( const covreg::NeuralNetworkInfo& info,
                            YAML::Node& yaml )
{
	yaml["activation_type"] = info.activation_type;
	yaml["activation_params"] = info.activation_params;
	yaml["output_dim"] = info.output_dim;
	yaml["input_dim"] = info.input_dim;
	yaml["num_hidden_layers"] = info.num_hidden_layers;
	yaml["parameters"] = info.parameters;
}

bool ReadNeuralNetInfo( const std::string& path,
                        covreg::NeuralNetworkInfo& info )
{
	YAML::Node yaml;
	try 
	{
		yaml = YAML::LoadFile( path );
	}
	catch( YAML::BadFile e ) { return false; }
	return ParseNeuralNetInfo( yaml, info );
}

bool WriteNeuralNetInfo( const std::string& path,
                         const covreg::NeuralNetworkInfo& info )
{
	std::ofstream output( path );
	if( !output.is_open() ) { return false; }
	YAML::Node yaml;
	PopulateNeuralNetInfo( info, yaml );
	output << yaml;
	return true;
}
*/
}