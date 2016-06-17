#pragma once

#include <yaml-cpp/yaml.h>
#include <fstream>

#include "covreg/CovarianceEstimatorInfo.h"
#include "covreg/NeuralNetworkInfo.h"

namespace argus
{

template <typename Info>
bool ParseInfo( const YAML::Node& yaml, Info& info );

template <typename Info>
void PopulateInfo( const Info& info, YAML::Node& yaml );

template <typename Info>
bool ReadInfo( const std::string& path, Info& info )
{
	YAML::Node yaml;
	try
	{
		yaml = YAML::LoadFile( path );
	}
	catch( YAML::BadFile e ) { return false; }
	return ParseInfo<Info>( yaml, info );
}

template <typename Info>
bool WriteInfo( const std::string& path, const Info& info )
{
	std::ofstream output( path );
	if( !output.is_open() ) { return false; }
	YAML::Node yaml;
	PopulateInfo<Info>( info, yaml );
	output << yaml;
	return true;
}

template <>
bool ParseInfo<covreg::CovarianceEstimatorInfo>( const YAML::Node& yaml,
                                                 covreg::CovarianceEstimatorInfo& info );

template <>
void PopulateInfo<covreg::CovarianceEstimatorInfo>( const covreg::CovarianceEstimatorInfo& info,
                                                    YAML::Node& yaml );

/*! \brief Parses estimator parameters from a YAML object. Returns success. */
bool ParseEstimatorInfo( const YAML::Node& yaml,
                         covreg::CovarianceEstimatorInfo& info );

/*! \brief Populates a YAML node from estimator parameters. */
void PopulateEstimatorInfo( const covreg::CovarianceEstimatorInfo& info,
                            YAML::Node& yaml );

/*! \brief Reads estimator parameters from a YAML files. Returns success. */
bool ReadEstimatorInfo( const std::string& path, 
                        covreg::CovarianceEstimatorInfo& info );

/*! \brief Writes estimator parameters to a YAML file. Returns success. */
bool WriteEstimatorInfo( const std::string& path, 
                         const covreg::CovarianceEstimatorInfo& info );



bool ParseNeuralNetInfo( const YAML::Node& yaml,
                         covreg::NeuralNetworkInfo& info );

void PopulateNeuralNetInfo( const covreg::NeuralNetworkInfo& info,
                            YAML::Node& yaml );

bool ReadNeuralNetInfo( const std::string& path,
                        covreg::NeuralNetworkInfo& info );

bool WriteNeuralNetInfo( const std::string& path,
                         const covreg::NeuralNetworkInfo& info );

}