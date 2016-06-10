#pragma once

#include <yaml-cpp/yaml.h>

#include "covreg/CovarianceEstimatorInfo.h"
#include "covreg/NeuralNetworkInfo.h"

namespace argus
{

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