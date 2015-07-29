#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include "argus_common/PoseSE3.h"

namespace argus_common
{
	
	/*! \brief Read/Write a PoseSE3 object to the node */
	YAML::Node SetPoseYaml( const PoseSE3& pose );
	bool GetPoseYaml( const YAML::Node& node, PoseSE3& pose );
	
	/*! \brief Read/Write a Quaternion object to the node */
	YAML::Node SetQuaternionYaml( const Eigen::Quaterniond& quat );
	bool GetQuaternionYaml( const YAML::Node& node, Eigen::Quaterniond& quat );
	
	/*! \brief Read/Write a Position object to the node */
	YAML::Node SetPositionYaml( const Eigen::Translation3d& trans );
	bool GetPositionYaml( const YAML::Node& node, Eigen::Translation3d& trans );
	
	/*! \brief Read/Write a matrix to the node */
	YAML::Node SetMatrixYaml( const Eigen::MatrixXd& mat, 
							  std::string idDim = "dimensions", std::string idVal = "values" );
	bool GetMatrixYaml( const YAML::Node& node, Eigen::MatrixXd& mat, 
						std::string idDim = "dimensions", std::string idVal = "values" );
	
}