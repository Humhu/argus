#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include "argus_common/PoseSE3.h"

namespace argus_common
{
	
	/*! \brief Read/Write a PoseSE3 object to the node */
	void SetPoseYaml( YAML::Node& node, const PoseSE3& pose, 
					  std::string id = "pose" );
	bool GetPoseYaml( YAML::Node& node, PoseSE3& pose, 
					  std::string id = "pose" );
	
	/*! \brief Read/Write a Quaternion object to the node */
	void SetQuaternionYaml( YAML::Node& node, const Eigen::Quaterniond& quat, 
							std::string id = "quaternion" );
	bool GetQuaternionYaml( YAML::Node& node, Eigen::Quaterniond& quat, 
							std::string id = "quaternion" );
	
	/*! \brief Read/Write a Position object to the node */
	void SetPositionYaml( YAML::Node& node, const Eigen::Translation3d& trans, 
						  std::string id = "position" );
	bool GetPositionYaml( YAML::Node& node, Eigen::Translation3d& trans, 
						  std::string id = "position" );
	
	/*! \brief Read/Write a matrix to the node */
	void SetMatrixYaml( YAML::Node& node, const Eigen::MatrixXd& mat, 
						std::string idDim = "dimensions", std::string idVal = "values" );
	bool GetMatrixYaml( YAML::Node& node, Eigen::MatrixXd& mat, 
						std::string idDim = "dimensions", std::string idVal = "values" );
	
}