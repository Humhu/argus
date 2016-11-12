#pragma once

#include "argus_msgs/FilterUpdate.h"
#include "nav_msgs/Odometry.h"

#include "argus_utils/geometry/PoseSE3.h"

#include <memory>

namespace argus
{

// The C++ counterpart of nav_msgs::Odometry
struct TargetState
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	std::string referenceFrame;
	std::string bodyFrame;
	ros::Time timestamp;
	PoseSE3 pose;
	PoseSE3::CovarianceMatrix poseCovariance;
	PoseSE3::TangentVector velocity;
	PoseSE3::CovarianceMatrix velocityCovariance;
	
	TargetState();
	TargetState( const nav_msgs::Odometry& odom );

	nav_msgs::Odometry ToMsg() const;
};

// The C++ counterpart of argus_msgs::FilterUpdate
struct FilterUpdate
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	std::string sourceName;
	ros::Time timestamp;
	MatrixType observationMatrix;
	MatrixType observationCov;
	VectorType observation;

	FilterUpdate();
	FilterUpdate( const argus_msgs::FilterUpdate& msg );

	argus_msgs::FilterUpdate ToMsg() const;
};


}
