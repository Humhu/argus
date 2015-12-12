#pragma once

#include "argus_msgs/CompactOdometry.h"
#include "nav_msgs/Odometry.h"

#include "argus_utils/PoseSE3.h"

#include <memory>

namespace fieldtrack
{

class TargetState
{
public:
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	typedef std::shared_ptr<TargetState> Ptr;
	typedef argus_utils::PoseSE3 PoseType;
	typedef argus_utils::PoseSE3::TangentVector VelocityType;
	typedef argus_utils::PoseSE3::CovarianceMatrix CovarianceType;
	
	PoseType pose;
	VelocityType velocity;
	CovarianceType poseCovariance;
	CovarianceType velocityCovariance;
	
	TargetState();

};
	
argus_msgs::CompactOdometry TargetToCompactOdom( const TargetState& state );
TargetState CompactOdomToTarget( const argus_msgs::CompactOdometry& odom );

nav_msgs::Odometry TargetToOdom( const TargetState& state );
TargetState OdomToTarget( const nav_msgs::Odometry& odom );

}
