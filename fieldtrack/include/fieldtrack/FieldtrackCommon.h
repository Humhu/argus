#pragma once

#include "argus_msgs/CompactOdometry.h"
#include "nav_msgs/Odometry.h"

#include "argus_utils/geometry/PoseSE3.h"

#include <memory>

namespace argus
{

class TargetState
{
public:
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	typedef std::shared_ptr<TargetState> Ptr;
	typedef PoseSE3 PoseType;
	typedef PoseSE3::TangentVector VelocityType;
	typedef PoseSE3::CovarianceMatrix CovarianceType;
	
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
