#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "argus_utils/geometry/PoseSE3.h"
#include "paraset/ParameterManager.hpp"

namespace argus
{
class VelocityPublisher
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VelocityPublisher( ros::NodeHandle& nh, ros::NodeHandle& ph );

	void ReportPose( const ros::Time& time, const std::string& frameId,
	                 const PoseSE3& pose, double transScale = 1.0 );
    const PoseSE3::TangentVector& GetLastVelocity() const;
	void Reset( const ros::Time& time );

private:

	ros::Publisher _velPub;
	NumericParam _minTimeDelta;

	ros::Time _lastTime;
	PoseSE3 _lastPose;
    PoseSE3::TangentVector _lastVelocity;
};
}