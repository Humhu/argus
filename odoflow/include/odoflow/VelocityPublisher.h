#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "argus_utils/geometry/PoseSE3.h"
#include "paraset/ParameterManager.hpp"

namespace argus
{
/*! \brief Differentiates a sequence of poses and publishes the estimated 
 * velocity.
 */
class VelocityPublisher
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	VelocityPublisher( ros::NodeHandle& nh, ros::NodeHandle& ph );

	/*! \brief Report a pose at the specified time. Scales translation component
	 * and if enough time has passed since last differentiation, publishes a twist
	 * message with the specified frame ID.
	 */
	void ReportPose( const ros::Time& time, const std::string& frameId,
	                 const PoseSE3& pose, double transScale = 1.0 );
	
	/*! \brief Returns the last successful differentiated velocity. Returns all
	 * zeros if no differentiations have occurred since reset.
	 */
	const PoseSE3::TangentVector& GetLastVelocity() const;
	
	/*! \brief Resets the pose chain, zeros the last velocity, and sets the time
	 * for the new first pose.
	*/
	void Reset( const ros::Time& time );

private:

	ros::Publisher _velPub;
	NumericParam _minTimeDelta;

	ros::Time _lastTime;
	PoseSE3 _lastPose;
	PoseSE3::TangentVector _lastVelocity;
};
}