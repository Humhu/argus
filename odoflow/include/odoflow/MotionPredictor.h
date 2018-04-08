#pragma once

#include <ros/ros.h>

#include "extrinsics_array/ExtrinsicsInterface.h"
#include "argus_utils/geometry/VelocityIntegrator.hpp"

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include "argus_utils/geometry/PoseSE2.h"
#include "paraset/ParameterManager.hpp"

namespace argus
{
/*! \brief Subscribes to odometry/velocity messages to predict the
 * displacement of the camera between two times.
 */
class MotionPredictor
{
public:

	MotionPredictor( ros::NodeHandle& nh, ros::NodeHandle& ph );

	/*! \brief Predict the motion of the specified frame between the specified times.
	Also applies a scale to the translational component. If integrated entropy exceeds
	threshold, returns zero prediction. */
	PoseSE2 PredictMotion( const ros::Time& fromTime,
	                       const ros::Time& currTime,
	                       const std::string& camFrame,
	                       double scale );

	/*! \brief Clears all bufferered odometry/velocity messages.
	*/
    void Reset();

private:

	void OdometryCallback( const nav_msgs::Odometry::ConstPtr& msg );
	void TwistStampedCallback( const geometry_msgs::TwistStamped::ConstPtr& msg );

	bool _enablePrediction;
	ros::Subscriber _motionSub;
	std::string _odomFrame;
	VelocityIntegratorSE3 _velIntegrator;
	ExtrinsicsInterface _extrinsics;

	NumericParam _maxPredictEntropy;    
};
}