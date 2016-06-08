#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include "argus_msgs/FilterUpdate.h"
#include "argus_msgs/FilterStepInfo.h"

#include "argus_utils/filters/FilterTypes.h"
#include "argus_utils/geometry/PoseSE3.h"

#include <unordered_map>

namespace argus
{

// TODO How to serialize SE2 covariance into odom message?

/*! \brief Subscribes to velocities from odometers and relative pose estimates
 * from localization sensors. Outputs nav_msgs::Odometry */
class SimpleStateEstimator
{
public:
	
	typedef ConstantAccelFilterSE3 FilterType;
	typedef FilterType::PoseType PoseType;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SimpleStateEstimator( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	FilterType filter;
	FilterType::FullCovType Qrate;
	ros::Time filterTime;

	bool twoDimensional;
	
	std::string referenceFrame;
	std::string bodyFrame;
	std::shared_ptr<ros::Timer> updateTimer;

	// Subscribers to argus_msgs::FilterUpdate
	std::unordered_map<std::string, ros::Subscriber> updateSubs;

	ros::Publisher odomPub; // Publishes nav_msgs::Odometry
	ros::Publisher stepPub; // Publishes argus_msgs::FilterStepInfo
	
	void UpdateCallback( const argus_msgs::FilterUpdate::ConstPtr& msg );
	argus_msgs::FilterStepInfo PoseUpdate( const PoseType& pose, 
	                                       const MatrixType& R );
	argus_msgs::FilterStepInfo DerivsUpdate( const VectorType& derivs, 
	                                         const MatrixType& C, 
	                                         const MatrixType& R );
	// TODO
	// argus_msgs::FilterStepInfo PositionUpdate( const Translation3Type& pos, 
	//                                            const MatrixType& R );
	// argus_msgs::FilterStepInfo OrientationUpdate( const QuaternionType& ori, 
	//                                               const MatrixType& R );
	// argus_msgs::FilterStepInfo JointUpdate( const PoseSE3& pose, 
	//                                         const VectorType& derivs, 
	//                                         const MatrixType& C, 
	//                                         const MatrixType& R );
	void TimerCallback( const ros::TimerEvent& event );
	void EnforceTwoDimensionality();
};

}
