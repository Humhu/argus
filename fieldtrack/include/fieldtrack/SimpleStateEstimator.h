#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "broadcast/BroadcastReceiver.h"

#include "covreg/CovarianceManager.h"

#include "argus_msgs/FilterUpdate.h"
#include "argus_msgs/FilterStepInfo.h"

#include "argus_utils/filters/FilterTypes.h"
#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <unordered_map>
#include <map>

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

	SimpleStateEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	MatrixType GetCovarianceRate( const ros::Time& time );

private:
	
	FilterType _filter;
	FilterType::FullCovType _Qrate;
	CovarianceManager _Qestimator;
	ros::Time _filterTime;
	ros::Duration _updateLag;

	bool twoDimensional;
	bool velocityOnly;
	
	std::string _referenceFrame;
	std::string _bodyFrame;
	ros::Timer _updateTimer;
	unsigned int _infoNumber;

	// Subscribers to argus_msgs::FilterUpdate
	std::unordered_map<std::string, ros::Subscriber> _updateSubs;

	Mutex _bufferMutex;
	typedef std::map<ros::Time, argus_msgs::FilterUpdate> UpdateBuffer;
	UpdateBuffer _updateBuffer;

	ros::Publisher _odomPub; // Publishes nav_msgs::Odometry
	ros::Publisher _stepPub; // Publishes argus_msgs::FilterStepInfo
	

	void UpdateCallback( const argus_msgs::FilterUpdate::ConstPtr& msg );
	
	// Process all messages until the specified time
	void ProcessUpdateBuffer( const ros::Time& until );

	// Predict the filter to the specified time
	void PredictUntil( const ros::Time& time );

	// Process a given message by predicting to that time and then updating
	void ProcessUpdate( const argus_msgs::FilterUpdate& msg );

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

	void SquashPoseUncertainty();
	void EnforceTwoDimensionality();
};

}
