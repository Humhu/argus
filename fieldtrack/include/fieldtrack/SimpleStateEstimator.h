#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "broadcast/BroadcastReceiver.h"
#include "broadcast/BroadcastTransmitter.h"

#include "covreg/CovarianceManager.h"
#include "covreg/AdaptiveCovarianceEstimator.h"

#include "argus_msgs/FilterUpdate.h"
#include "argus_msgs/FilterStepInfo.h"

#include "argus_utils/filters/FilterTypes.h"
#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <unordered_map>
#include <map>

namespace argus
{

class SimpleStateEstimator;

struct StampedFilter
{

	typedef ConstantAccelFilterSE3 FilterType;
	typedef FilterType::PoseType PoseType;

	StampedFilter( SimpleStateEstimator& p );

	// Predict the filter to the specified time
	argus_msgs::FilterStepInfo PredictUntil( const ros::Time& time );

	// Process a given message by predicting to that time and then updating
	// Returns predict and update step info
	std::pair<argus_msgs::FilterStepInfo,
	          argus_msgs::FilterStepInfo>
	 ProcessUpdate( const argus_msgs::FilterUpdate& msg );

	argus_msgs::FilterStepInfo PoseUpdate( const PoseType& pose,
	                                       const MatrixType& R );
	argus_msgs::FilterStepInfo DerivsUpdate( const VectorType& derivs,
	                                         const MatrixType& C,
	                                         const MatrixType& R );

	SimpleStateEstimator& parent;
	FilterType filter;
	ros::Time filterTime;
	unsigned int infoNumber;

	void SquashPoseUncertainty();
	void EnforceTwoDimensionality();

	nav_msgs::Odometry GetOdomMsg() const;

};

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
	
	StampedFilter _filter;

	FilterType::FullCovType _Qrate;
	bool _usingAdaptive;
	CovarianceManager _Qestimator;
	AdaptiveCovarianceEstimator _Qadapter;

	ros::Duration _updateLag;

	bool twoDimensional;
	bool velocityOnly;
	
	std::string _referenceFrame;
	std::string _bodyFrame;
	ros::Timer _updateTimer;
	unsigned int _infoNumber;

	// BroadcastTransmitter _xlTx;

	// Subscribers to argus_msgs::FilterUpdate
	std::unordered_map<std::string, ros::Subscriber> _updateSubs;

	Mutex _bufferMutex;
	Mutex _filterMutex;
	typedef std::map<ros::Time, argus_msgs::FilterUpdate> UpdateBuffer;
	UpdateBuffer _updateBuffer;

	ros::Publisher _latestOdomPub; // Publishes nav_msgs::Odometry
	ros::Publisher _laggedOdomPub; // Publishes lagged odometry
	ros::Publisher _stepPub; // Publishes argus_msgs::FilterStepInfo
	
	MatrixType lastCov;

	void UpdateCallback( const argus_msgs::FilterUpdate::ConstPtr& msg );
	
	// Process all messages until the specified time
	void ProcessUpdateBuffer( const ros::Time& until );

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
};

}
