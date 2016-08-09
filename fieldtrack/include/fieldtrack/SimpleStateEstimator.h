#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "broadcast/BroadcastReceiver.h"
#include "broadcast/BroadcastTransmitter.h"

#include "covreg/CovarianceManager.h"
#include "covreg/AdaptiveCovarianceEstimator.h"

#include "fieldtrack/ResetFilter.h"

#include "argus_msgs/FilterUpdate.h"
#include "argus_msgs/FilterStepInfo.h"
#include "argus_msgs/MatrixFloat64.h"

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
	typedef std::pair<argus_msgs::FilterStepInfo,argus_msgs::FilterStepInfo> InfoPair;

	StampedFilter( SimpleStateEstimator& p );

	// Predict the filter to the specified time
	argus_msgs::FilterStepInfo PredictUntil( const ros::Time& time );

	// Process a given message by predicting to that time and then updating
	// Returns predict and update step info
	 bool ProcessUpdate( const argus_msgs::FilterUpdate& msg, 
	                     InfoPair& info );

	argus_msgs::FilterStepInfo PoseUpdate( const PoseType& pose,
	                                       const MatrixType& R );
	argus_msgs::FilterStepInfo DerivsUpdate( const VectorType& derivs,
	                                         const MatrixType& C,
	                                         const MatrixType& R );

	bool CheckPoseUpdate( const PoseType& pose, const MatrixType& R );
	bool CheckDerivsUpdate( const VectorType& derivs,
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

	void Reset( double waitTime, const ros::Time& time );

	// TODO
	double _likelihoodThreshold;

private:
	
	StampedFilter _filter;

	FilterType::FullCovType _initCov;
	FilterType::FullCovType _Qrate;
	bool _usingAdaptiveTrans;
	
	// TODO For some reason an uninitialiezd estimtaor seems to cause a growing memory footprint over time...
	CovarianceManager _Qestimator;
	AdaptiveTransitionCovarianceEstimator _Qadapter;

	ros::Duration _updateLag;

	bool twoDimensional;
	bool velocityOnly;

	std::string _referenceFrame;
	std::string _bodyFrame;
	ros::Timer _updateTimer;
	unsigned int _infoNumber;

	// BroadcastTransmitter _xlTx;

	// Subscribers to argus_msgs::FilterUpdate
	struct UpdateRegistration
	{
		ros::Subscriber sub;
		bool usingAdaptive;
		AdaptiveObservationCovarianceEstimator Radapter;
	};
	typedef std::unordered_map<std::string, UpdateRegistration> UpdateRegistry;
	UpdateRegistry _updateSubs;

	Mutex _mutex;
	typedef std::map<ros::Time, argus_msgs::FilterUpdate> UpdateBuffer;
	UpdateBuffer _updateBuffer;

	ros::Publisher _latestOdomPub; // Publishes nav_msgs::Odometry
	ros::Publisher _laggedOdomPub; // Publishes lagged odometry
	ros::Publisher _transCovPub;
	ros::Publisher _stepPub; // Publishes argus_msgs::FilterStepInfo
	ros::ServiceServer _resetHandler;

	MatrixType lastCov;

	void UpdateCallback( const argus_msgs::FilterUpdate::ConstPtr& msg );
	
	// Process all messages until the specified time
	void ProcessUpdateBuffer( const ros::Time& until );

	bool ResetFilterCallback( fieldtrack::ResetFilter::Request& req,
	                          fieldtrack::ResetFilter::Response& res );

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
