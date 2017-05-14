#pragma once

#include <ros/ros.h>
#include "fieldtrack/FieldtrackCommon.h"
#include "fieldtrack/AdaptiveCovarianceEstimator.h"
#include "fieldtrack/CovarianceModels.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

namespace argus
{
class PoseSourceManager
	: public boost::static_visitor<PoseObservation>
{
public:

	PoseSourceManager();

	void Initialize( ros::NodeHandle& ph,
	                 bool twoDimensional,
	                 const std::string& refFrame,
	                 const std::string& targetFrame,
	                 ExtrinsicsInterface::Ptr extrinsics );

	void Update( const UpdateInfo& info );
	void Reset();

	CovarianceModel::Ptr InitializeModel() const;
	void SetModel( const CovarianceModel& model );

	PoseObservation operator()( const geometry_msgs::PoseStamped& msg );
	PoseObservation operator()( const geometry_msgs::PoseWithCovarianceStamped& msg );
	PoseObservation operator()( const geometry_msgs::TwistStamped& msg );
	PoseObservation operator()( const geometry_msgs::TwistWithCovarianceStamped& msg );
	PoseObservation operator()( const geometry_msgs::TransformStamped& msg );
	PoseObservation operator()( const sensor_msgs::Imu& msg );

private:

	std::string _referenceFrame;
	std::string _targetFrame;
	ExtrinsicsInterface::Ptr _extrinsics;

	std::string _obsRefFrame;
	bool _twoDimensional;

	CovarianceMode _mode;
	MatrixType _fixedCov;
	AdaptiveObsCovEstimator _adaptiveCov;

	unsigned int GetDim() const;
	PoseObservation ProcessPose( const PoseSE3& pose,
	                             const MatrixType& cov,
	                             const ros::Time& time,
	                             const std::string& refFrame,
	                             const std::string& tarFrame );
	MatrixType GetCovariance( const ros::Time& stamp,
	                          const MatrixType& recv );
};
}