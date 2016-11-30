#pragma once

#include "fieldtrack/FieldtrackCommon.h"
#include "fieldtrack/AdaptiveCovarianceEstimator.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

#include <boost/variant.hpp>

namespace argus
{

class ObservationSourceManager
: public boost::static_visitor<Observation>
{
public:

	// Initialize a manager, specifying the frame all observations should be
	// transformed into and giving the interface to do it with
	ObservationSourceManager( ros::NodeHandle& ph, 
	                          const std::string& targetFrame,
	                          const std::string& refFrame,
	                          ExtrinsicsInterface::Ptr extrinsics );

	void Update( const ros::Time& time, const UpdateInfo& info );
	void Reset();

	// NOTE We assume for Pose messages that their effective child_frame_id is _refFrame
	Observation operator()( const geometry_msgs::PoseStamped& msg );
	Observation operator()( const geometry_msgs::PoseWithCovarianceStamped& msg );
	Observation operator()( const geometry_msgs::TwistStamped& msg );
	Observation operator()( const geometry_msgs::TwistWithCovarianceStamped& msg );
	Observation operator()( const geometry_msgs::TransformStamped& msg );
	Observation operator()( const sensor_msgs::Imu& msg );

private:

	std::string _targetFrame;
	std::string _refFrame;
	ExtrinsicsInterface::Ptr _extrinsicsManager;
	CovarianceMode _mode;
	MatrixType _fixedCov;
	AdaptiveObservationCovarianceEstimator _adaptiveCov;

	// TODO Set these functions free!
	static std::vector<unsigned int> ParseCovarianceMask( const MatrixType& cov );

	static VectorType MaskVector( const VectorType& vec, 
	                              const std::vector<unsigned int>& inds );
	static MatrixType MaskMatrix( const MatrixType& mat,
	                              const std::vector<unsigned int>& inds );
	static void CheckMatrixSize( const MatrixType& mat, unsigned int s );

	static Observation ParsePoseMessage( const std_msgs::Header& header,
	                                     const PoseSE3& pose,
	                                     const MatrixType& cov,
	                                     const std::vector<unsigned int>& valid );
	static Observation ParseDerivatives( const std_msgs::Header& header,
	                                     const VectorType& derivs,
	                                     const MatrixType& cov,
	                                     const std::vector<unsigned int>& valid );
	static Observation ParseImu( const std_msgs::Header& header,
	                             const VectorType& derivs,
	                             const MatrixType& cov,
	                             const std::vector<unsigned int>& valid );
};

}