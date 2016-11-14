#pragma once

#include "fieldtrack/FieldtrackCommon.h"
#include "fieldtrack/AdaptiveCovarianceEstimator.h"
#include <boost/variant.hpp>

namespace argus
{

class ObservationSourceManager
: public boost::static_visitor<Observation>
{
public:

	ObservationSourceManager( ros::NodeHandle& ph );

	void Update( const UpdateInfo& info );
	void Reset();

	Observation operator()( const geometry_msgs::PoseStamped& msg );
	Observation operator()( const geometry_msgs::PoseWithCovarianceStamped& msg );
	Observation operator()( const geometry_msgs::TwistStamped& msg );
	Observation operator()( const geometry_msgs::TwistWithCovarianceStamped& msg );
	Observation operator()( const sensor_msgs::Imu& msg );

private:

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