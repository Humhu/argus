#pragma once

#include <ros/ros.h>
#include "fieldtrack/FieldtrackCommon.h"
#include "fieldtrack/AdaptiveCovarianceEstimator.h"
#include "fieldtrack/CovarianceModels.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

namespace argus
{
/*! \brief Manages sources of velocity/derivative information.
 */
class VelocitySourceManager
	: public boost::static_visitor<DerivObservation>
{
public:

	VelocitySourceManager();

	/*! \brief Initialize the manager */
	void Initialize( ros::NodeHandle& ph,
	                 bool twoDimensional,
	                 unsigned int filterOrder,
	                 const std::string& targetFrame,
	                 ExtrinsicsInterface::Ptr extrinsics );

	void Update( const UpdateInfo& info );
	void Reset();

	bool CheckLogLikelihood( double ll ) const;
	const MatrixType& GetIndexMatrix() const;

	CovarianceModel::Ptr InitializeModel() const;
	void SetModel( const CovarianceModel& model );

	DerivObservation operator()( const geometry_msgs::PoseStamped& msg );
	DerivObservation operator()( const geometry_msgs::PoseWithCovarianceStamped& msg );
	DerivObservation operator()( const geometry_msgs::TwistStamped& msg );
	DerivObservation operator()( const geometry_msgs::TwistWithCovarianceStamped& msg );
	DerivObservation operator()( const geometry_msgs::TransformStamped& msg );
	DerivObservation operator()( const sensor_msgs::Imu& msg );

private:

	std::string _targetFrame;
	ExtrinsicsInterface::Ptr _extrinsicsManager;
	
	CovarianceMode _mode;
	MatrixType _fixedCov;
	AdaptiveObsCovEstimator _adaptiveCov;

	unsigned int _filterOrder;
	MatrixType _filterToThreeD; // Maps filter state to 3d derivs
	MatrixType _obsMatrix;
	double _minLogLikelihood;

	bool _twoDimensional;
	std::vector<unsigned int> _obsInds; // Dimensions of state that are observed
	std::vector<unsigned int> _sensorInds; // Dimensions of sensor that are active

	DerivObservation ProcessDerivatives( const VectorType& derivs,
	                                     const MatrixType& cov,
	                                     const ros::Time& stamp,
	                                     const std::string& frame );
	MatrixType GetCovariance( const ros::Time& stamp, const MatrixType& recv );
};
}
