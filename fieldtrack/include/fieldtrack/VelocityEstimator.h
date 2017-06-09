#pragma once

#include "argus_utils/filter/KalmanFilter.h"

#include "fieldtrack/BufferedEstimator.h"
#include "fieldtrack/VelocitySourceManager.h"
#include "fieldtrack/CovarianceModels.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <unordered_map>

namespace argus
{
class VelocityEstimator
	: public BufferedEstimator
{
public:

	VelocityEstimator();

	void Initialize( ros::NodeHandle& ph, ExtrinsicsInterface::Ptr extrinsics );

	nav_msgs::Odometry GetOdom() const;
	geometry_msgs::TwistStamped GetTwist() const;
	geometry_msgs::TwistWithCovarianceStamped GetTwistWithCovariance() const;

	CovarianceModel::Ptr InitTransCovModel() const;
	std::unordered_map<std::string, CovarianceModel::Ptr> InitObsCovModels() const;
	void SetTransCovModel( const CovarianceModel& model );
	void SetObsCovModel( const std::string& name, const CovarianceModel& model );

	unsigned int StateDim() const;
	unsigned int FullDim() const;

private:

	KalmanFilter _filter;

	unsigned int _filterOrder;
	bool _twoDimensional;

	double _logLikelihoodThreshold; // Min likelihood before rejecting observations
	double _maxEntropyThreshold; // Maximum state entropy before reset

	VectorType _initialState;
	MatrixType _initialCovariance; // Cov to reset to

	std::string _bodyFrame;

	MatrixType _transCovRate;

	typedef std::unordered_map<std::string, VelocitySourceManager> SourceRegistry;
	SourceRegistry _sourceRegistry;

	void GetFullVels( PoseSE3::TangentVector& vel,
	                  PoseSE3::CovarianceMatrix& cov ) const;


	virtual void ResetDerived( const ros::Time& time,
	                           const VectorType& state = VectorType(),
	                           const MatrixType& cov = MatrixType() );
	virtual PredictInfo PredictUntil( const ros::Time& until );
	virtual bool ProcessMessage( const std::string& source,
	                             const ObservationMessage& msg,
	                             UpdateInfo& info );
	virtual void CheckFilter();

	MatrixType GetTransitionCov( double dt );
};
}