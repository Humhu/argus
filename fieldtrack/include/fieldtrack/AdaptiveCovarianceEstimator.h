#pragma once

#include <ros/ros.h>
#include "argus_utils/filters/FilterInfo.h"
#include "argus_utils/filters/FilterUtils.h"

namespace argus
{

/*! \brief Estimates the transition covariance of a Kalman filter.
 *
 * Implementation of the AKF approach detailed in Mohamed and 
 * Schwarz 1999. Modified to use decaying weights on the correction
 * outer products to more smoothly handle outliers.
 *
 */
class AdaptiveTransitionCovarianceEstimator
{
public:

	AdaptiveTransitionCovarianceEstimator();

	// Read parameters for the estimator
	void Initialize( ros::NodeHandle& ph );

	bool IsReady() const;
	MatrixType GetQ() const;

	void Update( const PredictInfo& predict, 
	             const UpdateInfo& update );

	void Reset();

private:

	unsigned int _windowLength;
	unsigned int _minLength;
	bool _useDiag;

	VectorType _prodWeights;

	std::deque<MatrixType> _delXOuterProds;
	MatrixType _lastFSpostFT;
	MatrixType _currSpost;
};

class AdaptiveObservationCovarianceEstimator
{
public:

	AdaptiveObservationCovarianceEstimator();

	void Initialize( ros::NodeHandle& ph );

	bool IsReady() const;
	MatrixType GetR() const;

	void Update( const UpdateInfo& update );

	void Reset();

private:

	unsigned int _windowLength;
	unsigned int _minLength;
	bool _useDiag;

	VectorType _prodWeights;

	std::deque<MatrixType> _innoOuterProds;
	MatrixType _lastHPHT;
};

}