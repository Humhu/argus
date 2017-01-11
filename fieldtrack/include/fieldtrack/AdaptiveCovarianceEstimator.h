#pragma once

#include <ros/ros.h>
#include "fieldtrack/utils/FilterInfo.h"
#include <deque>

#include "broadcast/BroadcastTransmitter.h"

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

	unsigned int NumSamples() const;
	MatrixType GetQ( const ros::Time& time );

	void Update( const ros::Time& time,
	             const PredictInfo& predict, 
	             const UpdateInfo& update );

	void Reset();

private:

	typedef std::pair<ros::Time,MatrixType> InnoStamped;

	unsigned int _maxSamples;
	unsigned int _minSamples;
	ros::Duration _maxAge;

	MatrixType _offset;
	MatrixType _initialCov;
	bool _useDiag;
	double _decayRate;

	std::deque<InnoStamped> _innoProds; // Ordered so head is newest
	MatrixType _lastFSpostFT;
	MatrixType _currSpost;
	MatrixType _lastK;

	BroadcastTransmitter _tx;

	void CheckBuffer( const ros::Time& now );
};

class AdaptiveObservationCovarianceEstimator
{
public:

	AdaptiveObservationCovarianceEstimator();

	void Initialize( ros::NodeHandle& ph );

	unsigned int NumSamples() const;
	MatrixType GetR( const ros::Time& time );

	void Update( const ros::Time& time, const UpdateInfo& update );

	void Reset();

private:

	typedef std::pair<ros::Time,MatrixType> InnoStamped;

	unsigned int _maxSamples;
	unsigned int _minSamples;
	ros::Duration _maxAge;

	MatrixType _offset;
	MatrixType _initialCov;
	bool _useDiag;
	double _decayRate;

	std::deque<InnoStamped> _innoProds; // Ordered so head is newest
	MatrixType _lastHPHT;

	void CheckBuffer( const ros::Time& now );
};

}