#pragma once

#include <ros/ros.h>
#include "argus_utils/filters/FilterInfo.h"
#include "argus_utils/filters/FilterUtils.h"

namespace argus
{

// Based on Mohamed and Schwarz 1999
class AdaptiveTransitionCovarianceEstimator
{
public:

	AdaptiveTransitionCovarianceEstimator();

	void Initialize( ros::NodeHandle& ph, const std::string& field );

	MatrixType GetQ() const;

	void ProcessInfo( const argus_msgs::FilterStepInfo& msg );

	bool IsReady() const;

	void Reset();

private:

	unsigned int _windowLength;

	std::deque<MatrixType> _delXOuterProds;
	MatrixType _lastFSpostFT;
	MatrixType _currSpost;
	MatrixType _lastF;
	MatrixType _offset;
	double _lastDt;

	void InfoCallback( const argus_msgs::FilterStepInfo::ConstPtr& msg );

};

class AdaptiveObservationCovarianceEstimator
{
public:

	AdaptiveObservationCovarianceEstimator();

	void Initialize( ros::NodeHandle& ph, const std::string& field );

	MatrixType GetR() const;

	bool IsReady() const;

	void ProcessInfo( const argus_msgs::FilterStepInfo& msg );

	void Reset();

private:
	std::string _sourceName;
	unsigned int _windowLength;
	std::deque<MatrixType> _innoOuterProds;
	MatrixType _lastHPHT;
	MatrixType _initCov;

};

}