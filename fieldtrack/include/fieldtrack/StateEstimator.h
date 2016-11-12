#pragma once

#include <ros/ros.h>
#include "fieldtrack/FieldtrackCommon.h"
#include "argus_utils/filters/FilterTypes.h"
#include "covreg/AdaptiveCovarianceEstimator.h"
#include <unordered_map>

namespace argus
{

// Types of filter covariance models
enum CovarianceMode
{
	COV_PASS,     // Pass-through
	COV_FIXED,    // Fixed value
	COV_ADAPTIVE, // Window adaptive 
	//COV_PREDICTIVE // TODO
};
CovarianceMode StringToCovMode( const std::string& str );
std::string CovModeToString( CovarianceMode mode );

class StateEstimator
{
public:

	// TODO Make runtime settable?
	typedef ConstantAccelFilterSE3 FilterType;
	typedef FilterType::PoseType PoseType;

	StateEstimator();

	void Initialize( ros::NodeHandle& ph );
	void Reset( const ros::Time& time );

	// Adds a new update to the internal update buffer
	void BufferUpdate( FilterUpdate update );

	// TODO Return all Predict/Update info pairs
	// Perform predicts and updates to the specified time
	void Process( const ros::Time& until );

	// Get the current filter state
	TargetState GetState() const;

private:

	FilterType _filter;
	ros::Time _filterTime;

	bool _noPose;
	bool _twoDimensional;
	double _likelihoodThreshold; // Min likelihood before rejecting observations
	double _maxEntropyThreshold; // Maximum state entropy before reset
	MatrixType _initialCovariance;

	std::string _referenceFrame;
	std::string _bodyFrame;
	unsigned int _updateCounter;

	CovarianceMode _transitionMode;
	MatrixType _fixedTransCov;
	percepto::AdaptiveTransitionCovarianceEstimator _adaptiveTransCov;;

	struct SourceRegistration
	{
		CovarianceMode mode;
		unsigned int dim;
		MatrixType fixedCov;
		percepto::AdaptiveObservationCovarianceEstimator adaptiveCov;
	};
	typedef std::unordered_map<std::string, SourceRegistration> SourceRegistry;
	SourceRegistry _obsRegistry;

	typedef std::map<ros::Time, FilterUpdate> UpdateBuffer;
	UpdateBuffer _updateBuffer;

	// Performs a predict and update step for a given update
	void ProcessUpdate( FilterUpdate update );

	// Forward predicts the filter to the specified time
	PredictInfo PredictUntil( const ros::Time& until );

	// Performs a pose update using the update. Returns whether the update
	// was used or rejected
	bool PoseUpdate( const FilterUpdate& update, UpdateInfo& info );

	// Performs a derivative update using the update. Returns whether the update
	// was used or rejected
	bool DerivsUpdate( const FilterUpdate& update, UpdateInfo& info );

	// Checks the filter health
	void CheckFilter();
	void Enforce2D();
	void SquashPose();

	MatrixType GetTransitionCov( double dt ) const;
	MatrixType GetObservationCov( const FilterUpdate& update ) const;

};

}