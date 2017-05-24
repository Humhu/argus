#pragma once

#include "fieldtrack/PoseDerivativeFilter.h"
#include "fieldtrack/FieldtrackCommon.h"
#include "fieldtrack/ObservationSourceManager.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

#include <ros/ros.h>
#include <unordered_map>

namespace argus
{

class StateEstimator
{
public:

	StateEstimator();

	void Initialize( ros::NodeHandle& ph, ExtrinsicsInterface::Ptr extrinsics );
	void Reset( const ros::Time& time );

	template <typename M>
	void BufferObservation( const std::string& sourceName, M msg )
	{
		// Make sure message is from a registered source
		if( _sourceRegistry.count( sourceName ) == 0 )
		{
			throw std::invalid_argument( "Unknown source: " + sourceName );
		}

		// Make sure message does not precede filter
		if( msg.header.stamp < _filterTime )
		{
			ROS_WARN_STREAM( "Dropping measurement from " << sourceName << " since timestamp "
			                 << msg.header.stamp << " precedes filter time " << _filterTime );
			return;
		}

		// For some reason the resolution on the ROS timestamp compare is limited
		// We avoid overwriting observations by bumping up their timestamps
		while( _updateBuffer.count( msg.header.stamp ) > 0 )
		{
			msg.header.stamp.nsec += 2; 
		}
		_updateBuffer[ msg.header.stamp ] = SourceMsg( sourceName, msg );
	}

	// TODO Return all Predict/Update info pairs
	// Perform predicts and updates to the specified time
	std::vector<FilterInfo> Process( const ros::Time& until );

	// Get the current filter state
	TargetState GetState() const;

	const MatrixType& GetTransitionCovRate() const;
	void SetTransitionCovRate( const MatrixType& Q );

private:

	ExtrinsicsInterface::Ptr _extrinsicsManager;
	PoseDerivativeFilter _filter;
	ros::Time _filterTime;
	unsigned int _stepCounter;

	bool _noPose;
	bool _noDerivs;
	bool _twoDimensional;
	double _likelihoodThreshold; // Min likelihood before rejecting observations
	double _maxEntropyThreshold; // Maximum state entropy before reset
	MatrixType _initialCovariance;

	std::string _referenceFrame;
	std::string _bodyFrame;

	MatrixType _transCovRate;

	typedef std::unordered_map<std::string, ObservationSourceManager> SourceRegistry;
	SourceRegistry _sourceRegistry;

	typedef std::pair<std::string, ObservationMessage> SourceMsg;
	typedef std::map<ros::Time, SourceMsg> UpdateBuffer;
	UpdateBuffer _updateBuffer;

	// Forward predicts the filter to the specified time
	PredictInfo PredictUntil( const ros::Time& until );

	// Checks the filter health
	void CheckFilter();
	void Enforce2D();
	void SquashPose();
	void SquashDerivs();

	MatrixType GetTransitionCov( const ros::Time& time, double dt );
};

}
