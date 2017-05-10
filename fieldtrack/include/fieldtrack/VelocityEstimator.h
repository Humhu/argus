#pragma once

#include "argus_utils/filter/KalmanFilter.h"
#include "fieldtrack/FieldtrackCommon.h"
#include "fieldtrack/VelocitySourceManager.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

#include <unordered_map>

namespace argus
{
class VelocityEstimator
{
public:

	VelocityEstimator();

	void Initialize( ros::NodeHandle& ph, ExtrinsicsInterface::Ptr extrinsics );

	// NOTE This is copied from StateEstimator - we should be able to generalize somehow
	template<typename M>
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
		_updateBuffer[msg.header.stamp] = SourceMsg( sourceName, msg );
	}

	std::vector<FilterInfo> Process( const ros::Time& until );

	void Reset( const ros::Time& time );
	nav_msgs::Odometry GetOdom() const;

private:

	ExtrinsicsInterface::Ptr _extrinsicsManager;
	KalmanFilter _filter;
	ros::Time _filterTime;
	unsigned int _stepCounter;

	unsigned int _filterOrder;
	bool _twoDimensional;

	double _logLikelihoodThreshold; // Min likelihood before rejecting observations
	double _maxEntropyThreshold; // Maximum state entropy before reset

	MatrixType _initialCovariance; // Cov to reset to

	std::string _bodyFrame;

	MatrixType _transCovRate;

	typedef std::unordered_map<std::string, VelocitySourceManager> SourceRegistry;
	SourceRegistry _sourceRegistry;

	typedef std::pair<std::string, ObservationMessage> SourceMsg;
	typedef std::map<ros::Time, SourceMsg> UpdateBuffer;
	UpdateBuffer _updateBuffer;

	unsigned int StateDim() const;
	unsigned int FullDim() const;

	// Forward predicts the filter to the specified time
	PredictInfo PredictUntil( const ros::Time& until );

	// Checks the filter health
	void CheckFilter();
	void Enforce2D();

	MatrixType GetTransitionCov( double dt );
};
}