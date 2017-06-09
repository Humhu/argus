#pragma once

#include <ros/ros.h>
#include "argus_utils/filter/FilterInfo.h"
#include "fieldtrack/FieldtrackCommon.h"

namespace argus
{
/*! \brief Base class for all time-lag buffered estimators. */
class BufferedEstimator
{
public:

	BufferedEstimator();
	virtual ~BufferedEstimator();

	// NOTE Messages are buffered as variants, but we need to process their timestamps
	// to get super-resolution since ros::Time has some weird min comparison resolution
	template <typename M>
	void BufferObservation( const std::string& sourceName, M msg )
	{
		// Make sure message does not precede filter
		if( msg.header.stamp < _filterTime )
		{
			ROS_WARN_STREAM( "Dropping measurement from " << sourceName <<
			                 " since timestamp " << msg.header.stamp <<
			                 " precedes filter time " << _filterTime );
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

	// Process all messages in the buffer until the specified time
	std::vector<FilterInfo> Process( const ros::Time& until );

	// Clear the buffer and reset the filter state and time
	void Reset( const ros::Time& time,
	            const VectorType& state = VectorType(),
	            const MatrixType& cov = MatrixType() );

	// Retrieve the current filter time
	const ros::Time& GetFilterTime() const;

private:

	ros::Time _filterTime;
	unsigned int _stepCounter;

	typedef std::pair<std::string, ObservationMessage> SourceMsg;
	typedef std::map<ros::Time, SourceMsg> UpdateBuffer;
	UpdateBuffer _updateBuffer;

	// Reset method called by the derived filter class
	virtual void ResetDerived( const ros::Time& time, const VectorType& state,
	                           const MatrixType& cov ) = 0;

	// Forward predicts the filter to the specified time
	virtual PredictInfo PredictUntil( const ros::Time& until ) = 0;

	// Processes the message and applies an update
	virtual bool ProcessMessage( const std::string& source,
	                             const ObservationMessage& msg,
	                             UpdateInfo& info ) = 0;

	// Checks the filter health
	virtual void CheckFilter() = 0;
};
}