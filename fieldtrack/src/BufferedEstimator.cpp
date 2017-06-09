#include "fieldtrack/BufferedEstimator.h"

namespace argus
{
BufferedEstimator::BufferedEstimator()
	: _stepCounter( 0 ) {}

BufferedEstimator::~BufferedEstimator() {}

void BufferedEstimator::Reset( const ros::Time& time,
                               const VectorType& state,
                               const MatrixType& cov )
{
	_filterTime = time;
	_stepCounter = 0;
	_updateBuffer.clear();
	ResetDerived( time, state, cov );
}

const ros::Time& BufferedEstimator::GetFilterTime() const { return _filterTime; }

std::vector<FilterInfo> BufferedEstimator::Process( const ros::Time& until )
{
	std::vector<FilterInfo> infos;
	if( until < _filterTime )
	{
		ROS_WARN_STREAM( "Cannot process to time " << until <<
		                 " as it precedes filter time " << _filterTime );
		return infos;
	}

	while( !_updateBuffer.empty() )
	{
		UpdateBuffer::const_iterator oldest = _updateBuffer.begin();
		const ros::Time& obsTime = oldest->first;
		const SourceMsg& sourceMsg = oldest->second;
		const std::string& sourceName = sourceMsg.first;
		const ObservationMessage& msg = sourceMsg.second;

		if( obsTime > until ) { break; }
		if( obsTime < _filterTime )
		{
			ROS_WARN_STREAM( "Observation from " << obsTime << " before filter time " <<
			                 _filterTime << " popped from buffer!" );
			_updateBuffer.erase( oldest );
			continue;
		}
		// Perform predict
		PredictInfo predInfo = PredictUntil( obsTime );
		predInfo.time = _filterTime;
		predInfo.frameId = "predict";
		_filterTime = obsTime;
		predInfo.stepNum = _stepCounter++;
		infos.emplace_back( predInfo );

		UpdateInfo upInfo;
		if( ProcessMessage( sourceName, msg, upInfo ) )
		{
			upInfo.time = _filterTime;
			upInfo.frameId = sourceName;
			upInfo.stepNum = _stepCounter++;
			infos.emplace_back( upInfo );
		}
		else
		{
			ROS_WARN_STREAM( "Could not apply update from " << sourceName );
		}

		_updateBuffer.erase( oldest );
		CheckFilter();
	}

	// Predict the remainder of requested time
	PredictInfo predInfo = PredictUntil( until );
	_filterTime = until;
	infos.emplace_back( predInfo );

	// Have to check after final predict
	CheckFilter();

	return infos;
}
}
