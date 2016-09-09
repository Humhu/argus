#pragma once

#include "broadcast/BroadcastReceiver.h"

#include <memory>
#include <deque>

namespace argus
{

class BroadcastMultiReceiver
{
public:

	typedef std::shared_ptr<BroadcastMultiReceiver> Ptr;

	BroadcastMultiReceiver();

	void Initialize( ros::NodeHandle& ph );

	template <class ... Args>
	void AddStream( const std::string& streamName,
	                Args ... args )
	{
		_receivers.emplace_back();
		_receivers.back().Initialize( streamName, args... );
		_dim += _receivers.back().GetDim();
		if( _streamName.empty() )
		{
			_streamName = _receivers.back().GetStreamName();
		}
		else
		{
			_streamName += "_" + _receivers.back().GetStreamName();
		}
	}

	const std::string& GetStreamName() const;
	unsigned int GetDim() const;

	bool IsReady() const;

	bool ReadStream( const ros::Time& time, StampedFeatures& features ) const;

	ros::Time EarliestTime() const;
	ros::Time LatestTime() const;

private:

	// NOTE Need a container that doesn't invalidate pointers as it grows
	typedef std::deque<BroadcastReceiver> Receivers;

	unsigned int _dim;
	std::string _streamName;
	Receivers _receivers;
};

}