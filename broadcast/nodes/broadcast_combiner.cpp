#include <ros/ros.h>

#include "broadcast/BroadcastTransmitter.h"
#include "broadcast/BroadcastReceiver.h"
#include "broadcast/BroadcastInfoManager.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

class BroadcastCombiner
{
public:

	// TODO Get more parameters in through streams_to_combine
	BroadcastCombiner( ros::NodeHandle& nodeHandle, ros::NodeHandle& privHandle )
	: _lookup(), 
	_broadcastManager( _lookup ), _outputDim( 0 )
	{
		std::string outputName;
		GetParamRequired( privHandle, "output_stream", outputName );
		double outputRate;
		GetParam( privHandle, "output_rate", outputRate, 10.0 );

		std::vector<std::string> descriptions;
		std::vector<std::string> streams;
		GetParamRequired( privHandle, "streams_to_combine", streams );
		for( unsigned int i = 0; i < streams.size(); i++ )
		{
			if( !_broadcastManager.CheckMemberInfo( streams[i] ) )
			{
				ROS_ERROR_STREAM( "Could not retrieve broadcast info for: " << streams[i] );
				exit( -1 );
			}
			_receivers.emplace_back( std::make_shared<BroadcastReceiver>( streams[i], 10 ) );
			const BroadcastInfo& info = _broadcastManager.GetInfo( streams[i] );
			_outputDim += info.featureSize;
			streams.insert( streams.end(), info.featureDescriptions.begin(), 
			                info.featureDescriptions.end() );
		}

		_transmitter = std::make_shared<BroadcastTransmitter>( outputName,
		                                                       _outputDim,
		                                                       descriptions );
		_timer = std::make_shared<ros::Timer>( 
		     nodeHandle.createTimer( ros::Duration( 1.0/outputRate ),
		                              &BroadcastCombiner::TimerCallback,
		                              this ) );
	}

private:

	LookupInterface _lookup;
	BroadcastInfoManager _broadcastManager;
	std::vector<std::shared_ptr<BroadcastReceiver>> _receivers;
	std::shared_ptr<BroadcastTransmitter> _transmitter;
	std::shared_ptr<ros::Timer> _timer;
	unsigned int _outputDim;

	void TimerCallback( const ros::TimerEvent& event )
	{
		VectorType feats( _outputDim );
		unsigned int ind = 0;
		for( unsigned int i = 0; i < _receivers.size(); i++ )
		{
			VectorType rx = _receivers[i]->GetClosestPrevious( event.current_real );
			feats.segment( ind, ind + rx.size() ) = rx;
			ind += rx.size();
		}
		_transmitter->Publish( event.current_real, feats );
	}
};


int main( int argc, char** argv )
{

}
