#include "broadcast/BroadcastTransmitter.h"
#include "broadcast/StampedFeatures.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "lookup/LookupUtils.hpp"

using namespace broadcast;

namespace argus
{

BroadcastTransmitter::BroadcastTransmitter( const std::string& streamName, 
                                            unsigned int featureSize,
                                            const std::vector<std::string> featureDescriptions,
                                            const std::string& streamNs,
                                            unsigned int outgoingQueueSize,
                                            const std::string& topic )
: _nodeHandle( streamNs ), _streamName( streamName ), _streamSize( featureSize )
{
	std::string lookupNamespace;
	GetParam<std::string>( _nodeHandle, "lookup_namespace", 
	                       lookupNamespace, "lookup" );
	register_lookup_target( _nodeHandle, streamName, streamNs, lookupNamespace );

	_nodeHandle.setParam( "feature_size", (int) _streamSize );
	_nodeHandle.setParam( "feature_descriptions", featureDescriptions );
	_streamPub = _nodeHandle.advertise<StampedFeatures>( topic, outgoingQueueSize );
}

void BroadcastTransmitter::Publish( const ros::Time& stamp, const VectorType& features )
{
	if( features.size() != _streamSize )
	{
		throw std::runtime_error( "Received features of size " + std::to_string( features.size() )
		                          + " for stream " + _streamName + " but expected " 
		                          + std::to_string( _streamSize ) );
	}
	StampedFeatures msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = _streamName;
	GetVectorView( msg.features ) = features;
	_streamPub.publish( msg );
}

}