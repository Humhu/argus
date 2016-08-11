#include "broadcast/BroadcastMultiReceiver.h"
#include "argus_utils/utils/ParamUtils.h"
#include <boost/foreach.hpp>

namespace argus
{

BroadcastMultiReceiver::BroadcastMultiReceiver() 
: _dim( 0 ), _streamName( "" ) {}

void BroadcastMultiReceiver::Initialize( ros::NodeHandle& ph )
{
	YAML::Node topics;
	GetParam( ph, "", topics );
	YAML::const_iterator iter;
	for( iter = topics.begin(); iter != topics.end(); ++iter )
	{
		const std::string& name = iter->first.as<std::string>();
		AddStream( name, iter->second );
	}
}

const std::string& BroadcastMultiReceiver::GetStreamName() const
{
	return _streamName;
}

unsigned int BroadcastMultiReceiver::GetDim() const
{
	return _dim;
}

bool BroadcastMultiReceiver::IsReady() const
{
	BOOST_FOREACH( const BroadcastReceiver& rx, _receivers )
	{
		if( !rx.IsReady() ) { return false; }
	}
	return true;
}

bool BroadcastMultiReceiver::ReadStream( const ros::Time& time,
                                         StampedFeatures& features )
{
	if( !IsReady() ) { return false; }
	features.name = GetStreamName();;
	features.time = time;
	features.features = VectorType( GetDim() );
	StampedFeatures subF;
	unsigned int ind = 0;
	BOOST_FOREACH( BroadcastReceiver& rx, _receivers )
	{
		if( !rx.ReadStream( time, subF ) ) { return false; }
		features.features.segment( ind, rx.GetDim() ) = subF.features;
		ind += rx.GetDim();
	}
	return true;
}

}