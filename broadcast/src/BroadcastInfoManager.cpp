#include "broadcast/BroadcastInfoManager.h"
#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

BroadcastInfoManager::BroadcastInfoManager( LookupInterface& interface )
: InfoManager( interface ) {}

bool BroadcastInfoManager::ParseMemberInfo( const std::string& memberNamespace,
                                            BroadcastInfo& info )
{
	std::string sizeKey = memberNamespace + "feature_size";
	std::string descKey = memberNamespace + "descriptions";
	std::string topicKey = memberNamespace + "topic";
	std::string modeKey = memberNamespace + "mode";

	std::string mode;
	if( !GetParam<unsigned int>( _nodeHandle, sizeKey, info.featureSize )
	 || !GetParam( _nodeHandle, descKey, info.featureDescriptions ) 
	 || !GetParam( _nodeHandle, topicKey, info.topic ) 
	 || !GetParam( _nodeHandle, modeKey, mode ) )
	{
		ROS_WARN_STREAM( "Could not parse broadcast info at: " << memberNamespace );
		return false;
	}
	info.mode = StringToBroadcastMode( mode );
	return true;
}

void BroadcastInfoManager::PopulateMemberInfo( const BroadcastInfo& info,
                                               const std::string& memberNamespace )
{
	std::string sizeKey = memberNamespace + "feature_size";
	std::string descKey = memberNamespace + "descriptions";
	std::string topicKey = memberNamespace + "topic";
	std::string modeKey = memberNamespace + "mode";
	_nodeHandle.setParam( sizeKey, (int)info.featureSize );
	_nodeHandle.setParam( descKey, info.featureDescriptions );
	_nodeHandle.setParam( topicKey, info.topic );
	_nodeHandle.setParam( modeKey, BroadcastModeToString( info.mode ) );
}

}