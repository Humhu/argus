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
	std::string descKey = memberNamespace + "feature_descriptions";
	if( !GetParam<unsigned int>( _nodeHandle, sizeKey, info.featureSize )
	 || !GetParam( _nodeHandle, descKey, info.featureDescriptions ) )
	{
		ROS_WARN_STREAM( "Could not parse broadcast info at: " << memberNamespace );
		return false;
	}
	return true;
}

void BroadcastInfoManager::PopulateMemberInfo( const BroadcastInfo& info,
                                               const std::string& memberNamespace )
{
	std::string sizeKey = memberNamespace + "feature_size";
	std::string descKey = memberNamespace + "feature_descriptions";
	_nodeHandle.setParam( sizeKey, (int)info.featureSize );
	_nodeHandle.setParam( descKey, info.featureDescriptions );
}

}