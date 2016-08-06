#pragma once

#include "broadcast/BroadcastCommon.h"
#include "lookup/InfoManager.h"

namespace argus
{

struct BroadcastInfo
{
	unsigned int featureSize;
	std::vector<std::string> featureDescriptions;
	std::string topic;
	BroadcastMode mode;
};

class BroadcastInfoManager
: public InfoManager<BroadcastInfo>
{
public:

	BroadcastInfoManager( LookupInterface& interface );

protected:

	virtual bool ParseMemberInfo( const std::string& memberNamespace,
	                              BroadcastInfo& info );
	virtual void PopulateMemberInfo( const BroadcastInfo& info,
	                                 const std::string& memberNamespace );

private:

	ros::NodeHandle _nodeHandle;
};

}