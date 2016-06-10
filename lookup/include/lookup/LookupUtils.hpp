#pragma once

#include "lookup/LookupInterface.h"

namespace argus
{

void register_lookup_target( ros::NodeHandle& nodeHandle,
                             const std::string& targetName,
                             const std::string& targetNamespace = "",
                             const std::string& lookupNamespace = "/lookup" )
{
	LookupInterface lookupInterface( nodeHandle );
	lookupInterface.SetLookupNamespace( lookupNamespace );
	lookupInterface.WriteNamespace( targetName, targetNamespace );
}

}