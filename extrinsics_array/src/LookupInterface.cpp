#include "extrinsics_array/LookupInterface.h"

namespace extrinsics_array
{

LookupInterface::LookupInterface( ros::NodeHandle& nh )
: nodeHandle( nh ), lookupNamespace( "/lookup/" ) {}

void LookupInterface::SetLookupNamespace( const std::string& ns )
{
	lookupNamespace = ns;
	if( lookupNamespace.back() != '/' ) { lookupNamespace += "/"; }
}

void LookupInterface::WriteLookup( const std::string& child, const std::string& parent )
{
	std::string lookupPath = FormLookupPath( child );
	
	// Parent paths should always end in a slash /
	std::string p = parent;
	if( p.back() != '/' ) { p += "/"; }
	nodeHandle.setParam( lookupPath, parent );
}

bool LookupInterface::ReadLookup( const std::string& child, std::string& parent )
{
	std::string lookupPath = FormLookupPath( child );
	return nodeHandle.getParam( lookupPath, parent );
}

std::string LookupInterface::FormLookupPath( const std::string& child ) const
{
	std::string lookupPath = lookupNamespace + child;
	if( lookupPath.back() != '/' ) { lookupPath += "/"; }
	lookupPath += "parent_array";
	return lookupPath;
}


	
} // end namespace extrinsics_array 
