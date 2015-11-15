#include "lookup/LookupInterface.h"

namespace lookup
{

LookupInterface::LookupInterface( ros::NodeHandle& nh )
: nodeHandle( nh ), lookupNamespace( "/lookup/" ) {}

void LookupInterface::SetLookupNamespace( const std::string& ns )
{
	lookupNamespace = ns;
	if( lookupNamespace.back() != '/' ) { lookupNamespace += "/"; }
}

void LookupInterface::WriteParent( const std::string& child, const std::string& parent )
{
	std::string lookupPath = FormLookupPath( child ) + "parent_array";
	
	// Parent paths should always end in a slash /
	std::string p = parent;
	if( p.back() != '/' ) { p += "/"; }
	nodeHandle.setParam( lookupPath, p );
}

bool LookupInterface::ReadParent( const std::string& child, std::string& parent )
{
	std::string lookupPath = FormLookupPath( child ) + "parent_array";
	return nodeHandle.getParam( lookupPath, parent );
}

std::string LookupInterface::FormLookupPath( const std::string& child ) const
{
	std::string lookupPath = lookupNamespace + child;
	if( lookupPath.back() != '/' ) { lookupPath += "/"; }
	return lookupPath;
}
	
} // end namespace lookup 
