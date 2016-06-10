#include "lookup/LookupInterface.h"

namespace argus
{

LookupInterface::LookupInterface()
: nodeHandle(), lookupNamespace( "/lookup/" ) {}

LookupInterface::LookupInterface( const ros::NodeHandle& nh )
: nodeHandle( nh ), lookupNamespace( "/lookup/" ) {}

void LookupInterface::SetLookupNamespace( const std::string& ns )
{
	lookupNamespace = ns;
	if( lookupNamespace.back() != '/' ) { lookupNamespace += "/"; }
}

void LookupInterface::WriteNamespace( const std::string& targetName, const std::string& ns )
{
	std::string resolvedNamespace = CleanPath( nodeHandle.resolveName( ns ) );
	std::string lookupPath = FormLookupPath( targetName );
	
	ROS_INFO_STREAM( "Registering target (" << targetName 
	    << ") with fully resolved namespace (" << resolvedNamespace 
	    << ") at lookup path (" << lookupPath << ")" );
	
	nodeHandle.setParam( lookupPath, resolvedNamespace );
}

bool LookupInterface::ReadNamespace( const std::string& targetName, 
                                     std::string& resolvedNamespace,
                                     const ros::Duration& timeout )
{
	std::string path = FormLookupPath( targetName );
	if( nodeHandle.getParam( path, resolvedNamespace ) ) { return true; }
	else if( timeout.toSec() == 0.0 ) { return false; }

	ros::Time start = ros::Time::now();
	do
	{
		ros::Duration( 0.5 ).sleep();
		if( nodeHandle.getParam( path, resolvedNamespace ) ) { return true; }
	} while( ros::Time::now() < start + timeout );
	
	return false;
}

std::string LookupInterface::CleanPath( const std::string& path )
{
	std::string p = path;
	if( p.back() != '/' ) { p += "/"; }
	return p;
}

std::string LookupInterface::FormLookupPath( const std::string& key ) const
{
	std::string lookupPath = CleanPath( lookupNamespace + key );
	//lookupPath += "namespace";
	return lookupPath;
}
	
} // end namespace lookup 
