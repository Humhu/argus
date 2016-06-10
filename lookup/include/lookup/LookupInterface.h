#pragma once

#include <ros/ros.h>

namespace argus
{

/*! \brief Provides an interface to a lookup service that resolves unique object
 * names to fully-qualified namespaces. Uses the ROS parameter server. */
class LookupInterface
{
public:

	// TODO Deprecate
	LookupInterface();

	LookupInterface( const ros::NodeHandle& nh );
	
	/*! \brief Sets the lookup namespace. */
	void SetLookupNamespace( const std::string& ns );
	
	/*! \brief Fully resolves the namespace and registers it to the target name. */
	void WriteNamespace( const std::string& targetName, const std::string& ns );
	
	/*! \brief Tries to look up the corresponding namespace. Namespaces end in /. 
	 * Returns success. */
	bool ReadNamespace( const std::string& key, std::string& ns,
	                    const ros::Duration& timeout = ros::Duration( 0.0 ) );
	
private:
	
	ros::NodeHandle nodeHandle;
	std::string lookupNamespace;
	
	static std::string CleanPath( const std::string& path );
	std::string FormLookupPath( const std::string& key ) const;
	
};
	
} // end namespace lookup 
