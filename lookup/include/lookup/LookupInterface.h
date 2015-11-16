#pragma once

#include <ros/ros.h>

namespace lookup
{

/*! \brief Provides an interface to a lookup service that resolves unique object
 * names to fully-qualified namespaces. Uses the ROS parameter server. */
class LookupInterface
{
public:

	LookupInterface();
	
	/*! \brief Sets the lookup namespace. */
	void SetLookupNamespace( const std::string& ns );
	
	/*! \brief Fully resolves the namespace and registers it to the target name. */
	void WriteNamespace( const std::string& targetName, const std::string& ns );
	
	/*! \brief Tries to look up the corresponding namespace. Returns success. */
	bool ReadNamespace( const std::string& key, std::string& ns );
	
private:
	
	ros::NodeHandle nodeHandle;
	std::string lookupNamespace;
	
	static std::string CleanPath( const std::string& path );
	std::string FormLookupPath( const std::string& key ) const;
	
};
	
} // end namespace lookup 
