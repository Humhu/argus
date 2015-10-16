#pragma once

#include <ros/ros.h>

namespace extrinsics_array
{

/*! \brief Provides an interface to a ROS parameter server lookup service. */
class LookupInterface
{
public:

	LookupInterface( ros::NodeHandle& nh );
	
	/*! \brief Sets the lookup namespace. */
	void SetLookupNamespace( const std::string& ns );
	
	// TODO Convert parent path to fully qualified?
	/*! \brief Writes the parent entry. */
	void WriteLookup( const std::string& child, const std::string& parent );
	
	/*! \brief Tries to look up the parent. Returns success. */
	bool ReadLookup( const std::string& child, std::string& parent );
	
private:
	
	ros::NodeHandle nodeHandle;
	std::string lookupNamespace;
	
	std::string FormLookupPath( const std::string& child ) const;
	
};
	
} // end namespace extrinsics_array 
