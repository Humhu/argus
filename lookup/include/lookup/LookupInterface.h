#pragma once

#include <ros/ros.h>

namespace lookup
{

/*! \brief Provides an interface to a ROS parameter server lookup service. */
class LookupInterface
{
public:

	LookupInterface( ros::NodeHandle& nh );
	
	/*! \brief Sets the lookup namespace. */
	void SetLookupNamespace( const std::string& ns );
	
	/*! \brief Writes the parent entry. */
	void WriteParent( const std::string& child, const std::string& parent );
	
	/*! \brief Tries to look up the parent. Returns success. */
	bool ReadParent( const std::string& child, std::string& parent );
	
	/*! \brief Writes the frame displacement topic. */
	
private:
	
	ros::NodeHandle nodeHandle;
	std::string lookupNamespace;
	
	std::string FormLookupPath( const std::string& child ) const;
	
};
	
} // end namespace lookup 
