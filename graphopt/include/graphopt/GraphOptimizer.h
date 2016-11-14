# pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <isam/Slam.h>

namespace argus
{

/*! \brief Wraps the ISAM graph optimizer with ROS functionality.
*/
class GraphOptimizer
{
public:

	GraphOptimizer();

	/*! \brief Construct an optimizer from a private namespace. */
	GraphOptimizer( ros::NodeHandle& ph );

	/*! \brief Retrieve a reference to the raw optimizer object. */
	isam::Slam& GetOptimizer();
	const isam::Slam& GetOptimizer() const;

private:

	isam::Slam::Ptr _optimizer;
};

}