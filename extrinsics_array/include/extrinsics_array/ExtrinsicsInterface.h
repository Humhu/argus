#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <argus_utils/geometry/PoseSE3.h>

#include <exception>

namespace argus
{

// Exception denoting an exception lookup error
class ExtrinsicsException : public std::exception
{
public:

	ExtrinsicsException( const std::string& msg );

	virtual const char* what() const throw();

private:

	std::string _msg;
};

/*! \brief Wrapper around the tf2 transform lookup system. */
class ExtrinsicsInterface
{
public:

	ExtrinsicsInterface( ros::NodeHandle& nh, ros::NodeHandle& ph );

	/*! \brief Look up the pose of 'target' relative to 'ref' at an optional time. 
	 * Throws an ExtrinsicsException if the lookup fails. */
	PoseSE3 GetExtrinsics( const std::string& target, 
	                       const std::string& ref,
	                       const ros::Time& time = ros::Time( 0 ) );

	/*! \brief Look up the displacement of 'target' from start to stop. Throws an
	 * ExtrinsicsException if the lookup fails. */
	PoseSE3 GetDisplacement( const std::string& target,
	                         const ros::Time& start,
	                         const ros::Time& stop );

	PoseSE3 GetExtrinsics( const std::string& target,
	                       const ros::Time& targetTime,
	                       const std::string& ref,
	                       const ros::Time& refTime );

private:

	std::shared_ptr<tf2_ros::Buffer> _tfBuffer;
	std::shared_ptr<tf2_ros::TransformListener> _tfListener;

};

}