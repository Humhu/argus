#include <extrinsics_array/ExtrinsicsInterface.h>
#include <argus_utils/utils/ParamUtils.h>
#include <argus_utils/geometry/GeometryUtils.h>

namespace argus
{

ExtrinsicsException::ExtrinsicsException( const std::string& msg )
: _msg( msg ) {}

const char* ExtrinsicsException::what() const throw()
{
	return ("ExtrinsicsException: " + _msg).c_str();
}

ExtrinsicsInterface::ExtrinsicsInterface( ros::NodeHandle& nh,
                                          ros::NodeHandle& ph )
{
	double maxCacheTime;
	GetParam( ph, "max_cache_time", maxCacheTime, 10.0 );
	_tfBuffer = std::make_shared<tf2_ros::Buffer>( ros::Duration( maxCacheTime ) );
	_tfListener = std::make_shared<tf2_ros::TransformListener>( *_tfBuffer, nh );
}

PoseSE3 ExtrinsicsInterface::GetExtrinsics( const std::string& target,
                                            const std::string& ref,
                                            const ros::Time& time )
{
	return GetExtrinsics( target, time, ref, time );
}

PoseSE3 ExtrinsicsInterface::GetDisplacement( const std::string& target,
                                              const ros::Time& start,
                                              const ros::Time& stop )
{
	return GetExtrinsics( target, start, target, stop );
}

PoseSE3 ExtrinsicsInterface::GetExtrinsics( const std::string& target,
                                            const ros::Time& targetTime,
                                            const std::string& ref,
                                            const ros::Time& refTime )
{
	std::string err;
	// NOTE Assuming the transform should be static in the ref frame
	if( !_tfBuffer->canTransform( target, targetTime, 
	                              ref, refTime, 
	                              ref, ros::Duration(0), 
	                              &err ) )
	{
		throw ExtrinsicsException( err );
	}
	geometry_msgs::TransformStamped msg = _tfBuffer->lookupTransform( ref,
	                                                                  refTime,
	                                                                  target, 
	                                                                  targetTime,
	                                                                  target,
	                                                                  ros::Duration(0) );
	return TransformToPose( msg.transform );
}

}