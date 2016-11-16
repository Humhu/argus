#include <extrinsics_array/ExtrinsicsInterface.h>
#include <argus_utils/utils/ParamUtils.h>
#include <argus_utils/geometry/GeometryUtils.h>

namespace argus
{

ExtrinsicsInterface::ExtrinsicsInterface( ros::NodeHandle& nh,
                                          ros::NodeHandle& ph )
{
	double maxCacheTime;
	GetParam( ph, "max_cache_time", maxCacheTime, 10.0 );
	_tfBuffer = std::make_shared<tf2_ros::Buffer>( ros::Duration( maxCacheTime ) );
	_tfListener = std::make_shared<tf2_ros::TransformListener>( *_tfBuffer, nh );
}

PoseSE3 ExtrinsicsInterface::Convert( const std::string& fromIn,
                                      const std::string& toIn,
                                      const ros::Time& timeIn,
                                      const PoseSE3& poseIn,
                                      const std::string& fromOut, 
                                      const std::string& toOut )
{
	try
	{
		PoseSE3 fromExt, parentExt;
		if( fromIn != fromOut ) { fromExt = GetExtrinsics( fromOut, fromIn, timeIn ); }
		if( toIn != toOut ) { parentExt = GetExtrinsics( toIn, toOut, timeIn ); }
		return parentExt * poseIn * fromExt;
	}
	catch( ExtrinsicsException ex ) 
	{}
	
	try
	{
		PoseSE3 fromExt, parentExt;
		if( fromOut != toIn ) { fromExt = GetExtrinsics( fromOut, toIn, timeIn ); }
		if( fromIn != toOut ) { parentExt = GetExtrinsics( fromIn, toOut, timeIn ); }
		return parentExt * poseIn.Inverse() * fromExt;
	}
	catch( ExtrinsicsException ex )
	{
		throw ExtrinsicsException( "Could not convert " + fromIn + " -> " + toIn +
		                           " to requested " + fromOut + " -> " + toOut );
	}
}

PoseSE3 ExtrinsicsInterface::GetExtrinsics( const std::string& from,
                                            const std::string& to,
                                            const ros::Time& time )
{
	return GetExtrinsics( from, time, to, time );
}

PoseSE3 ExtrinsicsInterface::GetDisplacement( const std::string& from,
                                              const ros::Time& start,
                                              const ros::Time& stop )
{
	return GetExtrinsics( from, start, from, stop );
}

PoseSE3 ExtrinsicsInterface::GetExtrinsics( const std::string& from,
                                            const ros::Time& fromTime,
                                            const std::string& to,
                                            const ros::Time& toTime )
{
	std::string err;
	// NOTE Assuming the transform should be static in the to frame
	if( !_tfBuffer->canTransform( from, fromTime, 
	                              to, toTime, 
	                              to, ros::Duration(0), 
	                              &err ) )
	{
		throw ExtrinsicsException( "Could get extrinsics of " + from + " to " +
		                           to + " due to: " + err );
	}
	geometry_msgs::TransformStamped msg = _tfBuffer->lookupTransform( to,
	                                                                  toTime,
	                                                                  from, 
	                                                                  fromTime,
	                                                                  to,
	                                                                  ros::Duration(0) );
	return TransformToPose( msg.transform );
}

}