#include <ros/ros.h>

#include "extrinsics_array/ExtrinsicsInterface.h"
#include "camplex/FiducialInfoManager.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

int main( int argc, char**argv )
{
	ros::init( argc, argv, "atag_registrar" );

	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );

	LookupInterface lookup;
	FiducialInfoManager manager( lookup );
	ExtrinsicsInterface extrinsics( nh );

	YAML::Node info;
	GetParam( ph, "", info );
	YAML::Node::const_iterator iter;
	for( iter = info.begin(); iter != info.end(); ++iter )
	{
		YAML::Node info = iter->second;
		std::string targetNamespace = iter->first.as<std::string>();

		std::string family;
		double width;
		unsigned int id;
		GetParamRequired( info, "family", family );
		GetParamRequired( info, "width", width );
		GetParamRequired<unsigned int>( info, "id", id );
		std::string tagName = "apriltag_" + family + "_id" + std::to_string( id );

		lookup.WriteNamespace( tagName, targetNamespace );

		PoseSE3 pose;
		std::string parentId;
		if( GetParam( info, "pose", pose ) && GetParam( info, "parent_id", parentId ) )
		{
			ROS_INFO_STREAM( "Populating extrinsics " << pose << " relative to " << parentId );
			extrinsics.SetStaticExtrinsics( tagName, parentId, pose );
		}

		Fiducial intrinsics;
		double w = width / 2;
		Translation3Type point1( 0, -w, -w );
		Translation3Type point2( 0, w, -w );
		Translation3Type point3( 0, w, w );
		Translation3Type point4( 0, -w, w );
		intrinsics.points.push_back( point1 );
		intrinsics.points.push_back( point2 );
		intrinsics.points.push_back( point3 );
		intrinsics.points.push_back( point4 );

		manager.WriteMemberInfo( tagName, intrinsics );
	}

	ros::spin();
}
