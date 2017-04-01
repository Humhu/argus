#include <ros/ros.h>

#include "extrinsics_array/ExtrinsicsInterface.h"
#include "camplex/FiducialInfoManager.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

int main( int argc, char**argv )
{
	ros::init( argc, argv, "checkerboard_registrar" );

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

		unsigned int width, height;
        double dim;
		GetParamRequired<unsigned int>( info, "width", width );
        GetParamRequired<unsigned int>( info, "height", height );
        GetParamRequired( info, "dim", dim );
		std::stringstream ss;
        ss << "checkerboard_" << width << "_" << height;
		std::string boardName = ss.str();

		lookup.WriteNamespace( boardName, targetNamespace );

		PoseSE3 pose;
		std::string parentId;
		if( GetParam( info, "pose", pose ) && GetParam( info, "parent_id", parentId ) )
		{
			ROS_INFO_STREAM( "Populating extrinsics " << pose << " relative to " << parentId );
			extrinsics.SetStaticExtrinsics( boardName, parentId, pose );
		}


		Fiducial intrinsics;
        for( unsigned int i = 0; i < height; i++ )
        {
            for( unsigned int j = 0; j < width; j++ )
            {
                intrinsics.points.emplace_back( 0, i*dim, j*dim );
            }
        }

		manager.WriteMemberInfo( boardName, intrinsics );
	}

	exit( 0 );
}
