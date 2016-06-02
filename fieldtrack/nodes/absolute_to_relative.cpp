#include <ros/ros.h>

#include "nav_msgs/Odometry.h"
#include "argus_msgs/RelativePoseWithCovariance.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "lookup/LookupInterface.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

using namespace argus_msgs;
using namespace argus;

/*! \brief Translates nav_msgs::Odometry messages from an absolute frame
 * to a RelativePoseMessage. */
class OdometryTranslator 
{
public:

	OdometryTranslator( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
	: nodeHandle( nh ), privHandle( ph ), lookupInterface(),
	extrinsicsManager( lookupInterface )
	{
		if( !privHandle.getParam( "observer_frame", observerFrame) )
		{
			ROS_ERROR_STREAM( "Please specify observer frame name.");
		}
		
		std::string lookupNamespace;
		privHandle.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
		lookupInterface.SetLookupNamespace( lookupNamespace );
		
		odomSub = nodeHandle.subscribe( "odometry", 10, 
		                                &OdometryTranslator::OdomCallback, 
                                        this );
		posePub = nodeHandle.advertise<argus_msgs::RelativePoseWithCovariance>( "poses", 10 );
	}

private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	LookupInterface lookupInterface;
	ExtrinsicsInfoManager extrinsicsManager;
	
	ros::Publisher posePub;
	ros::Subscriber odomSub;
	std::string observerFrame;
	
	void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
	{
		if( !extrinsicsManager.CheckMemberInfo( observerFrame ) )
		{
			ROS_WARN_STREAM( "Could not find extrinsics for observer " << observerFrame );
			return;
		}
		PoseSE3 obsExt = extrinsicsManager.GetInfo( observerFrame ).extrinsics;
		PoseSE3 relPose = obsExt * MsgToPose( msg->pose.pose );

		RelativePoseWithCovariance relMsg;
		relMsg.header = msg->header;
		relMsg.relative_pose.observer_name = observerFrame;
		relMsg.relative_pose.observer_time = msg->header.stamp;
		relMsg.relative_pose.target_name = msg->header.frame_id;
		relMsg.relative_pose.relative_pose = PoseToMsg( relPose );
		PoseSE3::CovarianceMatrix cov;
		ParseMatrix( msg->pose.covariance, cov );
		SerializeSymmetricMatrix( cov, relMsg.covariance );
		posePub.publish( relMsg );
	}
};

int main( int argc, char** argv )
{

	ros::init( argc, argv, "absolute_to_relative" );
	
	ros::NodeHandle nh, ph( "~" );
	OdometryTranslator converter( nh, ph );
	
	ros::spin();
	
	return 0;
}
