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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	OdometryTranslator( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
	: nodeHandle( nh ), privHandle( ph ), lookupInterface(),
	extrinsicsManager( lookupInterface )
	{		
		std::string lookupNamespace;
		privHandle.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
		lookupInterface.SetLookupNamespace( lookupNamespace );
		
		odomSub = nodeHandle.subscribe( "odometry", 10, 
		                                &OdometryTranslator::OdomCallback, 
                                        this );
		posePub = nodeHandle.advertise<argus_msgs::RelativePoseWithCovariance>( "poses", 10 );

		std::vector<double> covVals;
		if( !privHandle.getParam( "pose_covariance", covVals ) )
		  {
		    poseCov = PoseSE3::CovarianceMatrix::Identity();
		  }
		else
		  {

		    if( !ParseMatrix( covVals, poseCov ) )
		      {
			ROS_ERROR_STREAM( "Could not parse pose covariance." );
			exit( -1 );
		      }
		  }
	}

private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	LookupInterface lookupInterface;
	ExtrinsicsInfoManager extrinsicsManager;
	
	ros::Publisher posePub;
	ros::Subscriber odomSub;
  PoseSE3::CovarianceMatrix poseCov;
	
	void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
	{
	  std::string targetFrame = msg->child_frame_id;
		if( !extrinsicsManager.CheckMemberInfo( targetFrame ) )
		{
			ROS_WARN_STREAM( "Could not find extrinsics for target " << targetFrame );
			return;
		}
		const ExtrinsicsInfo& extInfo = extrinsicsManager.GetInfo( targetFrame );
		PoseSE3 relPose =  MsgToPose( msg->pose.pose ) * extInfo.extrinsics.Inverse();

		RelativePoseWithCovariance relMsg;
		relMsg.header = msg->header;
		relMsg.header.stamp = ros::Time::now(); // TODO
		relMsg.relative_pose.observer_name = msg->header.frame_id; //extInfo.referenceFrame;
		relMsg.relative_pose.observer_time = msg->header.stamp;
		relMsg.relative_pose.target_name = extInfo.referenceFrame; //msg->header.frame_id;
		relMsg.relative_pose.relative_pose = PoseToMsg( relPose );

		//PoseSE3::CovarianceMatrix poseCov;
		//ParseMatrix( msg->pose.covariance, poseCov );
		SerializeSymmetricMatrix( poseCov, relMsg.covariance );
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
