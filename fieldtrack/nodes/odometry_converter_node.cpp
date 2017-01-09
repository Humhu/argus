#include <ros/ros.h>

#include "nav_msgs/Odometry.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "lookup/LookupInterface.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

using namespace argus;

class OdometryTranslator 
{
public:

	OdometryTranslator( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: _extrinsicsManager( nh, ph )
	{
	  GetParam( ph, "override_reference_frame", _overrideFrameId );
	  GetParam( ph, "override_child_frame", _overrideChildId );

		GetParamRequired( ph, "reference_frame", _targetRefFrame );
		GetParamRequired( ph, "body_frame", _targetBodyFrame );
		_odomSub = nh.subscribe( "odometry", 10,
		                         &OdometryTranslator::OdomCallback,
		                         this );
		_posePub = nh.advertise<nav_msgs::Odometry>( "poses", 10 );
	}

private:
	
	ExtrinsicsInterface _extrinsicsManager;
	
	ros::Publisher _posePub;
	ros::Subscriber _odomSub;

	std::string _overrideFrameId;
	std::string _overrideChildId;

	std::string _targetRefFrame;
	std::string _targetBodyFrame;
	
	void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
	{
	  std::string bodyFrame = _overrideFrameId.empty() ? msg->header.frame_id : _overrideFrameId;
	  std::string refFrame = _overrideChildId.empty() ? msg->child_frame_id : _overrideChildId;

		nav_msgs::Odometry output( *msg );

		// Convert the pose
		PoseSE3 origPose = MsgToPose( msg->pose.pose );
		PoseSE3 convPose = _extrinsicsManager.Convert( bodyFrame,
		                                               refFrame,
		                                               msg->header.stamp,
		                                               origPose,
		                                               _targetBodyFrame,
		                                               _targetRefFrame );
		output.pose.pose = PoseToMsg( convPose );
		// TODO Convert the covariance too

		// Convert the velocity
		PoseSE3::TangentVector origVel = MsgToTangent( msg->twist.twist );
		PoseSE3::CovarianceMatrix origVelCov;
		ParseMatrix( msg->twist.covariance, origVelCov );

		PoseSE3 extrinsics = _extrinsicsManager.GetExtrinsics( bodyFrame,
		                                                       _targetBodyFrame,
		                                                       msg->header.stamp );
		PoseSE3::TangentVector convVel = TransformTangent( origVel, 
		                                                   extrinsics );
		PoseSE3::CovarianceMatrix convVelCov = TransformCovariance( origVelCov,
		                                                            extrinsics );
		output.twist.twist = TangentToMsg( convVel );
		SerializeMatrix( convVelCov, output.twist.covariance );
		_posePub.publish( output );
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "odometry_converter" );
	
	ros::NodeHandle nh, ph( "~" );
	OdometryTranslator converter( nh, ph );
	
	ros::spin();
	
	return 0;
}
