#include <ros/ros.h>

#include "geometry_msgs/TwistStamped.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "lookup/LookupInterface.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

using namespace argus;

class VelocityTranslator 
{
public:

	VelocityTranslator( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: _extrinsicsManager( nh, ph )
	{
		GetParamRequired( ph, "target_body_frame", _targetBodyFrame );
		_rawSub = nh.subscribe( "twist", 10,
		                        &VelocityTranslator::TwistCallback,
		                        this );
		_convPub = nh.advertise<geometry_msgs::TwistStamped>( "twist_converted", 10 );
	}

private:
	
	ExtrinsicsInterface _extrinsicsManager;
	
	ros::Subscriber _rawSub;
	ros::Publisher _convPub;

	std::string _targetBodyFrame;
	
	void TwistCallback( const geometry_msgs::TwistStamped::ConstPtr& msg )
	{
		geometry_msgs::TwistStamped output( *msg );

		// Convert the velocity
		PoseSE3::TangentVector origVel = MsgToTangent( msg->twist );
		// TODO Implement switch for TwistWithCovarianceStamped type
		// PoseSE3::CovarianceMatrix origVelCov;
		// ParseMatrix( msg->twist.covariance, origVelCov );

		PoseSE3 extrinsics = _extrinsicsManager.GetExtrinsics( msg->header.frame_id,
		                                                       _targetBodyFrame,
		                                                       msg->header.stamp );
		PoseSE3::TangentVector convVel = TransformTangent( origVel, 
		                                                   extrinsics );
		// PoseSE3::CovarianceMatrix convVelCov = TransformCovariance( origVelCov,
		                                                            // extrinsics );
		output.twist = TangentToMsg( convVel );
		output.header.frame_id = _targetBodyFrame;
		// SerializeMatrix( convVelCov, output.twist.covariance );
		_convPub.publish( output );
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "velocity_converter" );
	
	ros::NodeHandle nh, ph( "~" );
	VelocityTranslator converter( nh, ph );
	
	ros::spin();
	
	return 0;
}
