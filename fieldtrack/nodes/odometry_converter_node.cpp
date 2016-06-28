#include <ros/ros.h>

#include "nav_msgs/Odometry.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "lookup/LookupInterface.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

using namespace argus;

/*! \brief Translates nav_msgs::Odometry messages from an absolute frame
 * to a RelativePoseMessage. */
class OdometryTranslator 
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	OdometryTranslator( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
	: _nodeHandle( nh ), _privHandle( ph ), _lookupInterface(),
	_extrinsicsManager( _lookupInterface )
	{		
		std::string lookupNamespace;
		_privHandle.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
		_lookupInterface.SetLookupNamespace( lookupNamespace );
		
		_odomSub = _nodeHandle.subscribe( "odometry", 10, 
		                                  &OdometryTranslator::OdomCallback, 
                                          this );
		_posePub = _nodeHandle.advertise<nav_msgs::Odometry>( "poses", 10 );

		if( !GetMatrixParam<double>( _privHandle, "pose_covariance", _poseCov ) )
		{
			if( !GetDiagonalParam<double>( _privHandle, "pose_covariance", _poseCov ) )
			{
		 		ROS_WARN_STREAM( "Could not parse pose covariance. Using identity." );
				_poseCov = PoseSE3::CovarianceMatrix::Identity();
			}
		}

		if( !GetMatrixParam<double>( _privHandle, "twist_covariance", _twistCov ) )
		{
			if( !GetDiagonalParam<double>( _privHandle, "twist_covariance", _twistCov ) )
			{
				ROS_WARN_STREAM( "Could not parse twist covariance. Using identity." );
				_twistCov = PoseSE3::CovarianceMatrix::Identity();
			}
		} 
	}

private:
	
	ros::NodeHandle _nodeHandle;
	ros::NodeHandle _privHandle;
	
	LookupInterface _lookupInterface;
	ExtrinsicsInfoManager _extrinsicsManager;
	
	ros::Publisher _posePub;
	ros::Subscriber _odomSub;
    PoseSE3::CovarianceMatrix _poseCov;
    PoseSE3::CovarianceMatrix _twistCov;
	
	void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
	{
	    std::string targetFrame = msg->child_frame_id;
		if( !_extrinsicsManager.CheckMemberInfo( targetFrame ) )
		{
			ROS_WARN_STREAM( "Could not find extrinsics for target " << targetFrame );
			return;
		}
		const PoseSE3& ext = _extrinsicsManager.GetInfo( targetFrame ).extrinsics;
		PoseSE3::AdjointMatrix extAdj = PoseSE3::Adjoint( ext );
		PoseSE3 transPose =  MsgToPose( msg->pose.pose ) * ext.Inverse();
		PoseSE3::TangentVector transTan = extAdj * MsgToTangent( msg->twist.twist );

		// Convert the covariances into the new frame as well
		PoseSE3::CovarianceMatrix transPoseCov = extAdj * _poseCov * extAdj.transpose();
		PoseSE3::CovarianceMatrix transTwistCov = extAdj * _twistCov * extAdj.transpose();

		nav_msgs::Odometry output( *msg );
		output.pose.pose = PoseToMsg( transPose );
		SerializeMatrix( transPoseCov, output.pose.covariance );
		output.twist.twist = TangentToMsg( transTan );
		SerializeMatrix( transTwistCov, output.twist.covariance );
		_posePub.publish( output );
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
