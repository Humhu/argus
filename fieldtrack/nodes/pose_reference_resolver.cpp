#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "argus_msgs/RelativePose.h"
#include "argus_msgs/RelativePoseWithCovariance.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "lookup/LookupInterface.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

using namespace argus_msgs;
using namespace argus;

class PoseReferenceResolver 
{
public:

	PoseReferenceResolver( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: _lookupInterface(),
	_extrinsicsManager( _lookupInterface )
	{
		// if( !ph.getParam( "reference_name", _referenceName ) )
		// {
		// 	ROS_ERROR_STREAM( "Please specify reference frame name." );
		// 	exit( -1 );
		// }
		
		GetParamRequired( ph, "resolve_observer", _resolveObserver );
		GetParamRequired( ph, "resolve_target", _resolveTarget );

		std::string lookupNamespace;
		ph.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
		_lookupInterface.SetLookupNamespace( lookupNamespace );
		
		// TODO In/Out buffer sizes

		std::string inputType;
		unsigned int inBuffSize;
		GetParamRequired( ph, "input_type", inputType );
		GetParam( ph, "input_buff_size", inBuffSize, (unsigned int) 10 );
		if( inputType == "relative_pose" )
		{
			_poseSub = nh.subscribe( "relative_pose",
				                     inBuffSize, 
			                         &PoseReferenceResolver::PoseCallback, 
	                                 this );
		}
		else if( inputType == "relative_pose_with_cov" )
		{
			_poseSub = nh.subscribe( "relative_pose",
				                     inBuffSize, 
			                         &PoseReferenceResolver::PoseWithCovCallback, 
	                                 this );
		}
		else
		{
			throw std::invalid_argument( "Unknown input type: " + inputType );
		}

		unsigned int outBuffSize;
		GetParamRequired( ph, "output_type", _outputType );
		GetParam( ph, "output_buff_size", outBuffSize, (unsigned int) 10 );
		if( _outputType == "pose" )
		{
			_posePub = nh.advertise<geometry_msgs::Pose>( "resolved_pose", 
				                                          outBuffSize );
		}
		else if( _outputType == "pose_stamped" )
		{
			_posePub = nh.advertise<geometry_msgs::PoseStamped>( "resolved_pose", 
				                                                 outBuffSize );
		}
		else if( _outputType == "pose_stamped_with_cov" )
		{
			_posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>( "resolved_pose", 
				                                                               outBuffSize );
		}
		else
		{
			throw std::invalid_argument( "Unknown output type: " + _outputType );
		}
	}

private:
	
	LookupInterface _lookupInterface;
	ExtrinsicsInfoManager _extrinsicsManager;
	
	std::string _outputType;
	bool _resolveObserver;
	bool _resolveTarget;

	ros::Publisher _posePub;
	ros::Subscriber _poseSub; std::string _referenceName;
	
	void PoseCallback( const RelativePose::ConstPtr& msg )
	{
		RelativePose out( *msg );
		if( !ProcessRelativePose( out ) ) { return; }

		if( _outputType == "pose" )
		{
			geometry_msgs::Pose po;
			po = out.relative_pose;
			_posePub.publish( po );
		}
		else if( _outputType == "pose_stamped" )
		{
			geometry_msgs::PoseStamped po;
			po.pose = out.relative_pose;
			po.header.stamp = out.observer_time;
			po.header.frame_id = out.observer_name;	
			_posePub.publish( po );
		}
		else if( _outputType == "pose_stamped_with_cov" )
		{
			geometry_msgs::PoseWithCovarianceStamped po;
			po.pose.pose = out.relative_pose;
			po.header.stamp = out.observer_time;
			po.header.frame_id = out.observer_name;
			po.pose.covariance.fill( 0 );
		}
	}

	void PoseWithCovCallback( const RelativePoseWithCovariance::ConstPtr& msg )
	{
		RelativePoseWithCovariance out( *msg );
		if( !ProcessRelativePose( out.relative_pose ) ) { return; }

		if( _outputType == "pose" )
		{
			geometry_msgs::Pose po;
			po = out.relative_pose.relative_pose;
			_posePub.publish( po );
		}
		else if( _outputType == "pose_stamped" )
		{
			geometry_msgs::PoseStamped po;
			po.pose = out.relative_pose.relative_pose;
			po.header.stamp = out.relative_pose.observer_time;
			po.header.frame_id = out.relative_pose.observer_name;	
			_posePub.publish( po );
		}
		else if( _outputType == "pose_stamped_with_cov" )
		{
			geometry_msgs::PoseWithCovarianceStamped po;
			po.pose.pose = out.relative_pose.relative_pose;
			po.header.stamp = out.relative_pose.observer_time;
			po.header.frame_id = out.relative_pose.observer_name;
			po.pose.covariance = out.covariance;
		}
	}

	bool ProcessRelativePose( RelativePose& rel )
	{
		PoseSE3 pose = MsgToPose( rel.relative_pose );
		if( _resolveObserver )
		{
			if( !_extrinsicsManager.CheckMemberInfo( rel.observer_name ) )
			{
				ROS_ERROR_STREAM( "Could not get extrinsics for observer: " << rel.observer_name );
				return false;
			}
			const ExtrinsicsInfo& info = _extrinsicsManager.GetInfo( rel.observer_name );
			pose = info.extrinsics * pose;
		}
		if( _resolveTarget )
		{
			if( !_extrinsicsManager.CheckMemberInfo( rel.target_name ) )
			{
				ROS_ERROR_STREAM( "Could not get extrinsics for observer: " << rel.target_name );
				return false;
			}
			const ExtrinsicsInfo& info = _extrinsicsManager.GetInfo( rel.target_name );
			pose = pose * info.extrinsics.Inverse();
		}
		rel.relative_pose = PoseToMsg( pose );
		return true;
	}
};

int main( int argc, char** argv )
{

	ros::init( argc, argv, "static_pose_converter" );
	
	ros::NodeHandle nh, ph( "~" );
	PoseReferenceResolver converter( nh, ph );
	
	ros::spin();
	
	return 0;
}
