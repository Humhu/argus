#include <ros/ros.h>

#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/utils/ParamUtils.h"

#include "argus_msgs/ImageFiducialDetections.h"

#include "camplex/FiducialCommon.h"
#include "camplex/FiducialInfoManager.h"
#include "camplex/FiducialVisualizer.h"

#include "vizard/PoseVisualizer.h"

#include "extrinsics_array/ExtrinsicsInterface.h"


#include <boost/foreach.hpp>

using namespace argus;

class FiducialPoseEstimator
{
public:

	FiducialPoseEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: _fiducialManager( _lookupInterface ),
		_extrinsicsInterface( nh, ph )
	{
		GetParam<std::string>( ph, "reference_frame", _refFrame, "" );

		unsigned int inBuffSize, outBuffSize;
		GetParam( ph, "input_buffer_size", inBuffSize, (unsigned int) 20 );
		GetParam( ph, "output_buffer_size", outBuffSize, (unsigned int) 20 );

		_enableVis = ph.hasParam( "visualization" );
		if( _enableVis )
		{
			ros::NodeHandle ch( ph.resolveName( "visualization/camera" ) );
			ros::NodeHandle fh( ph.resolveName( "visualization/fiducial" ) );
			_camVis.ReadParams( ch );
			_fidVis.ReadParams( fh );

			std::string refFrame;
			GetParamRequired( ph, "visualization/reference_frame", refFrame );
			_camVis.SetFrameID( refFrame );
			_fidVis.SetFrameID( refFrame );
			_visPub = nh.advertise<MarkerMsg>( "markers", 10 );
		}

		_detSub = nh.subscribe( "detections",
		                        inBuffSize,
		                        &FiducialPoseEstimator::DetectionsCallback,
		                        this );
		_posePub = ph.advertise<geometry_msgs::TransformStamped>( "relative_pose",
		                                                          outBuffSize );
	}

private:

	std::string _refFrame;
	ros::Subscriber _detSub;
	ros::Publisher _posePub;

	LookupInterface _lookupInterface;
	FiducialInfoManager _fiducialManager;
	ExtrinsicsInterface _extrinsicsInterface;

	bool _enableVis;
	PoseVisualizer _camVis;
	FiducialVisualizer _fidVis;
	ros::Publisher _visPub;

	std::unordered_map<std::string, Fiducial> _transformedFiducials;

	// Gets the specified fiducial transformed into _refFrame at the specified time
	bool GetFiducial( const std::string& name, const ros::Time& time,
	                  Fiducial& fid, PoseSE3& extrinsics )
	{
		// Force lookup of fiducials in case initialization is slow
		// NOTE This means "rogue" undocumented fiducials will slow the system down
		if( !_fiducialManager.HasMember( name ) )
		{
			if( !_fiducialManager.ReadMemberInfo( name, true ) )
			{
				ROS_INFO_STREAM( "Could not read intrinsics for " << name );
				return false;
			}
		}
		fid = _fiducialManager.GetInfo( name );
		extrinsics = PoseSE3();
		if( _refFrame.empty() ) { return true; }

		try
		{
			extrinsics = _extrinsicsInterface.GetExtrinsics( name, _refFrame, time );
			return true;
		}
		catch( ExtrinsicsException& ex )
		{
			ROS_INFO_STREAM( "Could not get extrinsics for " << name << std::endl << ex.what() );
			return false;
		}
	}

	void DetectionsCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg )
	{
		const std::string& cameraName = msg->header.frame_id;
		const ros::Time& detTime = msg->header.stamp;

		std::vector<Fiducial> fids;
		std::vector<FiducialDetection> detections;
		std::vector<PoseSE3> fidExts;
		// 1. Process all fiducials
		BOOST_FOREACH( const argus_msgs::FiducialDetection & det, msg->detections )
		{
			Fiducial fid;
			PoseSE3 extrinsics;
			if( !GetFiducial( det.name, detTime, fid, extrinsics ) ) { continue; }

			Fiducial fidTrans = fid.Transform( extrinsics );
			fids.push_back( fid );
			fidExts.push_back( extrinsics );
			detections.push_back( det );
		}

		if( detections.empty() )
		{
			return;
		}

		PoseSE3 relPose = EstimateArrayPose( detections, fids );

		geometry_msgs::TransformStamped poseMsg;
		poseMsg.header.stamp = msg->header.stamp;
		poseMsg.header.frame_id = _refFrame;
		poseMsg.child_frame_id = cameraName;
		poseMsg.transform = PoseToTransform( relPose );
		_posePub.publish( poseMsg );

		if( _enableVis && _visPub.getNumSubscribers() > 0 )
		{
			_camVis.SetFrameID( cameraName );
			_fidVis.SetFrameID( cameraName );

			std::vector<PoseSE3> camRelFidPoses;
			std::vector<std::string> fidNames;
			for( unsigned int i = 0; i < detections.size(); ++i )
			{
				camRelFidPoses.push_back( relPose * fidExts[i] );
				fidNames.push_back( detections[i].name );
			}

			std::vector<PoseSE3> posesToVis;
			std::vector<std::string> namesToVis;
			posesToVis.push_back( PoseSE3() ); // Camera pose
			namesToVis.push_back( cameraName );
			posesToVis.push_back( relPose ); // Ref frame pose
			namesToVis.push_back( _refFrame );

			std::vector<MarkerMsg> fidMarkers = _fidVis.ToMarkers( camRelFidPoses, fids, fidNames );
			std::vector<MarkerMsg> camMarkers = _camVis.ToMarkers( posesToVis, namesToVis );

			BOOST_FOREACH( const MarkerMsg& msg, fidMarkers )
			{
				_visPub.publish( msg );
			}
			BOOST_FOREACH( const MarkerMsg& msg, camMarkers )
			{
				_visPub.publish( msg );
			}
		}
	}

};

int main( int argc, char**argv )
{
	ros::init( argc, argv, "fiducial_pose_estimator" );

	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );

	FiducialPoseEstimator estimator( nh, ph );
	ros::spin();

	exit( 0 );
}
