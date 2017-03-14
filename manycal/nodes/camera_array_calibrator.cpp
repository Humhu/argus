#include <ros/ros.h>

#include "manycal/CameraArrayCalibrator.h"
#include "manycal/ManycalVisualization.h"
#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>

using namespace argus;

class CameraCalibrationNode
{
public:

	CameraCalibrationNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: _calibrator( nh, ph )
	{
		if( ph.hasParam( "visualization" ) )
		{
			ros::NodeHandle	ch( ph.resolveName( "visualization/camera" ) );
			ros::NodeHandle	fh( ph.resolveName( "visualization/fiducial" ) );

			_camVis.ReadParams( ch );
			_fidVis.ReadParams( fh );

			std::string refFrame;
			GetParamRequired( ph, "reference_frame", refFrame );
			_camVis.SetFrameID( refFrame );
			_fidVis.SetFrameID( refFrame );

			double rate;
			GetParamRequired( ph, "visualization/rate", rate );
			_visTimer = nh.createTimer( ros::Duration( 1.0 / rate ), &CameraCalibrationNode::TimerCallback, this );

			_visPub = nh.advertise<MarkerMsg>( "markers", 10 );
		}
	}

private:

	CameraArrayCalibrator _calibrator;

	ros::Publisher _visPub;
	ros::Timer _visTimer;
	PoseVisualizer _camVis;
	FiducialVisualizer _fidVis;

	void TimerCallback( const ros::TimerEvent& event )
	{
		std::vector<FiducialCalibration> fids = _calibrator.GetFiducials();
		std::vector<CameraCalibration> cams = _calibrator.GetCameras();

		std::vector<PoseSE3> fidPoses;
		std::vector<Fiducial> fidInts;
		std::vector<std::string> fidNames;
		BOOST_FOREACH( const FiducialCalibration& fid, fids )
		{
			fidPoses.push_back( fid.extrinsics );
			fidInts.push_back( fid.intrinsics );
			fidNames.push_back( fid.name );
		}

		std::vector<PoseSE3> camPoses;
		std::vector<std::string> camNames;
		BOOST_FOREACH( const CameraCalibration& cam, cams )
		{
			camPoses.push_back( cam.extrinsics );
			camNames.push_back( cam.name );
		}
		
		std::vector<MarkerMsg> fidMarkers = _fidVis.ToMarkers( fidPoses, fidInts, fidNames );
		std::vector<MarkerMsg> camMarkers = _camVis.ToMarkers( camPoses, camNames );
		BOOST_FOREACH( const MarkerMsg& msg, fidMarkers )
		{
			_visPub.publish( msg );
		}
		BOOST_FOREACH( const MarkerMsg& msg, camMarkers )
		{
			_visPub.publish( msg );
		}
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "camera_array_calibrator" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	CameraCalibrationNode node( nh, ph );
	ros::spin();
	
	return 0;
}
