#include <ros/ros.h>

#include "visualization_msgs/Marker.h"

#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/ParamUtils.h"

#include <nav_msgs/Odometry.h>

#include <boost/foreach.hpp>

using namespace argus;

class Visualizer
{
public:

Visualizer( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph )
{	
	_visPub = nodeHandle.advertise<visualization_msgs::Marker>( "vis_markers", 100 );
	
	GetParam<double>( privHandle, "robot/base_size/x", _xSize );
	GetParam<double>( privHandle, "robot/base_size/y", _ySize );
	GetParam<double>( privHandle, "robot/base_size/z", _zSize );

	GetParam<double>( privHandle, "robot/wheel_size/diameter", _wheelDiameter );
	GetParam<double>( privHandle, "robot/wheel_size/thickness", _wheelThickness );
	GetParam<double>( privHandle, "robot/wheel_size/protrusion", _wheelProtrusion );

	_odomSub = nodeHandle.subscribe( "odom", 
	                                 1,
	                                 &Visualizer::OdomCallback,
	                                 this );
}

void OdomCallback( const nav_msgs::Odometry::ConstPtr& smsg )
{

}

void PlotRobot( unsigned int id,
                const std::string& frameName,
                const ros::Time& time,
                const PoseSE3& pose,
                double alpha )
{
	// A robot consists of a box body and 4 wheels
	visualization_msgs::Marker msg;
	msg.header.stamp = time;
	msg.header.frame_id = frameName;
	msg.id = id;

	// Plot box
	msg.ns = "velocity_vis/robot/body"; // TODO Add the node name too
	msg.type = visualization_msgs::Marker::CUBE;
	msg.action = visualization_msgs::Marker::ADD;

	msg.scale.x = _xSize;
	msg.scale.y = _ySize;
	msg.scale.z = _zSize;

	msg.color.a = alpha;
	msg.color.r = 0.1;
	msg.color.b = 0.9;
	msg.color.g = 0.1;

	msg.pose = PoseToMsg( pose );

	_visPub.publish( msg );

	// Plot wheel
	msg.ns = "velocity_vis/robot/wheel1";
	msg.type = visualization_msgs::Marker::CYLINDER;
	msg.action = visualization_msgs::Marker::ADD;

	msg.scale.x = _wheelDiameter;
	msg.scale.y = _wheelDiameter;
	msg.scale.z = _wheelThickness;

	msg.color.a = alpha;
	msg.color.r = 0.5;
	msg.color.g = 0.5;
	msg.color.b = 0.5;
	
	double wheelX = _xSize/2 - _wheelDiameter/2;
	double wheelY = _ySize/2 + _wheelThickness/2;
	double wheelZ = -_zSize/2 + _wheelDiameter/2 - _wheelProtrusion;
	PoseSE3 offset( wheelX, wheelY, wheelZ, 0.7, 0.7, 0, 0 );
	msg.pose = PoseToMsg( pose * offset );

	_visPub.publish( msg );
}

private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	ros::Subscriber _odomSub;
	ros::Publisher _visPub;
	
	double _xSize;
	double _ySize;
	double _zSize;

	double _wheelDiameter;
	double _wheelThickness;
	double _wheelProtrusion;

};

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "odom_visualizer" );
	
	ros::NodeHandle nh, ph( "~" );
	Visualizer vis( nh, ph );
	
	ros::spin();
	return 0;
}
