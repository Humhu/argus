#include <ros/ros.h>
#include <image_transport/image_transport.h>

/*! \brief A node to broadcast camera info messages. */
class DummyCameraInfoNode
{
public:

DummyCameraInfoNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
{
  std::string frame_id;
  if (!getParameter(ph, "frame_id", frame_id)) exit(-1);

  msg.header.frame_id = frame_id;

	info_pub = ph.advertise<sensor_msgs::CameraInfo>( "/" + frame_id + "/camera_info", 1 );

  double fx, fy, cx, cy;
  if (!getParameter(ph, "fx", fx)) exit(-1);
  if (!getParameter(ph, "fy", fy)) exit(-1);
  if (!getParameter(ph, "cx", cx)) exit(-1);
  if (!getParameter(ph, "cy", cy)) exit(-1);

  msg.K[0] = fx;
  msg.K[1] = 0.0;
  msg.K[2] = cx;
  msg.K[3] = 0.0;
  msg.K[4] = fy;
  msg.K[5] = cy;
  msg.K[6] = 0.0;
  msg.K[7] = 0.0;
  msg.K[8] = 1.0;

  for (unsigned int i = 0; i < 9; i++)
    msg.R[i] = 0.0;

  msg.R[0] = 1.0;
  msg.R[4] = 1.0;
  msg.R[8] = 1.0;

  int width;
  int height;

  if (!getParameter(ph, "width", width)) exit(-1);
  if (!getParameter(ph, "height", height)) exit(-1);

  msg.width = width;
  msg.height = height;

  image_sub = ph.subscribe("/" + frame_id + "/image_rect", 1, &DummyCameraInfoNode::ImageCallback, this);
}

private:

sensor_msgs::CameraInfo msg;

ros::Publisher info_pub;
ros::Subscriber image_sub;

void ImageCallback(const sensor_msgs::Image::ConstPtr& m)
{
  msg.header.stamp = m->header.stamp;
  info_pub.publish(msg);
}

bool getParameter(ros::NodeHandle& n, const std::string& s, std::string& val)
{
  if (!n.getParam( s, val ))
  {
    ROS_ERROR("Failed to load parameter %s", s.c_str());
    return false;
  }
  return true;
}

bool getParameter(ros::NodeHandle& n, const std::string& s, double& val)
{
  if (!n.getParam( s, val ))
  {
    ROS_ERROR("Failed to load parameter %s", s.c_str());
    return false;
  }
  return true;
}

bool getParameter(ros::NodeHandle& n, const std::string& s, int& val)
{
  if (!n.getParam( s, val ))
  {
    ROS_ERROR("Failed to load parameter %s", s.c_str());
    return false;
  }
  return true;
}

};

int main( int argc, char** argv )
{

	ros::init( argc, argv, "dummy_camera_info" );

	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );

	DummyCameraInfoNode node( nh, ph );

	int numSpinners;
	ph.param( "num_threads", numSpinners, 1 );

	ros::AsyncSpinner spinner( numSpinners );
	spinner.start();
	ros::waitForShutdown();

	return 0;

}
