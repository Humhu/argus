#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

void ImageCallback( const sensor_msgs::ImageConstPtr& msg, 
					const std::string& imageDir,
					const std::string& imagePrefix )
{
	
	static unsigned int counter = 0;
	
	cv_bridge::CvImageConstPtr frame;
	try
	{
		frame = cv_bridge::toCvShare( msg, "mono8" );
	}
	catch( cv_bridge::Exception& e )
	{
		ROS_ERROR( "VisualOdometryNode cv_bridge exception: %s", e.what() );
		return;
	}
	
	std::stringstream name;
	name << imageDir << imagePrefix << counter << ".png";
	counter++;
	cv::imwrite( name.str(), frame->image );
}

int main( int argc, char** argv )
{
	ros::init( argc, argv, "image_recorder" );
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle("~");
	
	std::string topicName = nodeHandle.resolveName( "image" );
	
	std::string imageDir, imagePrefix;
	privHandle.param<std::string>( "image_directory", imageDir, "~/" );
	if( imageDir.back() != '/' ) { imageDir += "/"; };
	
	privHandle.param<std::string>( "image_prefix", imagePrefix, "image_" );
	
	image_transport::ImageTransport rImagePort( nodeHandle );
	image_transport::Subscriber rImageSub = 
		rImagePort.subscribe( topicName, 5, 
							  boost::bind( &ImageCallback, _1, imageDir, imagePrefix ) );
	
	cv::namedWindow( topicName );
	
	ros::spin();
	
	cv::destroyWindow( topicName );
	
	return 0;
}
