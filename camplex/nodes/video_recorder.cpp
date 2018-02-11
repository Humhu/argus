#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

class VideoRecorder 
{
public:
	
	VideoRecorder( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
	: imagePort( nh )
	{
		ph.param<std::string>( "output_file", outputName, "output.mkv" );
		ph.param<int>( "framerate", framerate, 30 );
		
		imageSub = imagePort.subscribe( "image", 5, &VideoRecorder::ImageCallback, this );	
		
		ROS_INFO_STREAM( "Recording video to output: " << outputName );

		std::string ccCode;
		ph.param<std::string>( "encoding", ccCode, "XVID" );
		if( ccCode.size() != 4 )
		{
			throw std::invalid_argument("Encoding must be length 4 but got " + ccCode );
		}
		fourCC = cv::VideoWriter::fourcc(ccCode[0], ccCode[1], ccCode[2], ccCode[3] );
	}
	
	void ImageCallback( const sensor_msgs::ImageConstPtr& msg )
	{
		cv_bridge::CvImageConstPtr frame;
		try
		{
			frame = cv_bridge::toCvShare( msg, "bgr8" );
		}
		catch( cv_bridge::Exception& e )
		{
			ROS_ERROR( "cv_bridge exception: %s", e.what() );
			return;
		}
		
		if( !vidWriter.isOpened() )
		{
			vidWriter.open( outputName, fourCC, framerate, frame->image.size() );
		}
		
		vidWriter.write( frame->image );
		
	}
	
private:

	image_transport::ImageTransport imagePort;
	image_transport::Subscriber imageSub;
	cv::VideoWriter vidWriter;
	
	int fourCC;
	std::string outputName;
	int framerate;
	
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "video_recorder" );
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle("~");
	VideoRecorder recorder( nodeHandle, privHandle );
	
	ros::spin();
	
	return 0;
}
