#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <apriltags/TagDetector.h>
#include <apriltags/Tag16h5.h>
#include <apriltags/Tag25h7.h>
#include <apriltags/Tag25h9.h>
#include <apriltags/Tag36h9.h>
#include <apriltags/Tag36h11.h>

#include "atags/AtagCommon.h"
#include "argus_msgs/ImageFiducialDetections.h"
#include "camplex/CameraCalibration.h"
#include "fiducials/FiducialCommon.h"

/*! \brief A single-threaded AprilTag detector. */
class AtagNode
{
public:

AtagNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
: imagePort( nh )
{
	ph.param( "enable_undistortion", enableUndistortion, false );
	ph.param( "enable_normalization", enableNormalization, false );

	rawPublisher = ph.advertise<argus_msgs::ImageFiducialDetections>( "detections_raw", 20 );
	if( enableUndistortion || enableNormalization )
	{
		processedPublisher = ph.advertise<argus_msgs::ImageFiducialDetections>( "detections_processed", 20 );
	}
	
	ph.param<std::string>( "tag_family", tagFamily, "36h11" );
	if( tagFamily == "16h5" )
	{
		detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes16h5 );
	}
	else if( tagFamily == "25h7" )
	{
		detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes25h7 );
	}
	else if( tagFamily == "25h9" )
	{
		detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes25h9 );
	}
	else if( tagFamily == "36h9" )
	{
		detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes36h9 );
	}
	else if( tagFamily == "36h11" )
	{
		detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes36h11 );
	}
	else
	{
		ROS_ERROR( "Invalid tag family. Must be 16h5, 25h7, 25h9, 36h9, or 36h11" );
		exit( -1 );
	}
	
	ph.param( "max_skewness_ratio", maxSkewnessRatio, 3.0 );
	ph.param( "min_area_product", minAreaProduct, 4000.0 );
	
	int buffLen;
	ph.param( "buffer_size", buffLen, 5 );
	cameraSub = imagePort.subscribeCamera( "image", buffLen, &AtagNode::ImageCallback, this );
}
	
private:
	
std::string tagFamily;
AprilTags::TagDetector::Ptr detector;

bool enableUndistortion;
bool enableNormalization;

double maxSkewnessRatio; // Used to filter out skew detections
double minAreaProduct; // Used to filter out small detections

ros::Publisher rawPublisher;
ros::Publisher processedPublisher;

image_transport::ImageTransport imagePort;
image_transport::CameraSubscriber cameraSub;

void ImageCallback( const sensor_msgs::Image::ConstPtr& msg,
                    const sensor_msgs::CameraInfo::ConstPtr& info )
{
	camplex::CameraCalibration cameraModel( "camear", *info );
	
	// Detection occurs in grayscale
	cv::Mat frame = cv_bridge::toCvShare( msg, "mono8" )->image;
	std::vector<AprilTags::TagDetection> tagDetections = detector->extractTags( frame );
	
	if( tagDetections.size() == 0 ) { return; }
	
	// Publish raw
	std::vector<argus_msgs::FiducialDetection> fidDetections;
	fidDetections.reserve( tagDetections.size() );
	for( unsigned int i = 0; i < tagDetections.size(); i++ )
	{
		std::pair<double,double> diagLengths = atags::ComputeDiagonals( tagDetections[i] );
		double elarge = std::max( diagLengths.first, diagLengths.second );
		double esmall = std::min( diagLengths.first, diagLengths.second );
		double eratio = elarge / esmall;
		if( eratio > maxSkewnessRatio ) { continue; }
		double eprod = elarge * esmall;
		if( eprod < minAreaProduct ) { continue; }
		
		fidDetections.push_back( atags::TagToFiducial( tagDetections[i], tagFamily ) );
	}
	if( fidDetections.empty() ) { return; }
	
	argus_msgs::ImageFiducialDetections detMsg;
	detMsg.detections = fidDetections;
	detMsg.header.frame_id = msg->header.frame_id;
	detMsg.header.stamp = msg->header.stamp;
	
	rawPublisher.publish( detMsg );
	
	// Publish processed
	if( enableUndistortion || enableNormalization )
	{
		// Check for uncalibrated camera
		if( cameraModel.GetFx() == 0 || cameraModel.GetFy() == 0 ) { 
			ROS_WARN_STREAM( "Cannot undistort or normalize detection with uninitialized camera." );
			return; 
		}
		
		
		fiducials::UndistortDetections( fidDetections, cameraModel,
		                                enableUndistortion, enableNormalization,
		                                detMsg.detections );
		
		processedPublisher.publish( detMsg );
	}
}
	
};

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "atag_detector" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	AtagNode node( nh, ph );
	
	int numSpinners;
	ph.param( "num_threads", numSpinners, 1 );
	
	ros::AsyncSpinner spinner( numSpinners );
	spinner.start();
	ros::waitForShutdown();
	
	return 0;
	
}
