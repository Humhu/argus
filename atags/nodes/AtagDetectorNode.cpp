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

using namespace argus;

/*! \brief A single-threaded AprilTag _detector. */
class AtagNode
{
public:

AtagNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
: _imagePort( nh )
{
	ph.param( "enable_undistortion", _enableUndistortion, false );
	ph.param( "enable_normalization", _enableNormalization, false );

	_rawPub = ph.advertise<argus_msgs::ImageFiducialDetections>( "detections_raw", 20 );
	if( _enableUndistortion || _enableNormalization )
	{
		_procPub = ph.advertise<argus_msgs::ImageFiducialDetections>( "detections_processed", 20 );
	}
	
	ph.param<std::string>( "tag_family", _tagFamily, "36h11" );
	if( _tagFamily == "16h5" )
	{
		_detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes16h5 );
	}
	else if( _tagFamily == "25h7" )
	{
		_detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes25h7 );
	}
	else if( _tagFamily == "25h9" )
	{
		_detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes25h9 );
	}
	else if( _tagFamily == "36h9" )
	{
		_detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes36h9 );
	}
	else if( _tagFamily == "36h11" )
	{
		_detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes36h11 );
	}
	else
	{
		ROS_ERROR( "Invalid tag family. Must be 16h5, 25h7, 25h9, 36h9, or 36h11" );
		exit( -1 );
	}
	
	ph.param( "max_skewness_ratio", _maxSkewnessRatio, 3.0 );
	ph.param( "min_area_product", _minAreaProduct, 4000.0 );
	
	int buffLen;
	ph.param( "buffer_size", buffLen, 5 );
	_cameraSub = _imagePort.subscribeCamera( "image", buffLen, &AtagNode::ImageCallback, this );
}
	
private:
	
std::string _tagFamily;
AprilTags::TagDetector::Ptr _detector;

bool _enableUndistortion;
bool _enableNormalization;

double _maxSkewnessRatio; // Used to filter out skew detections
double _minAreaProduct; // Used to filter out small detections

ros::Publisher _rawPub;
ros::Publisher _procPub;

image_transport::ImageTransport _imagePort;
image_transport::CameraSubscriber _cameraSub;

void ImageCallback( const sensor_msgs::Image::ConstPtr& msg,
                    const sensor_msgs::CameraInfo::ConstPtr& info )
{
	camplex::CameraCalibration cameraModel( "camera", *info );
	
	// Detection occurs in grayscale
	cv::Mat msgFrame = cv_bridge::toCvShare( msg )->image;
	cv::Mat frame;
	if( msgFrame.channels() > 1 )
	{
		cv::cvtColor( msgFrame, frame, CV_BGR2GRAY );
	}
	else
	{
		frame = msgFrame;
	}

	std::vector<AprilTags::TagDetection> tagDetections = _detector->extractTags( frame );
	
	if( tagDetections.size() == 0 ) { return; }
	
	argus_msgs::ImageFiducialDetections rawMsg;
	rawMsg.header.frame_id = msg->header.frame_id;
	rawMsg.header.stamp = msg->header.stamp;
	rawMsg.detections.reserve( tagDetections.size() );

	argus_msgs::ImageFiducialDetections procMsg( rawMsg );
	for( unsigned int i = 0; i < tagDetections.size(); i++ )
	{
		std::pair<double,double> diagLengths = ComputeDiagonals( tagDetections[i] );
		Eigen::Matrix2d cov = ComputeCovariance( tagDetections[i] );
		Eigen::Vector2cd eigenvalues = cov.eigenvalues();
		double elarge = std::max( eigenvalues(0).real(), eigenvalues(1).real() );
		double esmall = std::min( eigenvalues(0).real(), eigenvalues(1).real() );
		double eratio = elarge / esmall;
		double eprod = diagLengths.first * diagLengths.second;
		//ROS_INFO_STREAM( "ID: " << tagDetections[i].id << " ratio: " << eratio << " area: " << eprod );
;		if( eratio > _maxSkewnessRatio ) { continue; }
		if( eprod < _minAreaProduct ) { continue; }
		
		FiducialDetection fid = TagToFiducial( tagDetections[i], _tagFamily );
		rawMsg.detections.push_back( fid.ToMsg() );

		if( _enableUndistortion || _enableNormalization )
		{
			FiducialDetection fidProc = fid.Undistort( cameraModel, 
			                                           _enableUndistortion,
			                                           _enableNormalization );
			procMsg.detections.push_back( fidProc.ToMsg() );
		}
	}

	// Don't publish if no detections
	if( rawMsg.detections.empty() ) { return; }
	_rawPub.publish( rawMsg );
	
	if( _enableUndistortion || _enableNormalization )
	{
		_procPub.publish( procMsg );
	}
}
	
};

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "atag__detector" );
	
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
