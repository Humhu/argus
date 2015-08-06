#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "odoflow/VisualOdometryPipeline.h"
#include "odoflow/ROSParser.h"
#include "odoflow/InterfaceBindings.h"

using namespace argus_utils;
using namespace odoflow;

// TODO Add image pyramids
class VisualOdometryNode
{
	// ROS Node members
	ros::NodeHandle rNode;
	ros::NodeHandle pNode;
	
	ros::Publisher posePub;
	
	image_transport::ImageTransport rImagePort;
	image_transport::CameraSubscriber rImageSub;
	
	image_geometry::PinholeCameraModel cameraModel;
	
	VisualOdometryPipeline pipeline;
	
	PoseSE3 pose;
	
public:
	
	VisualOdometryNode()
		: pNode( "~" ),
		rImagePort( rNode )
	{
		
		// Subscribe to image stream from some input source
		std::string topicName = rNode.resolveName( "image_raw" );
		rImageSub = rImagePort.subscribeCamera( topicName, 1,
			&VisualOdometryNode::ImageCallback, this );
		
		posePub = pNode.advertise<geometry_msgs::PoseStamped>( "odometry", 10 );
		
		InterestPointDetector::Ptr detector = ParseDetector( pNode );
		InterestPointTracker::Ptr tracker = ParseTracker( pNode );
		MotionEstimator::Ptr estimator = ParseEstimator( pNode );
		
		// TODO Disallow default construction of pipeline?
		pipeline.SetDetector( detector );
		pipeline.SetTracker( tracker );
		pipeline.SetEstimator( estimator );
		
		bool showOutput;
		pNode.param( "show_output", showOutput, false );
		pipeline.SetVisualization( showOutput );
		
		int redetectionThreshold;
		pNode.param( "redetection_threshold", redetectionThreshold, 10 );
		pipeline.SetRedetectionThreshold( redetectionThreshold );
		
		pose = PoseSE3(); // Initialization (technically unnecessary)
		
	}
	
	~VisualOdometryNode()
	{}
	
	void ImageCallback( const sensor_msgs::ImageConstPtr& msg,
						const sensor_msgs::CameraInfoConstPtr& info_msg )
	{
		
		cameraModel.fromCameraInfo( info_msg );
		pipeline.SetRectificationParameters( cameraModel.intrinsicMatrix(),
											 cameraModel.distortionCoeffs() );
		
		Timepoint timestamp = msg->header.stamp.toBoost();
		
		// Decode the received image into an OpenCV object
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
		
		cv::Mat image = frame->image;
		
		ros::Time start = ros::Time::now();
		PoseSE3 displacement;
		bool success = pipeline.ProcessImage( image, timestamp, displacement );
		ros::Time finish = ros::Time::now();
		
		if( success )
		{
			pose = pose*displacement.Inverse();
			
			PoseSE3::Vector poseVector = pose.ToVector();
			
			geometry_msgs::PoseStamped poseMsg;
			poseMsg.header = msg->header;
			poseMsg.header.frame_id = "0";
			poseMsg.pose.position.x = poseVector(0);
			poseMsg.pose.position.y = poseVector(1);
			poseMsg.pose.position.z = poseVector(2);
			poseMsg.pose.orientation.w = poseVector(3);
			poseMsg.pose.orientation.x = poseVector(4);
			poseMsg.pose.orientation.y = poseVector(5);
			poseMsg.pose.orientation.z = poseVector(6);
			
			
			
			posePub.publish( poseMsg );
		}
		
		ros::Duration dt = finish - start;
		
// 		std::cout << "Displacement: " << displacement << std::endl;
// 		std::cout << "Pose: " << pose << std::endl;
// 		std::cout << "Took " << dt.toSec() << " seconds to process." << std::endl;
		
	}
	
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "optical_flow" );
	VisualOdometryNode flow;
	ros::spin();
	return 0;
}
