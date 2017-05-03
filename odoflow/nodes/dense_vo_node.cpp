#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/TwistStamped.h>

#include "odoflow/ECCDenseTracker.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "paraset/ParameterManager.hpp"

using namespace argus;

class DenseVONode
{
EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
DenseVONode(ros::NodeHandle& nh, ros::NodeHandle& ph)
	: _imageTrans(nh), _tracker(nh, ph)
{
	_imageSub = _imageTrans.subscribe("image",
	                                  2,
	                                  boost::bind(&DenseVONode::ImageCallback, this, _1));
	_twistPub = ph.advertise<geometry_msgs::TwistStamped>("velocity_raw", 10);

	_pyramidDepth.InitializeAndRead(ph, 0, "pyramid_depth", "Number of image pyramids");
	_pyramidDepth.AddCheck<IntegerValued>();
	_pyramidDepth.AddCheck<GreaterThanOrEqual>(0);

	_prevImgVel = PoseSE3::TangentVector::Zero();
	GetParamRequired(ph, "scale", _scale);
	GetParam(ph, "min_dt", _minDt, 1E-3);

	ResetTracking();
}

void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImageConstPtr frame;
	try
	{
		frame = cv_bridge::toCvShare(msg, "mono8");
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("VisualOdometryNode cv_bridge exception: %s", e.what());
		return;
	}

	ProcessImage(frame->image, msg->header);
}

void CreatePyramid(const cv::Mat& img, std::vector<cv::Mat>& pyr,
                   unsigned int depth)
{
	pyr.clear();
	pyr.resize(depth + 1);
	pyr[0] = img;
	for(unsigned int i = 0; i < depth; ++i)
	{
		cv::pyrDown(pyr[i], pyr[i + 1]);
	}
}

void ResetTracking()
{
	_prevImgVel.setZero();
	_prevPyramid.clear();
}

void ProcessImage(const cv::Mat& img, const std_msgs::Header& header)
{
	std::vector<cv::Mat> pyramid;
	unsigned int depth = _pyramidDepth;

	CreatePyramid(img, pyramid, depth);

	// Check for initialization or change in image size
	if(_prevPyramid.empty() || img.size() != _prevPyramid[0].size())
	{
		_prevPyramid = pyramid;
		_prevTime = header.stamp;
		return;
	}

	// In case the depth increased since last time
	if(_prevPyramid.size() <= (depth + 1))
	{
		_prevPyramid.resize(depth + 1);
		for(unsigned int i = _prevPyramid.size() - 1; i < depth; ++i)
		{
			cv::pyrDown(_prevPyramid[i], _prevPyramid[i + 1]);
		}
	}

	// Check for too small dt
	double dt = (header.stamp - _prevTime).toSec();
	if(dt < _minDt)
	{
		// ROS_WARN_STREAM( "Got dt " << dt << " less than min " << _minDt );
		return;
	}

	// Have to prescale the last velocity by timestep and lowest level size
	PoseSE3::TangentVector prevLogDisp = _prevImgVel * dt;
	prevLogDisp.head<3>() = prevLogDisp.head<3>() / std::pow(2, _pyramidDepth);
	PoseSE3 disp, imgDisp;
	for(int i = _pyramidDepth; i >= 0; --i)
	{
		disp = PoseSE3::Exp(prevLogDisp);

		const cv::Mat& currImg = pyramid[i];
		const cv::Mat& prevImg = _prevPyramid[i];

		// TODO Clean up the interface - why is disp the prediction for imgdisp?
		// ROS_INFO_STREAM( "Depth: " << i << " predicted disp: " << disp );
		if(!_tracker.TrackImages(prevImg, currImg, disp, imgDisp))
		{
			ROS_INFO_STREAM("Tracking failed!");
			ResetTracking();
			return;
		}
		// ROS_INFO_STREAM( "Got disp: " << imgDisp );

		// Next level will be twice as much resolution, so we double the translation prediction
		prevLogDisp = PoseSE3::Log(imgDisp);
		prevLogDisp.head<3>() *= 2;
	}

	// Last iteration doubles prevLogDisp so we halve it again and normalize by time
	_prevImgVel = prevLogDisp / (2 * dt);
	_prevPyramid = pyramid;
	_prevTime = header.stamp;

	geometry_msgs::TwistStamped tmsg;
	tmsg.header = header;

	// TODO Normalize by x and y resolution separately and have one scale for each
	double imgWidth = (double) img.size().width;
	PoseSE3::TangentVector dvel = _scale * PoseSE3::Log(disp) / (dt * imgWidth);
	tmsg.twist = TangentToMsg(dvel);
	_twistPub.publish(tmsg);
}

private:
image_transport::ImageTransport _imageTrans;
image_transport::Subscriber _imageSub;
ros::Publisher _twistPub;

std::vector<cv::Mat> _prevPyramid;
ros::Time _prevTime;
PoseSE3::TangentVector _prevImgVel;     // At the lowest level pyramid

double _scale;
double _minDt;
NumericParam _pyramidDepth;

ECCDenseTracker _tracker;
};

int main(int argc, char**argv)
{
	ros::init(argc, argv, "image_viewer");

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle("~");
	DenseVONode ecc(nodeHandle, privHandle);

	ros::spin();

	return 0;
}
