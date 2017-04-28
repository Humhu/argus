#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/TwistStamped.h>

#include "odoflow/ECCDenseTracker.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

class DenseVONode
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  public:
    DenseVONode(ros::NodeHandle &nh, ros::NodeHandle &ph)
        : _imageTrans(nh), _tracker(nh, ph)
    {
        _imageSub = _imageTrans.subscribe("image",
                                          2,
                                          boost::bind(&DenseVONode::ImageCallback, this, _1));
        _twistPub = ph.advertise<geometry_msgs::TwistStamped>("velocity_raw", 10);

        _prevImgVel = PoseSE3::TangentVector::Zero();
        GetParamRequired( ph, "scale", _scale );
    }

    void ImageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImageConstPtr frame;
        try
        {
            frame = cv_bridge::toCvShare(msg, "mono8");
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("VisualOdometryNode cv_bridge exception: %s", e.what());
            return;
        }

        if (!_prevMat.empty())
        {
            ros::Time start = ros::Time::now();
            double dt = ( msg->header.stamp - _prevTime ).toSec();
            
            PoseSE3 pose = PoseSE3::Exp( _prevImgVel * dt );
            PoseSE3 imgPose;
            if( !_tracker.TrackImages(_prevMat, frame->image, pose, imgPose) )
            {
                ROS_INFO_STREAM( "Tracking failed!" );
                _prevImgVel = PoseSE3::TangentVector::Zero();
            }
            else
            {
                ros::Time stop = ros::Time::now();

                _prevMat = frame->image;

                PoseSE3::TangentVector vel = _scale * PoseSE3::Log( pose ) / dt;
                geometry_msgs::TwistStamped tmsg;
                tmsg.header = msg->header;
                tmsg.twist = TangentToMsg( vel );
                _twistPub.publish( tmsg );
                
                _prevImgVel = PoseSE3::Log( imgPose ) / dt;
            }
        }

        _prevMat = frame->image;
        _prevTime = msg->header.stamp;
    }

  private:
    image_transport::ImageTransport _imageTrans;
    image_transport::Subscriber _imageSub;
    ros::Publisher _twistPub;

    cv::Mat _prevMat;
    ros::Time _prevTime;
    PoseSE3::TangentVector _prevImgVel;
    double _scale;

    ECCDenseTracker _tracker;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_viewer");

    ros::NodeHandle nodeHandle;
    ros::NodeHandle privHandle("~");
    DenseVONode ecc(nodeHandle, privHandle);

    ros::spin();

    return 0;
}
