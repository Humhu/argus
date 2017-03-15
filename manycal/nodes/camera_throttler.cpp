#include <ros/ros.h>
#include <deque>
#include <boost/foreach.hpp>

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "manycal/MessageThrottler.hpp"
#include "manycal/SetThrottleWeight.h"

#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

class CameraThrottler
{
public:

    CameraThrottler( ros::NodeHandle& nh, ros::NodeHandle& ph )
    : publicPort( nh )
    {
        // TODO Parameters for _throttle?
        unsigned int buffLen;
        GetParamRequired( ph, "buffer_length", buffLen );
        double r;
        GetParamRequired( ph, "max_rate", r );

        _throttle.SetBufferLength( buffLen );
        _throttle.SetTargetRate( r );

        GetParamRequired( ph, "update_rate", r );
        _updateTimer = nh.createTimer( ros::Duration( 1.0/r ), &CameraThrottler::TimerCallback, this );

        YAML::Node sources;
        GetParamRequired( ph, "sources", sources );
        YAML::Node::const_iterator iter;
        for( iter = sources.begin(); iter != sources.end(); ++iter )
        {
            const std::string& name = iter->first.as<std::string>();
            const YAML::Node& info = iter->second;

            std::string input_topic, output_topic;
            GetParamRequired( info, "input", input_topic );
            GetParamRequired( info, "output", output_topic );

            ROS_INFO_STREAM( "Registering source " << name << " with input " << input_topic <<
                             " and output " << output_topic );

            _throttle.RegisterSource( name );
            _throttle.SetSourceWeight( name, 1.0 );
            _subscribers.emplace_back( 
                publicPort.subscribeCamera(input_topic, 10, 
                    boost::bind(&CameraThrottler::CameraCallback, this, name, _1, _2)) );
            
            _publishers[name] = publicPort.advertiseCamera(output_topic, 10);
        }

        _throttleServer = ph.advertiseService( "set_throttle", &CameraThrottler::SetThrottleCallback,
                                               this );
    }

private:

    ros::Timer _updateTimer;
    ros::ServiceServer _throttleServer;
    image_transport::ImageTransport publicPort;
    std::deque<image_transport::CameraSubscriber> _subscribers;
    std::map<std::string, image_transport::CameraPublisher> _publishers;
    
    typedef std::pair<sensor_msgs::Image::ConstPtr, sensor_msgs::CameraInfo::ConstPtr> CameraData;
    typedef MessageThrottler<CameraData> DataThrottler;
    DataThrottler _throttle;

    bool SetThrottleCallback( manycal::SetThrottleWeight::Request& req,
                              manycal::SetThrottleWeight::Response& res )
    {
        try
        {
            _throttle.SetSourceWeight( req.name, req.weight );
            return true;
        }
        catch ( const std::invalid_argument& e )
        {
            return false;
        }
    }

    void TimerCallback( const ros::TimerEvent& event )
    {
        DataThrottler::KeyedData out;
        if( _throttle.GetOutput( event.current_real.toSec(), out ) )
        {
            _publishers[out.first].publish( out.second.first, out.second.second );
        }
    }

    void CameraCallback( const std::string& name,
                         const sensor_msgs::Image::ConstPtr& image,
                         const sensor_msgs::CameraInfo::ConstPtr& info )
    {
        _throttle.BufferData( name, CameraData( image, info ) );
    }

};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "camera_throttler" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
    unsigned int numThreads;
    GetParam<unsigned int>( ph, "num_threads", numThreads, 1 );

	CameraThrottler throttler( nh, ph );
	
	// ros::spin();
    ros::AsyncSpinner spinner( numThreads );
    spinner.start();
	ros::waitForShutdown();
    
	return 0;
}
