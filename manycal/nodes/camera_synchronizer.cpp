#include <ros/ros.h>
#include <deque>
#include <boost/foreach.hpp>

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "manycal/MessageSynchronizer.hpp"

#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

class CameraSynchronizer
{
public:

    CameraSynchronizer( ros::NodeHandle& nh, ros::NodeHandle& ph )
    : publicPort( nh )
    {
        // TODO Parameters for _sync?
        unsigned int buffLen;
        GetParamRequired( ph, "buffer_length", buffLen );
        double dt;
        GetParamRequired( ph, "max_dt", dt );

        _sync.SetBufferLength( buffLen );
        _sync.SetMaxDt( dt );

        YAML::Node sources;
        GetParamRequired( ph, "sources", sources );
        YAML::Node::const_iterator iter;
        for( iter = sources.begin(); iter != sources.end(); ++iter )
        {
            const std::string& name = iter->first.as<std::string>();
            const YAML::Node& info = iter->second;

            std::string input_topic, output_topic;
            GetParamRequired( info, "input", input_topic );
            _sync.RegisterSource( name );
            _subscribers.emplace_back( 
                publicPort.subscribeCamera(input_topic, 10, 
                    boost::bind(&CameraSynchronizer::CameraCallback, this, name, _1, _2)) );
            
            GetParamRequired( info, "output", output_topic );
            _publishers[name] = publicPort.advertiseCamera(output_topic, 10);
        }
    }

private:

    image_transport::ImageTransport publicPort;
    std::deque<image_transport::CameraSubscriber> _subscribers;
    std::map<std::string, image_transport::CameraPublisher> _publishers;
    
    typedef std::pair<sensor_msgs::Image::ConstPtr, sensor_msgs::CameraInfo::ConstPtr> CameraData;
    typedef MessageSynchronizer<CameraData> DataSynchronizer;
    DataSynchronizer _sync;

    void CameraCallback( const std::string& name,
                         const sensor_msgs::Image::ConstPtr& image,
	                     const sensor_msgs::CameraInfo::ConstPtr& info )
    {
        CameraData data( image, info );
        _sync.BufferData( name, image->header.stamp.toSec(), data );
        while( _sync.HasOutput() )
        {
            DataSynchronizer::DataMap out = _sync.GetOutput();
            typedef DataSynchronizer::DataMap::value_type Item;
            BOOST_FOREACH(const Item& item, out )
            {
                const std::string& name = item.first;
                const CameraData& data = item.second.second;
                _publishers[name].publish( data.first, data.second );
            }
        }
    }

};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "camera_synchronizer" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	CameraSynchronizer calibrator( nh, ph );
	
	ros::spin();
	
	return 0;
}
