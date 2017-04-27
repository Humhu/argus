#include <ros/ros.h>

#include "argus_utils/utils/ParamUtils.h"
#include "odoscan/OdoscanCommon.h"
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

using namespace argus;

class ScanConverter
{
public:

	ScanConverter( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		_cloudPublisher = ph.advertise<LaserCloudType>( "cloud", 0 );

		unsigned int buffSize;
		GetParam<unsigned int>( ph, "buffer_size", buffSize, 0 );
		_scanSubscriber = nh.subscribe( "scan", buffSize, &ScanConverter::ScanCallback, this );

		GetParam( ph, "min_range", _minRange, std::numeric_limits<float>::min() );
		GetParam( ph, "max_range", _maxRange, std::numeric_limits<float>::max() );
	}

private:

	ros::Subscriber _scanSubscriber;
	ros::Publisher _cloudPublisher;
	float _minRange;
	float _maxRange;

	void ScanCallback( const sensor_msgs::LaserScan::ConstPtr& msg )
	{
		LaserCloudType::Ptr cloud = boost::make_shared<LaserCloudType>();
		const sensor_msgs::LaserScan& scan = *msg;
		cloud->header.frame_id = msg->header.frame_id;
		pcl_conversions::toPCL( msg->header.stamp, cloud->header.stamp );
		cloud->height = 1;
		cloud->width = scan.ranges.size();

		float minRange = std::max( scan.range_min, _minRange );
		float maxRange = std::min( scan.range_max, _maxRange );
		float ang = scan.angle_min;
		for( unsigned int i = 0; i < scan.ranges.size(); ++i )
		{
			float range = scan.ranges[i];
			if( range >= minRange && range <= maxRange )
			{
				float x = cos( ang ) * range;
				float y = sin( ang ) * range;
				cloud->push_back( LaserPointType( x, y, 0.0 ) );
			}
			ang += scan.angle_increment;
		}

		_cloudPublisher.publish( cloud );
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "scan_to_cloud_converter" );

	ros::NodeHandle nh, ph( "~" );
	ScanConverter sc( nh, ph );
	ros::spin();
	return 0;
}