#include <ros/ros.h>

#include "pcl_ros/point_cloud.h"

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

class ScanFeaturesNode
{
public:

	ScanFeaturesNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		_cloudSub = nh.subscribe( "cloud", )
	}

private:

	ros::Subscriber _cloudSub;

	void CloudCallback( const CloudType::ConstPtr& cloud )
	{
		
	}

};

int main( int argc, char** argv )
{

	ros::init( argc, argv, "scan_features_node" );


	return 0;
}