#include <ros/ros.h>

#include "odoscan/ScanFilter.h"
#include "odoscan/VoxelGridFilter.h"
#include "odoscan/ApproximateVoxelGridFilter.h"

#include "odoscan/ScanMatcher.h"
#include "odoscan/ICPMatcher.h"

#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/LaserScan.h"

#include <laser_geometry/laser_geometry.h>

#include "lookup/LookupInterface.h"
#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_map>
#include <unordered_set>

using namespace argus;

class LaserOdometryNode
{
public:
	LaserOdometryNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: _nodeHandle( nh ), _privHandle( ph )

	{
		ros::NodeHandle mh( _privHandle.resolveName( "matcher" ) );
		std::string type;
		GetParamRequired( mh, "type", type );
		if( type == "icp" )
		{
			_matcher = std::make_shared<ICPMatcher>();
		}
		else
		{
			throw std::invalid_argument( "Unknown matcher type: " + type );
		}
		_matcher->Initialize( mh );

		ros::NodeHandle fh( _privHandle.resolveName( "filter" ) );
		GetParamRequired( fh, "type", type );
		if( type == "voxel" )
		{
			_filter = std::make_shared<VoxelGridFilter>();
		}
		else if( type == "approximate_voxel" )
		{
			_filter = std::make_shared<ApproximateVoxelGridFilter>();
		}
		else if( type == "none" )
		{
			// Do nothing
		}
		else
		{
			throw std::invalid_argument( "Unknown filter type: " + type );
		}
		if( _filter )
		{
			_filter->Initialize( fh );
		}

		YAML::Node sources;
		GetParamRequired( ph, "sources", sources );
		YAML::const_iterator iter;
		for( iter = sources.begin(); iter != sources.end(); ++iter )
		{
			const std::string& name = iter->first.as<std::string>();
			const YAML::Node& info = iter->second;
			RegisterCloudSource( name, info );
		}

		GetParamRequired( ph, "max_dt", _maxDt );
	}

private:
	ros::NodeHandle _nodeHandle;
	ros::NodeHandle _privHandle;

	ScanFilter::Ptr _filter;
	ScanMatcher::Ptr _matcher;

	LookupInterface _lookupInterface;
	double _maxDt;

	struct CloudRegistration
	{
		Mutex mutex;

		laser_geometry::LaserProjection projector;
		ros::Subscriber cloudSub;
		ros::Publisher velPub;

		bool showOutput;
		ros::Publisher debugAlignedPub;
		ros::Publisher debugKeyPub;

		LaserCloudType::Ptr lastCloud;
		ros::Time lastCloudTime;
	};
	typedef std::unordered_map<std::string, CloudRegistration> CloudRegistry;
	CloudRegistry _cloudRegistry;

	void RegisterCloudSource( const std::string& name, const YAML::Node& info )
	{
		ROS_INFO_STREAM( "LaserOdometryNode: Registering cloud source: " << name );

		CloudRegistration& reg = _cloudRegistry[name];

		GetParam( info, "show_output", reg.showOutput, false );
		if( reg.showOutput )
		{
			std::string debugAlignedTopic = name + "/aligned_cloud";
			ROS_INFO_STREAM( "Publishing debug aligned cloud on: " << debugAlignedTopic );
			reg.debugAlignedPub = _privHandle.advertise<LaserCloudType>( debugAlignedTopic, 0 );

			std::string debugKeyTopic = name + "/key_cloud";
			ROS_INFO_STREAM( "Publishing debug key cloud on: " << debugKeyTopic );
			reg.debugKeyPub = _privHandle.advertise<LaserCloudType>( debugKeyTopic, 0 );
		}

		std::string outputTopic;
		GetParamRequired( info, "output_topic", outputTopic );
		reg.velPub = _nodeHandle.advertise<geometry_msgs::TwistStamped>( outputTopic, 0 );

		unsigned int buffSize;
		std::string inputTopic;
		GetParam<unsigned int>( info, "buffer_size", buffSize, 0 );

		if( GetParam( info, "cloud_topic", inputTopic ) )
		{
			reg.cloudSub = _nodeHandle.subscribe<LaserCloudType>
			                   ( inputTopic,
			                   buffSize,
			                   boost::bind( &LaserOdometryNode::CloudCallback,
			                                this,
			                                boost::ref( reg ),
			                                _1 ) );
		}
		else if( GetParam( info, "scan_topic", inputTopic ) )
		{
			reg.cloudSub = _nodeHandle.subscribe<sensor_msgs::LaserScan>
			                   ( inputTopic,
			                   buffSize,
			                   boost::bind( &LaserOdometryNode::ScanCallback,
			                                this,
			                                boost::ref( reg ),
			                                _1 ) );
		}
	}

	void ScanCallback( CloudRegistration& reg,
	                   const sensor_msgs::LaserScan::ConstPtr& msg )
	{
		sensor_msgs::PointCloud2 cloudMsg;
		reg.projector.projectLaser( *msg, cloudMsg );
		pcl::PCLPointCloud2 pclMsg;
		pcl_conversions::toPCL( cloudMsg, pclMsg );
		LaserCloudType::Ptr cloud = boost::make_shared<LaserCloudType>();
		pcl::fromPCLPointCloud2( pclMsg, *cloud );
		ProcessCloud( reg, cloud );
	}

	void CloudCallback( CloudRegistration& reg,
	                    const LaserCloudType::ConstPtr& msg )
	{
		ProcessCloud( reg, msg );
	}

	void ProcessCloud( CloudRegistration& reg,
	                   const LaserCloudType::ConstPtr& cloud )
	{
		// Parse message fields
		LaserCloudType::Ptr currCloud;
		if( _filter )
		{
			currCloud = boost::make_shared<LaserCloudType>();
			_filter->Filter( cloud, *currCloud );
		}
		else
		{
			currCloud = boost::make_shared<LaserCloudType>( *cloud );
		}

		ros::Time currTime;
		pcl_conversions::fromPCL( cloud->header.stamp, currTime );

		// Synchronize registration access
		WriteLock lock( reg.mutex );

		if( !reg.lastCloud )
		{
			reg.lastCloud = currCloud;
			reg.lastCloudTime = currTime;
			return;
		}

		double dt = ( currTime - reg.lastCloudTime ).toSec();
		if( dt > _maxDt )
		{
			reg.lastCloud = currCloud;
			reg.lastCloudTime = currTime;
			return;
		}

		LaserCloudType::Ptr aligned = boost::make_shared<LaserCloudType>();

		PoseSE3 laserDisplacement;
		bool success = _matcher->Match( reg.lastCloud, currCloud, laserDisplacement, aligned );

		if( reg.showOutput )
		{
			reg.debugAlignedPub.publish( aligned );
			reg.debugKeyPub.publish( reg.lastCloud );
		}

		if( !success )
		{
			ROS_WARN_STREAM( "Scan matching failed!" );
			reg.lastCloud = currCloud;
			reg.lastCloudTime = currTime;
			return;
		}

		PoseSE3::TangentVector laserVelocity = PoseSE3::Log( laserDisplacement ) / dt;
		// ROS_INFO_STREAM( "Frame velocity: " << frameVelocity.transpose() << std::endl <<
		//                  "Laser displacement: " << laserDisplacement << std::endl <<
		//                  "dt: " << dt );

		geometry_msgs::TwistStamped out;
		out.header.stamp = currTime;
		out.header.frame_id = cloud->header.frame_id;
		out.twist = TangentToMsg( laserVelocity );
		reg.velPub.publish( out );

		// Update
		reg.lastCloud = currCloud;
		reg.lastCloudTime = currTime;
	}
};

int main( int argc, char**argv )
{
	ros::init( argc, argv, "laser_odometry_node" );

	ros::NodeHandle nh, ph( "~" );
	LaserOdometryNode lon( nh, ph );

	unsigned int numThreads;
	GetParamRequired( ph, "num_threads", numThreads );
	ros::AsyncSpinner spinner( numThreads );
	spinner.start();
	ros::waitForShutdown();

	return 0;
}
