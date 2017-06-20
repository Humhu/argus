#include <ros/ros.h>

#include "odoscan/ScanFilter.h"
#include "odoscan/VoxelGridFilter.h"
#include "odoscan/ApproximateVoxelGridFilter.h"

#include "odoscan/ScanMatcher.h"
#include "odoscan/DegeneracyDetector.h"
#include "odoscan/ICPMatcher.h"
#include "odoscan/MatchRestarter.h"

#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>

#include <laser_geometry/laser_geometry.h>

#include "lookup/LookupInterface.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

#include "argus_utils/geometry/VelocityIntegrator.hpp"
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
		: _nh( nh ), _ph( ph ), _extrinsics( nh, ph )
	{
		ros::NodeHandle mh( ph.resolveName( "matcher" ) );
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

		GetParam( ph, "enable_prediction", _enablePrediction, false );
		if( _enablePrediction )
		{
			double odomLen;
			GetParam( ph, "max_odom_buff_len", odomLen, 5.0 );
			_velIntegrator.SetMaxBuffLen( odomLen );
			_odomSub = nh.subscribe( "odom", 10, &LaserOdometryNode::OdomCallback, this );
		}

		ros::NodeHandle ch( ph.resolveName( "checker" ) );
		_checker.Initialize( ch );

		ros::NodeHandle rh( ph.resolveName( "restarter" ) );
		_restarter.Initialize( rh, _matcher );

		ros::NodeHandle fh( ph.resolveName( "filter" ) );
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
		GetParam( ph, "max_keyframe_age", _maxKeyframeAge, std::numeric_limits<double>::infinity() );

		_maxError.InitializeAndRead( ph, 0.25, "max_error",
		                             "Maximum alignment mean sum of squared errors." );
		_maxError.AddCheck<GreaterThan>( 0.0 );

		_minInlierRatio.InitializeAndRead( ph, 0.5, "min_inlier_ratio",
		                                   "Minimum solution inlier to full scan ratio." );
		_minInlierRatio.AddCheck<GreaterThanOrEqual>( 0.0 );
		_minInlierRatio.AddCheck<LessThanOrEqual>( 1.0 );

		_minNumInliers.InitializeAndRead( ph, 50, "min_num_inliers",
						  "Minimum number of inliers required" );
	}

private:

	ros::NodeHandle _nh;
	ros::NodeHandle _ph;

	ExtrinsicsInterface _extrinsics;

	bool _enablePrediction;
	ros::Subscriber _odomSub;
	std::string _odomFrame;
	VelocityIntegratorSE3 _velIntegrator;

	ScanFilter::Ptr _filter;
	ScanMatcher::Ptr _matcher;
	DegeneracyDetector _checker;
	MatchRestarter _restarter;

	LookupInterface _lookupInterface;
	double _maxDt;
	double _maxKeyframeAge; // TODO Make parameters as well

	NumericParam _maxError;
	NumericParam _minInlierRatio;
  NumericParam _minNumInliers;

	struct CloudRegistration
	{
		Mutex mutex;

		laser_geometry::LaserProjection projector;
		ros::Subscriber cloudSub;
		ros::Publisher velPub;

		bool overrideTimestamp;
		bool showOutput;
		ros::Publisher debugAlignedPub;
		ros::Publisher debugKeyPub;

		LaserCloudType::Ptr keyframeCloud;
		ros::Time keyframeTime;
		PoseSE3 lastPose;
		ros::Time lastPoseTime;
	};

	typedef std::unordered_map<std::string, CloudRegistration> CloudRegistry;
	CloudRegistry _cloudRegistry;

	void RegisterCloudSource( const std::string& name,
	                          const YAML::Node& info )
	{
		ROS_INFO_STREAM( "LaserOdometryNode: Registering cloud source: " << name );

		CloudRegistration& reg = _cloudRegistry[name];

		GetParam( info, "override_timestamp", reg.overrideTimestamp, false );
		GetParam( info, "show_output", reg.showOutput, false );
		if( reg.showOutput )
		{
			std::string debugAlignedTopic = name + "/aligned_cloud";
			ROS_INFO_STREAM( "Publishing debug aligned cloud on: " <<
			                 _ph.resolveName( debugAlignedTopic ) );
			reg.debugAlignedPub = _ph.advertise<LaserCloudType>( debugAlignedTopic, 0 );

			std::string debugKeyTopic = name +  "/key_cloud";
			ROS_INFO_STREAM( "Publishing debug key cloud on: " <<
			                 _ph.resolveName( debugKeyTopic ) );
			reg.debugKeyPub = _ph.advertise<LaserCloudType>( debugKeyTopic, 0 );
		}

		std::string outputTopic;
		GetParamRequired( info, "output_topic", outputTopic );
		reg.velPub = _nh.advertise<geometry_msgs::TwistStamped>( outputTopic, 0 );

		unsigned int buffSize;
		std::string inputTopic;
		GetParam<unsigned int>( info, "buffer_size", buffSize, 0 );

		if( GetParam( info, "cloud_topic", inputTopic ) )
		{
			ROS_INFO_STREAM( "Subscribing to cloud at " << inputTopic );
			reg.cloudSub = _nh.subscribe<LaserCloudType>( inputTopic,
			                                              buffSize,
			                                              boost::bind( &LaserOdometryNode::CloudCallback,
			                                                           this,
			                                                           boost::ref( reg ),
			                                                           _1 ) );
		}
		else if( GetParam( info, "scan_topic", inputTopic ) )
		{
			ROS_INFO_STREAM( "Subscribing to scan at " << inputTopic );
			reg.cloudSub = _nh.subscribe<sensor_msgs::LaserScan>( inputTopic,
			                                                      buffSize,
			                                                      boost::bind( &LaserOdometryNode::ScanCallback,
			                                                                   this,
			                                                                   boost::ref( reg ),
			                                                                   _1 ) );
		}
		else
		{
			throw std::runtime_error( "No input topic specified for " + name );
		}
	}

	void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
	{
		PoseSE3::TangentVector vel = MsgToTangent( msg->twist.twist );
		PoseSE3::CovarianceMatrix cov;
		ParseMatrix( msg->twist.covariance, cov );
		_velIntegrator.BufferInfo( msg->header.stamp.toSec(), vel, cov );
		_odomFrame = msg->child_frame_id;
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

	void ResetKeyframe( CloudRegistration& reg,
	                    const LaserCloudType::Ptr& cloud,
	                    const ros::Time& time )
	{
		reg.keyframeCloud = cloud;
		reg.keyframeTime = time;
		reg.lastPose = PoseSE3();
		reg.lastPoseTime = time;
	}

	void PredictMotion( CloudRegistration& reg,
	                    const std::string laserFrame,
	                    const ros::Time& currTime,
	                    PoseSE3& laserDisp,
	                    PoseSE3::CovarianceMatrix& laserDispCov )
	{
		// Initialization of default values
		laserDisp = PoseSE3();
		laserDispCov = PoseSE3::CovarianceMatrix::Identity(); // TODO

		// If not predicting or haven't received odometry yet, return default values
		if( !_enablePrediction || _odomFrame.empty() ) { return; }

		// First try to integrate odom frame motion
		// If fails, return defaults
		PoseSE3 odomDisp;
		PoseSE3::CovarianceMatrix odomCov;
		if( !_velIntegrator.Integrate( reg.lastPoseTime.toSec(), currTime.toSec(),
		                               odomDisp, odomCov ) )
		{
			ROS_WARN_STREAM( "Could not integrate motion from " << reg.lastPoseTime <<
			                 " to " << currTime );
			return;
		}

		// Then try to get extrinsics
		// If fails, return defaults
		PoseSE3 odomToLaser;
		try
		{
			odomToLaser = _extrinsics.GetExtrinsics( _odomFrame,
			                                         laserFrame,
			                                         currTime );

			laserDisp = odomToLaser * odomDisp * odomToLaser.Inverse();
			laserDispCov = TransformCovariance( odomCov, odomToLaser );
		}
		catch( ExtrinsicsException& e )
		{
			ROS_WARN_STREAM( "Could not get extrinsics: " << e.what() );
			return;
		}
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
		if( reg.overrideTimestamp )
		{
			currTime = ros::Time::now();
		}
		else
		{
			pcl_conversions::fromPCL( cloud->header.stamp, currTime );
		}

		// Synchronize registration access
		WriteLock lock( reg.mutex );

		double keyframeAge = (currTime - reg.keyframeTime).toSec();
		double dt = (currTime - reg.lastPoseTime).toSec();
		if( dt < 0 )
		{
			ROS_WARN_STREAM( "Negative dt detected. Resetting..." );
			_velIntegrator.Reset();
			ResetKeyframe( reg, currCloud, currTime );
			return;
		}
		if( !reg.keyframeCloud || dt > _maxDt || keyframeAge > _maxKeyframeAge )
		{
			ResetKeyframe( reg, currCloud, currTime );
			return;
		}

		LaserCloudType::Ptr aligned = boost::make_shared<LaserCloudType>();

		// Attempt to use velocity prediction
		PoseSE3 laserDisp;
		PoseSE3::CovarianceMatrix laserDispCov;
		PredictMotion( reg, cloud->header.frame_id, currTime, laserDisp, laserDispCov );
		PoseSE3 guessDisp = reg.lastPose * laserDisp;
		_restarter.SetDisplacementDistribution( guessDisp, laserDispCov );
		ScanMatchResult result = _restarter.Match( reg.keyframeCloud, currCloud, aligned );

		if( reg.showOutput )
		{
			if( result.success && aligned )
			{
			  //result.inliers->header = aligned->header;
			        reg.debugAlignedPub.publish( aligned );
			}
			if( reg.keyframeCloud )
			{
				reg.debugKeyPub.publish( reg.keyframeCloud );
			}
		}

		if( !result.success )
		{
			ROS_WARN_STREAM( "Scan matching failed! Resetting keyframe..." );
			ResetKeyframe( reg, currCloud, currTime );
			return;
		}


		if( _checker.HasDegeneracy( result.inliers ) )
		{
			ResetKeyframe( reg, currCloud, currTime );
			return;
		}

		double inlierRatio = result.inliers->size() / (float) reg.keyframeCloud->size();
		if( inlierRatio < _minInlierRatio || result.inliers->size() < _minNumInliers )
		{
			ROS_WARN_STREAM( "Found " << result.inliers->size() <<
			                 " inliers out of " << reg.keyframeCloud->size() <<
			                 " input points, less than min ratio " << _minInlierRatio <<
					 " or number " << _minNumInliers );
			ResetKeyframe( reg, currCloud, currTime );
			return;
		}

		if( result.fitness > _maxError )
		{
			ROS_WARN_STREAM( "Scan match result has error " << result.fitness << " greater than threshold " << _maxError );
			ResetKeyframe( reg, currCloud, currTime );
			return;
		}

		// TODO Logic for 2D flip case

		PoseSE3 displacement = reg.lastPose.Inverse() * result.transform;
		PoseSE3::TangentVector laserVelocity = PoseSE3::Log( displacement ) / dt;

		geometry_msgs::TwistStamped out;
		out.header.stamp = currTime;
		out.header.frame_id = cloud->header.frame_id;
		out.twist = TangentToMsg( laserVelocity );
		reg.velPub.publish( out );

		// Update
		reg.lastPose = result.transform;
		reg.lastPoseTime = currTime;
	}
};

int main( int argc, char ** argv )
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
