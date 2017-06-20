#include "odoscan/DegeneracyDetector.h"

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

namespace argus
{
DegeneracyDetector::DegeneracyDetector()
{}

void DegeneracyDetector::Initialize( ros::NodeHandle& ph )
{
	_checkPlane.InitializeAndRead( ph, true, "check_plane",
	                               "Whether to check for planar degeneracies" );
	_checkSphere.InitializeAndRead( ph, false, "check_sphere",
	                                "Whether to check for spherical degeneracies" );
        _checkLine.InitializeAndRead( ph, true, "check_line",
                                      "Whether to check for linear degeneracies" );
        _checkCircle.InitializeAndRead( ph, false, "check_circle",
                                        "Whether to check for circular degeneracies" );

	_inlierThreshold.InitializeAndRead( ph, 0.02, "inlier_threshold",
	                                    "RANSAC inlier threshold" );
        _inlierThreshold.AddCheck<GreaterThanOrEqual>( 0 );

	_maxIterations.InitializeAndRead( ph, 1, "max_iterations",
	                                  "RANSAC max iterations" );
	_maxIterations.AddCheck<IntegerValued>();
        _maxIterations.AddCheck<GreaterThan>( 0 );

	_maxDegeneracyRatio.InitializeAndRead( ph, 0.95, "max_degeneracy_ratio",
	                                       "Maximum allowable proportion of degenerate points" );
	_maxDegeneracyRatio.AddCheck<GreaterThanOrEqual>( 0 );
	_maxDegeneracyRatio.AddCheck<LessThanOrEqual>( 1.0 );
}

bool DegeneracyDetector::HasDegeneracy( const LaserCloudType::ConstPtr& cloud )
{
	typedef pcl::SampleConsensusModelPlane<LaserPointType> SACPlane;
	typedef pcl::SampleConsensusModelSphere<LaserPointType> SACSphere;
        typedef pcl::SampleConsensusModelLine<LaserPointType> SACLine;
        typedef pcl::SampleConsensusModelCircle2D<LaserPointType> SACCircle;

	double maxInliers = _maxDegeneracyRatio * cloud->size();

	if( _checkPlane )
	{
		SACPlane::Ptr plane = boost::make_shared<SACPlane>( cloud );
		RANSAC planeRansac( plane );
		unsigned int planeInliers = CountInliers( planeRansac );
		if( planeInliers > maxInliers )
		{
			ROS_WARN_STREAM( "Planar degeneracy with " << planeInliers <<
			" inliers detected, greater than max allowed " << maxInliers );
			return true;
		}
	}
	if( _checkSphere )
	{
		SACSphere::Ptr sphere = boost::make_shared<SACSphere>( cloud );
		RANSAC sphereRansac( sphere );
		unsigned int sphereInliers = CountInliers( sphereRansac );
		if( sphereInliers > maxInliers )
		{
			ROS_WARN_STREAM( "Spherical degeneracy with " << sphereInliers <<
			" inliers detected, greater than max allowed " << maxInliers );
			return true;
		}
	}
	if( _checkLine )
	  {
	    SACLine::Ptr line = boost::make_shared<SACLine>( cloud );
	    RANSAC lineRansac( line );
	    unsigned int lineInliers = CountInliers( lineRansac );
	    if( lineInliers > maxInliers )
	      {
		ROS_WARN_STREAM( "Linear degeneracy with " << lineInliers <<
				 " inliers detected, greater than max allowed " << maxInliers );
		return true;
	      }
	  }
	if( _checkCircle )
	  {
	    SACCircle::Ptr circle = boost::make_shared<SACCircle>( cloud );
	    RANSAC circleRansac( circle );
	    unsigned int circleInliers = CountInliers( circleRansac );
	    if( circleInliers > maxInliers )
	      {
		ROS_WARN_STREAM( "Circular degeneracy with " << circleInliers <<
				 " inliers detected, greater than max allowed " << maxInliers );
		return true;
	      }
	  }


	return false;
}

unsigned int DegeneracyDetector::CountInliers( RANSAC& ransac )
{
  ransac.setDistanceThreshold( 0.01 ); //_inlierThreshold );
	ransac.setMaxIterations( _maxIterations );
	ransac.computeModel();
	std::vector<int> inliers;
	ransac.getInliers( inliers );
	return inliers.size();
}

}
