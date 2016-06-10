#include <manycal/slamse3.h>
#include <manycal/sclam_fiducial.h>

#include <isam/SlamInterface.h>
#include <manycal/OdometryGraph.hpp>

#include <iostream>

#undef NDEBUG
#include <cassert>

using namespace argus;

typedef OdometryGraph <isam::PoseSE3> OGraph;

void TimeSeriesTest()
{
	std::cout << "Beginning map time series test..." << std::endl;
	
	std::map<double,int> testMap;
	std::map<double,int>::iterator iter;
	
	assert( !get_closest_lesser( testMap, 0.0, iter ) );
	assert( !get_closest_greater( testMap, 0.0, iter ) );
	assert( !get_closest_lesser_eq( testMap, 0.0, iter ) );
	assert( !get_closest_greater_eq( testMap, 0.0, iter ) );
	
	testMap[0] = 10;
	
	assert( !get_closest_lesser( testMap, 0.0, iter ) );
	assert( get_closest_lesser( testMap, 0.1, iter ) );
	
	assert( !get_closest_greater( testMap, 0.0, iter ) );
	assert( get_closest_greater( testMap, -0.1, iter ) );

	assert( get_closest_lesser_eq( testMap, 0.0, iter ) );
	assert( get_closest_lesser_eq( testMap, 0.1, iter ) );
	assert( iter->first == 0 );
	assert( get_closest_greater_eq( testMap, 0.0, iter ) );
	assert( get_closest_greater_eq( testMap, -0.1, iter ) );
	assert( iter->first == 0 );
	
	testMap[1] = 11;

	assert( !get_closest_lesser( testMap, 0.0, iter ) );
	assert( get_closest_lesser( testMap, 0.5, iter ) );
	assert( iter->first == 0 );
	assert( get_closest_lesser_eq( testMap, 0.0, iter ) );
	assert( get_closest_lesser_eq( testMap, 1.0, iter ) );
	assert( get_closest_lesser_eq( testMap, 0.5, iter ) );
	assert( iter->first == 0 );

	assert( !get_closest_greater( testMap, 1.0, iter ) );
	assert( get_closest_greater( testMap, 0.5, iter ) );
	assert( iter->first == 1 );
	assert( get_closest_greater_eq( testMap, 0.0, iter ) );
	assert( get_closest_greater_eq( testMap, 1.0, iter ) );
	assert( get_closest_greater_eq( testMap, 0.5, iter ) );
	assert( iter->first == 1 );

	assert( get_closest( testMap, 0.5, iter ) );
	assert( get_closest_eq( testMap, 0.5, iter ) );

	std::cout << "Map time series tests completed." << std::endl;
}

/* Small test with extrinsics and fiducial detections. */
int main( int argc, char** argv )
{

	ros::Time::init();
	
	TimeSeriesTest();
	
	isam::Slam::Ptr slam = std::make_shared <isam::Slam> ();
	
	OGraph ograph( slam );
	
	std::cout << "Initializing ograph..." << std::endl;
	ros::Time now = ros::Time::now();
	isam::PoseSE3_Node::Ptr initNode = ograph.CreateNode( now.toBoost(),
	                                                      isam::PoseSE3( 0, 0, 0, 1, 0, 0, 0 ) );
	ograph.CreatePrior( now.toBoost(),
	                    isam::PoseSE3( 0, 0, 0, 1, 0, 0, 0 ),
	                    isam::Covariance( isam::eye(6) ) );
	
	ros::Duration( 0.5 ).sleep();
	ros::Time midSleep = ros::Time::now();
	ros::Duration( 0.5 ).sleep();
	
	// Add a node at the specified time and displacement
	std::cout << "Adding odometry..." << std::endl;

	ros::Time prev = now;
	now = ros::Time::now();
	isam::PoseSE3_Node::Ptr lastNode = ograph.CreateNode( now.toBoost(), isam::PoseSE3() );
	ograph.CreateEdge( prev.toBoost(), now.toBoost(),
	                   isam::PoseSE3( 1, 0, 0, 1, 0, 0, 0 ), 
	                   isam::Covariance( isam::eye(6) ) );
	
	std::cout << "Splitting odometry..." << std::endl;
	isam::PoseSE3_Node::Ptr midNode = ograph.RetrieveNode( midSleep.toBoost() );
	
	// Print results
	std::cout << "Dumping results..." << std::endl;
	slam->write( std::cout );
	slam->batch_optimization();
	slam->write( std::cout );
	
	return 0;
}
