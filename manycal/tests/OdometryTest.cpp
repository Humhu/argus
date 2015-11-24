#include <manycal/slamse3.h>
#include <manycal/sclam_fiducial.h>

#include <isam/SlamInterface.h>
#include <manycal/OdometryGraph.hpp>

#include <iostream>
#include <cassert>

typedef manycal::OdometryGraph <isam::PoseSE3> OGraph;

void TimeSeriesTest()
{
	std::cout << "Beginning map time series test..." << std::endl;
	
	std::map<int,int> testMap;
	std::map<int,int>::iterator iter;
	
	assert( !get_closest_lower( testMap, 0, iter ) );
	assert( !get_closest_upper( testMap, 0, iter ) );
	
	testMap[0] = 10;
	
	assert( !get_closest_lower( testMap, 0, iter ) );
	assert( !get_closest_upper( testMap, 0, iter ) );
	
	assert( get_closest_lower( testMap, 1, iter ) );
	assert( iter->first == 0 );
	assert( iter->second == 10 );
	assert( get_closest_upper( testMap, -1, iter ) );
	assert( iter->first == 0 );
	assert( iter->second == 10 );
	
	testMap[1] = 11;
	assert( !get_closest_lower( testMap, 0, iter ) );
	assert( !get_closest_upper( testMap, 1, iter ) );
	
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
	isam::PoseSE3_Node::Ptr initNode = ograph.Initialize( now.toBoost(),
	                                                      isam::PoseSE3( 0, 0, 0, 1, 0, 0, 0 ) );
	ograph.AddPrior( now.toBoost(),
	                 isam::PoseSE3( 0, 0, 0, 1, 0, 0, 0 ),
	                 isam::Covariance( isam::eye(6) ) );
	
	ros::Duration( 0.5 ).sleep();
	ros::Time midSleep = ros::Time::now();
	ros::Duration( 0.5 ).sleep();
	
	// Add a node at the specified time and displacement
	std::cout << "Adding odometry..." << std::endl;
	isam::PoseSE3_Node::Ptr lastNode = ograph.AddOdometry( ros::Time::now().toBoost(),
	                                                       isam::PoseSE3( 1, 0, 0, 1, 0, 0, 0 ), 
	                                                       isam::Covariance( isam::eye(6) ) );
	
	std::cout << "Splitting odometry..." << std::endl;
	isam::PoseSE3_Node::Ptr midNode = ograph.SplitOdometry( midSleep.toBoost() );
	
	// Print results
	std::cout << "Dumping results..." << std::endl;
	slam->write( std::cout );
	slam->batch_optimization();
	slam->write( std::cout );
	
	return 0;
}
