#include <manycal/slam3dvel.h>
#include <manycal/sclam_fiducial.h>

#include <isam/SlamInterface.h>

#include <iostream>

/* Small test with extrinsics and fiducial detections. */
int main( int argc, char** argv )
{
	
	isam::Slam::Ptr slam = std::make_shared <isam::Slam> ();
	isam::SlamInterface interface( slam );
	
	// Camera extrinsics node
	isam::PoseSE3 camExtrinsicsInit( argus_utils::PoseSE3( 0, 0, 1, 1, 0, 0, 0 ) );
	isam::PoseSE3_Node::Ptr camExtrinsicsNode = std::make_shared <isam::PoseSE3_Node>();
	camExtrinsicsNode->init( camExtrinsicsInit );
	
	interface.add_node( camExtrinsicsNode );
	
	isam::Noise camExtrinsicsPriorInfo = isam::Covariance( isam::eye(6) );
	isam::PoseSE3_Prior::Ptr camExtrinsicsPrior = std::make_shared <isam::PoseSE3_Prior>
	    ( camExtrinsicsNode.get(), camExtrinsicsInit, camExtrinsicsPriorInfo );
		
	interface.add_factor( camExtrinsicsPrior );
	
	// Camera intrinsics node
	isam::MonocularIntrinsics camIntrinsicsInit( 600, 600, 320, 240 );
	isam::MonocularIntrinsics_Node::Ptr camIntrinsicsNode = std::make_shared <isam::MonocularIntrinsics_Node>();
	camIntrinsicsNode->init( camIntrinsicsInit );
	
	interface.add_node( camIntrinsicsNode );
	
	isam::Noise camIntrinsicsPriorInfo = isam::Covariance( isam::eye(4) );
	isam::MonocularIntrinsics_Prior::Ptr camIntrinsicsPrior = std::make_shared <isam::MonocularIntrinsics_Prior>
	    ( camIntrinsicsNode.get(), camIntrinsicsInit, camIntrinsicsPriorInfo );
		
	interface.add_factor( camIntrinsicsPrior );
	
	// Fiducial extrinsics node
	isam::PoseSE3 fidExtrinsicsInit( argus_utils::PoseSE3( 0, 0, 1, 1, 0, 0, 0 ) );
	isam::PoseSE3_Node::Ptr fidExtrinsicsNode = std::make_shared <isam::PoseSE3_Node>();
	fidExtrinsicsNode->init( fidExtrinsicsInit );
	
	interface.add_node( fidExtrinsicsNode );
	
	isam::Noise fidExtrinsicsPriorInfo = isam::Covariance( isam::eye(6) );
	isam::PoseSE3_Prior::Ptr fidExtrinsicsPrior = std::make_shared<isam::PoseSE3_Prior>
	    ( fidExtrinsicsNode.get(), fidExtrinsicsInit, fidExtrinsicsPriorInfo );
		
	interface.add_factor( fidExtrinsicsPrior );
	
	// Fiducial intrinsics node
	std::vector <isam::Point3d> fidPoints;
	fidPoints.emplace_back( 0.0, 0.1, 0.1 );
	fidPoints.emplace_back( 0.0, 0.1, -0.1 );
	fidPoints.emplace_back( 0.0, -0.1, 0.1 );
	fidPoints.emplace_back( 0.0, -0.1, -0.1 );
	std::cout << "fidPoints.size(): " << fidPoints.size() << std::endl;
	isam::FiducialIntrinsics fidPointsInit( fidPoints );
	// TODO Clean up manual dimensionality setting
	isam::FiducialIntrinsics_Node::Ptr fidIntrinsicsNode = std::make_shared<isam::FiducialIntrinsics_Node>( 12 );
	fidIntrinsicsNode->init( fidPointsInit );
	
	interface.add_node( fidIntrinsicsNode );
	
	isam::Noise fidIntrinsicsPriorInfo = isam::Covariance( isam::eye(12) );
	isam::FiducialIntrinsics_Prior::Ptr fidIntrinsicsPrior = std::make_shared<isam::FiducialIntrinsics_Prior>
	    ( fidIntrinsicsNode.get(), fidPointsInit, fidIntrinsicsPriorInfo );
	
	// Camera reference poses
	isam::PoseSE3 cameraRefPose1( argus_utils::PoseSE3( 0, 0, 0, 1, 0, 0, 0 ) );
	isam::PoseSE3_Node::Ptr cameraRefNode1 = std::make_shared<isam::PoseSE3_Node>();
	
	cameraRefNode1->init( cameraRefPose1 );
	
	isam::PoseSE3 cameraRefPose2( argus_utils::PoseSE3( 1, 0, 0, 1, 0, 0, 0 ) );
	isam::PoseSE3_Node::Ptr cameraRefNode2 = std::make_shared<isam::PoseSE3_Node>();
	cameraRefNode2->init( cameraRefPose1 );
	
	interface.add_node( cameraRefNode2 );
	
	// Velocity initialized by poses
	isam::VelocitySE3_Node::Ptr cameraRefVel = std::make_shared <isam::VelocitySE3_Node>();
	
	interface.add_node( cameraRefVel );
	
	isam::VelocitySE3 velPriorValue; // Zeros
	isam::Noise velPriorInfo = isam::Covariance( 1000*isam::eye(6) );
	isam::VelocitySE3_Prior::Ptr cameraRefVelPrior = std::make_shared <isam::VelocitySE3_Prior>
	    ( cameraRefVel.get(), velPriorValue, velPriorInfo );
		
	interface.add_factor( cameraRefVelPrior );
	
	// Fiducial reference poses
	isam::PoseSE3 fidRefPose1( argus_utils::PoseSE3( 1, 0, 0, 1, 0, 0, 0 ) );
	isam::PoseSE3_Node::Ptr fidRefNode1 = std::make_shared<isam::PoseSE3_Node>();
	
	fidRefNode1->init( fidRefPose1 );
	
	isam::PoseSE3 fidRefPose2( argus_utils::PoseSE3( 2, 0, 0, 1, 0, 0, 0 ) );
	isam::PoseSE3_Node::Ptr fidRefNode2 = std::make_shared<isam::PoseSE3_Node>();
	fidRefNode2->init( fidRefPose1 );
	
	interface.add_node( fidRefNode2 );
	
	// Velocity initialized by poses
	isam::VelocitySE3_Node::Ptr fidRefVel = std::make_shared <isam::VelocitySE3_Node>();
	
	interface.add_node( fidRefVel );
	
	isam::VelocitySE3_Prior::Ptr fidRefVelPrior = std::make_shared <isam::VelocitySE3_Prior>
	    ( fidRefVel.get(), velPriorValue, velPriorInfo );
		
	interface.add_factor( cameraRefVelPrior );
	
	// Velocity factors
	isam::Noise velFactorNoise = isam::Covariance( isam::eye(6) );
	isam::PoseSE3Vel_Factor::Ptr camVelFactor = std::make_shared<isam::PoseSE3Vel_Factor>
	    ( cameraRefNode1.get(), cameraRefNode2.get(), cameraRefVel.get(), 1.0, velFactorNoise );
	
	interface.add_factor( camVelFactor );
	
	isam::PoseSE3Vel_Factor::Ptr fidVelFactor = std::make_shared<isam::PoseSE3Vel_Factor>
	    ( fidRefNode1.get(), fidRefNode2.get(), fidRefVel.get(), 1.0, velFactorNoise );
		
	interface.add_factor( fidVelFactor );
	
	// Add a fiducial detection
	isam::PoseSE3 camRel1( camExtrinsicsInit.pose.Inverse() * cameraRefPose1.pose.Inverse() *
	                       fidRefPose1.pose * fidExtrinsicsInit.pose );
	std::cout << "camRel1: " << camRel1.pose << std::endl;
	isam::PoseSE3 camRel2( camExtrinsicsInit.pose.Inverse() * cameraRefPose2.pose.Inverse() *
	                       fidRefPose1.pose * fidExtrinsicsInit.pose );
	
	isam::FiducialDetection predictedDetection1 = isam::Predict( fidPointsInit, 
	                                                             camIntrinsicsInit,
	                                                             camRel1 );
	isam::FiducialDetection predictedDetection2 = isam::Predict( fidPointsInit, 
	                                                             camIntrinsicsInit,
	                                                             camRel2 );
	
	std::cout << "Detection1: " << predictedDetection1 << std::endl;
	std::cout << "Detection2: " << predictedDetection2 << std::endl;
	
	isam::FiducialFactor::Properties props;
	props.optCamReference = true;
	props.optCamIntrinsics = false;
	props.optCamExtrinsics = true;
	props.optFidReference = true;
	props.optFidIntrinsics = false;
	props.optFidExtrinsics = true;
	
	// Print results
	interface.get_slam()->write( std::cout );
	interface.get_slam()->batch_optimization();
	interface.get_slam()->write( std::cout );
	
	return 0;
}
