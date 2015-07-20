#ifndef _MANYCAL_TAG_CALIBRATOR_H_
#define _MANYCAL_TAG_CALIBRATOR_H_

#include "manycal/PoseGraph.hpp"

namespace manycal 
{

	/*! \brief Presents a pose graph wrapper for managing graphical tag-based calibration. Optimizes
	 * extrinsics, and optionally intrinsics, of cameras and tags. */
	class TagGraph : public PoseGraph3d
	{
	public:
		
		TagGraph( isam::SlamInterface& _slam )
			: PoseGraph3d( _slam ) {}
		
		void AddCamera( const std::string& name, isam::Pose3d extrinsics, isam::MonocularIntrinsics intrinsics );
		void AddTag( const std::string& name, isam::Pose3d extrinsics, isam::TagIntrinsics intrinsics );
		
		isam::Pose3d_Node::Ptr GetTagExtrinsics( const std::string& name );
		isam::Tag_Node::Ptr GetTagIntrinsics( const std::string& name );
		
		isam::Pose3d_Node::Ptr GetCameraExtrinsics( const std::string& name );
		isam::MonocularIntrinsics_Node::Ptr GetCameraIntrinsics( const std::string& name );
		
		void AddCameraExtrinsicsPrior( const std::string& name, isam::Pose3d prior,
									   const isam::Noise& noise );
		void AddCameraIntrinsicsPrior( const std::string& name, isam::MonocularIntrinsics prior, 
									   const isam::Noise& noise );
		void AddTagExtrinsicsPrior( const std::string& name, isam::Pose3d prior,
									const isam::Noise& noise );
		void AddTagIntrinsicsPrior( const std::string& name, isam::TagIntrinsics prior,
									const isam::Noise& noise );
		
		/*! \brief Adds a factor corresponding to an observation between tag and camera. Chooses
		 * a factor type based on the calibration specifications. */
		void AddObservation( const std::string& tagName, const std::string& cameraName,
							 isam::TagCorners corners, isam::Covariance& cov );
		
	protected:
		
		struct TagRegistration
		{
			bool isStatic;
			bool isRelative;
			bool optimizeIntrinsics;
			
			isam::Tag_Node::Ptr intrinsics;
			isam::Extrinsics3d_Node::Ptr extrinsics;
		};
		
		struct CameraRegistration
		{
			isam::MonocularIntrinsics_Node::Ptr intrinsics;
			isam::Extrinsics3d_Node::Ptr extrinsics;
		};
		
		std::unordered_map< std::string, TagRegistration > tagRegistry;
		std::unordered_map< std::string, CameraRegistration> cameraRegistry;
		
	};
  
}

#endif