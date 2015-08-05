#ifndef _MANYCAL_POSE_GRAPH_PUBLISHER_H_
#define _MANYCAL_POSE_GRAPH_PUBLISHER_H_

#include "manycal/PoseGraph.hpp"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <type_traits>

namespace manycal
{
	/*! \brief A class that allows easy ros::tf publishing of pose graph contents. */
	template< class PG >
	class PoseGraphPublisher
	{
	public:
		
		typedef argus_common::Time Time;
		typedef std::shared_ptr<PoseGraphPublisher> Ptr;
		typedef typename PG::DynamicSeries::Timepoint Timepoint;
		
		PoseGraphPublisher( const PG& _pg ) 
			: pg( _pg ) {}
		
		void PublishStatic( const std::string& name, const std::string& baseName )
		{
			Time now = ros::Time::now().toBoost();
			Timepoint timepoint;
			timepoint.time = now;
			timepoint.point = pg.GetStatic( name );
			PublishPose( name, baseName,  );
		}
		
		void PublishDynamic( const std::string& name, Time time, const std::string& baseName )
		{
			
		}
		
	private:
		
		tf::TransformBroadcaster transformPub;
		const PG& pg;
		
		void PublishPose( const std::string& name, const std::string& baseName,
						  & timepoint )
		{
			tf::Transform trans = ConvertTransform( timepoint.point->value() );
			tf::StampedTransform ttrans( trans, ros::Time( timepoint.time ), baseName, name );
			transformPub.sendTransform( ttrans );
		}
		
		tf::Transform ConvertTransform( typename PG::PoseNode::DataType& data )
		{
			static_assert( false, "Must specialize ConvertTransform for PoseGraphPublisher." );
		}
		
	};
	
	template<>
	tf::Transform PoseGraphPublisher<PoseGraph3d>::ConvertTransform( isam::Pose3d& data )
	{
		Eigen::Quaterniond q = data.rot().quaternion();
		tf::Quaternion quat( q.x(), q.w(), q.z(), q.w() );
		tf::Vector3 pos( data.x(), data.y(), data.z() );
			
		return tf::Transform( quat, pos );
	}
	
	template<>
	tf::Transform PoseGraphPublisher<PoseGraph2d>::ConvertTransform( isam::Pose2d& data )
	{
		tf::Quaternion quat( data.t(), 0, 0 );
		tf::Vector3 pos( data.x(), data.y(), 0 );
			
		return tf::Transform( quat, pos );
	}
	
}

#endif
