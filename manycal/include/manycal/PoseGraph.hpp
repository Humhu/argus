#ifndef _MANYCAL_POSE_GRAPH_H_
#define _MANYCAL_POSE_GRAPH_H_

#include "isam/SlamInterface.h"

#include "argus_utils/ArgusTypes.h"
#include "argus_utils/TimeSeries.hpp"

#include <unordered_map>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace manycal 
{
	
	/*! \brief Presents a convenient pose graph wrapper around an iSAM Slam object. */
	template< class PoseNode, class UnaryFactor, class BinaryFactor >
	class PoseGraph
	{
	public:
		
		typedef argus_utils::Time Time;
		typedef std::shared_ptr<PoseGraph> Ptr;
		typedef PoseNode PoseNodeType; // Represents all poses
		typedef UnaryFactor PriorType; // Used for priors
		typedef BinaryFactor EdgeType; // Used for odometry
		typedef argus_utils::TimeSeries< typename PoseNode::Ptr > DynamicSeries;
		
		PoseGraph( isam::SlamInterface& _slam )
			: slam( _slam )
		{}
		
		/*! \brief Add a new static object. */
		typename PoseNode::Ptr AddStatic( const std::string& name, 
										  const typename PoseNode::DataType& data )
		{
			typename PoseNode::Ptr node = std::make_shared<PoseNode>();
			node->init( data );
			staticMap[ name ] = node;
			slam.add_node( node );
			return node;
		}
		
		/*! \brief Retrieve a static object. */
		typename PoseNode::Ptr GetStatic( const std::string& name ) const
		{
			return staticMap.at( name );
		}
		
		/*! \brief Add a prior to a static object. */
		typename UnaryFactor::Ptr AddStaticPrior( const std::string& name, 
												  const typename UnaryFactor::DataType& data,
												  const isam::Noise& noise )
		{
			typename PoseNode::Ptr pose = GetStatic( name );
			if( !pose )
			{
				throw std::out_of_range( "Cannot add prior to non-existant static object." );
			}
			typename UnaryFactor::Ptr unary = std::make_shared<UnaryFactor>( pose.get(), data, noise );
			slam.add_factor( unary );
			return unary;
		}
		
		/*! \brief Add a new dynamic object pose. */
		typename PoseNode::Ptr AddDynamic( const std::string& name, Time time, 
										   const typename PoseNode::DataType& data )
		{
			if( dynamicMap.count( name ) == 0 )
			{
				dynamicMap[ name ] = std::make_shared<DynamicSeries>();
			}
			
			typename PoseNode::Ptr pose = std::make_shared<PoseNode>();
			pose->init( data );
			dynamicMap[ name ]->AddTimepoint( time, pose );
			slam.add_node( pose );
			return pose;
		}
		
		/*! \brief Retrieve a dynamic pose at a specified time if it exists. */
		typename PoseNode::Ptr GetDynamic( const std::string& name, Time time ) const
		{
			typename PoseNode::Ptr pos;
			try
			{
				pos = dynamicMap.at( name )->GetExact( time ).point;
			}
			catch( std::out_of_range e ) {}
			
			return pos;
		}
		
		/*! \brief Return the entire time series for a dynamic object. */
		typename PoseGraph::DynamicSeries::Ptr GetDynamicSeries( const std::string& name ) const
		{
			typename DynamicSeries::Ptr series;
			try
			{
				series = dynamicMap.at( name );
			}
			catch( std::out_of_range e ) {}
			
			return series;
		}
		
		/*! \brief Adds a prior to an existing dynamic pose. */
		typename UnaryFactor::Ptr AddDynamicPrior( const std::string& name, Time time, 
												   const typename UnaryFactor::DataType& data,
												   const isam::Noise& noise )
		{
			typename PoseNode::Ptr pose = GetDynamic( name, time );
			if( !pose )
			{
				throw std::out_of_range( "Cannot add prior to non-existant dynamic object." );
			}
			typename UnaryFactor::Ptr unary = std::make_shared<UnaryFactor>( pose.get(), data, noise );
			slam.add_factor( unary );
			return unary;
		}
		
		/*! \brief Creates a new dynamic pose at the specified time and links it to its time
		 * predecessor with a binary factor. */
		typename PoseNode::Ptr AddDynamicOdometry( const std::string& name, Time time, 
												   const typename BinaryFactor::DataType& data,
												   const isam::Noise& noise )
		{
			typename PoseNode::Ptr older = dynamicMap.at( name )->GetClosestLower( time ).point;
			typename PoseNode::DataType init = older->value().oplus( data );
			typename PoseNode::Ptr newer = AddDynamic( name, time, init );
			
			typename BinaryFactor::Ptr binary = std::make_shared<BinaryFactor>
				( older.get(), newer.get(), data, noise );
			slam.add_factor( binary );
			return newer;
		}
		
	protected:
		
		std::unordered_map< std::string, typename DynamicSeries::Ptr > dynamicMap;
		std::unordered_map< std::string, typename PoseNode::Ptr > staticMap;
		
		isam::SlamInterface& slam;
		
	};
	
	typedef PoseGraph<isam::Pose3d_Node, isam::Pose3d_Factor, isam::Pose3d_Pose3d_Factor> PoseGraph3d;
	typedef PoseGraph<isam::Pose2d_Node, isam::Pose2d_Factor, isam::Pose2d_Pose2d_Factor> PoseGraph2d;
	
}

#endif
