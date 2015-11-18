#pragma once

#include "argus_utils/MapUtils.hpp"
#include "isam/Slam.h"

#include <iostream> // TODO

namespace manycal
{

template <class T>
double index_difference( const T& a, const T& b ) { return 0; }

template <>
double index_difference <boost::posix_time::ptime>( const boost::posix_time::ptime& a,
                                                    const boost::posix_time::ptime& b )
{
	return (a - b).total_microseconds() * 1E-6;
}

/*! \brief Provides an interface for managing a linked set of pose nodes. Allows
 * retrieval and insertion of nodes. */
template <class P, typename IndexType = boost::posix_time::ptime>
class OdometryGraph
{
public:

	typedef typename isam::Slam_Traits<P>::PoseType PoseType;
	typedef typename isam::Slam_Traits<P>::NodeType NodeType;
	typedef typename isam::Slam_Traits<P>::PriorType PriorType;
	typedef typename isam::Slam_Traits<P>::EdgeType EdgeType;
	typedef isam::Noise NoiseType;
	typedef std::shared_ptr<OdometryGraph> Ptr;
	
	OdometryGraph( isam::Slam::Ptr s ) 
	: slam( s ) {}
	
	/*! \brief Retrieve a node at the specified index if it exists. Returns nullptr otherwise. */
	typename NodeType::Ptr GetNode( const IndexType& ind ) const
	{
		if( timeSeries.count( ind ) == 0 ) { return nullptr; }
		return timeSeries.at( ind ).node;
	}
	
	/*! \brief Retrieve a node at the specified index if it exists. Creates the node
	 * if it is between the first and last index otherwise. Returns the node. */
	typename NodeType::Ptr GetNodeInterpolate( const IndexType& ind )
	{
		typename NodeType::Ptr node = GetNode( ind );
		if( node ) { return node; }
		
		return SplitOdometry( ind );
	}
	
	typename NodeType::Ptr Initialize( const IndexType& ind, const PoseType& pose )
	{
		Datum datum;
		datum.node = std::make_shared <NodeType>();
		datum.node->init( pose );
		datum.toPrev = nullptr;
		
		timeSeries[ ind ] = datum;
		
		slam->add_node( datum.node.get() );
		
		return datum.node;
	}
	
	/*! \brief Creates a node and adds an edge to the previous node. */
	typename NodeType::Ptr AddOdometry( const IndexType& ind, const PoseType& displacement, 
	                                    const NoiseType& noise )
	{
		if( ind <= argus_utils::get_lowest_key( timeSeries ) ) { return nullptr; }
		
		typename TimeSeries::iterator iter;
		if( !argus_utils::get_closest_lower( timeSeries, ind, iter) ) { return nullptr; }
		
		Datum next;
		next.node = std::make_shared <NodeType>();
		next.toPrev = std::make_shared <EdgeType>
		    ( iter->second.node.get(), next.node.get(), displacement, noise );
		next.toPrev->initialize();
		
		timeSeries[ ind ] = next;
		
		slam->add_node( next.node.get() );
		slam->add_factor( next.toPrev.get() );
		
		return next.node;
	}
	
	typename NodeType::Ptr AddPrior( const IndexType& ind, const PoseType& pose,
	                                 const isam::Noise& noise )
	{
		if( !GetNodeInterpolate( ind ) ) { return nullptr; }
		Datum& datum = timeSeries[ ind ];
		datum.prior = std::make_shared <PriorType>( datum.node.get(), pose, noise );
		slam->add_factor( datum.prior.get() );
		return datum.node;
	}
	
	// Inserts a node at the specified index in between two existing nodes.
	typename NodeType::Ptr SplitOdometry( const IndexType& ind )
	{
		typename TimeSeries::iterator prevIter, nextIter;
		// Make sure index has previous and following items
		if( !argus_utils::get_closest_lower( timeSeries, ind, prevIter ) || 
		    !argus_utils::get_closest_upper( timeSeries, ind, nextIter ) )
		{
			return nullptr;
		}
		
		IndexType prevTime = prevIter->first;
		IndexType nextTime = nextIter->first;
		
		double prevDt = index_difference(ind, prevTime);
		double nextDt = index_difference(nextTime, ind);
		double prevProp = prevDt / ( prevDt + nextDt );
		double nextProp = 1.0 - prevProp;
		
		// Split odometry
		typename EdgeType::Ptr odometry = nextIter->second.toPrev;
		
		// Add newest timepoint
		Datum midDatum;
		midDatum.node = std::make_shared<NodeType>();
		midDatum.toPrev = std::make_shared <EdgeType>
		    ( prevIter->second.node.get(), midDatum.node.get(),
		      isam::Slam_Traits<P>::ScaleDisplacement( odometry->measurement(), prevProp ), 
		      isam::SqrtInformation( prevProp * odometry->sqrtinf() ) );
		
		timeSeries[ ind ] = midDatum;
		
		// Update last timepoint with split odometry
		nextIter->second.toPrev = std::make_shared <EdgeType>
		    ( midDatum.node.get(), nextIter->second.node.get(), 
		      isam::Slam_Traits<P>::ScaleDisplacement( odometry->measurement(), nextProp ),
		      isam::SqrtInformation( nextProp * odometry->sqrtinf() ) );

		slam->remove_factor( odometry.get() );
		slam->add_node( midDatum.node.get() );
		slam->add_factor( midDatum.toPrev.get() );
		slam->add_factor( nextIter->second.toPrev.get() );
		return midDatum.node;
	}
	
	struct Datum
	{
		typename NodeType::Ptr node;
		typename EdgeType::Ptr toPrev;
		typename PriorType::Ptr prior;
	};
	
	// TODO Public?
	typedef std::map <IndexType, Datum> TimeSeries;
	TimeSeries timeSeries;
	
private:
	
	isam::Slam::Ptr slam;
	
	
};

}
