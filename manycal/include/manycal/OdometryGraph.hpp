#pragma once

#include "argus_utils/utils/MapUtils.hpp"
#include "manycal/PoseGraph.hpp"
#include "manycal/slam_traits.h"

#include <boost/foreach.hpp>
#include <iostream> // TODO

namespace argus
{

/*! \brief Provides an interface for managing a linked set of pose nodes. Allows
 * retrieval and insertion of nodes. */
template <class P, typename IndexType = boost::posix_time::ptime>
class OdometryGraph
: public PoseGraph <P, IndexType>
{
public:

	typedef typename PoseGraph<P,IndexType>::PoseType PoseType;
	typedef typename PoseGraph<P,IndexType>::NodeType NodeType;
	typedef typename PoseGraph<P,IndexType>::PriorType PriorType;
	typedef typename PoseGraph<P,IndexType>::EdgeType EdgeType;
	typedef typename PoseGraph<P,IndexType>::NoiseType NoiseType;
	typedef std::shared_ptr<OdometryGraph> Ptr;
	
	OdometryGraph( isam::Slam::Ptr s ) 
	: slam( s ) {}
	
	virtual IndexType EarliestIndex() const
	{
		return argus::get_lowest_key( timeSeries );
	}

	virtual IndexType LatestIndex() const
	{
		return argus::get_highest_key( timeSeries );
	}

	virtual bool IsGrounded( const IndexType& ind ) const
	{
		// Case where there is 1 or 0 nodes
		if( timeSeries.size() < 2 ) 
		{
			if( timeSeries.count( ind ) == 0 ) { return false; }
			return timeSeries.at( ind ).priors.size() > 0;
		}

		return ind >= EarliestIndex() && ind <= LatestIndex();
	}

	virtual typename NodeType::Ptr CreateNode( const IndexType& ind, 
	                                           const PoseType& pose )
	{
		Datum datum;
		datum.node = std::make_shared <NodeType>();
		datum.node->init( pose );
		datum.toPrev = nullptr;
		
		timeSeries[ ind ] = datum;
		slam->add_node( datum.node.get() );
		return datum.node;
	}

	/*! \brief Retrieve a node at the specified index if it exists. Creates the node
	 * if it is between the first and last index otherwise. Returns the node. */
	typename NodeType::Ptr RetrieveNode( const IndexType& ind )
	{
		if( timeSeries.count( ind ) > 0 ) { return timeSeries[ind].node; }
		return SplitOdometry( ind );
	}

	/*! \brief Removes the pose at the index and all connected factors from the graph.
	 * Note that this includes factors added from outside the pose graph. Fills the 
	 * resulting hole with a composed odometry factor. */
	virtual void RemoveNode( const IndexType& ind )
	{
		if( timeSeries.count( ind ) == 0 ) { return; }

		const Datum& datum = timeSeries[ ind ];
		slam->remove_node( datum.node.get() );

		typename TimeSeries::iterator prevIter;
		bool hasLower = argus::get_closest_lesser( timeSeries, ind, prevIter );
		Datum& prev = prevIter->second;

		typename TimeSeries::iterator nextIter;
		bool hasUpper = argus::get_closest_greater( timeSeries, ind, nextIter );
		Datum& next = nextIter->second;

		// Fill in hole in graph by joining odometry
		if( hasUpper && hasLower )
		{
			PoseType sumOdom;
			typename isam::Slam_Traits<P>::CovarianceType sumCov;
			isam::Slam_Traits<P>::ComposeDisplacement
			    ( datum.toPrev->measurement(), next.toPrev->measurement(),
			      datum.toPrev->noise().cov(), next.toPrev->noise().cov(),
			      sumOdom, sumCov );
			next.toPrev = std::make_shared<EdgeType>
			    ( prev.node.get(), next.node.get(), sumOdom, isam::Covariance( sumCov ) );
			slam->add_factor( next.toPrev.get() );
		}
		// Else just clear odometry factor pointing to node we removed
		else if( hasUpper )
		{
			next.toPrev = nullptr;
		}

		// Don't erase until we're done using datum
		timeSeries.erase( ind );
	}
	
	virtual void ClearNodes()
	{
		BOOST_FOREACH( typename TimeSeries::value_type& iter, timeSeries )
		{
			slam->remove_node( iter.second.node.get() );
		}
		timeSeries.clear();
	}

	/*! \brief Creates a node and adds an edge to the previous node. Index must
	 * come after the latest index. */
	virtual void CreateEdge( const IndexType& from, const IndexType& to,
	                         const PoseType& displacement, const NoiseType& noise )
	{
		IndexType first = std::min( from, to );
		IndexType last = std::max( from, to );

		typename NodeType::Ptr firstNode = RetrieveNode( first );
		typename NodeType::Ptr lastNode = RetrieveNode( last );
		if( !firstNode || !lastNode ) { return; }

		Datum& lastDatum = timeSeries.at( last );
		if( lastDatum.toPrev )
		{
			slam->remove_factor( lastDatum.toPrev.get() );
			lastDatum.toPrev.reset();
		}

		lastDatum.toPrev = std::make_shared <EdgeType>
		    ( firstNode.get(), lastNode.get(), displacement, noise );
		
		slam->add_factor( lastDatum.toPrev.get() );
	}
	
	void CreatePrior( const IndexType& ind, const PoseType& pose,
	                  const isam::Noise& noise )
	{
		if( !RetrieveNode( ind ) ) { return; }
		Datum& datum = timeSeries[ ind ];
		typename PriorType::Ptr prior = std::make_shared <PriorType>
		    ( datum.node.get(), pose, noise );
		datum.priors.push_back( prior );
		slam->add_factor( prior.get() );
	}

	/*! \brief Removes nodes that have only odometry factors. */
	void CompressGraph()
	{
		std::vector<IndexType> toRemove;
		BOOST_FOREACH( typename TimeSeries::value_type& iter, timeSeries )
		{
			const IndexType& ind = iter.first;
			const Datum& data = iter.second;

			// Don't remove if there are any priors
			if( data.priors.count() > 0 ) { continue; }

			// Else collect all the local factors and check against the node's list
			std::set<isam::Factor*> localFactors;

			if( data.toPrev ) { localFactors.insert( data.toPrev.get() ); }

			typename TimeSeries::iterator nextIter;
			if( argus::get_closest_greater( timeSeries, ind, nextIter ) )
			{
				localFactors.insert( nextIter->toPrev.get() );
			}

			const std::list<isam::Factor*>& factors = data.node->factors();
			bool remove = true;
			BOOST_FOREACH( isam::Factor* factor, factors )
			{
				// If the node has a non-local factor, we can't remove it
				if( localFactors.count( factor ) == 0 )
				{
					remove = false;
					break;
				}
			}

			if( remove ) { toRemove.push_back( ind ); }
		}

		BOOST_FOREACH( const IndexType& ind, toRemove )
		{
			RemoveNode( ind );
		}
	}

private:
	
	// Inserts a node at the specified index in between two existing nodes.
	typename NodeType::Ptr SplitOdometry( const IndexType& ind )
	{
		typename TimeSeries::iterator prevIter, nextIter;
		// Make sure index has previous and following items
		if( !argus::get_closest_lesser( timeSeries, ind, prevIter ) || 
		    !argus::get_closest_greater( timeSeries, ind, nextIter ) )
		{
			return nullptr;
		}
		
		IndexType prevTime = prevIter->first;
		IndexType nextTime = nextIter->first;
		
		double prevDt = IndexTraits<IndexType>::Difference(ind, prevTime);
		double nextDt = IndexTraits<IndexType>::Difference(nextTime, ind);
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
		std::vector<typename PriorType::Ptr> priors;
	};
	
	typedef std::map <IndexType, Datum> TimeSeries;
	TimeSeries timeSeries;	
	isam::Slam::Ptr slam;
	
};

}
