#pragma once

#include "manycal/PoseGraph.hpp"

namespace manycal
{

template <class P, typename IndexType = boost::posix_time::ptime>
class JumpPoseGraph
: public PoseGraph<P,IndexType>
{
public:

	typedef typename PoseGraph<P,IndexType>::PoseType PoseType;
	typedef typename PoseGraph<P,IndexType>::NodeType NodeType;
	typedef typename PoseGraph<P,IndexType>::PriorType PriorType;
	typedef typename PoseGraph<P,IndexType>::EdgeType EdgeType;
	typedef typename PoseGraph<P,IndexType>::NoiseType NoiseType;
	typedef std::shared_ptr<JumpPoseGraph> Ptr;

	JumpPoseGraph( isam::Slam::Ptr s ) 
	: slam( s ) {}

	virtual IndexType EarliestIndex() const
	{
		return argus_utils::get_lowest_key( data );
	}

	virtual IndexType LatestIndex() const
	{
		return argus_utils::get_highest_key( data );
	}

	virtual bool IsGrounded( const IndexType& ind ) const
	{
		if( data.count( ind ) == 0 ) { return false; }
		return data.at( ind ).priors.size() > 0;
	}

	virtual typename NodeType::Ptr CreateNode( const IndexType& ind,
	                                           const PoseType& pose )
	{
		Datum datum;
		datum.node = std::make_shared <NodeType>();
		datum.node->init( pose );
		
		data[ ind ] = datum;
		slam->add_node( datum.node.get() );
		return datum.node;
	}

	virtual typename NodeType::Ptr RetrieveNode( const IndexType& ind )
	{
		return data[ ind ].node;
	}

	virtual void RemoveNode( const IndexType& ind )
	{
		if( data.count( ind ) > 0 ) 
		{ 
			slam->remove_node( data[ ind ].node.get() );
			data.erase( ind ); 
		}
	}

	virtual void ClearNodes()
	{
		BOOST_FOREACH( const typename DataMap::value_type& iter, data ) 
		{
			RemoveNode( iter.first );
		}
		data.clear();
	}

	virtual void CreatePrior( const IndexType& ind, const PoseType& pose,
	                          const isam::Noise& noise )
	{
		if( data.count( ind ) == 0 ) { return; }

		Datum& d = data[ ind ];
		typename PriorType::Ptr prior = std::make_shared <PriorType>
		    ( d.node.get(), pose, noise );
		slam->add_factor( prior.get() );
		d.priors.push_back( prior );
	}

	virtual void CreateEdge( const IndexType& from, const IndexType& to,
	                         const PoseType& pose, const NoiseType& noise ) {}

private:

	struct Datum
	{
		typename NodeType::Ptr node;
		std::vector<typename PriorType::Ptr> priors;
	};

	isam::Slam::Ptr slam;
	typedef std::map<IndexType,Datum> DataMap;
	DataMap data;

};

}