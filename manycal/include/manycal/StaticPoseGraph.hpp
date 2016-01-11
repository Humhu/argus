#pragma once

#include "manycal/PoseGraph.hpp"

namespace manycal
{

template <class P, typename IndexType = boost::posix_time::ptime>
class StaticPoseGraph
: public PoseGraph<P,IndexType>
{
public:

	typedef typename PoseGraph<P,IndexType>::PoseType PoseType;
	typedef typename PoseGraph<P,IndexType>::NodeType NodeType;
	typedef typename PoseGraph<P,IndexType>::PriorType PriorType;
	typedef typename PoseGraph<P,IndexType>::EdgeType EdgeType;
	typedef typename PoseGraph<P,IndexType>::NoiseType NoiseType;
	typedef std::shared_ptr<StaticPoseGraph> Ptr;

	StaticPoseGraph( isam::Slam::Ptr s ) 
	: slam( s ) {}

	virtual IndexType EarliestIndex() const
	{
		return IndexTraits<IndexType>::Earliest();
	}

	virtual IndexType LatestIndex() const
	{
		return IndexTraits<IndexType>::Latest();
	}

	virtual bool IsGrounded( const IndexType& ind ) const 
	{
		return node && priors.size() > 0;
	}

	virtual typename NodeType::Ptr CreateNode( const IndexType& ind,
	                                           const PoseType& pose )
	{
		if( node ) { RemoveNode( IndexType() ); }
		node = std::make_shared<NodeType>();
		slam->add_node( node.get() );
		node->init( pose );
		return node;
	}

	virtual typename NodeType::Ptr RetrieveNode( const IndexType& ind )
	{
		if( !node ) 
		{
			CreateNode( ind, PoseType() );
		}
		return node;
	}

	virtual void RemoveNode( const IndexType& ind )
	{
		if( !node ) { return; }
		slam->remove_node( node.get() );
		node.reset();
		priors.clear();
	}

	virtual void ClearNodes()
	{
		RemoveNode( IndexType() );
	}

	virtual void CreatePrior( const IndexType& ind, const PoseType& pose,
	                                   const isam::Noise& noise )
	{
		typename NodeType::Ptr n = RetrieveNode( ind );
		
		typename PriorType::Ptr prior = std::make_shared <PriorType>
		    ( node.get(), pose, noise );
		slam->add_factor( prior.get() );
		priors.push_back( prior );
	}

	virtual void CreateEdge( const IndexType& from, const IndexType& to,
	                         const PoseType& pose, const NoiseType& noise ) {}

private:

	isam::Slam::Ptr slam;
	typename NodeType::Ptr node;
	std::vector<typename PriorType::Ptr> priors;

};

}