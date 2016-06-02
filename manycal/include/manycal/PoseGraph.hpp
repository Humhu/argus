#pragma once

#include <ros/time.h>
#include <memory>
#include "isam/Slam.h"

namespace argus
{

/*! \brief Defines basic properties and operators for time indices. */
template <typename IndexType>
struct IndexTraits
{
	static double Difference( const IndexType& a, const IndexType& b ) 
	{ 
		return (double)( a - b ); 
	}
	
	/*! \brief Returns the earliest representable time. */
	static IndexType Earliest() 
	{ 
		return std::numeric_limits<IndexType>::min(); 
	}
	
	/*!\brief Returns the latest representable time. */
	static IndexType Latest() 
	{ 
		return std::numeric_limits<IndexType>::max(); 
	}
};

template <>
struct IndexTraits <boost::posix_time::ptime>
{
	typedef boost::posix_time::ptime Time;
	
	static double Difference( const Time& a, const Time& b )
	{
		// TODO Set precision
		return (a - b).total_microseconds() * 1E-6;
	}

	static Time Earliest() 
	{
		return Time( boost::posix_time::min_date_time );
	}

	static Time Latest()
	{
		return Time( boost::posix_time::max_date_time );
	}
};

template <>
struct IndexTraits <ros::Time>
{

	static double Difference( const ros::Time& a, const ros::Time& b )
	{
		return (a - b).toSec();
	}

	static ros::Time Earliest()
	{
		return ros::Time::fromBoost( IndexTraits<boost::posix_time::ptime>::Earliest() );
	}

	static ros::Time Latest()
	{
		return ros::Time::fromBoost( IndexTraits<boost::posix_time::ptime>::Latest() );
	}
};

/*! \brief Interface for pose graphs grouping all pose variables for a dynamic
 * system indexable by time. */
template <class P, typename IndexType = boost::posix_time::ptime>
class PoseGraph
{
public:

	typedef typename isam::Slam_Traits<P>::PoseType PoseType;
	typedef typename isam::Slam_Traits<P>::NodeType NodeType;
	typedef typename isam::Slam_Traits<P>::PriorType PriorType;
	typedef typename isam::Slam_Traits<P>::EdgeType EdgeType;
	typedef isam::Noise NoiseType;
	typedef std::shared_ptr<PoseGraph> Ptr;

	PoseGraph() {}

	virtual ~PoseGraph() {}

	/*! \brief Return the earliest index in the pose graph. */
	virtual IndexType EarliestIndex() const = 0;

	/*! \brief Return the latest index in the pose graph. */
	virtual IndexType LatestIndex() const = 0;

	/*! \brief Return whether the (hypothetical) node at the time index is grounded.
	 * If not and the node will be optimized, a prior should be added. */
	virtual bool IsGrounded( const IndexType& ind ) const = 0;

	/*! \brief Creates a node at the specified index. */
	virtual typename NodeType::Ptr CreateNode( const IndexType& ind, 
	                                           const PoseType& pose ) = 0;

	/*! \brief Retrieve the node corresponding to the specified index, creating
	 * if necessary. Returns null if no correspondence and index invalid. */
	virtual typename NodeType::Ptr RetrieveNode( const IndexType& ind ) = 0;

	/*! \brief Remove the node at the specified index if it exists. */
	virtual void RemoveNode( const IndexType& ind ) = 0;

	/*! \brief Remove all nodes. */
	virtual void ClearNodes() = 0;

	/*! \brief Create a prior for the specified index. */
	virtual void CreatePrior( const IndexType& ind, const PoseType& pose,
	                          const NoiseType& noise ) = 0;

	virtual void CreateEdge( const IndexType& from, const IndexType& to,
	                         const PoseType& pose, const NoiseType& noise ) = 0;

};

}