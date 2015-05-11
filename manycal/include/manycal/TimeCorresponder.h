#ifndef _TIME_CORRESPONDER_H_
#define _TIME_CORRESPONDER_H_

#include "manycal/DataCorresponder.hpp"

#include "atags/TagDetection.h"

namespace manycal
{
	
	// TODO Think about how this should even work...
	/*! \brief Corresponder class that associates data in a sliding window. */
	template <class D, class C>
	class TimeCorresponder
		: public DataCorresponder<D,C>
	{
	public:
		
		TimeCorresponder( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
			: DataCorresponder( nh, ph )
		{}
		
		virtual void DataCallback( const DConstPtr& msg );
		
	private:
		
		ros::Time startTime;
		ros::Time endTime;
		ros::Duration windowLength;
	};
	
}

#endif
