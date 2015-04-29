#ifndef _ODOFLOW_BINDINGS_H_
#define _ODOFLOW_BINDINGS_H_

#include "boost/date_time/posix_time/posix_time.hpp"

namespace odoflow
{
	typedef boost::posix_time::ptime Timepoint;
	typedef boost::posix_time::time_duration TimeDuration;
	
	class Interface
	{
	public:
		
		/*! \brief Gets the system time. Implementation depends on whether 
		* ROS is being used or not. */
		static Timepoint GetTime();
		
	};
}

#endif
