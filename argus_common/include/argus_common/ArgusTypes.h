#ifndef _ARGUS_TYPES_H_
#define _ARGUS_TYPES_H_

#include <boost/date_time/posix_time/posix_time.hpp>

namespace argus_common
{
	
	// Compatible with ros::Time, but keeps internal libraries ROS-free
	typedef boost::posix_time::ptime Time;
	typedef boost::posix_time::time_duration TimeDuration;
	
	double to_seconds( const TimeDuration& td )
	{
		return td.total_nanoseconds()*1E-9;
	}
}

#endif
