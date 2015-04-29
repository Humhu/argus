#include "odoflow/InterfaceBindings.h"
	
#if USE_ROS
	
#include <ros/ros.h>
namespace odoflow
{
	Timepoint Interface::GetTime()
	{
		Timepoint time = ros::Time::now().toBoost();
		return time;
	}
}

#else

namespace odoflow
{
	Timepoint Interface::GetTime()
	{
		Timepoint now( boost::posix_time::microsec_clock::local_time() );
		return now;
	}
}

#endif
