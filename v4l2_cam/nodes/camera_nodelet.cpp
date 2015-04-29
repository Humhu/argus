#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "v4l2_cam/DriverNode.h"
#include <boost/thread.hpp>

namespace v4l2_cam
{

	class CameraNodelet 
		: public nodelet::Nodelet
	{
		
	public:
		
		CameraNodelet() {}
		~CameraNodelet() {}
		
		void Spin()
		{
			while( ros::ok() )
			{
				driver->Process();
			}
		}
		
	private:
		
		DriverNode::Ptr driver;
		boost::thread worker;
		
		virtual void onInit()
		{
			driver = boost::make_shared<DriverNode>(
				getMTNodeHandle(), getMTPrivateNodeHandle() );
			worker = boost::thread( boost::bind( &CameraNodelet::Spin, this ) );
		}
		
	};
	
}

PLUGINLIB_DECLARE_CLASS(v4l2_cam, camera_nodelet,
						v4l2_cam::CameraNodelet, nodelet::Nodelet);
