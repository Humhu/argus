#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <boost/algorithm/string.hpp>

#include "v4l2_cam/DriverNode.h"

namespace v4l2_cam
{

	class CameraNodelet 
		: public nodelet::Nodelet
	{
		
	public:
		
		CameraNodelet() {}
		~CameraNodelet() {}
		
	private:
		
		DriverNode::Ptr driver;
		
		virtual void onInit()
		{
			driver = std::make_shared<DriverNode>(
				getNodeHandle(), getPrivateNodeHandle() );
		}
		
	};
	
}

PLUGINLIB_DECLARE_CLASS( v4l2_cam, camera_nodelet,
						 v4l2_cam::CameraNodelet, nodelet::Nodelet );
