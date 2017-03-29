#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <boost/algorithm/string.hpp>

#include "camplex/DriverNode.h"

namespace camplex
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

PLUGINLIB_DECLARE_CLASS( camplex, camera_nodelet,
						 camplex::CameraNodelet, nodelet::Nodelet );
