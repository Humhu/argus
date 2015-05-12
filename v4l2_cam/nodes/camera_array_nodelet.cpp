#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "v4l2_cam/CameraArray.h"

namespace v4l2_cam
{

	class CameraArrayNodelet 
		: public nodelet::Nodelet
	{
		
	public:
		
		CameraArrayNodelet() {}
		~CameraArrayNodelet() {}
		
	private:
		
		CameraArray::Ptr array;
		
		virtual void onInit()
		{
			array = std::make_shared<CameraArray>(
				getNodeHandle(), getPrivateNodeHandle() );
		}
		
	};
	
}

PLUGINLIB_DECLARE_CLASS( v4l2_cam, camera_array_nodelet,
						 v4l2_cam::CameraArrayNodelet, nodelet::Nodelet );
