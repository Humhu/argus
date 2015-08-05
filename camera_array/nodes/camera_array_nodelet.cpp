#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "camera_array/CameraArray.h"

namespace camera_array
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

PLUGINLIB_DECLARE_CLASS( camera_array, camera_array_nodelet,
						 camera_array::CameraArrayNodelet, nodelet::Nodelet );
