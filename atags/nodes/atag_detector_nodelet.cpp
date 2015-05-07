#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "atags/AtagDetector.h"

namespace atags
{

	class AtagDetectorNodelet
		: public nodelet::Nodelet
	{
	public:
		
		AtagDetectorNodelet() {}
		~AtagDetectorNodelet() {}
		
	private:
	
		AtagDetector::Ptr detector;
		
		virtual void onInit()
		{
			detector = std::make_shared<AtagDetector>(
				getNodeHandle(), getPrivateNodeHandle() );
		}
	};
	
}

PLUGINLIB_DECLARE_CLASS( atags, atag_detector_nodelet,
						 atags::AtagDetectorNodelet, nodelet::Nodelet );
