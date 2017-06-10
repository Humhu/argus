#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include "camplex/CameraDriver.h"

// Services auto-generated by ROS
#include "camplex/CaptureFrames.h"
#include "camplex/GetCameraInfo.h"
#include "camplex/PrintCapabilities.h"
#include "camplex/SetStreaming.h"

#include "paraset/ParameterManager.hpp"

#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <memory>
#include <boost/thread/locks.hpp>
#include <deque>

namespace argus
{

/*! \brief Provides a ROS interface to a CameraDriver object.
 */
class DriverNode
{
public:

	typedef std::shared_ptr<DriverNode> Ptr;

	DriverNode( ros::NodeHandle& nh, ros::NodeHandle& ph );
	~DriverNode();

private:

	typedef camera_info_manager::CameraInfoManager InfoManager;

	ros::Timer _timer;

	// Service handlers
	ros::ServiceServer _getInfoServer;
	ros::ServiceServer _capabilitiesServer;
	ros::ServiceServer _setStreamingServer;

	image_transport::ImageTransport _it;
	image_transport::CameraPublisher _itPub;

	std::shared_ptr<InfoManager> _cameraInfoManager;
	sensor_msgs::CameraInfo::Ptr _cameraInfo;

	/*! \brief Denotes the camera's mode of operation. */
	enum StreamingMode
	{
		STREAM_OFF,
		STREAM_CONTINUOUS,
	};

	mutable Mutex _mutex;
	ConditionVariable _blocked;
	StreamingMode _mode;

	std::string _cameraName;
	CameraDriver _driver;

	std::deque<NumericParam> _numericParams;
	std::deque<BooleanParam> _booleanParams;

	// Externally-locked functions to set the streaming state
	void StartStreaming( WriteLock& lock );
	void StopStreaming( WriteLock& lock );

	void IntControlCallback( int id, double value );
	void BoolControlCallback( int id, bool value );

	bool GetCameraInfoService( camplex::GetCameraInfo::Request& req,
	                           camplex::GetCameraInfo::Response& res );

	bool SetStreamingService( camplex::SetStreaming::Request& req,
	                          camplex::SetStreaming::Response& res );

	bool PrintCapabilitiesService( camplex::PrintCapabilities::Request& req,
	                               camplex::PrintCapabilities::Response& res );

	void Spin( const ros::TimerEvent& event );

};

}
