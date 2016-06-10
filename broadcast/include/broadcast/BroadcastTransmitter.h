#pragma once

#include <ros/ros.h>
#include "argus_utils/utils/LinalgTypes.h"

namespace argus
{

/*! \brief Wraps registering and publishing a feature broadcast. */
class BroadcastTransmitter
{
public:

	BroadcastTransmitter( const std::string& streamName, 
	                      unsigned int featureSize,
	                      const std::vector<std::string> featureDescriptions,
	                      const std::string& streamNs = "~",
	                      unsigned int outgoingQueueSize = 10,
	                      const std::string& topic = "features_raw" );

	void Publish( const ros::Time& stamp, const VectorType& feats );

private:

	ros::NodeHandle _nodeHandle;
	std::string _streamName;
	unsigned int _streamSize;
	ros::Publisher _streamPub;

};

}