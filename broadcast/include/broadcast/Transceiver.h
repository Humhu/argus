#pragma once

#include <ros/ros.h>

#include "broadcast/StampedFeatures.h"

#include <argus_utils/utils/BoundedMap.hpp>
#include <argus_utils/utils/LinalgTypes.h>

namespace argus
{

class Transceiver
{
public:

	Transceiver( ros::NodeHandle& nh, ros::NodeHandle& ph );

	/*! \brief Retrieve the latest received registered streams appended 
	 * into a single vector. The order of appending is the same as the
	 * order of registration. */
	VectorType GetLatestData() const;

private:

	/*! \brief Subscribe to the globally-unique stream name. */
	void RegisterStream( const std::string& featureName );


	void FeatureCallback( const broadcast::StampedFeatures::ConstPtr& msg );

	ros::NodeHandle _nodeHandle;

	struct StreamRegistration
	{
		BoundedMap<ros::Time, VectorType> _buffer;
		ros::Subscriber _streamSub;
	};

	std::vector<StreamRegistration> _registrations;

};

}