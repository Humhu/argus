#pragma once

#include "odoscan/OdoscanCommon.h"
#include "paraset/ParameterManager.hpp"

#include <pcl/filters/filter.h>

namespace argus
{

/*! \brief Base for all classes that filter point clouds. */
class ScanFilter
{
public:

	typedef std::shared_ptr<ScanFilter> Ptr;
	typedef pcl::Filter<LaserPointType> FilterType;

	ScanFilter();
	virtual ~ScanFilter();

	void Initialize( ros::NodeHandle& ph );

	void Filter( const LaserCloudType::ConstPtr& input, LaserCloudType& output );

protected:

	virtual void InitializeDerived( ros::NodeHandle& ph ) = 0;

	virtual FilterType::Ptr CreateFilter() = 0;

};

}