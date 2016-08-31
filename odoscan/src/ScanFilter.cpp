#include "odoscan/ScanFilter.h"

namespace argus
{

ScanFilter::ScanFilter() {}

ScanFilter::~ScanFilter() {}

void ScanFilter::Initialize( ros::NodeHandle& ph )
{
	InitializeDerived( ph );
}

void ScanFilter::Filter( const LaserCloudType::ConstPtr& input, LaserCloudType& output )
{
	FilterType::Ptr filter = CreateFilter();
	filter->setInputCloud( input );
	filter->filter( output );
}

}