#pragma once

#include "odoscan/ScanFilter.h"

#include <pcl/filters/approximate_voxel_grid.h>

namespace argus
{

class ApproximateVoxelGridFilter
: public ScanFilter
{
public:

	typedef std::shared_ptr<ApproximateVoxelGridFilter> Ptr;
	typedef pcl::ApproximateVoxelGrid<LaserPointType> VoxelFilterType;

	ApproximateVoxelGridFilter();

private:

	NumericParam _logLeafSize;
	// NumericParam _minPointsPerVox;

	virtual void InitializeDerived( ros::NodeHandle& ph );
	virtual FilterType::Ptr CreateFilter();

};

}