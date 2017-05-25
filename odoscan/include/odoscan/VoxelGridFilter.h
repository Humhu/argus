#pragma once

#include "odoscan/ScanFilter.h"

#include <pcl/filters/voxel_grid.h>

namespace argus
{

class VoxelGridFilter
: public ScanFilter
{
public:

	typedef std::shared_ptr<VoxelGridFilter> Ptr;
	typedef pcl::VoxelGrid<LaserPointType> VoxelFilterType;

	VoxelGridFilter();

private:

	NumericParam _logLeafSize;
	// NumericParam _minPointsPerVox;

	virtual void InitializeDerived( ros::NodeHandle& ph );
	virtual FilterType::Ptr CreateFilter();

};

}