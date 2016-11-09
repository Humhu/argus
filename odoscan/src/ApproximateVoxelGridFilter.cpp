#include "odoscan/ApproximateVoxelGridFilter.h"
#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

ApproximateVoxelGridFilter::ApproximateVoxelGridFilter() {}

void ApproximateVoxelGridFilter::InitializeDerived( ros::NodeHandle& ph )
{
	double leafSize;
	GetParamRequired( ph, "voxel_size", leafSize );
	_leafSize.Initialize( ph, leafSize, "voxel_size", 
	                      "Voxel grid size" );

	// unsigned int minPointsPerVox;
	// GetParamRequired( ph, "min_voxel_points", minPointsPerVox );
	// _minPointsPerVox.Initialize( ph, minPointsPerVox, "min_voxel_points",
	//                              "Minimum number of points per voxel." );
}

ScanFilter::FilterType::Ptr ApproximateVoxelGridFilter::CreateFilter()
{
	boost::shared_ptr<VoxelFilterType> filter = boost::make_shared<VoxelFilterType>();
	
	double ls = _leafSize;
	filter->setLeafSize( ls, ls, ls );
	// filter->setMinimumPointsNumberPerVoxel( _minPointsPerVox ); Needs > PCL 1.7
	return filter;
}

}