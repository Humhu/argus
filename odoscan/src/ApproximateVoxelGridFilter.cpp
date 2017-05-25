#include "odoscan/ApproximateVoxelGridFilter.h"
#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

ApproximateVoxelGridFilter::ApproximateVoxelGridFilter() {}

void ApproximateVoxelGridFilter::InitializeDerived( ros::NodeHandle& ph )
{
	_logLeafSize.InitializeAndRead( ph, -1, "log_voxel_size", "Log voxel grid size" );

	// unsigned int minPointsPerVox;
	// GetParamRequired( ph, "min_voxel_points", minPointsPerVox );
	// _minPointsPerVox.Initialize( ph, minPointsPerVox, "min_voxel_points",
	//                              "Minimum number of points per voxel." );
}

ScanFilter::FilterType::Ptr ApproximateVoxelGridFilter::CreateFilter()
{
	boost::shared_ptr<VoxelFilterType> filter = boost::make_shared<VoxelFilterType>();
	
	double ls = std::pow( 10, _logLeafSize );
	filter->setLeafSize( ls, ls, ls );
	// NOTE Needs > PCL 1.7
	// filter->setMinimumPointsNumberPerVoxel( _minPointsPerVox ); 
	return filter;
}

}