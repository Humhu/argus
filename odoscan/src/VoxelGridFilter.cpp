#include "odoscan/VoxelGridFilter.h"
#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

VoxelGridFilter::VoxelGridFilter() {}

void VoxelGridFilter::InitializeDerived( ros::NodeHandle& ph )
{
	_logLeafSize.InitializeAndRead( ph, -1, "log_voxel_size", "Log voxel grid size" );

	// FullNumericRange minPointsPerVox;
	// GetParamRequired( ph, "min_voxel_points", minPointsPerVox );
	// _minPointsPerVox.Initialize( ph, minPointsPerVox.init, "min_voxel_points",
	//                              "Minimum number of points per voxel." );
	// _minPointsPerVox.AddCheck<GreaterThanOrEqual>( minPointsPerVox.min );
	// _minPointsPerVox.AddCheck<LessThanOrEqual>( minPointsPerVox.max );
	// _minPointsPerVox.AddCheck<IntegerValued>( ROUND_CLOSEST );
}

ScanFilter::FilterType::Ptr VoxelGridFilter::CreateFilter()
{
	boost::shared_ptr<VoxelFilterType> filter = boost::make_shared<VoxelFilterType>();
	
	double ls = std::pow( 10, _logLeafSize );
	filter->setLeafSize( ls, ls, ls );
	// filter->setMinimumPointsNumberPerVoxel( _minPointsPerVox ); Needs > PCL 1.7
	return filter;
}

}
