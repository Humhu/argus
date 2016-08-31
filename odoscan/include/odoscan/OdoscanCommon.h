#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

namespace argus
{

typedef pcl::PointXYZ LaserPointType;
typedef pcl::PointCloud<LaserPointType> LaserCloudType;

}