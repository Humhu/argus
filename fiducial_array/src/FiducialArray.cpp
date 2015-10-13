#include "fiducial_array/FiducialArray.h"

#include "argus_utils/GeometryUtils.h"

namespace fiducial_array
{
	
FiducialArray::FiducialArray() {}

void FiducialArray::FromInfo( const FiducialArrayInfo& info )
{

	typedef Eigen::Matrix<double,3,Eigen::Dynamic> Mat3D;
	
	Mat3D pointMat, transMat;
	Eigen::Vector3d v;
	for( unsigned int i = 0; i < info.fiducialNames.size(); i++ )
	{
		size_t numPoints = info.intrinsics[i].points.size();
		
		// First populate an Eigen matrix with the fiducial-frame positions
		pointMat = Mat3D( 3, numPoints );
		for( unsigned int j = 0; j < numPoints; j++ )
		{
			v << info.intrinsics[i].points[j].x, 
			     info.intrinsics[i].points[j].y, 
			     info.intrinsics[i].points[j].z;
			pointMat.col(j) = v;
		}
		
		// Transform the point positions into the array frame
		argus_utils::PoseSE3 extrinsic = argus_utils::MsgToPose( info.extrinsics[i] );
		transMat = extrinsic.ToTransform() * pointMat;
		
		// Convert the transformed point positions into a cv::Point3f array
		std::vector< cv::Point3f > points( numPoints );
		for( unsigned int j = 0; j < numPoints; j++ )
		{
			points[j] = cv::Point3f( transMat(0,j), transMat(1,j), transMat(2,j) );
		}
		fiducialPoints[ info.fiducialNames[i] ] = points;
	}
}

const std::vector< cv::Point3f >& FiducialArray::GetFiducialPoints( const std::string& name ) const
{
	return fiducialPoints.at( name );
}
	
}
