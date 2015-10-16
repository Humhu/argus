#include "fiducial_array/FiducialArray.h"
#include "fiducial_array/FiducialCommon.h"
#include "argus_utils/GeometryUtils.h"

namespace fiducial_array
{
	
FiducialArray::FiducialArray() {}

FiducialArray::~FiducialArray() {}

FiducialArray::FiducialArray( const FiducialArrayInfo& info )
: ExtrinsicsArray( info.extrinsics )
{
	std::vector< argus_utils::PoseSE3 > poses;
	std::vector< std::vector< cv::Point3f > > points;
	poses.reserve( info.fiducials.size() );
	points.reserve( info.fiducials.size() );
	for( unsigned int i = 0; i < info.extrinsics.memberNames.size(); i++ )
	{
		poses.push_back( GetPose( info.extrinsics.memberNames[i] ) );
		points.push_back( MsgToPoints( info.fiducials[i].points ) );
	}
	
	Populate( info.extrinsics.memberNames, poses, points );
}

FiducialArray::FiducialArray( const std::string& refName,
                              const std::vector< std::string >& names,
                              const std::vector< argus_utils::PoseSE3 >& poses,
                              const std::vector< std::vector< cv::Point3f > >& points )
: ExtrinsicsArray( refName, names, poses )
{
	Populate( names, poses, points );
}

const std::vector< cv::Point3f >& FiducialArray::GetFiducialPoints( const std::string& name ) const
{
	return fiducialPoints.at( name );
}

void FiducialArray::Populate( const std::vector< std::string >& names,
                              const std::vector< argus_utils::PoseSE3 >& poses,
                              const std::vector< std::vector< cv::Point3f > >& points )
{
	typedef Eigen::Matrix<double,3,Eigen::Dynamic> Mat3D;
	
	Mat3D pointMat, transMat;
	Eigen::Vector3d v;
	for( unsigned int i = 0; i < points.size(); i++ )
	{
		size_t numPoints = points[i].size();
		
		// First populate an Eigen matrix with the fiducial-frame positions
		pointMat = Mat3D( 3, numPoints );
		for( unsigned int j = 0; j < numPoints; j++ )
		{
			v << points[i][j].x, points[i][j].y, points[i][j].z;
			pointMat.col(j) = v;
		}
		
		// Transform the point positions into the array frame
		transMat = poses[i].ToTransform() * pointMat;
		
		// Convert the transformed point positions into a cv::Point3f array
		std::vector< cv::Point3f > points( numPoints );
		for( unsigned int j = 0; j < numPoints; j++ )
		{
			points[j] = cv::Point3f( transMat(0,j), transMat(1,j), transMat(2,j) );
		}
		fiducialPoints[ names[i] ] = points;
	}
}
	
}
