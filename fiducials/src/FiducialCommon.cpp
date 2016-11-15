#include "fiducials/FiducialCommon.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/foreach.hpp>

namespace argus
{

Fiducial::Fiducial() {}

Fiducial::Fiducial( const fiducials::FiducialInfo& info )
{
	points.reserve( info.points.size() );
	for( unsigned int i = 0; i < info.points.size(); ++i )
	{
		points.emplace_back( info.points[i].x, 
		                     info.points[i].y, 
		                     info.points[i].z );
	}
}

Fiducial Fiducial::Transform( const PoseSE3& pose ) const
{
	Fiducial transformed;
	PoseSE3::Transform tform = pose.ToTransform();
	transformed.points.reserve( points.size() );
	for( unsigned int i = 0; i < points.size(); ++i )
	{
		Translation3Type::VectorType p;
		p << points[i].x(), points[i].y(), points[i].z();
		Translation3Type::VectorType pt = tform * p;
		transformed.points.emplace_back( pt );
	}
	return transformed;
}

fiducials::FiducialInfo Fiducial::ToMsg() const
{
	fiducials::FiducialInfo info;
	info.points.resize( points.size() );
	for( unsigned int i = 0; i < info.points.size(); ++i )
	{
		info.points[i].x = points[i].x();
		info.points[i].y = points[i].y();
		info.points[i].z = points[i].z();
	}
	return info;
}

FiducialDetection::FiducialDetection() {}

FiducialDetection::FiducialDetection( const argus_msgs::FiducialDetection& msg )
{
	name = msg.name;
	undistorted = msg.undistorted;
	normalized = msg.normalized;
	points.reserve( msg.points.size() );
	for( unsigned int i = 0; i < msg.points.size(); i++ )
	{
		points.emplace_back( msg.points[i].x, msg.points[i].y );
	}
}

FiducialDetection 
FiducialDetection::Undistort( const camplex::CameraCalibration& cameraModel,
                              bool undistort, 
                              bool normalize )
{
	FiducialDetection undistortedDet( *this );
	if( points.empty() ) { return undistortedDet; }

	bool needsUndistortion = undistort && !undistorted;
	bool needsNormalization = normalize && !normalized;
	// TODO Move undistorted and normalized out of detections and into the ImageDetection instead?
	// Shortcut work if possible
	if( !needsUndistortion && !needsNormalization ) { return undistortedDet; }
	
	// TODO A better camera model validity catch
	// Quick catch of uncalibrated camera model
	if( std::isnan( cameraModel.GetFx() ) )
	{
		throw std::runtime_error( "UndistortDetections: Camera model is invalid." );
	}
	

	cv::Mat distortionCoeffs;
	cv::Matx33d outputCameraMatrix;
	if( needsUndistortion ) { distortionCoeffs = cameraModel.GetDistortionCoeffs(); }
	outputCameraMatrix = needsNormalization ? cv::Matx33d::eye() : cameraModel.GetIntrinsicMatrix();

	// TODO Preallocate size?
	std::vector<cv::Point2f> undistortedCv;
	cv::undistortPoints( PointsToCv( points ),
	                     undistortedCv,
	                     cameraModel.GetIntrinsicMatrix(),
	                     distortionCoeffs,
	                     cv::noArray(),
	                     outputCameraMatrix );

	if( needsUndistortion ) { undistortedDet.undistorted = true; }
	if( needsNormalization ) { undistortedDet.normalized = true; }
	undistortedDet.points = CvToPoints( undistortedCv );
	return undistortedDet;
}

argus_msgs::FiducialDetection FiducialDetection::ToMsg() const
{
	argus_msgs::FiducialDetection msg;
	msg.name = name;
	msg.undistorted = undistorted;
	msg.normalized = normalized;
	msg.points.resize( points.size() );
	for( unsigned int i = 0; i < points.size(); i++ )
	{
		msg.points[i].x = points[i].x();
		msg.points[i].y = points[i].y();
	}
	return msg;
}

std::vector<cv::Point2f> PointsToCv( const std::vector<Translation2Type>& points )
{
	std::vector<cv::Point2f> cv;
	cv.reserve( points.size() );
	BOOST_FOREACH( const Translation2Type& point, points )
	{
		cv.emplace_back( point.x(), point.y() );
	}
	return cv;
}

std::vector<Translation2Type> CvToPoints( const std::vector<cv::Point2f>& cv )
{
	std::vector<Translation2Type> points;
	points.reserve( cv.size() );
	BOOST_FOREACH( const cv::Point2f& c, cv )
	{
		points.emplace_back( c.x, c.y );
	}
	return points;
}

std::vector<cv::Point3f> PointsToCv( const std::vector<Translation3Type>& points )
{
	std::vector<cv::Point3f> cv;
	cv.reserve( points.size() );
	BOOST_FOREACH( const Translation3Type& point, points )
	{
		cv.emplace_back( point.x(), point.y(), point.z() );
	}
	return cv;
}

std::vector<Translation3Type> CvToPoints( const std::vector<cv::Point3f>& cv )
{
	std::vector<Translation3Type> points;
	points.reserve( cv.size() );
	BOOST_FOREACH( const cv::Point3f& c, cv )
	{
		points.emplace_back( c.x, c.y, c.z );
	}
	return points;
}

bool ProjectDetection( const Fiducial& fiducial,
                       const camplex::CameraCalibration& cameraModel,
                       const PoseSE3& fiducialToCam,
                       FiducialDetection& detection )
{
	detection.undistorted = true;
	detection.normalized = false;
	
	typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Matrix3Dynamic;
	Matrix3Dynamic fidPoints = Matrix3Dynamic( 3, fiducial.points.size() );
	for( unsigned int i = 0; i < fiducial.points.size(); i++ )
	{
		fidPoints(0,i) = fiducial.points[i].x();
		fidPoints(1,i) = fiducial.points[i].y();
		fidPoints(2,i) = fiducial.points[i].z();
	}

	Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
	K(0,0) = cameraModel.GetFx();
	K(1,1) = cameraModel.GetFy();
	K(0,2) = cameraModel.GetCx();
	K(1,2) = cameraModel.GetCy();
	
	// Have to switch to z-forward coordinates
	static PoseSE3 camToObj( 0, 0, 0, -0.5, 0.5, -0.5, 0.5 );
// 	Eigen::Transform <double, 2, Eigen::Affine> cameraMatrix( K.block<2,2>(0,0) );
	PoseSE3 fidToCam = camToObj.Inverse() * fiducialToCam;
	
	Matrix3Dynamic relPoints = fidToCam.ToTransform() * fidPoints;
	for( unsigned int i = 0; i < relPoints.cols(); i++ )
	{
		if( relPoints(2,i) < 0 ) { return false; }
	}
	
	typedef Eigen::Matrix<double, 2, Eigen::Dynamic> Matrix2Dynamic;
	Matrix2Dynamic imgPoints = (K * relPoints).colwise().hnormalized();
	for( unsigned int i = 0; i < imgPoints.cols(); ++i )
	{
		detection.points.emplace_back( imgPoints(0,i), imgPoints(1,i) );
	}
	
	if( !CheckDetectionROI( detection, cameraModel.GetRoi() ) ) { return false; }
		
	return true;
}

bool CheckDetectionROI( const FiducialDetection& det, const cv::Rect& roi )
{
	BOOST_FOREACH( const Translation2Type& point, det.points )
	{
		if( std::isnan( point.x() ) || std::isnan( point.y() ) ||
		    point.x() < 0 || point.x() > roi.width ||
		    point.y() < 0 || point.y() > roi.height ) 
		{
			return false;
		}
	}
	return true;
}

double FindMinDistance( const std::vector <argus_msgs::Point2D>& points )
{
	if( points.empty() ) { return 0; }
	
	double minSeen = std::numeric_limits<double>::infinity();
	for( unsigned int i = 0; i < points.size(); i++ )
	{
		const argus_msgs::Point2D& pi = points[i];
		for( unsigned int j = i + 1; j < points.size(); j++ )
		{
			const argus_msgs::Point2D& pj = points[j];
			double dx = pi.x - pj.x;
			double dy = pi.y - pj.y;
			double dist = dx*dx + dy*dy;
			if( dist < minSeen ) { minSeen = dist; }
		}
	}
	return std::sqrt( minSeen );
}


// TODO if info is null, assume normalized and undistorted detections
PoseSE3 EstimateArrayPose( const std::vector<FiducialDetection>& detections,
                           const std::vector<Fiducial>& fiducials,
                           const PoseSE3& guess )
{
	if( detections.empty() )
	{
		throw std::runtime_error( "Cannot estimate pose with no detections." );
	}

	std::vector<cv::Point2f> points2d;
	BOOST_FOREACH( const FiducialDetection& det, detections )
	{
		if( !det.normalized )
		{
			throw std::runtime_error( "EstimateArrayPose: Detections must be normalized." );
		}
		std::vector<cv::Point2f> pts = PointsToCv( det.points );
		points2d.insert( points2d.end(), pts.begin(), pts.end() );
	}

	std::vector<cv::Point3f> points3d;
	BOOST_FOREACH( const Fiducial& fid, fiducials )
	{
		std::vector<cv::Point3f> pts = PointsToCv( fid.points );
		points3d.insert( points3d.end(), pts.begin(), pts.end() );
	}

	// Maps from z-forward convention to x-forward
	static PoseSE3 zToX( 0, 0, 0, -0.5, 0.5, -0.5, 0.5 );

	// Initialize guess
	cv::Mat rvec;
	cv::Mat tvec( 3, 1, CV_64FC1 ); // Must allocate tvec
	cv::Matx33d R;
	PoseSE3 postGuess = zToX.Inverse() * guess;
	Eigen::Matrix4d H = postGuess.ToTransform().matrix();
	for( unsigned int i = 0; i < 3; i++ )
	{
		for( unsigned int j = 0; j < 3; j++ )
		{
			R(i,j) = H(i,j);
		}
		tvec.at<double>(i) = H(i,3);
	}

	cv::Mat distortionCoeffs;
	cv::Matx33f cameraMat = cv::Matx33f::eye();
	cv::Rodrigues( R, rvec );
	cv::solvePnP( points3d, points2d, cameraMat, distortionCoeffs, rvec, tvec, false );
	cv::Rodrigues( rvec, R );
	H << R(0,0), R(0,1), R(0,2), tvec.at<double>(0),
	     R(1,0), R(1,1), R(1,2), tvec.at<double>(1),
	     R(2,0), R(2,1), R(2,2), tvec.at<double>(2),
	          0,      0,      0,      1;
	return zToX * PoseSE3( H );
}

} // end namespace fiducials
