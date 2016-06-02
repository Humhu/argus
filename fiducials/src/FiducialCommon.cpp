#include "fiducials/FiducialCommon.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/foreach.hpp>

namespace argus
{

std::vector< cv::Point2f > MsgToPoints( const std::vector< argus_msgs::Point2D >& msg )
{
	std::vector< cv::Point2f > points;
	points.reserve( msg.size() );
	BOOST_FOREACH( const argus_msgs::Point2D& p, msg)
	{
		cv::Point2f pc( p.x, p.y );
		points.push_back( pc );
	}
	return points;
}
	
std::vector< cv::Point3f > MsgToPoints( const std::vector< geometry_msgs::Point >& msg )
{
	std::vector< cv::Point3f > points;
	points.reserve( msg.size() );
	BOOST_FOREACH( const geometry_msgs::Point& p, msg)
	{
		cv::Point3f pc( p.x, p.y, p.z );
		points.push_back( pc );
	}
	return points;
}
	
std::vector <argus_msgs::Point2D> PointsToMsg( const std::vector <cv::Point2f>& points )
{
	std::vector <argus_msgs::Point2D> msgs;
	msgs.reserve( points.size() );
	for( unsigned int i = 0; i < points.size(); i++ )
	{
		argus_msgs::Point2D msg;
		msg.x = points[i].x;
		msg.y = points[i].y;
		msgs.push_back( msg );
	}
	return msgs;
}

std::vector <geometry_msgs::Point> PointsToMsg( const std::vector <cv::Point3f>& points )
{
	std::vector <geometry_msgs::Point> msgs;
	msgs.reserve( points.size() );
	for( unsigned int i = 0; i < points.size(); i++ )
	{
		geometry_msgs::Point msg;
		msg.x = points[i].x;
		msg.y = points[i].y;
		msg.z = points[i].z;
		msgs.push_back( msg );
	}
	return msgs;
}

Eigen::Matrix <double, 2, Eigen::Dynamic> PointsToMatrix( const std::vector <cv::Point2f>& points )
{
	if( points.size() == 0 )
	{
		throw std::runtime_error( "Cannot convert zero size point array." );
	}
	
	Eigen::Matrix <double, 2, Eigen::Dynamic> mat( 2, points.size() );
	for( unsigned int i = 0; i < points.size(); i++ )
	{
		mat.col( i ) = Eigen::Vector2d( points[i].x, points[i].y );
	}
	return mat;
}

Eigen::Matrix <double, 3, Eigen::Dynamic> PointsToMatrix( const std::vector <cv::Point3f>& points )
{
	if( points.size() == 0 )
	{
		throw std::runtime_error( "Cannot convert zero size point array." );
	}
	
	Eigen::Matrix <double, 3, Eigen::Dynamic> mat( 3, points.size() );
	for( unsigned int i = 0; i < points.size(); i++ )
	{
		mat.col( i ) = Eigen::Vector3d( points[i].x, points[i].y, points[i].z );
	}
	return mat;
}

std::vector <cv::Point2f> MatrixToPoints( const Eigen::Matrix <double, 2, Eigen::Dynamic>& mat )
{
	std::vector <cv::Point2f> points;
	points.reserve( mat.cols() );
	for( unsigned int i = 0; i < mat.cols(); i++ )
	{
		points.emplace_back( mat(0,i), mat(1,i) );
	}
	return points;
}

std::vector <cv::Point3f> MatrixToPoints( const Eigen::Matrix <double, 3, Eigen::Dynamic>& mat )
{
	std::vector <cv::Point3f> points;
	points.reserve( mat.cols() );
	for( unsigned int i = 0; i < mat.cols(); i++ )
	{
		points.emplace_back( mat(0,i), mat(1,i), mat(2,i) );
	}
	return points;	
}


std::vector <argus_msgs::Point2D> MatrixToMsg( const Eigen::Matrix <double, 2, Eigen::Dynamic>& mat )
{
	std::vector <argus_msgs::Point2D> msgs;
	msgs.reserve( mat.cols() );
	for( unsigned int i = 0; i < mat.cols(); i++ )
	{
		argus_msgs::Point2D msg;
		msg.x = mat(0,i);
		msg.y = mat(1,i);
		msgs.push_back( msg );
	}
	return msgs;
}

std::vector <geometry_msgs::Point> MatrixToMsg( const Eigen::Matrix <double, 3, Eigen::Dynamic>& mat )
{
	std::vector <geometry_msgs::Point> msgs;
	msgs.reserve( mat.cols() );
	for( unsigned int i = 0; i < mat.cols(); i++ )
	{
		geometry_msgs::Point msg;
		msg.x = mat(0,i);
		msg.y = mat(1,i);
		msg.z = mat(2,i);
		msgs.push_back( msg );
	}
	return msgs;
}

Eigen::Matrix <double, 2, Eigen::Dynamic> MsgToMatrix( const std::vector <argus_msgs::Point2D>& msg )
{
	if( msg.size() == 0 )
	{
		throw std::runtime_error( "Cannot convert zero size message array." );
	}
	Eigen::Matrix <double, 2, Eigen::Dynamic> mat( 2, msg.size() );
	for( unsigned int i = 0; i < msg.size(); i++ )
	{
		mat.col(i) = Eigen::Vector2d( msg[i].x, msg[i].y );
	}
	return mat;
}

Eigen::Matrix <double, 3, Eigen::Dynamic> MsgToMatrix( const std::vector <geometry_msgs::Point>& msg )
{
	if( msg.size() == 0 )
	{
		throw std::runtime_error( "Cannot convert zero size message array." );
	}
	Eigen::Matrix <double, 3, Eigen::Dynamic> mat( 3, msg.size() );
	for( unsigned int i = 0; i < msg.size(); i++ )
	{
		mat.col(i) = Eigen::Vector3d( msg[i].x, msg[i].y, msg[i].z );
	}
	return mat;
}

bool UndistortDetections( const std::vector< argus_msgs::FiducialDetection >& detections,
                          const camplex::CameraCalibration& cameraModel,
                          bool undistort, bool normalize,
                          std::vector< argus_msgs::FiducialDetection >& undistorted )
{
	undistorted.clear();
	
	if( detections.empty() ) { return true; }
	
	// Shortcut work if possible
	if( detections[0].undistorted == undistort 
	    && detections[0].normalized == normalize )
	{ return true; }
	
	// Quick catch of uncalibrated camera model
	if( std::isnan( cameraModel.GetFx() ) ) { return false; }
	
	// TODO Preallocate size?
	std::vector< cv::Point2f > points2d, undistortedPoints;
	for( unsigned int i = 0; i < detections.size(); i++ )
	{
		for( unsigned int j = 0; j < detections[i].points.size(); j++ )
		{
			cv::Point2f point( detections[i].points[j].x, detections[i].points[j].y );
			points2d.push_back( point );
		}
	}
	
	// Perform undistortion
	cv::Mat distortionCoeffs;
	cv::Matx33d outputCameraMatrix;
	
	if( undistort ) { distortionCoeffs = cameraModel.GetDistortionCoeffs(); }
	outputCameraMatrix = normalize ? cv::Matx33d::eye() : cameraModel.GetIntrinsicMatrix();
	
	cv::undistortPoints( points2d, undistortedPoints, cameraModel.GetIntrinsicMatrix(),
	                     distortionCoeffs, cv::noArray(), outputCameraMatrix );
	
	// Assign to undistorted output 
	undistorted.reserve( detections.size() );
	unsigned int ind = 0;
	argus_msgs::FiducialDetection det;
	for( unsigned int i = 0; i < detections.size(); i++ )
	{
		det.name = detections[i].name;
		det.undistorted = undistort;
		det.normalized = normalize;
		det.points.clear();
		det.points.reserve( detections[i].points.size() );
		for( unsigned int j = 0; j < detections[i].points.size(); j++ )
		{
			argus_msgs::Point2D point;
			point.x = undistortedPoints[ind].x;
			point.y = undistortedPoints[ind].y;
			det.points.push_back( point );
			ind++;
		}
		undistorted.push_back( det );
	}
	return true;
}

bool 
ProjectDetection( const Fiducial& fiducial,
                  const camplex::CameraCalibration& cameraModel,
                  const PoseSE3& fiducialToCam,
                  argus_msgs::FiducialDetection& detection )
{
	detection.undistorted = true;
	detection.normalized = false;
	
	Eigen::Matrix<double, 3, Eigen::Dynamic> fidPoints = MsgToMatrix( fiducial.points );
	Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
	K(0,0) = cameraModel.GetFx();
	K(1,1) = cameraModel.GetFy();
	K(0,2) = cameraModel.GetCx();
	K(1,2) = cameraModel.GetCy();
	
	// Have to switch to z-forward coordinates
	static PoseSE3 camToObj( 0, 0, 0, -0.5, 0.5, -0.5, 0.5 );
// 	Eigen::Transform <double, 2, Eigen::Affine> cameraMatrix( K.block<2,2>(0,0) );
	PoseSE3 fidToCam = camToObj.Inverse() * fiducialToCam;
	
	Eigen::Matrix <double, 3, Eigen::Dynamic> relPoints = fidToCam.ToTransform() * fidPoints;
	for( unsigned int i = 0; i < relPoints.cols(); i++ )
	{
		if( relPoints(2,i) < 0 ) { return false; }
	}
	
	Eigen::Matrix <double, 2, Eigen::Dynamic> imgPoints = (K * relPoints).colwise().hnormalized();
	detection.points = MatrixToMsg( imgPoints );
	
	if( !CheckDetectionROI( detection, cameraModel.GetRoi() ) ) { return false; }
		
	return true;
}

bool CheckDetectionROI( const argus_msgs::FiducialDetection& det, const cv::Rect& roi )
{
	BOOST_FOREACH( const argus_msgs::Point2D& point, det.points )
	{
		if( std::isnan( point.x ) || std::isnan( point.y ) ||
			point.x < 0 || point.x > roi.width ||
		    point.y < 0 || point.y > roi.height ) { return false; }
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

} // end namespace fiducials
