#include "atags/AtagDetector.h"
#include "atags/AtagCommon.h"

#include <apriltags/Tag16h5.h>
#include <apriltags/Tag25h7.h>
#include <apriltags/Tag25h9.h>
#include <apriltags/Tag36h9.h>
#include <apriltags/Tag36h11.h>

#include <boost/foreach.hpp>

#include "argus_utils/utils/ParamUtils.h"

namespace April = AprilTags;

namespace argus
{

AtagDetector::AtagDetector()
{
	SetFamily( "36h11" );
	SetUndistortion( false );
	SetNormalization( false );
	SetMaxSkewness( 3.0 );
	SetMinArea( 4000 );
}

void AtagDetector::ReadParams( const ros::NodeHandle& ph )
{
	bool und, nor;
	if( GetParam( ph, "enable_undistortion", und ) ) { SetUndistortion( und ); }
	if( GetParam( ph, "enable_normalization", nor ) ) { SetNormalization( nor ); }

	std::string family;
	if( GetParam( ph, "tag_family", family ) ) { SetFamily( family ); }

	double ms, ma;
	if( GetParam( ph, "max_skewness_ratio", ms ) ) { SetMaxSkewness( ms ); }
	if( GetParam( ph, "min_area_product", ma ) ) { SetMinArea( ma ); }
}

void AtagDetector::SetFamily( const std::string& family )
{
	if( family == "16h5" )
	{
		_detector = std::make_shared<April::TagDetector>( April::tagCodes16h5 );
	}
	else if( family == "25h7" )
	{
		_detector = std::make_shared<April::TagDetector>( April::tagCodes25h7 );
	}
	else if( family == "25h9" )
	{
		_detector = std::make_shared<April::TagDetector>( April::tagCodes25h9 );
	}
	else if( family == "36h9" )
	{
		_detector = std::make_shared<April::TagDetector>( April::tagCodes36h9 );
	}
	else if( family == "36h11" )
	{
		_detector = std::make_shared<April::TagDetector>( April::tagCodes36h11 );
	}
	else
	{
		throw std::invalid_argument( "Invalid tag family: Must be 16h5, 25h7, 25h9, 36h9, or 36h11" );
	}
	_family = family;
}

void AtagDetector::SetUndistortion( bool enable )
{
	_undistort = enable;
}

void AtagDetector::SetNormalization( bool enable )
{
	_normalize = enable;
}

void AtagDetector::SetMaxSkewness( double s )
{
	if( s < 0 )
	{
		throw std::invalid_argument( "Max skewness must be positive." );
	}
	_maxSkewnessRatio = s;
}

void AtagDetector::SetMinArea( double a )
{
	if( a < 0 )
	{
		throw std::invalid_argument( "Min area must be positive." );
	}
	_minAreaProduct = a;
}

std::vector<FiducialDetection> AtagDetector::ProcessImage( const cv::Mat& image,
                                                           const CameraCalibration& cal ) const
{
	std::vector<April::TagDetection> rawDetections = _detector->extractTags( image );

	std::vector<FiducialDetection> detections;
	detections.reserve( rawDetections.size() );
	BOOST_FOREACH( const April::TagDetection & rawDet, rawDetections )
	{
		if( !CheckDetection( rawDet ) ) { continue; }

		FiducialDetection det = TagToFiducial( rawDet, _family );
		if( _undistort || _normalize )
		{
			det = det.Undistort( cal, _undistort, _normalize );
		}
		detections.push_back( det );
	}
	return detections;
}

bool AtagDetector::CheckDetection( const April::TagDetection& det ) const
{
	std::pair<double, double> diagLengths = ComputeDiagonals( det );
	Eigen::Matrix2d cov = ComputeCovariance( det );
	Eigen::Vector2cd eigenvalues = cov.eigenvalues();
	double eLarge = std::max( eigenvalues( 0 ).real(), eigenvalues( 1 ).real() );
	double eSmall = std::min( eigenvalues( 0 ).real(), eigenvalues( 1 ).real() );
	double eRatio = eLarge / eSmall;
	double eProd = diagLengths.first * diagLengths.second;

	bool passSkew = eRatio < _maxSkewnessRatio;
	bool passArea = eProd > _minAreaProduct;
	return passSkew && passArea;
}

}