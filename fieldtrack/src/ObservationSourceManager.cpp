#include "fieldtrack/ObservationSourceManager.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"

#define POSE_DIM (PoseSE3::TangentDimension)
#define POSITION_DIM (PoseSE3::TangentDimension/2)
#define ORIENTATION_DIM (PoseSE3::TangentDimension/2)
#define IMU_IND_OFFSET (PoseSE3::TangentDimension/2)

namespace argus
{

ObservationSourceManager::ObservationSourceManager( ros::NodeHandle& ph,
                                                    const std::string& targetFrame,
                                                    const std::string& refFrame,
                                                    ExtrinsicsInterface::Ptr extrinsics )
: _targetFrame( targetFrame ),
  _refFrame( refFrame ),
  _extrinsicsManager( extrinsics )
{
	std::string mode;
	GetParamRequired( ph, "mode", mode );
	_mode = StringToCovMode( mode );

	if( _mode == COV_PASS )
	{
		// Nothing to parse
	}
	else if( _mode == COV_FIXED )
	{
		// TODO HACK!!
		_fixedCov = MatrixType( 6, 6 );
		GetParamRequired( ph, "fixed_covariance", _fixedCov );
	}
	else if( _mode == COV_ADAPTIVE )
	{
		if( HasParam( ph, "obs_mask" ) )
		{
			// TODO HACK!!
			_fixedCov = MatrixType( 6, 6 );
			GetParam( ph, "obs_mask", _fixedCov );
		}

		_adaptiveCov.Initialize( 6, ph );
	}
}

void ObservationSourceManager::Update( const ros::Time& time,
                                       const UpdateInfo& info )
{
	if( _mode == COV_ADAPTIVE )
	{
		_adaptiveCov.Update( time, info );
	}
}

void ObservationSourceManager::Reset()
{
	if( _mode == COV_ADAPTIVE )
	{
		_adaptiveCov.Reset();
	}
}

Observation
ObservationSourceManager::ParsePoseMessage( const std_msgs::Header& header,
                                            const PoseSE3& pose,
                                            const MatrixType& cov,
                                            const std::vector<unsigned int>& valid )
{
	std::vector<bool> mask( POSE_DIM, false );
	for( unsigned int i = 0; i < valid.size(); i++ )
	{
		mask[ valid[i] ] = true;
	}
	bool pos = mask[0] && mask[1] && mask[2];
	bool ori = mask[3] && mask[4] && mask[5];
	
	if( pos && ori )
	{
		PoseObservation obs;
		obs.timestamp = header.stamp;
		obs.referenceFrame = header.frame_id;
		obs.pose = pose;
		obs.covariance = cov;
		return obs;
	}
	else if( pos )
	{
		PositionObservation obs;
		obs.timestamp = header.stamp;
		obs.referenceFrame = header.frame_id;
		obs.position = pose.GetTranslation();
		obs.covariance = (cov.rows() == POSITION_DIM) ? cov : MaskMatrix( cov, valid );
		return obs;
	}
	else if( ori )
	{
		OrientationObservation obs;
		obs.timestamp = header.stamp;
		obs.referenceFrame = header.frame_id;
		obs.orientation = pose.GetQuaternion();
		obs.covariance = (cov.rows() == ORIENTATION_DIM) ? cov : MaskMatrix( cov, valid );
		return obs;
	}
	throw std::invalid_argument( "Pose message has invalid configuration." );
}

Observation
ObservationSourceManager::ParseDerivatives( const std_msgs::Header& header,
                                            const VectorType& derivs,
                                            const MatrixType& cov,
                                            const std::vector<unsigned int>& valid )
{
	DerivObservation obs;
	obs.timestamp = header.stamp;
	obs.referenceFrame = header.frame_id;
	obs.indices = valid;
	obs.derivatives = (derivs.size() != valid.size()) ? MaskVector( derivs, valid )
	                                                  : derivs;
	obs.covariance = (cov.rows() != valid.size()) ? MaskMatrix( cov, valid )
	                                              : cov;
	return obs;
}

Observation
ObservationSourceManager::ParseImu( const std_msgs::Header& header,
                                    const VectorType& derivs,
                                    const MatrixType& cov,
                                    const std::vector<unsigned int>& valid )
{
	DerivObservation obs;
	obs.timestamp = header.stamp;
	obs.referenceFrame = header.frame_id;
	obs.indices.clear();
	for( unsigned int i = 0; i < valid.size(); i++ )
	{
		// IMU obs start with angular velocities, so indices need to be offset
		obs.indices.push_back( IMU_IND_OFFSET + valid[i] );
	}
	obs.derivatives = (derivs.size() != valid.size()) ? MaskVector( derivs, valid )
	                                                  : derivs;
	obs.covariance = (cov.rows() != valid.size()) ? MaskMatrix( cov, valid )
	                                              : cov;
	return obs;
}

Observation ObservationSourceManager::operator()( const geometry_msgs::PoseStamped& msg )
{
	PoseSE3 pose = MsgToPose( msg.pose );
	if( msg.header.frame_id != _targetFrame )
	{
		PoseSE3 ext = _extrinsicsManager->GetExtrinsics( _targetFrame, msg.header.frame_id );
		pose = pose * ext;
	}

	MatrixType cov( POSE_DIM, POSE_DIM );
	
	std::vector<unsigned int> valid;
	if( _mode == COV_PASS )
	{
		throw std::invalid_argument( "PoseStamped cannot have COV_PASS mode." );
	}
	else if( _mode == COV_FIXED )
	{
		cov = _fixedCov;
		CheckMatrixSize( cov, POSE_DIM );
		valid = ParseCovarianceMask( cov );
	}
	else if( _mode == COV_ADAPTIVE )
	{
		if( _fixedCov.size() == 0 )
		{
			throw std::runtime_error( "PoseStamped does not have init_covariance in COV_ADAPTIVE mode." );
		}
		CheckMatrixSize( _fixedCov, POSE_DIM );
		valid = ParseCovarianceMask( _fixedCov );
		cov = _adaptiveCov.GetR( msg.header.stamp );
	}
	else
	{
		throw std::invalid_argument( "Unknown covariance mode." );
	}
	return ParsePoseMessage( msg.header, pose, cov, valid );
}

Observation ObservationSourceManager::operator()( const geometry_msgs::PoseWithCovarianceStamped& msg )
{
	PoseSE3 pose = MsgToPose( msg.pose.pose );
	if( msg.header.frame_id != _targetFrame )
	{
		PoseSE3 ext = _extrinsicsManager->GetExtrinsics( _targetFrame, msg.header.frame_id );
		pose = pose * ext;
	}

	MatrixType cov( POSE_DIM, POSE_DIM );
	ParseMatrix( msg.pose.covariance, cov );

	std::vector<unsigned int> valid;
	if( _mode == COV_PASS )
	{
		valid = ParseCovarianceMask( cov );
	}
	else if( _mode == COV_FIXED )
	{
		cov = _fixedCov;
		CheckMatrixSize( cov, POSE_DIM );
		valid = ParseCovarianceMask( cov );
	}
	else if( _mode == COV_ADAPTIVE )
	{
		MatrixType init = _fixedCov.size() == 0 ? cov : _fixedCov;
		CheckMatrixSize( init, POSE_DIM );
		valid = ParseCovarianceMask( init );
		cov = _adaptiveCov.GetR( msg.header.stamp );
	}
	else
	{
		throw std::invalid_argument( "Unknown covariance mode." );
	}
	return ParsePoseMessage( msg.header, pose, cov, valid );
}

Observation ObservationSourceManager::operator()( const geometry_msgs::TwistStamped& msg )
{
	PoseSE3::TangentVector derivs = MsgToTangent( msg.twist );
	if( msg.header.frame_id != _targetFrame )
	{
		// TODO Transform covariances according to extrinsics also
		PoseSE3 ext = _extrinsicsManager->GetExtrinsics( msg.header.frame_id, _targetFrame );
		derivs = TransformTangent( derivs, ext );
	}

	MatrixType cov( POSE_DIM, POSE_DIM );
	std::vector<unsigned int> valid;
	if( _mode == COV_PASS )
	{
		throw std::runtime_error( "TwistStamped cannot have COV_PASS mode!" );
	}
	else if( _mode == COV_FIXED )
	{
		cov = _fixedCov;
		CheckMatrixSize( cov, POSE_DIM );
		valid = ParseCovarianceMask( cov );
	}
	else if( _mode == COV_ADAPTIVE )
	{
		if( _fixedCov.size() == 0 )
		{
			throw std::runtime_error( "TwistStamped does not have init_covariance in COV_ADAPTIVE mode." );
		}
		CheckMatrixSize( _fixedCov, POSE_DIM );
		valid = ParseCovarianceMask( _fixedCov );
		cov = _adaptiveCov.GetR( msg.header.stamp );
	}
	else
	{
		throw std::invalid_argument( "Unknown covariance mode." );
	}
	return ParseDerivatives( msg.header, derivs, cov, valid );
}

Observation ObservationSourceManager::operator()( const geometry_msgs::TwistWithCovarianceStamped& msg )
{
	PoseSE3::TangentVector derivs = MsgToTangent( msg.twist.twist );
	if( msg.header.frame_id != _targetFrame )
	{
		// TODO Transform covariances according to extrinsics also
		PoseSE3 ext = _extrinsicsManager->GetExtrinsics( msg.header.frame_id, _targetFrame );
		derivs = TransformTangent( derivs, ext );
	}

	MatrixType cov( POSE_DIM, POSE_DIM );
	ParseMatrix( msg.twist.covariance, cov );
	
	std::vector<unsigned int> valid;
	if( _mode == COV_PASS )
	{
		valid = ParseCovarianceMask( cov );
	}
	else if( _mode == COV_FIXED )
	{
		cov = _fixedCov;
		CheckMatrixSize( cov, POSE_DIM );
		valid = ParseCovarianceMask( cov );
	}
	else if( _mode == COV_ADAPTIVE )
	{
		MatrixType init = _fixedCov.size() == 0 ? cov : _fixedCov;
		CheckMatrixSize( init, POSE_DIM );
		valid = ParseCovarianceMask( init );
		cov = _adaptiveCov.GetR( msg.header.stamp );
	}
	else
	{
		throw std::invalid_argument( "Unknown covariance mode." );
	}
	return ParseDerivatives( msg.header, derivs, cov, valid );
}

Observation ObservationSourceManager::operator()( const geometry_msgs::TransformStamped& msg )
{
	PoseSE3 pose = TransformToPose( msg.transform );
	pose = _extrinsicsManager->Convert( msg.header.frame_id, 
	                                    msg.child_frame_id,
	                                    msg.header.stamp,
	                                    pose,
	                                    _targetFrame,
	                                    _refFrame );

	MatrixType cov( POSE_DIM, POSE_DIM );
	std::vector<unsigned int> valid;
	if( _mode == COV_PASS )
	{
		throw std::runtime_error( "TransformStamped cannot have COV_PASS mode!" );
	}
	else if( _mode == COV_FIXED )
	{
		cov = _fixedCov;
		CheckMatrixSize( cov, POSE_DIM );
		valid = ParseCovarianceMask( cov );
	}
	else if( _mode == COV_ADAPTIVE )
	{
		if( _fixedCov.size() == 0 )
		{
			throw std::runtime_error( "TransformStamped does not have init_covariance in COV_ADAPTIVE mode." );
		}
		CheckMatrixSize( _fixedCov, POSE_DIM );
		valid = ParseCovarianceMask( _fixedCov );
		cov = _adaptiveCov.GetR( msg.header.stamp );
	}
	else
	{
		throw std::invalid_argument( "Unknown covariance mode." );
	}
	return ParsePoseMessage( msg.header, pose, cov, valid );
}

Observation ObservationSourceManager::operator()( const sensor_msgs::Imu& msg )
{
	// TODO Support orientation updates from IMU
	// Begin by parsing all data from msg
	VectorType derivs = VectorType::Zero(POSE_DIM);
	derivs.head<3>() = MsgToVector3( msg.angular_velocity );
	derivs.tail<3>() = MsgToVector3( msg.linear_acceleration );
	
	if( msg.header.frame_id != _targetFrame )
	{
		PoseSE3 ext = _extrinsicsManager->GetExtrinsics( msg.header.frame_id, _targetFrame );
		PoseSE3::TangentVector v = PoseSE3::TangentVector::Zero();
		v.head<3>() = derivs.tail<3>();
		v = TransformTangent( v, ext );
		derivs.tail<3>() = v.head<3>();
	}

	// We assume no correlation between gyro/xl measurements in the message
	MatrixType cov = MatrixType::Zero(POSE_DIM,POSE_DIM);
	MatrixType temp(3,3);
	ParseMatrix( msg.angular_velocity_covariance, temp );
	cov.topLeftCorner(3,3) = temp;
	ParseMatrix( msg.linear_acceleration_covariance, temp );
	cov.bottomRightCorner(3,3) = temp;

	std::vector<unsigned int> valid;
	if( _mode == COV_PASS )
	{
		valid = ParseCovarianceMask( cov );
	}
	else if( _mode == COV_FIXED )
	{
		cov = _fixedCov;
		CheckMatrixSize( cov, POSE_DIM );
		valid = ParseCovarianceMask( cov );
	}
	else if( _mode == COV_ADAPTIVE )
	{
		MatrixType init = (_fixedCov.size() == 0) ? cov : _fixedCov;
		CheckMatrixSize( init, POSE_DIM );
		valid = ParseCovarianceMask( init );
		cov = _adaptiveCov.GetR( msg.header.stamp );
	}
	else
	{
		throw std::invalid_argument( "Unknown covariance mode." );
	}
	return ParseImu( msg.header, derivs, cov, valid );
}

std::vector<unsigned int>
ObservationSourceManager::ParseCovarianceMask( const MatrixType& cov )
{
	std::vector<unsigned int> inds;
	for( unsigned int i = 0; i < cov.rows(); i++ )
	{
		if( cov(i,i) >= 0 ) { inds.push_back( i ); }
	}
	return inds;
}

VectorType
ObservationSourceManager::MaskVector( const VectorType& vec, 
                                      const std::vector<unsigned int>& inds )
{
	VectorType out( inds.size() );
	GetSubmatrix( vec, out, inds );
	return out;
}

MatrixType
ObservationSourceManager::MaskMatrix( const MatrixType& mat,
                                      const std::vector<unsigned int>& inds )
{
	MatrixType out( inds.size(), inds.size() );
	GetSubmatrix( mat, out, inds, inds );
	return out;
}

void ObservationSourceManager::CheckMatrixSize( const MatrixType& mat, 
                                                unsigned int s )
{
	if( mat.rows() != s || mat.cols() != s )
	{
		std::stringstream ss;
		ss << "Matrix has size: (" << mat.rows() << ", " << mat.cols() 
		   << ") but expected " << s;
		throw std::invalid_argument( ss.str() );
	}
}

}