#include "fieldtrack/PoseSourceManager.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"

namespace argus
{
PoseSourceManager::PoseSourceManager() {}

void PoseSourceManager::Initialize( ros::NodeHandle& ph,
                                    bool twoDimensional,
                                    const std::string& refFrame,
                                    const std::string& targetFrame,
                                    ExtrinsicsInterface::Ptr extrinsics )
{
	_referenceFrame = refFrame;
	_targetFrame = targetFrame;
	_twoDimensional = twoDimensional;
	_extrinsics = extrinsics;

	std::string mode;
	GetParamRequired( ph, "mode", mode );
	_mode = StringToCovMode( mode );

	GetParam( ph, "reference_frame", _obsRefFrame );

	// Parse covariance operating mode
	if( _mode == COV_PASS )
	{
		// Nothing to parse
	}
	else if( _mode == COV_FIXED )
	{
		_fixedCov = MatrixType( GetDim(), GetDim() );
		GetParamRequired( ph, "fixed_covariance", _fixedCov );
	}
	else if( _mode == COV_ADAPTIVE )
	{
		_adaptiveCov.Initialize( GetDim(), ph );
	}
}

unsigned int PoseSourceManager::GetDim() const
{
	// TODO Support 2D mode correctly
	return _twoDimensional ?
	       PoseSE2::TangentDimension :
	       PoseSE3::TangentDimension;
}

CovarianceModel::Ptr PoseSourceManager::InitializeModel() const
{
	if( _mode == COV_PASS )
	{
		return std::make_shared<PassCovariance>();
	}
	else if( _mode == COV_FIXED )
	{
		FixedCovariance::Ptr cov = std::make_shared<FixedCovariance>();
		cov->Initialize( _fixedCov );
		return cov;
	}
	else if( _mode == COV_ADAPTIVE )
	{
		AdaptiveCovariance::Ptr cov = std::make_shared<AdaptiveCovariance>();
		// TODO HACK
		cov->SetWindowSize( 10 );
		cov->SetDefaultValue( _adaptiveCov.GetPriorCov() );
		return cov;
	}
	else
	{
		throw std::runtime_error( "Invalid covariance mode" );
	}
}

void PoseSourceManager::SetModel( const CovarianceModel& model )
{
	try
	{
		if( _mode == COV_PASS )
		{
			// Nothing to do
		}
		else if( _mode == COV_FIXED )
		{
			const FixedCovariance& fcov = dynamic_cast<const FixedCovariance&>( model );
			_fixedCov = fcov.GetValue();
		}
		else if( _mode == COV_ADAPTIVE )
		{
			// TODO Currently nothing to do
		}
	}
	catch( std::bad_cast& e )
	{
		throw std::invalid_argument( "Incorrect model type: " + std::string( e.what() ) );
	}
}

void PoseSourceManager::Update( const UpdateInfo& info )
{
	if( _mode == COV_ADAPTIVE )
	{
		_adaptiveCov.Update( info );
	}
}

void PoseSourceManager::Reset()
{
	if( _mode == COV_ADAPTIVE )
	{
		_adaptiveCov.Reset();
	}
}

PoseObservation
PoseSourceManager::operator()( const geometry_msgs::PoseStamped& msg )
{
	PoseSE3 pose = MsgToPose( msg.pose );
	MatrixType cov;
	return ProcessPose( pose, cov,
	                    msg.header.stamp,
	                    _obsRefFrame,
	                    msg.header.frame_id );
}

PoseObservation
PoseSourceManager::operator()( const geometry_msgs::PoseWithCovarianceStamped& msg )
{
	PoseSE3 pose = MsgToPose( msg.pose.pose );
	MatrixType cov;
	ParseMatrix( msg.pose.covariance, cov );
	return ProcessPose( pose, cov,
	                    msg.header.stamp,
	                    _obsRefFrame,
	                    msg.header.frame_id );
}

PoseObservation
PoseSourceManager::operator()( const geometry_msgs::TwistStamped& msg )
{
	throw std::runtime_error( "PoseSourceManager cannot process TwistStamped" );
}

PoseObservation
PoseSourceManager::operator()( const geometry_msgs::TwistWithCovarianceStamped& msg )
{
	throw std::runtime_error( "PoseSourceManager cannot process TwistWithCovarianceStamped" );
}

PoseObservation
PoseSourceManager::operator()( const geometry_msgs::TransformStamped& msg )
{
	PoseSE3 pose = TransformToPose( msg.transform );
	MatrixType cov;
	return ProcessPose( pose, cov,
	                    msg.header.stamp,
	                    msg.child_frame_id,
	                    msg.header.frame_id );
}

PoseObservation
PoseSourceManager::operator()( const sensor_msgs::Imu& msg )
{
	throw std::runtime_error ( "PoseSourceManager does not yet support IMU" );
}

PoseObservation
PoseSourceManager::ProcessPose( const PoseSE3& pose,
                                const MatrixType& cov,
                                const ros::Time& stamp,
                                const std::string& refFrame,
                                const std::string& tarFrame )
{
	PoseSE3 obsPose;
	MatrixType obsCov = GetCovariance( stamp, cov );
	try
	{
		PoseSE3 orefToRef = _extrinsics->GetExtrinsics( refFrame,
		                                                _referenceFrame );
		PoseSE3 tarToOtar = _extrinsics->GetExtrinsics( tarFrame,
		                                                _targetFrame );
		obsPose = orefToRef * pose * tarToOtar;
		MatrixType adj = PoseSE3::Adjoint( tarToOtar.Inverse() );
		obsCov = adj * cov * adj.transpose();
	}
	catch( ExtrinsicsException )
	{
		try
		{
			PoseSE3 tarToOref = _extrinsics->GetExtrinsics( _targetFrame,
			                                                refFrame );
			PoseSE3 otarToRef = _extrinsics->GetExtrinsics( tarFrame,
			                                                _referenceFrame );
			obsPose = otarToRef * pose.Inverse() * tarToOref;
			MatrixType adj = PoseSE3::Adjoint( pose * tarToOref.Inverse()  );
			obsCov = adj * cov * adj.transpose();
		}
		catch( ExtrinsicsException )
		{
			std::stringstream ss;
			ss << "Could not convert obs of " << tarFrame << " to " <<
			refFrame << " to desired pose of " << _targetFrame << "to " <<
			_referenceFrame; 
			throw std::runtime_error( ss.str() );
		}
	}

	PoseObservation obs;
	obs.timestamp = stamp;
	obs.referenceFrame = _referenceFrame;
	obs.pose = obsPose;
	obs.covariance = obsCov;
	return obs;
}

MatrixType PoseSourceManager::GetCovariance( const ros::Time& stamp,
                                             const MatrixType& recv )
{
	if( _mode == COV_PASS )
	{
		if( recv.size() == 0 )
		{
			throw std::runtime_error( "Cannot run in COV_PASS mode with non-covariance message" );
		}
		return recv;
	}
	else if( _mode == COV_FIXED )
	{
		return _fixedCov;
	}
	else if( _mode == COV_ADAPTIVE )
	{
		return _adaptiveCov.GetR( stamp );
	}
	else
	{
		throw std::runtime_error( "Unknown covariance mode!" );
	}
}
}
