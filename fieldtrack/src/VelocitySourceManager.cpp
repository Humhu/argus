#include "fieldtrack/VelocitySourceManager.h"
#include "fieldtrack/DimensionParser.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/geometry/PoseSE2.h"

namespace argus
{
VelocitySourceManager::VelocitySourceManager() {}

void VelocitySourceManager::Initialize( ros::NodeHandle& ph,
                                        bool twoDimensional,
                                        unsigned int filterOrder,
                                        const std::string& targetFrame,
                                        ExtrinsicsInterface::Ptr extrinsics )
{
	_targetFrame = targetFrame;
	_twoDimensional = twoDimensional;
	_extrinsicsManager = extrinsics;

	std::string mode;
	GetParamRequired( ph, "mode", mode );
	_mode = StringToCovMode( mode );

	std::vector<std::string> indSpecs;
	GetParamRequired( ph, "dim_specs", indSpecs );
	_dimInds = parse_dim_string( indSpecs, twoDimensional, filterOrder + 1, 1 );
	// NOTE These are hard-coded limits for twist, IMU messages which will have
	// at most acceleration info
	// TODO Currently am not validating obsInds vs the expected message type
	_obsInds = parse_dim_string( indSpecs, false, 2, 1 );

	GetParam( ph, "min_log_likelihood", _minLogLikelihood,
	          -std::numeric_limits<double>::infinity() );

	// Construct observation matrix
	unsigned int stateDim = twoDimensional ?
	                        PoseSE2::TangentDimension :
	                        PoseSE3::TangentDimension;
	unsigned int fullDim = stateDim * (filterOrder + 1);
	_obsMatrix = MatrixType::Zero( _dimInds.size(), fullDim );
	for( unsigned int i = 0; i < _dimInds.size(); ++i )
	{
		_obsMatrix( i, _dimInds[i] ) = 1.0;
	}

	// Parse covariance operating mode
	if( _mode == COV_PASS )
	{
		// Nothing to parse
	}
	else if( _mode == COV_FIXED )
	{
		_fixedCov = MatrixType( _dimInds.size(), _dimInds.size() );
		GetParamRequired( ph, "fixed_covariance", _fixedCov );
	}
	else if( _mode == COV_ADAPTIVE )
	{
		_adaptiveCov.Initialize( _dimInds.size(), ph );
	}
}

const MatrixType& VelocitySourceManager::GetObservationMatrix() const
{
	return _obsMatrix;
}

CovarianceModel::Ptr VelocitySourceManager::InitializeModel() const
{
	if( _mode == COV_PASS )
	{
		return std::make_shared<PassCovariance>();
	}
	else if( _mode == COV_FIXED )
	{
		FixedCovariance::Ptr cov = std::make_shared<FixedCovariance>();
		cov->Initialize( _fixedCov );
		// TODO Make settable
		cov->EnableL( false );
		return cov;
	}
	else if( _mode == COV_ADAPTIVE )
	{
		AdaptiveCovariance::Ptr cov = std::make_shared<AdaptiveCovariance>();
		// TODO HACK
		// TODO Read parameters correctly
		// TODO Enable/disable diagonal
		// TODO Add temporal weighting and prior
		cov->SetWindowSize( 10 );
		cov->SetDefaultValue( _adaptiveCov.GetPriorCov() );
		return cov;
	}
	else
	{
		throw std::runtime_error( "Invalid covariance mode" );
	}
}

void VelocitySourceManager::SetModel( const CovarianceModel& model )
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

void VelocitySourceManager::Update( const UpdateInfo& info )
{
	if( _mode == COV_ADAPTIVE )
	{
		_adaptiveCov.Update( info );
	}
}

void VelocitySourceManager::Reset()
{
	if( _mode == COV_ADAPTIVE )
	{
		_adaptiveCov.Reset();
	}
}

bool VelocitySourceManager::CheckLogLikelihood( double ll ) const
{
	return !std::isnan(ll) && ll > _minLogLikelihood;
}

DerivObservation
VelocitySourceManager::operator()( const geometry_msgs::PoseStamped& msg )
{
	throw std::runtime_error( "VelocitySourceManager does not yet support pose differentiation" );
}

DerivObservation
VelocitySourceManager::operator()( const geometry_msgs::PoseWithCovarianceStamped& msg )
{
	throw std::runtime_error( "VelocitySourceManager does not yet support pose differentiation" );
}

DerivObservation
VelocitySourceManager::operator()( const geometry_msgs::TwistStamped& msg )
{
	VectorType derivs = MsgToTangent( msg.twist );
	MatrixType cov;
	return ProcessDerivatives( derivs, cov, msg.header.stamp,
	                           msg.header.frame_id );
}

DerivObservation
VelocitySourceManager::operator()( const geometry_msgs::TwistWithCovarianceStamped& msg )
{
	VectorType derivs = MsgToTangent( msg.twist.twist );
	MatrixType cov( 6, 6 );
	ParseMatrix( msg.twist.covariance, cov );
	return ProcessDerivatives( derivs, cov, msg.header.stamp,
	                           msg.header.frame_id );
}

DerivObservation
VelocitySourceManager::operator()( const geometry_msgs::TransformStamped& msg )
{
	throw std::runtime_error( "VelocitySourceManager does not yet support pose differentiation" );
}

DerivObservation
VelocitySourceManager::operator()( const sensor_msgs::Imu& msg )
{
	throw std::runtime_error ( "VelocitySourceManager does not yet support IMU" );
}

DerivObservation
VelocitySourceManager::ProcessDerivatives( const VectorType& derivs,
                                           const MatrixType& cov,
                                           const ros::Time& stamp,
                                           const std::string& frame )
{
	PoseSE3 ext = _extrinsicsManager->GetExtrinsics( frame, _targetFrame, stamp );
	VectorType transDerivs = TransformTangent( derivs, ext );

	DerivObservation obs;
	obs.timestamp = stamp;
	obs.referenceFrame = _targetFrame;
	obs.derivatives = VectorType( _obsInds.size() );
	GetSubmatrix( transDerivs, obs.derivatives, _obsInds );
	obs.covariance = GetCovariance( stamp, cov );
	obs.indices = _dimInds;
	return obs;
}

MatrixType VelocitySourceManager::GetCovariance( const ros::Time& stamp,
                                                 const MatrixType& recv )
{
	if( _mode == COV_PASS )
	{
		if( recv.size() == 0 )
		{
			throw std::runtime_error( "Cannot run in COV_PASS mode with non-covariance message" );
		}
		MatrixType out;
		GetSubmatrix( recv, out, _obsInds, _obsInds );
		return out;
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