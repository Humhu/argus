#pragma once

#include <isam/Point3d.h>
#include <isam/Point2d.h>
#include <Eigen/Dense>
#include <iostream>

#include "manycal/slamse3.h"
#include "isam/slam_monocular.h"
#include "isam/sclam_monocular.h"

namespace isam
{

/*! \brief Represents a fiducial as a uniquely ordered group of points in space.
 * The points are ordered uniquely and specified in the fiducial frame of reference. */
class FiducialIntrinsics
{
public:
	
	typedef Eigen::Matrix <double, 3, 1> PointType;
	typedef Eigen::Matrix <double, Eigen::Dynamic, 1> VectorType;
	typedef Eigen::Matrix <double, 3, Eigen::Dynamic> MatrixType;
	
	// We have fixed-size Eigen members
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	static const char* name();
	
	/*! \brief Construct a fiducial from a vector of points. */
	FiducialIntrinsics( const std::vector <isam::Point3d>& pts );
	
	/*! \brief Construct from a stacked vector of points. */
	FiducialIntrinsics( const VectorType& v );
	
	/*! \brief Create a new fiducial by applying a small change to this fiducial. 
	 * The change should be aggregated X Y and Z for each point in order. */
	FiducialIntrinsics exmap( const Eigen::VectorXd& delta );
	
	/*! \brief Returns the dimensionality, equal to 3 times the number of points. */
	int dim() const;
	
	/*! \brief Set the fiducial point positions. */
	void set( const Eigen::VectorXd& v );
	
	/*! \brief Returns boolean vector indicating angle elements. */
	Eigen::VectorXb is_angle() const;
	
	MatrixType matrix() const;
	
	/*! \brief Returns aggregated X Y and Z point coordinates. */
	Eigen::VectorXd vector() const;
	
	void write( std::ostream& out ) const;
	
private:

	Eigen::VectorXd points;
	
};

std::ostream& operator<<( std::ostream& out, const FiducialIntrinsics& in );

/*! \brief Represents the image or camera coordinate detections of a fiducial. 
 * The points are expected to be ordered uniquely such that naively comparing
 * detections is appropriate. */
class FiducialDetection
{
public:
	
	/*! \brief Construct a detection from aggregated point x y. */
	FiducialDetection( const Eigen::VectorXd& p );
	
	/*! \brief Return the total dimensionality equal to the number of points times 2. */
	int dim() const;
	
	/*! \brief Return the detection in aggregate vector form. */
	Eigen::VectorXd vector() const;
	
	void write( std::ostream& out ) const;
	
private:
	
	Eigen::VectorXd points;
	
};

std::ostream& operator<<( std::ostream& out, const FiducialDetection& det );


FiducialDetection Predict( const FiducialIntrinsics& fiducial,
                           const MonocularIntrinsics& camera,
                           const PoseSE3& fiducialToCamera );
	
typedef NodeT <FiducialIntrinsics> FiducialIntrinsics_Node;

/*! \brief Provides a prior on a fiducial intrinsic. */
class FiducialIntrinsics_Prior
: public FactorT <FiducialIntrinsics>
{
public:
	
	typedef std::shared_ptr <FiducialIntrinsics_Prior> Ptr;
	typedef FactorT <FiducialIntrinsics> FactorType;
	
	FiducialIntrinsics_Prior( FiducialIntrinsics_Node* fiducial,
	                          const FiducialIntrinsics& prior,
	                          const Noise& noise )
	: FactorType( "FiducialIntrinsics_Prior", prior.dim(), noise, prior ),
	_fiducial( fiducial )
	{
		_nodes.resize( 1 );
		_nodes[ 0 ] = fiducial;
	}
	
	void initialize()
	{
		if( !_fiducial->initialized() )
		{
			_fiducial->init( _measure );
		}
	}
	
	Eigen::VectorXd basic_error( Selector s = ESTIMATE ) const
	{
		return _fiducial->vector( s ) - _measure.vector();
	}
	
private:
	
	FiducialIntrinsics_Node* _fiducial;
	
};

/*! \brief Factor that allows full camera and fiducial intrinsic and extrinsic
 * calibration and estimation. If camExt or fidExt are nullptr, then their transforms
 * are assumed to be identity. */
class FiducialFactor 
: public FactorT <FiducialDetection>
{
public:

	struct Properties
	{
		bool optCamReference;
		bool optCamIntrinsics;
		bool optCamExtrinsics;
		bool optFidReference;
		bool optFidIntrinsics;
		bool optFidExtrinsics;
	};
	
	typedef FactorT <FiducialDetection> FactorType;
	
	FiducialFactor( PoseSE3_Node* camRef, MonocularIntrinsics_Node* camInt,
	                PoseSE3_Node* camExt, PoseSE3_Node* fidRef,
	                FiducialIntrinsics_Node* fidInt, PoseSE3_Node* fidExt,
	                const FiducialDetection& detection, const Noise& noise,
	                Properties prop )
	: FactorType( "FiducialFactor", detection.dim(), noise, detection ),
	_cam_ref( camRef ), _cam_int( camInt ), _cam_ext( camExt ),
	_fid_ref( fidRef ), _fid_int( fidInt ), _fid_ext( fidExt ),
	properties( prop )
	{
		require( !properties.optCamExtrinsics || _cam_ext != nullptr,
		         "Cannot optimize camera extrinsics for null extrinsics node." );
		require( !properties.optFidExtrinsics || _fid_ext != nullptr,
		         "Cannot optimize fiducial extrinsics for null extrinsics node." );
		
		if( properties.optCamReference ) { _nodes.push_back( _cam_ref ); }
		if( properties.optCamIntrinsics ) { _nodes.push_back( _cam_int ); }
		if( properties.optCamExtrinsics ) { _nodes.push_back( _cam_ext ); }
		if( properties.optFidReference ) { _nodes.push_back( _fid_ref ); }
		if( properties.optFidIntrinsics ) { _nodes.push_back( _fid_int ); }
		if( properties.optFidExtrinsics ) { _nodes.push_back( _fid_ext ); }
		require( _nodes.size() > 0, "FiducialFactor created with no optimization variables." );
	}
	
	void initialize()
	{
		bool fidExtOk = ( _fid_ext == nullptr ) || _fid_ext->initialized();
		bool camExtOk = ( _cam_ext == nullptr ) || _cam_ext->initialized();
		
		require( _cam_ref->initialized() && _cam_int->initialized() && 
		         camExtOk && _fid_ref->initialized() &&
		         _fid_int->initialized() && fidExtOk,
		         "FiducialFactor requires all nodes to be initialized." );
	}
	
	Eigen::VectorXd basic_error( Selector s = ESTIMATE ) const
	{
		PoseSE3::PoseType camExtPose;
		if( _cam_ext ) { camExtPose = _cam_ext->value(s).pose; }
		
		PoseSE3::PoseType fidExtPose;
		if( _fid_ext ) { fidExtPose = _fid_ext->value(s).pose; }
		
		PoseSE3 relPose( camExtPose.Inverse() * _cam_ref->value(s).pose.Inverse() *
                         _fid_ref->value(s).pose * fidExtPose );
		FiducialDetection predicted = Predict( _fid_int->value(s), _cam_int->value(s), relPose );
		
		return predicted.vector() - _measure.vector();
	}
	
private:
	
	PoseSE3_Node* _cam_ref;
	MonocularIntrinsics_Node* _cam_int;
	PoseSE3_Node* _cam_ext;
	PoseSE3_Node* _fid_ref;
	FiducialIntrinsics_Node* _fid_int;
	PoseSE3_Node* _fid_ext;
	Properties properties;
	
};
	
}
