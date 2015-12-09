#include "fieldtrack/ConstantVelocityFilter.h"

using namespace argus_utils;

namespace fieldtrack
{

ConstantVelocityFilter::ConstantVelocityFilter()
{
	// Initialize velocity filter
	velocityFilter.TransMatrix() = VelocityFilterType::StateTransition::Identity();
	velocityFilter.ObsMatrix() = VelocityFilterType::ObservationMatrix::Identity();
	velocityFilter.EstimateMean() = VelocityFilterType::StateVector::Zero();
	velocityFilter.EstimateCovariance() = VelocityFilterType::StateCovariance::Identity();
	
	// Initialize pose filter
	poseFilter.EstimateMean() = PoseSE3();
	poseFilter.EstimateCovariance() = PoseFilterType::StateCovariance::Identity();

	filterTimestamp = ros::Time::now();
}

PoseSE3::CovarianceMatrix& ConstantVelocityFilter::PoseCovarianceRate()
{
	return poseCovarianceRate;
}

PoseSE3::CovarianceMatrix& ConstantVelocityFilter::VelocityCovarianceRate()
{
	return velocityCovarianceRate;
}

ConstantVelocityFilter::PoseFilterType& ConstantVelocityFilter::PoseFilter()
{
	return poseFilter;
}

ConstantVelocityFilter::VelocityFilterType& ConstantVelocityFilter::VelocityFilter()
{
	return velocityFilter;
}

const ConstantVelocityFilter::PoseFilterType& ConstantVelocityFilter::PoseFilter() const
{
	return poseFilter;
}

const ConstantVelocityFilter::VelocityFilterType& ConstantVelocityFilter::VelocityFilter() const
{
	return velocityFilter;
}

void ConstantVelocityFilter::Predict( const ros::Time& until )
{
	double dt = ( until - filterTimestamp ).toSec();
	if( dt < 0 ) { return; }
	
	PoseSE3 displacement = PoseSE3::Exp( velocityFilter.EstimateMean() * dt );
	PoseSE3::CovarianceMatrix covRate = velocityFilter.EstimateCovariance() + poseCovarianceRate;
	poseFilter.PredictBody( displacement, covRate * dt, BodyFrame );
	filterTimestamp = until;

	velocityFilter.Predict( velocityCovarianceRate * dt );
}

void ConstantVelocityFilter::DisplaceReference( const PoseSE3& displacement,
                                                const PoseSE3::CovarianceMatrix& cov )
{
	// TODO
	poseFilter.PredictWorld( displacement.Inverse(), cov, WorldFrame );
}

void ConstantVelocityFilter::DisplaceTo( const ros::Time& until )
{
	double dt = ( until - filterTimestamp ).toSec();
	PoseSE3 displacement = PoseSE3::Exp( velocityFilter.EstimateMean() * dt );
	poseFilter.PredictBody( displacement, 
	                        PoseSE3::CovarianceMatrix::Zero(), 
	                        BodyFrame );
	filterTimestamp = until;
}

void ConstantVelocityFilter::PoseUpdate( const PoseSE3& pose,
                                         const PoseSE3::CovarianceMatrix& cov,
                                         const ros::Time& timestamp )
{
	// If observation is from before current time, rewind pose, update,
	// and then forward predict again
	if( timestamp < filterTimestamp )
	{
		ros::Time current = filterTimestamp;
		DisplaceTo( timestamp );
		poseFilter.UpdateBody( pose, cov, BodyFrame );
		DisplaceTo( current );
	}
	// Else if observation is from after current time, predict forward and update
	else
	{
		Predict( timestamp );
		poseFilter.UpdateBody( pose, cov, BodyFrame );
	}
}

void ConstantVelocityFilter::VelocityUpdate( const PoseSE3::TangentVector& velocity,
                                             const PoseSE3::CovarianceMatrix& cov,
                                             const ros::Time& timestamp )
{
	// If observation is from before current time, rewind, update, and then 
	// forward predict
	if( timestamp < filterTimestamp )
	{
		ros::Time current = filterTimestamp;
		DisplaceTo( timestamp );
		velocityFilter.Update( velocity, cov );
		DisplaceTo( current );
	}
	else
	{
		Predict( timestamp );
		velocityFilter.Update( velocity, cov );
	}
}


}
