#pragma once

#include <ros/ros.h>
#include "argus_utils/PoseSE3.h"
#include "argus_utils/KalmanFilter.hpp"
#include "argus_utils/ManifoldKalmanFilter.hpp"

namespace fieldtrack
{

/*! \brief A pair of Kalman filters that track body velocity and pose independently. */	
class ConstantVelocityFilter
{
public:
	
	typedef std::shared_ptr<ConstantVelocityFilter> Ptr;
		
	typedef argus_utils::KalmanFilter<double, 6, Eigen::Dynamic, 6> VelocityFilterType;
	typedef argus_utils::ManifoldKalmanFilter<argus_utils::PoseSE3, argus_utils::BodyFrame> PoseFilterType;
	
	ConstantVelocityFilter();
	
	argus_utils::PoseSE3::CovarianceMatrix& PoseCovarianceRate();
	argus_utils::PoseSE3::CovarianceMatrix& VelocityCovarianceRate();
	PoseFilterType& PoseFilter();
	const PoseFilterType& PoseFilter() const;
	VelocityFilterType& VelocityFilter();
	const VelocityFilterType& VelocityFilter() const;
	
	/*! \brief Use the current velocity to predict to a future time. */
	void Predict( const ros::Time& until );
	
	/*! \brief Apply a displacement due to a moving reference frame. */
	void DisplaceReference( const argus_utils::PoseSE3& displacement,
	                        const argus_utils::PoseSE3::CovarianceMatrix& cov );
	
	/*! \brief Applies a pose update. If update is from before current filter time,
	 * rewinds the filter mean using the current velocity estimate, applies 
	 * the update, and unrewinds the mean. Otherwise, forward predicts the filter
	 * mean and covariance and applies the update. */
	void PoseUpdate( const argus_utils::PoseSE3& pose,
	                 const argus_utils::PoseSE3::CovarianceMatrix& cov,
                     const ros::Time& until = ros::Time::now() );
	
	/*! \brief Applies a velocity update. If update is from before current filter
	 * time, rewinds the filter mean using the current velocity estimate ,applies 
	 * the update, and unrewinds the mean. Otherwise, forward predicts the filter
	 * mean and covariance and applies the update. */
	void VelocityUpdate( const argus_utils::PoseSE3::TangentVector& velocity,
	                     const argus_utils::PoseSE3::CovarianceMatrix& cov,
	                     const ros::Time& until = ros::Time::now() );
	
private:
	
	ros::Time filterTimestamp;
	
	VelocityFilterType velocityFilter;
	VelocityFilterType::StateCovariance velocityCovarianceRate;
	
	PoseFilterType poseFilter;
	PoseFilterType::StateCovariance poseCovarianceRate;
	
	/*! \brief Use the current velocity to predict forward or backward in time. Does not
	 * change the estimate covariance. Intended only for late pose updates. */
	void DisplaceTo( const ros::Time& until );
	
};
	
}
