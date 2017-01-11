#pragma once

#include "argus_utils/utils/LinalgTypes.h"
#include <boost/variant.hpp>

#include "argus_msgs/FilterStepInfo.h"
#include "argus_msgs/FilterPredictStep.h"
#include "argus_msgs/FilterUpdateStep.h"

namespace argus
{

struct FilterInfoBase
{
	ros::Time time;
	std::string frameId;
	unsigned int stepNum;

	FilterInfoBase();
	FilterInfoBase( const argus_msgs::FilterStepInfo& msg );
};

/*! \brief Information from a filter predict step used to learn
 * noise models. */
struct PredictInfo
: public FilterInfoBase
{
	double step_dt; // The predict time step size
	MatrixType prior_state_cov; // State covariance before predict step
	MatrixType trans_jacobian; // Matrix used to propagate covariance
	MatrixType trans_noise_cov; // The Q matrix used in this step
	MatrixType post_state_cov; // State covariance after update step

	PredictInfo();
	PredictInfo( const argus_msgs::FilterPredictStep& msg );
	PredictInfo( const argus_msgs::FilterStepInfo& msg );
	argus_msgs::FilterPredictStep ToStepMsg() const;
	argus_msgs::FilterStepInfo ToInfoMsg() const;

	void FromStepMsg( const argus_msgs::FilterPredictStep& msg );
};

/*! \brief Information from a filter update step used to learn 
 * noise models. */
struct UpdateInfo
: public FilterInfoBase
{
	MatrixType prior_state_cov; // State covariance before update
	VectorType prior_obs_error; // Observation prediction error
	MatrixType obs_error_cov; // Observation error covariance
	
	MatrixType post_state_cov; // State covariance after update
	VectorType state_delta; // Change applied to state
	VectorType post_obs_error; // Post-update observation prediction error
	
	MatrixType kalman_gain; // Kalman gain matrix
	MatrixType obs_jacobian; // Matrix used to map state covariance to observation
	MatrixType obs_noise_cov; // The R matrix used in this step

	UpdateInfo();
	UpdateInfo( const argus_msgs::FilterUpdateStep& msg );
	UpdateInfo( const argus_msgs::FilterStepInfo& msg );
	argus_msgs::FilterUpdateStep ToStepMsg() const;
	argus_msgs::FilterStepInfo ToInfoMsg() const;

	void FromStepMsg( const argus_msgs::FilterUpdateStep& msg );
};

typedef boost::variant<PredictInfo, UpdateInfo> FilterInfo;

struct FilterInfoMessageVisitor
: public boost::static_visitor<argus_msgs::FilterStepInfo>
{
	typedef argus_msgs::FilterStepInfo MsgType;

	FilterInfoMessageVisitor() {};

	template <typename M>
	MsgType operator()( const M& m ) const
	{
		return m.ToInfoMsg();
	}
};

}