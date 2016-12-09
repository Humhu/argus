#include "fieldtrack/utils/FilterInfo.h"
#include "argus_utils/utils/MatrixUtils.h"

namespace argus
{

FilterInfoBase::FilterInfoBase() {}

FilterInfoBase::FilterInfoBase( const argus_msgs::FilterStepInfo& msg )
: time( msg.header.stamp ),
  frameId( msg.header.frame_id ),
  stepNum( msg.step_num )
{}

PredictInfo::PredictInfo() {}

PredictInfo::PredictInfo( const argus_msgs::FilterPredictStep& msg )
{
	FromStepMsg( msg );
}

PredictInfo::PredictInfo( const argus_msgs::FilterStepInfo& msg )
: FilterInfoBase( msg )
{
	if( msg.info_type != argus_msgs::FilterStepInfo::PREDICT_STEP )
	{
		throw std::runtime_error( "Non-predict message type." );
	}
	FromStepMsg( msg.predict );

}

argus_msgs::FilterPredictStep PredictInfo::ToStepMsg() const
{
	argus_msgs::FilterPredictStep msg;
	msg.step_dt = step_dt;
	msg.trans_jacobian = MatrixToMsg( trans_jacobian );
	msg.trans_noise_cov = MatrixToMsg( trans_noise_cov );
	msg.prior_state_cov = MatrixToMsg( prior_state_cov );
	msg.post_state_cov = MatrixToMsg( post_state_cov );
	return msg;
}

argus_msgs::FilterStepInfo PredictInfo::ToInfoMsg() const
{
	argus_msgs::FilterStepInfo msg;
	msg.header.stamp = time;
	msg.header.frame_id = frameId;
	msg.step_num = stepNum;
	msg.info_type = argus_msgs::FilterStepInfo::PREDICT_STEP;
	msg.predict = ToStepMsg();
	return msg;
}

void PredictInfo::FromStepMsg( const argus_msgs::FilterPredictStep& msg )
{
	step_dt = msg.step_dt;
	trans_jacobian = MsgToMatrix( msg.trans_jacobian );
	trans_noise_cov = MsgToMatrix( msg.trans_noise_cov );
	prior_state_cov = MsgToMatrix( msg.prior_state_cov );
	post_state_cov = MsgToMatrix( msg.post_state_cov );
}

UpdateInfo::UpdateInfo() {}

UpdateInfo::UpdateInfo( const argus_msgs::FilterUpdateStep& msg )
{
	FromStepMsg( msg );
}

UpdateInfo::UpdateInfo( const argus_msgs::FilterStepInfo& msg )
: FilterInfoBase( msg )
{
	if( msg.info_type != argus_msgs::FilterStepInfo::UPDATE_STEP )
	{
		throw std::runtime_error( "Non-update message type." );
	}
	FromStepMsg( msg.update );
}

argus_msgs::FilterUpdateStep UpdateInfo::ToStepMsg() const
{
	argus_msgs::FilterUpdateStep msg;
	msg.prior_state_cov = MatrixToMsg( prior_state_cov );
	SerializeMatrix( prior_obs_error, msg.prior_obs_error );
	msg.obs_error_cov = MatrixToMsg( obs_error_cov );
	msg.post_state_cov = MatrixToMsg( post_state_cov );
	SerializeMatrix( state_delta, msg.state_delta );
	SerializeMatrix( post_obs_error, msg.post_obs_error );
	msg.obs_jacobian = MatrixToMsg( obs_jacobian );
	msg.obs_noise_cov = MatrixToMsg( obs_noise_cov );
	return msg;
}

argus_msgs::FilterStepInfo UpdateInfo::ToInfoMsg() const
{
	argus_msgs::FilterStepInfo msg;
	msg.header.stamp = time;
	msg.header.frame_id = frameId;
	msg.step_num = stepNum;
	msg.info_type = argus_msgs::FilterStepInfo::UPDATE_STEP;
	msg.update = ToStepMsg();
	return msg;
}

void UpdateInfo::FromStepMsg( const argus_msgs::FilterUpdateStep& msg )
{
	prior_state_cov = MsgToMatrix( msg.prior_state_cov );
	prior_obs_error = GetVectorView( msg.prior_obs_error );
	obs_error_cov = MsgToMatrix( msg.obs_error_cov );
	post_state_cov = MsgToMatrix( msg.post_state_cov );
	state_delta = GetVectorView( msg.state_delta );
	post_obs_error = GetVectorView( msg.post_obs_error );
	obs_jacobian = MsgToMatrix( msg.obs_jacobian );
	obs_noise_cov = MsgToMatrix( msg.obs_jacobian );
}

}