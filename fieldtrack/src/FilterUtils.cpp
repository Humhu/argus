#include "fieldtrack/utils/FilterUtils.h"

namespace argus
{

PredictInfo MsgToPredict( const argus_msgs::FilterStepInfo& msg )
{
	if( !msg.isPredict )
	{
		throw std::runtime_error( "Cannot parse update msg to predict." );
	}
	PredictInfo info;
	info.Spre = MsgToMatrix( msg.Spre );
	info.xpre = VectorType( info.Spre.rows() );
	ParseMatrix( msg.xpre, info.xpre );

	info.dt = msg.dt;
	info.F = MsgToMatrix( msg.F );
	info.Q = MsgToMatrix( msg.noiseCov );
	return info;
}

argus_msgs::FilterStepInfo PredictToMsg( const PredictInfo& info )
{
	argus_msgs::FilterStepInfo msg;
	msg.Spre = MatrixToMsg( info.Spre );
	SerializeMatrix( info.xpre, msg.xpre );
	msg.dt = info.dt;
	msg.F = MatrixToMsg( info.F );
	msg.noiseCov = MatrixToMsg( info.Q );
	msg.isPredict = true;
	return msg;
}

UpdateInfo MsgToUpdate( const argus_msgs::FilterStepInfo& msg )
{
	if( msg.isPredict )
	{
		throw std::runtime_error( "Cannot parse predict msg to update." );
	}
	UpdateInfo info;
	info.Spre = MsgToMatrix( msg.Spre );
	info.Spost = MsgToMatrix( msg.Spost );
	info.xpre = VectorType( info.Spre.rows() );
	ParseMatrix( msg.xpre, info.xpre );
	info.delta_x = VectorType( info.Spre.rows() );
	ParseMatrix( msg.delta_x, info.delta_x );

	info.H = MsgToMatrix( msg.H );
	info.R = MsgToMatrix( msg.noiseCov );

	// NOTE Must initialize size or else parsing fails
	// TODO Update this parse function interface to avoid these kinds of errors in the future
	info.innovation = VectorType( info.H.rows() );
	ParseMatrix( msg.innovation, info.innovation );
	info.observation = VectorType( info.H.rows() );
	ParseMatrix( msg.observation, info.observation );
	info.post_innovation = VectorType( info.H.rows() );
	ParseMatrix( msg.post_innovation, info.post_innovation );
	return info;
}

argus_msgs::FilterStepInfo UpdateToMsg( const UpdateInfo& info )
{
	argus_msgs::FilterStepInfo msg;
	msg.Spre = MatrixToMsg( info.Spre );
	msg.Spost = MatrixToMsg( info.Spost );
	SerializeMatrix( info.xpre, msg.xpre );
	SerializeMatrix( info.delta_x, msg.delta_x );
	SerializeMatrix( info.innovation, msg.innovation );
	SerializeMatrix( info.observation, msg.observation );
	SerializeMatrix( info.post_innovation, msg.post_innovation );
	msg.H = MatrixToMsg( info.H );
	msg.noiseCov = MatrixToMsg( info.R );
	msg.isPredict = false;
	return msg;
}

}