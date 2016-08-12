#include "paraset/RunningAverageBaseline.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/MapUtils.hpp"
#include "paraset/RewardStamped.h"

namespace argus
{

RunningAverageBaseline::RunningAverageBaseline()
: _acc( 0 ), _source( nullptr ) {}

RunningAverageBaseline::RunningAverageBaseline( PolicyCritic::Ptr source )
: _acc( 0 ), _source( source ) {}


void RunningAverageBaseline::Initialize( ros::NodeHandle& nh,
                                         ros::NodeHandle& ph )
{
	GetParam( ph, "initial_value", _acc, 0.0 );
	GetParamRequired( ph, "discount_factor", _gamma );
	GetParamRequired( ph, "cache_time", _cacheTime );

	double pollOffsetTime, pollRate;
	if( GetParam( ph, "poll_offset", pollOffsetTime ) &&
	    GetParam( ph, "poll_rate", pollRate ) )
	{
		ROS_INFO_STREAM( "RunningAverageBaseline in poll mode." );
		_pollTimer = nh.createTimer( ros::Duration( 1.0/pollRate ),
		                             &RunningAverageBaseline::TimerCallback,
		                             this );
		_pollOffset = ros::Duration( pollOffsetTime );
	}
	else
	{
		ROS_INFO_STREAM( "RunningAverageBaseline in listen mode." );
		_valueSub = ph.subscribe( "input", 
		                          0,
		                          &RunningAverageBaseline::RewardCallback, 
		                          this );
	}

	_outputPub = ph.advertise<paraset::RewardStamped>( "output", 0 );
}

double RunningAverageBaseline::Evaluate( const ParamAction& act ) const
{
	CacheType::const_iterator before, after;
	if( !get_closest_greater_eq( _cache, act.time, before ) ||
	    !get_closest_lesser_eq( _cache, act.time, after ) )
	{
		throw std::out_of_range( "RunningAverageBaseline: Query out of cache range." );
	}

	double bdt = ( act.time - before->first ).toSec();
	double adt = ( after->first - act.time ).toSec();
	return ( bdt * after->second + adt * before->second ) / ( bdt + adt );
}

void RunningAverageBaseline::Publish( const ParamAction& x ) const
{
	paraset::RewardStamped msg;
	msg.header.stamp = x.time;
	msg.reward = Evaluate( x );
	_outputPub.publish( msg );
}

void RunningAverageBaseline::Update( const ros::Time& time, double reward )
{
	_acc = _acc * _gamma + (1.0 - _gamma) * reward;
	_cache[ time ] = _acc;
	while( GetSpan() > _cacheTime )
	{
		remove_lowest( _cache );
	}
}

void RunningAverageBaseline::RewardCallback( const paraset::RewardStamped::ConstPtr& msg )
{
	Update( msg->header.stamp, msg->reward );
}

double RunningAverageBaseline::GetSpan() const
{
	if( _cache.size() == 0 ) { return 0.0; }
	return ( get_highest_key( _cache ) - get_lowest_key( _cache ) ).toSec();
}

void RunningAverageBaseline::TimerCallback( const ros::TimerEvent& event )
{
	try
	{
		ros::Time queryTime = event.current_real - _pollOffset;
		double reward = _source->Evaluate( ParamAction( queryTime, VectorType() ) );
		Update( queryTime, reward );
	}
	catch( std::out_of_range e )
	{
		ROS_WARN_STREAM( "RunningAverageBaseline: Could not poll source." );
	}

	while( GetSpan() > _cacheTime )
	{
		remove_lowest( _cache );
	}
}

}