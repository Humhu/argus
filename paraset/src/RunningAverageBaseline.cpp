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

	_acc = 0;
	_accInitialized = false;
	_delayStarted = false;
	double initDelay;
	GetParam( ph, "init_delay", initDelay, 0.0 );
	_initDelay = ros::Duration( initDelay );

	_burnInStarted = false;
	double burnInTime;
	GetParam( ph, "burn_in_time", burnInTime, 0.0 );
	_burnInDuration = ros::Duration( burnInTime );

	double discountRate;
	GetParamRequired( ph, "discount_rate", discountRate );
	_gamma = std::exp( 1.0/pollRate * std::log( discountRate ) );
	ROS_INFO_STREAM( "Computed discount factor of " << _gamma << " from rate " << discountRate );


	_outputPub = ph.advertise<paraset::RewardStamped>( "output", 0 );
}

double RunningAverageBaseline::Evaluate( const ParamAction& act ) const
{
	if( !IsBurnedIn() )
	{
		throw std::out_of_range( "RunningAverageBaseline: Not ready - still burning in.");
	}

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
	if( !_delayStarted )
	{
		_delayStarted = true;
		_startDelayTime = time;
	}
	if( (time - _startDelayTime) < _initDelay ) { return; }

	if( !_accInitialized )
	{
		_acc = reward;
		_accInitialized = true;
	}
	else
	{
		_acc = _acc * _gamma + (1.0 - _gamma) * reward;
	}

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
	ros::Time queryTime;
	try
	{
		queryTime = event.current_expected - _pollOffset;
		double reward = _source->Evaluate( ParamAction( queryTime, VectorType() ) );
		Update( queryTime, reward );
	}
	catch( std::out_of_range e )
	{
		ROS_WARN_STREAM( "RunningAverageBaseline: Could not poll source." );
	}

	if( !_burnInStarted )
	{
		_burnInStarted = true;
		_startBurnTime = queryTime;
	}
	_lastBurnTime = queryTime;

	while( GetSpan() > _cacheTime )
	{
		remove_lowest( _cache );
	}
}

bool RunningAverageBaseline::IsBurnedIn() const
{
	if( !_burnInStarted ) { return false; }
	return ( _lastBurnTime - _startBurnTime ) > _burnInDuration;
}


}