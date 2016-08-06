#include <ros/ros.h>
#include <Eigen/Cholesky>

#include "broadcast/BroadcastTransmitter.h"
#include "broadcast/BroadcastReceiver.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"

#include "nav_msgs/Odometry.h"

using namespace argus;

#define POSE_DIM (7)
#define POSE_COV_DIM (21)
#define VEL_DIM (6)
#define VEL_COV_DIM (21)

// Takes odometry messages and converts them into feature vectors
class BeliefStateBroadcaster
{
public:

	BeliefStateBroadcaster( ros::NodeHandle& ph )
	{
		std::string streamName, modeStr;
		std::vector<std::string> descriptions;
		GetParamRequired( ph, "stream_name", streamName );
		GetParamRequired( ph, "mode", modeStr );
		GetParamRequired( ph, "descriptions", descriptions ); // TODO
		
		GetParam( ph, "use_pose", _usePose, false );
		GetParam( ph, "use_pose_cov", _usePoseCov, false );
		GetParam( ph, "use_vel", _useVel, false );
		GetParam( ph, "use_vel_cov", _useVelCov, false );
		if( !_usePose && !_usePoseCov && !_useVel && !_useVelCov )
		{
			throw std::runtime_error( "Must specify at least one part of state to publish." );
		}
		
		_featureDim = 0;
		if( _usePose ) { _featureDim += POSE_DIM; }     // Full pose with quaternion
		if( _usePoseCov ) { _featureDim += POSE_COV_DIM; } // Independent covariance elements
		if( _useVel ) { _featureDim += VEL_DIM; }      // Full velocity
		if( _useVelCov ) { _featureDim += VEL_COV_DIM; }  // Independent covariance elements

		BroadcastMode mode = StringToBroadcastMode( modeStr );
		if( mode == PUSH_TOPIC )
		{
			unsigned int queueSize;
			GetParam<unsigned int>( ph, "queue_size", queueSize, 0 );
			_tx.InitializePushStream( streamName, 
			                          ph, 
			                          _featureDim,
			                          descriptions,
			                          queueSize );
		}
		else if( mode == PULL_TOPIC )
		{
			double cacheTime;
			GetParam<double>( ph, "cache_time", cacheTime, 1.0 );
			_tx.InitializePullStream( streamName,
			                          ph,
			                          _featureDim,
			                          descriptions,
			                          cacheTime );
		}
		else
		{
			throw std::runtime_error( "Invalid broadcast mode." );
		}

		ros::NodeHandle nh;
		_odomSub = nh.subscribe( "odom", 
		                         0, 
		                         &BeliefStateBroadcaster::OdomCallback, 
		                         this );
	}

private:

	BroadcastTransmitter _tx;
	ros::Subscriber _odomSub;
	unsigned int _featureDim;
	bool _usePose, _usePoseCov, _useVel, _useVelCov;

	void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
	{
		typedef Eigen::LDLT<PoseSE3::CovarianceMatrix> CovLDL;

		VectorType feat( _featureDim );
		unsigned int ind = 0;
		if( _usePose )
		{
			PoseSE3 pose = MsgToPose( msg->pose.pose );
			feat.segment( ind, POSE_DIM ) = pose.ToVector();
			ind += POSE_DIM;
		}
		if( _usePoseCov )
		{
			PoseSE3::CovarianceMatrix poseCov;
			ParseMatrix( msg->pose.covariance, poseCov );
			CovLDL ldlt( poseCov );
			const CovLDL::TranspositionType& P = ldlt.transpositionsP();
			MatrixType Lmat( ldlt.matrixL() );
			Lmat = P.transpose() * Lmat;
			VectorType Lvec = GetLowerTriangular( Lmat, 1 );
			VectorType Dvec = P.transpose() * ldlt.vectorD();
			feat.segment( ind, Dvec.size() ) = Dvec;
			feat.segment( ind + Dvec.size(), Lvec.size() ) = Lvec;
			ind += Lvec.size() + Dvec.size();
		}
		if( _useVel )
		{
			PoseSE3::TangentVector vel = MsgToTangent( msg->twist.twist );
			feat.segment( ind, VEL_DIM ) = vel;
			ind += VEL_DIM;
		}
		if( _useVelCov )
		{
			PoseSE3::CovarianceMatrix velCov;
			ParseMatrix( msg->twist.covariance, velCov );
			CovLDL ldlt( velCov );
			const CovLDL::TranspositionType& P = ldlt.transpositionsP();
			MatrixType Lmat( ldlt.matrixL() );
			Lmat = P.transpose() * Lmat;
			VectorType Lvec = GetLowerTriangular( Lmat, 1 );
			VectorType Dvec = P.transpose() * ldlt.vectorD();

			feat.segment( ind, Dvec.size() ) = Dvec;
			feat.segment( ind + Dvec.size(), Lvec.size() ) = Lvec;
			ind += Lvec.size() + Dvec.size();
		}
		if( ind != _featureDim )
		{
			throw std::runtime_error( "BeliefStateBroadcaster: Vector population error." );
		}

		_tx.Publish( msg->header.stamp, feat );
	}

};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "belief_state_broadcaster" );

	ros::NodeHandle ph("~");
	BeliefStateBroadcaster broadcaster( ph );
	ros::spin();
	return 0;
}
