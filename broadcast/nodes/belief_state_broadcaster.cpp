#include <ros/ros.h>
#include <Eigen/Cholesky>

#include "broadcast/BroadcastTransmitter.h"
#include "broadcast/BroadcastReceiver.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"

#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/PoseSE3.h"

#include "nav_msgs/Odometry.h"

using namespace argus;

struct BeliefFeatureTraits
{
	BeliefFeatureTraits( bool twoD = false )
	: twoDimensional( twoD ) {}

	size_t PoseDim() const 
	{ 
		if( twoDimensional )
		{
			return PoseSE2::VectorDimension;
		}
		else
		{
			return PoseSE3::VectorDimension;
		}
	}

	size_t PoseCovDim() const 
	{
		if( twoDimensional )
		{
			return PoseSE2::TangentDimension * (PoseSE2::TangentDimension + 1) / 2;
		}
		else
		{
			return PoseSE3::TangentDimension * (PoseSE3::TangentDimension + 1) / 2;
		}
	}

	size_t VelDim() const 
	{
		if( twoDimensional )
		{
			return PoseSE2::TangentDimension;
		}
		else
		{
			return PoseSE3::TangentDimension;
		}
	}

	size_t VelCovDim() const { return PoseCovDim(); }

	VectorType GetPoseFeatures( const geometry_msgs::Pose& msg )
	{
		PoseSE3 pose = MsgToPose( msg );
		if( twoDimensional )
		{
			PoseSE2 pose2 = PoseSE2::FromSE3( pose );
			return pose2.ToVector();
		}
		else
		{
			return pose.ToVector();
		}
	}

	VectorType GetVelFeatures( const geometry_msgs::Twist& msg )
	{
		PoseSE3::TangentVector vel = MsgToTangent( msg );
		if( twoDimensional )
		{
			VectorType sub( 3 );
			std::vector<unsigned int> colInds = { 0 };
			std::vector<unsigned int> rowInds = { 0, 1, 5 };
			GetSubmatrix( vel, sub, rowInds, colInds );
			return sub;
		}
		else
		{
			return vel;
		}
	}

	template <typename SerializedType>
	VectorType GetPoseCovFeatures( const SerializedType& msg )
	{
		MatrixType poseCov;
		PoseSE3::CovarianceMatrix msgCov;
		ParseMatrix( msg, msgCov );
		if( twoDimensional )
		{
			MatrixType sub( 3, 3 );
			std::vector<unsigned int> inds = { 0, 1, 5 };
			GetSubmatrix( msgCov, sub, inds, inds );
			poseCov = sub;
		}
		else
		{
			poseCov = msgCov;
		}

		Eigen::LDLT<MatrixType> ldlt( poseCov );
		const Eigen::LDLT<MatrixType>::TranspositionType& P = ldlt.transpositionsP();
		MatrixType Lmat( ldlt.matrixL() );
		VectorType Lvec = GetLowerTriangular( Lmat, 1 );
		VectorType Dvec = P.transpose() * ldlt.vectorD();

		VectorType out( Lvec.size() + Dvec.size() );
		out.segment( 0, Dvec.size() ) = Dvec.array().log().matrix();
		out.segment( Dvec.size(), Lvec.size() ) = Lvec;
		return out;
	}

	template <typename SerializedType>
	VectorType GetVelCovFeatures( const SerializedType& msg )
	{
		return GetPoseCovFeatures( msg );
	}

	bool twoDimensional;
};

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
		
		bool twoD;
		GetParamRequired( ph, "two_dimensional", twoD );
		_featTraits = BeliefFeatureTraits( twoD );

		GetParam( ph, "var_offset", _varOffset, 1E-6 );

		_featureDim = 0;
		if( _usePose ) { _featureDim += _featTraits.PoseDim(); }     // Full pose with quaternion
		if( _usePoseCov ) { _featureDim += _featTraits.PoseCovDim(); } // Independent covariance elements
		if( _useVel ) { _featureDim += _featTraits.VelDim(); }      // Full velocity
		if( _useVelCov ) { _featureDim += _featTraits.VelCovDim(); }  // Independent covariance elements
		ROS_INFO_STREAM( "Feature dim: " << _featureDim );

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
	BeliefFeatureTraits _featTraits;
	ros::Subscriber _odomSub;
	unsigned int _featureDim;
	bool _usePose, _usePoseCov, _useVel, _useVelCov;
	double _varOffset;
	bool _twoDimensional;

	void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
	{
		VectorType feat( _featureDim );
		VectorType temp;
		unsigned int ind = 0;
		if( _usePose )
		{
			temp = _featTraits.GetPoseFeatures( msg->pose.pose );
			feat.segment( ind, temp.size() ) = temp;
			ind += temp.size();
		}
		if( _usePoseCov )
		{
			temp = _featTraits.GetPoseCovFeatures( msg->pose.covariance );
			feat.segment( ind, temp.size() ) = temp;
			ind += temp.size();
		}
		if( _useVel )
		{
			temp = _featTraits.GetVelFeatures( msg->twist.twist );
			feat.segment( ind, temp.size() ) = temp;
			ind += temp.size();
		}
		if( _useVelCov )
		{
			temp = _featTraits.GetVelCovFeatures( msg->twist.covariance );
			feat.segment( ind, temp.size() ) = temp;
			ind += temp.size();
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
