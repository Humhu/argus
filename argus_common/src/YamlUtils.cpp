#include "argus_common/YamlUtils.h"

namespace argus_common
{
	
	void SetPoseYaml( YAML::Node& node, const PoseSE3& pose, std::string id )
	{
		YAML::Node sub;
		SetQuaternionYaml( sub, pose.GetQuaternion() );
		SetPositionYaml( sub, pose.GetTranslation() );
		node[id] = sub;
	}
	
	bool GetPoseYaml( YAML::Node& node, PoseSE3& pose, std::string id )
	{
		if( !node[id] ) { return false; }
		YAML::Node n = node[id];
		Eigen::Quaterniond quat; 
		if( !GetQuaternionYaml( n, quat ) ) { return false; }
		Eigen::Translation3d pos; 
		if( !GetPositionYaml( n, pos ) ) { return false; }
		pose = PoseSE3( quat, pos );
		return true;
	}
	
	void SetQuaternionYaml( YAML::Node& node, const Eigen::Quaterniond& quat, std::string id )
	{
		std::vector<double> vals(4);
		vals[0] = quat.w();
		vals[1] = quat.x();
		vals[2] = quat.y();
		vals[3] = quat.z();
		node[id] = vals;
	}
	
	bool GetQuaternionYaml( YAML::Node& node, Eigen::Quaterniond& quat, std::string id )
	{
		if( !node[id] ) { return false; }
		std::vector<double> vals = node[id].as< std::vector<double> >();
		if( vals.size() != 4 )
		{
			throw std::runtime_error( "Incorrect number of elements for quaternion." );
		}
		quat = Eigen::Quaterniond( vals[0], vals[1], vals[2], vals[3] );
		return true;
	}
	
	void SetPositionYaml( YAML::Node& node, const Eigen::Translation3d& trans, std::string id )
	{
		std::vector<double> vals(3);
		vals[0] = trans.x();
		vals[1] = trans.y();
		vals[2] = trans.z();
		node[id] = vals;
	}
	
	bool GetPositionYaml( YAML::Node& node, Eigen::Translation3d& trans, std::string id )
	{
		if( !node[id] ) { return false; }
		std::vector<double> vals = node[id].as< std::vector<double> >();
		if( vals.size() != 3 )
		{
			throw std::runtime_error( "Incorrect number of elements for position." );
		}
		Eigen::Vector3d vec( vals[0], vals[1], vals[2] );
		trans = Eigen::Translation3d( vec );
		return true;
	}
	
	void SetMatrixYaml( YAML::Node& node, const Eigen::MatrixXd& mat, std::string idDim, std::string idVal )
	{
		unsigned int numEls = mat.rows()*mat.cols();
		std::vector<double> vals( numEls );
		std::vector<double> dimensions( 2 );
		dimensions[0] = mat.rows();
		dimensions[1] = mat.cols();
		for( unsigned int i = 0; i < numEls; i++ )
		{
			vals[i] = mat(i);
		}
		node[idVal] = vals;
		node[idDim] = dimensions;
	}
	
	bool GetMatrixYaml( YAML::Node& node, Eigen::MatrixXd& mat, std::string idDim, std::string idVal )
	{
		if( !node[idDim] || !node[idVal] ) { return false; }
		std::vector<int> dimensions = node[idDim].as< std::vector<int> >();
		std::vector<double> vals = node[idVal].as< std::vector<double> >();
		if( dimensions.size() != 2 )
		{
			throw std::runtime_error( "Invalid matrix dimension field." );
		}
		
		if( dimensions[0]*dimensions[1] != vals.size() )
		{
			throw std::runtime_error( "Invalid number of values for matrix." );
		}
		
		mat = Eigen::MatrixXd( dimensions[0], dimensions[1] );
		for( unsigned int i = 0; i < dimensions[0]*dimensions[1]; i++ )
		{
			mat(i) = vals[i];
		}
		return false;
	}
	
}