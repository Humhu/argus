#include "manycal/ArrayCalibrator.h"

namespace manycal
{

	ArrayCalibrator::ArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ) 
	{
		XmlRpc::XmlRpcValue cameras;
		privHandle.getParam( "cameras", cameras);
		
		XmlRpc::XmlRpcValue::iterator iter = cameras.begin();
		while( iter != cameras.end() )
		{
// 			XmlRpc::XmlRpcValue::ValueStruct::value_type item = *iter;
// 			iter++;
// 			std::string name = item.first;
// 			
// 			if( registry.count( name ) > 0 )
// 			{
// 				throw std::runtime_error( "Duplicate camera  entry for " + name );
// 			}
// 			
// 			int avail = item.second["available"];
// 			int max = item.second["max"];
// 			int min = item.second["min"];
// 			
// 			ResourceEntry entry;
// 			entry.available = avail;
// 			entry.max = max;
// 			entry.min = min;
// 			registry[name] = entry;
		}
	}
	
}
