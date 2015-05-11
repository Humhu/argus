#ifndef _DATA_CORRESPONDER_H_
#define _DATA_CORRESPONDER_H_

#include <ros/ros.h>

#include <memory>

namespace manycal
{

	/*! \brief Interface for class of objects that listen to a single topic and
	 * generate correspondences based on some criterion. */
	template <class D, class C>
	class DataCorresponder
	{
	public:
		
		typedef std::shared_ptr<DataCorresponder> Ptr;
		typedef D DataType;
		typedef C CorrespondenceType;
		
		DataCorresponder( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
			: nodeHandle( nh ), privHandle( ph )
		{
			int bufferLen;
			privHandle.param( "buffer_length", bufferLen, 10 );
			
 			subscriber = nodeHandle.subscribe<D>( "source", bufferLen, 
												  &DataCorresponder::DataCallback, this );
			publisher = privHandle.advertise<C>( "correspondences", 10 );
		}
		
		virtual void DataCallback( const typename D::ConstPtr& msg ) = 0;
		
	protected:
		
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privHandle;
		
		ros::Subscriber subscriber;
		ros::Publisher publisher;
		
	};
	
}

#endif
