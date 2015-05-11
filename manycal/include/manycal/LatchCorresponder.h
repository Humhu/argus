#ifndef _LATCH_CORRESPONDER_H_
#define _LATCH_CORRESPONDER_H_

#include "manycal/DataCorresponder.hpp"

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

namespace manycal
{
	
	/*! \brief Corresponder class that associates all data in between latching calls. */
	template <class D, class C>
	class LatchCorresponder
		: public DataCorresponder<D,C>
	{
	public:
		
		typedef std::shared_ptr<LatchCorresponder> Ptr;
		
		LatchCorresponder( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
			: DataCorresponder<D,C>( nh, ph ), latchOn( false )
		{}
		
		virtual void DataCallback( const typename D::ConstPtr& msg )
		{
			boost::unique_lock<boost::mutex> lock( mutex );
			if( latchOn )
			{
				latchedData.push_back( msg );
			}
		}
		
		void StartLatch()
		{
			boost::unique_lock<boost::mutex> lock( mutex );
			latchOn = true;
			latchedData.clear();
		}
		
		void EndLatch()
		{
			boost::unique_lock<boost::mutex> lock( mutex );
			latchOn = false;
			PublishLatch();
			latchedData.clear();
		}
		
	protected:
		
		// Flush contents of latchedData in C message. Should be implemented by derived class.
		virtual void PublishLatch() = 0;
				
		boost::mutex mutex;
		bool latchOn;
		
		// TODO Make threadsafe queue
		std::vector<typename D::ConstPtr> latchedData;
		
	};
	
}

#endif
