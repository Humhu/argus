#ifndef _SEMAPHORE_H_
#define _SEMAPHORE_H_

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>

namespace argus_common {

	/*! \brief A standard semaphore class. */
	class Semaphore {
	public:

		Semaphore( int startCounter )
			: counter( startCounter )
		{}
		
		void Increment( unsigned int i = 1 )
		{
			boost::unique_lock<Mutex> lock( mutex );
			counter += i;
			hasCounters.notify_all(); // TODO all or one?
		}
		
		void Decrement( unsigned int i = 1 )
		{
			boost::unique_lock<Mutex> lock( mutex );
			
			while( counter < i )
			{
				hasCounters.wait( lock );
			}
			counter = counter - i;
		}

		// TODO IncrementWait and DecrementWait
		
		/*! \brief Returns how many counters are available. */
		int Query() const
		{
			boost::shared_lock<Mutex> lock( mutex );
			return counter;
		}
		
	protected:

		typedef boost::shared_mutex Mutex;
		typedef boost::condition_variable_any Condition;
		
		mutable Mutex mutex;
		int counter;
		Condition hasCounters;
		
	};

}

#endif
