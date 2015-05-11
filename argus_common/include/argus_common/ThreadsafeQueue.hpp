#ifndef _THREADSAFE_QUEUE_H_
#define _THREADSAFE_QUEUE_H_

#include <memory>
#include <deque>

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>

namespace argus_common {
	
	/*! \class ThreadsafeContainer ThreadsafeQueue.h
	* \brief is a mutex-wrapped container. */
	template <class T, template<typename,typename> class Container = std::deque >
	class ThreadsafeQueue {
	public:

		typedef T DataType;
		typedef Container<T, typename std::allocator<T> > ContainerType;

		ThreadsafeQueue( size_t initSize )
			: items( initSize ) {}

		void PushFront( const T& item )
		{
			Lock lock( mutex );
			items.push_front( item );
			hasContents.notify_one();
		}
		
		void PushBack( const T& item)
		{
			Lock lock( mutex );
			items.push_back( item );
			hasContents.notify_one();
		}
		
		void WaitPopFront( T& item )
		{
			Lock lock( mutex );
			while( items.empty() )
			{
				hasContents.wait( lock );
			}
			item = items.front();
			items.pop_front();
			
			if( items.empty() )
			{
				isEmpty.notify_all();
			}
		}
		
		void WaitPopBack( T& item )
		{
			Lock lock( mutex );
			while( items.empty() )
			{
				hasContents.wait( lock );
			}
			item = items.back();
			items.pop_back();	
			
			if( items.empty() )
			{
				isEmpty.notify_all();
			}
		}
		
		bool TryPopFront( T& item )
		{
			Lock lock( mutex );
			if( items.empty() )
			{
				return false;
			}
			item = items.front();
			items.pop_front();
			
			if( items.empty() )
			{
				isEmpty.notify_all();
			}
			return true;
		}
		
		void TryPopBack( T& item )
		{
			Lock lock( mutex );
			while( items.empty() )
			{
				return false;
			}
			item = items.back();
			items.pop_back();	
			
			if( items.empty() )
			{
				isEmpty.notify_all();
			}
			return true;
		}
		
		size_t Size() const
		{
			Lock lock( mutex );
			return items.size();
		}
		
		bool IsEmpty() const
		{
			return Size() == 0;
		}
		
		void Clear()
		{
			Lock lock( mutex );
			items.clear();
			isEmpty.notify_all();
		}

		/*! \brief Wait for this queue to be empty. */
		void WaitEmpty()
		{
			Lock lock( mutex );
			while( !items.empty() )
			{
				isEmpty.wait( lock );
			}
			
		}

	protected:

		typedef boost::mutex Mutex;
		typedef boost::unique_lock<Mutex> Lock;
		typedef boost::condition_variable ConditionVariable;
		
		mutable Mutex mutex;

		ContainerType items;
		ConditionVariable hasContents;
		ConditionVariable isEmpty;

	};

}

#endif //_THREADSAFE_QUEUE_H_
