#ifndef _THREADSAFE_QUEUE_H_
#define _THREADSAFE_QUEUE_H_

#include <memory>
#include "localizer/utils/ThreadsafeClass.h"
#include "localizer/utils/Condition.h"

namespace localizer {

	namespace utils {

		// TODO: Deprecate and replace with some boost thing
		template <class T>
		/*! \class ThreadsafeQueue ThreadsafeQueue.h
		* \brief is a mutex-wrapped std::deque. It implements the most basic STL
		* methods and adds a Condition for waiting. */
		class ThreadsafeQueue {
		public:

			typedef T DataType;
			typedef std::deque<T> ContainerType;

			// TODO Switch to use policies instead?
			enum QueuePolicy { KeepHead, KeepTail };

			const size_t maxSize; // Max size 0 means no bounds
			const QueuePolicy policy;

			ThreadsafeQueue( size_t _maxSize, QueuePolicy _policy = KeepTail );

			T& back();
			T& front();
			const T& back() const;
			const T& front() const;

			void push_back(T &item);
			void push_front(T &item);
			void pop_back();
			void pop_front();
			size_t size() const;
			bool empty() const;
			void clear();

			typename ThreadsafeQueue<T>::ContainerType GetContents() const;

			/*! \brief Wait for contents. */
			void Wait();

			/*! \brief Wait for this queue to be empty. */
			void WaitEmpty();

			/*! \brief Atomic combination of front() and pop_front(); */
			T DequeueFront();
			T DequeueBack();

			/*! \brief Atomic combination of Wait() followed by front() and pop_front() */
			T WaitFront();

			/*! \brief Atomic combination of Wait() followed by back() and pop_back() */
			T WaitBack();

		protected:

			mutable Mutex mutex;

			/*! \var The container in which items are stored. */
			ContainerType items;
			ConditionVariable hasContents;
			ConditionVariable isEmpty;

			T& back( WriteLock& lock );
			T& front( WriteLock& lock );
			void push_back( T& t, WriteLock& lock );
			void push_front( T& t, WriteLock& lock );
			void pop_back( WriteLock& lock );
			void pop_front( WriteLock& lock );
			void Wait( WriteLock& lock );
			void WaitEmpty( WriteLock& lock );

		};

		#include "localizer/utils/ThreadsafeQueue.hpp"

	}

}

#endif //_THREADSAFE_QUEUE_H_
