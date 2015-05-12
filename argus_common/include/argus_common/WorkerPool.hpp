#ifndef _WORKER_POOL_H_
#define _WORKER_POOL_H_

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>

#include <queue>

namespace argus_common {

	/*! \brief An asynchronous worker thread pool. */
	class WorkerPool {
	public:

		typedef std::shared_ptr<WorkerPool> Ptr;
		typedef boost::function<void()> Job;

		WorkerPool()
			: numWorkers( 4 ) {}
			
		WorkerPool( unsigned int n )
			: numWorkers( n ) {}
			
		~WorkerPool()
		{
			StopWorkers();
		}
		
		void SetNumWorkers( unsigned int n )
		{
			numWorkers = n;
		}
		
		void EnqueueJob( Job job )
		{
			jobQueue.push( job );
			hasJobs.notify_one();
		}

		void StartWorkers()
		{
			std::cout << "Starting with " << numWorkers << " workers." << std::endl;
			for( unsigned int i = 0; i < numWorkers; i++) {
				workerThreads.create_thread( boost::bind( &WorkerPool::WorkerLoop, this ) );
			}
		}
		
		void StopWorkers()
		{
			workerThreads.interrupt_all();
			workerThreads.join_all();
		}

		void WaitOnJobs()
		{
			Lock lock( mutex );
			while( !jobQueue.empty() )
			{
				hasNoJobs.wait( lock );
			}
		}
		
	protected:

		typedef boost::unique_lock< boost::shared_mutex > Lock;

		boost::shared_mutex mutex;
		unsigned int numWorkers;
		std::queue<Job> jobQueue;
		boost::condition_variable_any hasJobs;
		boost::condition_variable_any hasNoJobs;
		boost::thread_group workerThreads;
		
		void WorkerLoop()
		{
			while( true ) {

				try {
					
					Lock lock( mutex );
					while( jobQueue.empty() )
					{
						hasJobs.wait( lock );
					}
					
					Job job = jobQueue.front();
					jobQueue.pop();
					if( jobQueue.size() == 0 )
					{
						hasNoJobs.notify_all();
					}
					
					lock.unlock();
					
					job();

				} catch( boost::thread_interrupted e ) {
					return;
				}
			}
		}

	};

}


#endif
