/** \file ThreadsafeQueue.hpp Implementation for ThreadsafeQueue */

template <class T>
ThreadsafeQueue<T>::ThreadsafeQueue( size_t _maxSize, QueuePolicy _policy ) :
	maxSize( _maxSize ), policy( _policy ), hasContents( mutex ), isEmpty( mutex ) {};

template <class T>
T& ThreadsafeQueue<T>::back() {
	WriteLock lock(mutex);
	return back( lock );
}

template <class T>
T &ThreadsafeQueue<T>::front() {
	WriteLock lock(mutex);
	return front( lock );
}

template <class T>
const T& ThreadsafeQueue<T>::back() const {
	WriteLock lock(mutex);
	return items.back();
}

template <class T>
const T &ThreadsafeQueue<T>::front() const {
	WriteLock lock(mutex);
	return items.front();
}

template <class T>
void ThreadsafeQueue<T>::push_back(T &item) {
	WriteLock lock(mutex);
	push_back( item, lock );
}

template <class T>
void ThreadsafeQueue<T>::push_front(T &item) {
	WriteLock lock(mutex);
	push_front( item, lock );
}

template <class T>
void ThreadsafeQueue<T>::pop_back() {
	WriteLock lock(mutex);
	pop_back( lock );
}

template <class T>
void ThreadsafeQueue<T>::pop_front() {
	WriteLock lock(mutex);
	pop_front( lock );
}

template <class T>
size_t ThreadsafeQueue<T>::size() const {
	ReadLock lock(mutex);
	return items.size();
}

template <class T>
bool ThreadsafeQueue<T>::empty() const{
	ReadLock lock(mutex);
	return items.empty();
}

template <class T>
void ThreadsafeQueue<T>::clear() {
	WriteLock lock(mutex);
	items.clear();
}

template <class T>
void ThreadsafeQueue<T>::Wait() {
	WriteLock lock(mutex);
	while( items.empty() ) {
		hasContents.Wait(lock);
	}
}

template <class T>
void ThreadsafeQueue<T>::WaitEmpty() {
	WriteLock lock(mutex);
	WaitEmpty( lock );
}

template <class T>
T ThreadsafeQueue<T>::DequeueFront() {
	WriteLock lock(mutex);
	T t = front( lock );
	pop_front( lock );
	return t;
}

template <class T>
T ThreadsafeQueue<T>::DequeueBack() {
	WriteLock lock(mutex);
	T t = back( lock );
	pop_back( lock );
	return t;
}

template <class T>
T ThreadsafeQueue<T>::WaitFront() {
	WriteLock lock(mutex);
	Wait( lock );
	T t = front( lock );
	pop_front( lock );
	return t;
}

template <class T>
T ThreadsafeQueue<T>::WaitBack() {
	WriteLock lock(mutex);
	Wait( lock );
	T t = back( lock );
	pop_back( lock );
	return t;
}

template <class T>
T& ThreadsafeQueue<T>::back( WriteLock& lock ) {
	CheckExternalLock( mutex, lock );
	return items.back();
}

template <class T>
T& ThreadsafeQueue<T>::front( WriteLock& lock ) {
	CheckExternalLock( mutex, lock );
	return items.front();
}

template <class T>
void ThreadsafeQueue<T>::push_back( T& item, WriteLock& lock ) {
	CheckExternalLock( mutex, lock );

	hasContents.NotifyAll();

	// If we can push, push it
	if( maxSize == 0 || items.size() < maxSize ) {
		items.push_back( item );
		return;
	}

	// Else we have to check the policy
	switch( policy ) {
		case KeepHead:
			// We keep the head and do nothing
			break;

		case KeepTail:
			// Get rid of the head and push it on
			items.pop_front();
			items.push_back( item );
			break;
	}
}

template <class T>
void ThreadsafeQueue<T>::push_front( T& item, WriteLock& lock ) {
	CheckExternalLock( mutex, lock );

	hasContents.NotifyAll();

	if( maxSize == 0 || items.size() < maxSize ) {
		items.push_front( item );
		return;
	}

	switch( policy ) {
		case KeepHead:
			// Get rid of the tail and push it on
			items.pop_back();
			items.push_front( item );
			break;

		case KeepTail:
			// We keep the tail and do nothing
			break;
	}
}

template <class T>
void ThreadsafeQueue<T>::pop_back( WriteLock& lock ) {
	CheckExternalLock( mutex, lock );
	items.pop_back();

	if( items.empty() ) {
		isEmpty.NotifyAll();
	}
}

template <class T>
void ThreadsafeQueue<T>::pop_front( WriteLock& lock ) {
	CheckExternalLock( mutex, lock );
	items.pop_front();

	if( items.empty() ) {
		isEmpty.NotifyAll();
	}
}

template <class T>
void ThreadsafeQueue<T>::Wait( WriteLock& lock ) {
	CheckExternalLock( mutex, lock );
	while( items.empty() ) {
		hasContents.Wait( lock );
	}
}

template <class T>
void ThreadsafeQueue<T>::WaitEmpty( WriteLock& lock ) {
	CheckExternalLock( mutex, lock );
	while( !items.empty() ) {
		isEmpty.Wait( lock );
	}
}

template <class T>
typename ThreadsafeQueue<T>::ContainerType ThreadsafeQueue<T>::GetContents() const {
	return items;
}

