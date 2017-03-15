#pragma once

#include <deque>
#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>
#include <sstream>
#include <stdexcept>

#include "argus_utils/synchronization/SynchronizationTypes.h"

namespace argus
{

/*! \brief Synchronizes buffers of timestamped data within some amount of tolerance.
 * NOTE: Assumes that data arrives in temporal order.
 * NOTE: Accesses to the output buffer are synchronized, but parameter setting and registration
 * is not
 */
template <typename Msg, typename Key = std::string>
class MessageSynchronizer
{
public:

    typedef std::pair<double, Msg> StampedData;
    typedef std::map<Key, StampedData> DataMap;

    MessageSynchronizer( unsigned int buffLen = 10, double maxDt = 0.1 ) 
    : _bufferLen( buffLen ), _maxDt( maxDt ) {}

    // NOTE Does not change buffer length of existing buffers!
    void SetBufferLength( unsigned int buffLen )
    {
        if( buffLen != _bufferLen && _registry.size() > 0 )
        {
            std::cerr << "Warning: Changing buffer length does not modify existing buffers." << std::endl;
        }
        _bufferLen = buffLen;
    }

    void SetMaxDt( double dt )
    {
        _maxDt = dt;
    }

    void RegisterSource( const Key& key )
    {
        if( _registry.count( key ) > 0 )
        {
            std::stringstream ss;
            ss << "Source: " << key << " already registered!";
            throw std::invalid_argument( ss.str() );
        }

        _registry[ key ] = Buffer( _bufferLen );
    }
    
    void BufferData( const Key& key,
                     double t,
                     const Msg& m )
    {
        if( _registry.count( key ) == 0 )
        {
            std::stringstream ss;
            ss << "Source: " << key << " not registered!";
            throw std::invalid_argument( ss.str() );
        }

        Buffer& buff = _registry[ key ];
        if( !buff.empty() )
        {
            double lastTime = _registry[ key ].back().first;
            if( t < lastTime )
            {
                std::stringstream ss;
                ss << "Time " << t << " predecdes last time " << lastTime;
                throw std::invalid_argument( ss.str() );
            }
        }

        _registry[ key ].push_back( StampedData(t, m ) );
        CheckBuffers();
    }

    DataMap WaitForOutput()
    {
        WriteLock lock( _mutex );
        while( _outputBuff.empty() )
        {
            _hasOutput.wait( lock );
        }

        DataMap out = _outputBuff.front();
        _outputBuff.pop_front();
        return out;
    }

    bool HasOutput() const
    {
        WriteLock lock( _mutex );                
        return !_outputBuff.empty();
    }

    DataMap GetOutput()
    {
        WriteLock lock( _mutex );                
        if( _outputBuff.empty() )
        {
            throw std::runtime_error("Cannot get output from empty buffer!");
        }

        DataMap out = _outputBuff.front();
        _outputBuff.pop_front();
        return out;
    }

private:

    void CheckBuffers()
    {
        while( !AnyBuffersEmpty() )
        {
            DataMap oldest = GetOldest();
            
            Eigen::VectorXd times( oldest.size() );
            unsigned int i = 0;
            typedef typename DataMap::value_type Item;
            BOOST_FOREACH( const Item& item, oldest )
            {
                const StampedData& data = item.second;
                double t = data.first;
                times(i) = t;
                ++i;
            }

            double mostRecent = times.maxCoeff();
            double spread = mostRecent - times.minCoeff();
            if( spread > _maxDt )
            {
                RemoveBefore( mostRecent - _maxDt );
            }
            else
            {
                WriteLock lock( _mutex );                        
                _outputBuff.emplace_back(oldest);
                _hasOutput.notify_one();
                lock.release();

                RemoveOldest();
            }
        }
    }

    DataMap GetOldest() const
    {
        DataMap out;
        typedef typename SourceRegistry::value_type Item;
        BOOST_FOREACH( const Item& item, _registry )
        {
            const Key& key = item.first;
            const Buffer& buff = item.second;
            out[ key ] = buff.front();
        }
        return out;
    }

    bool AnyBuffersEmpty() const
    {
        typedef typename SourceRegistry::value_type Item;        
        BOOST_FOREACH( const Item& item, _registry )
        {
            const Buffer& buff = item.second;            
            if( buff.empty() ) { return true; }
        }
        return false;
    }

    /*! \brief Removes the oldest data from each buffer.*/
    void RemoveOldest()
    {
        typedef typename SourceRegistry::value_type Item;
        BOOST_FOREACH( Item& item, _registry )
        {
            Buffer& buff = item.second;
            buff.pop_front();
        }
    }

    /*! \brief Removes all stamped data with time before specified t. */
    void RemoveBefore( double t )
    {
        typedef typename SourceRegistry::value_type Item;
        BOOST_FOREACH( Item& item, _registry )
        {
            Buffer& buff = item.second;
            while( !buff.empty() && buff.front().first < t )
            {
                buff.pop_front();
            }
        }
    }

    mutable Mutex _mutex;
    ConditionVariable _hasOutput;

    typedef boost::circular_buffer<StampedData> Buffer;
    typedef std::map<Key, Buffer> SourceRegistry;
    SourceRegistry _registry;

    std::deque<DataMap> _outputBuff;

    // Parameters
    unsigned int _bufferLen;
    double _maxDt;

};

}