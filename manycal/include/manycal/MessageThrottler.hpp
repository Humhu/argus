#pragma once

#include <deque>
#include <cmath>
#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>
#include "argus_utils/synchronization/SynchronizationTypes.h"

namespace argus
{

/*! \brief Weighted subsampling and delaying of message streams to 
 * achieve a target message rate. 
 * // NOTE Accessing outputs is synchronized, but setting parameters is not!
 */
 template <typename Msg, typename Key = std::string>
 class MessageThrottler
 {
public:

    typedef std::pair<Key, Msg> KeyedData;

    MessageThrottler( double rate = 10.0, unsigned int buffLen = 10 ) 
    {
        SetTargetRate( rate );
    }

    void SetTargetRate( double rate )
    {
        if( rate < 0 )
        {
            throw std::invalid_argument( "Rate must be positive." );
        }
        _overallRate = rate;
        ComputeBufferRates();
    }

    void SetBufferLength( unsigned int buffLen )
    {
        if( buffLen != _bufferLen && _registry.size() > 0 )
        {
            std::cerr << "Warning: Changing buffer length does not modify existing buffers." << std::endl;
        }
        _bufferLen = buffLen;
    }

    void RegisterSource( const Key& key )
    {
        CheckStatus( key, false );
        _registry.emplace( std::piecewise_construct,
                           std::forward_as_tuple( key ), 
                           std::forward_as_tuple( _bufferLen ) );
        ComputeBufferRates();
    }

    void SetSourceWeight( const Key& key, double w )
    {
        CheckStatus( key, true );
        if( w < 0 )
        {
            throw std::invalid_argument( "Weights must be positive." );
        }
        _registry.at( key ).weight = w;
        ComputeBufferRates();
    }

    void BufferData( const Key& key,
                     const Msg& m )
    {
        CheckStatus( key, true );
        WriteLock lock( _mutex );
        _registry.at( key ).Buffer( m );
    }

    bool GetOutput( double now, KeyedData& out )
    {
        typedef typename SourceRegistry::value_type Item;

        WriteLock outlock( _mutex );

        Key highestKey;
        double highestScore = 0;
        BOOST_FOREACH( const Item& item, _registry )
        {
            double score = item.second.ComputeNumToOutput( now );
            if( score > highestScore )
            {
                highestScore = score;
                highestKey = item.first;
            }
        }

        // If no scores were nonzero, there are no outputs to be had!
        if( highestScore == 0 )
        {
            return false;
        }

        Msg m = _registry.at( highestKey ).PopAndMark( now );
        out = KeyedData( highestKey, m );
        return true;
    }

private:

    // Compute the bandwidth allocations for each buffer
    void ComputeBufferRates()
    {
        typedef typename SourceRegistry::value_type Item;
        
        double sumWeights = 0;
        BOOST_FOREACH( const Item& item, _registry )
        {
            sumWeights += item.second.weight;
        }
        if( sumWeights == 0 ) { sumWeights = 1.0; }
        BOOST_FOREACH( Item& item, _registry )
        {
            SourceRegistration& reg = item.second;
            reg.rate = _overallRate * reg.weight / sumWeights; 
        }
    }

    void CheckStatus( const std::string& key, bool expect_reg )
    {
        bool is_reg = _registry.count( key ) > 0;
        if( expect_reg != is_reg )
        {
            std::stringstream ss;
            ss << "Source: " << key << (expect_reg ? " not registered!" : " already_registered");
            throw std::invalid_argument( ss.str() );
        }
    }

    // TODO Clean up public/private here
    struct SourceRegistration
    {
        mutable Mutex mutex;
        boost::circular_buffer<Msg> buffer;
        double weight;
        double rate;
        double lastOutputTime;

        SourceRegistration( unsigned int len )
        : mutex(), buffer( len ), weight( 0 ), rate( 0 ),
          lastOutputTime( -std::numeric_limits<double>::infinity() ) {}

        double ComputeNumToOutput( double now ) const
        {
            ReadLock lock( mutex );
            
            double elapsed = now - lastOutputTime;
            if( elapsed < 0 )
            {
                throw std::runtime_error( "Negative time elapsed!" );
            }
            double maxOutput = elapsed * rate;
            double numBuffered = double( buffer.size() );
            return std::floor( std::min( maxOutput, numBuffered ) );
        }

        void Buffer( const Msg& m )
        {
            WriteLock lock( mutex );
            buffer.push_back( m );
        }

        Msg PopAndMark( double now )
        {
            WriteLock lock( mutex );

            lastOutputTime = now;
            Msg m = buffer.front();
            buffer.pop_front();
            return m;
        }
    };

    mutable Mutex _mutex;

    typedef std::map<Key, SourceRegistration> SourceRegistry;
    SourceRegistry _registry;

    double _lastOutput;
    std::deque<KeyedData> _outputBuffer;

    // Parameters
    unsigned int _bufferLen;
    double _overallRate;
 };

}