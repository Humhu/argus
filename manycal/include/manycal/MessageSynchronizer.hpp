#pragma once

#include <deque>
#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>
#include <sstream>
#include <stdexcept>

namespace argus
{

/*! \brief Synchronizes buffers of timestamped data within some amount of tolerance.
 * NOTE: Assumes that data arrives in temporal order.
 */
template <typename M>
class MessageSynchronizer
{
public:

    typedef std::pair<double, M> StampedData;
    typedef std::map<std::string, StampedData> DataMap;

    MessageSynchronizer( unsigned int buffLen = 10, double maxDt = 0.1 ) 
    : _bufferLen( buffLen ), _maxDt( maxDt ) {}

    void SetBufferLength( unsigned int buffLen )
    {
        _bufferLen = buffLen;
    }

    void SetMaxDt( double dt )
    {
        _maxDt = dt;
    }

    void RegisterSource( const std::string& name )
    {
        if( _registry.count(name) > 0 )
        {
            throw std::invalid_argument( "Source: " + name + " already registered!" );
        }

        _registry[name] = Buffer( _bufferLen );
    }
    
    void BufferData( const std::string& name,
                     double t,
                     const M& m )
    {
        if( _registry.count(name) == 0 )
        {
            throw std::invalid_argument( "Source: " + name + " not registered!" );
        }

        Buffer& buff = _registry[name];
        if( !buff.empty() )
        {
            double lastTime = _registry[name].back().first;
            if( t < lastTime )
            {
                std::stringstream ss;
                ss << "Time " << t << " predecdes last time " << lastTime;
                throw std::invalid_argument( ss.str() );
            }
        }

        _registry[name].push_back( StampedData(t, m ) );
        CheckBuffers();
    }

    bool HasOutput() const
    {
        return !_outputBuff.empty();
    }

    DataMap GetOutput()
    {
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
                _outputBuff.emplace_back(oldest);
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
            const std::string& name = item.first;
            const Buffer& buff = item.second;
            out[name] = buff.front();
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

    typedef boost::circular_buffer<StampedData> Buffer;
    typedef std::map<std::string, Buffer> SourceRegistry;
    SourceRegistry _registry;

    std::deque<DataMap> _outputBuff;

    // Parameters
    unsigned int _bufferLen;
    double _maxDt;

};

}