#include "camplex/CameraDriver.h"

#include <opencv2/highgui/highgui.hpp>

#include <cstring>
#include <stdexcept>

#include <boost/foreach.hpp>

#include <errno.h>
#include <fcntl.h>
#include <libv4l2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

namespace camplex
{

  CameraDriver::Mutex CameraDriver::classMutex;

	CameraDriver::CameraDriver() :
		camFD( -1 ), isOpen( false ), isStreaming( false ), buffersAllocated( false )
	{}

	CameraDriver::~CameraDriver() 
	{
		Close();
	}

	void CameraDriver::Open( const std::string& devPath, ReadMode m ) 
	{
		Lock lock(mutex);

		devicePath = devPath;
		readMode = m;
		
		switch(readMode) 
		{
			case BLOCKING:
				camFD = v4l2_open( devicePath.c_str(), O_RDWR );
				break;
			case NON_BLOCKING:
				// TODO NON_BLOCKING doesn't seem to work correctly
				camFD = v4l2_open( devicePath.c_str(), O_RDWR, O_NONBLOCK );
				break;
			default:
				throw std::runtime_error( "Invalid camera driver read mode." );
		}

		if( camFD == -1 ) 
		{
			throw std::runtime_error("Could not open device at " + devicePath);
		}
		isOpen = true;
	}

	bool CameraDriver::IsOpen() const
	{
		Lock lock(mutex);
		return isOpen;
	}

	void CameraDriver::Close() 
	{
		Lock lock(mutex);
		if( camFD == -1 ) { return; }
		
		if( isStreaming )
		{
			SetStreaming( false, lock );
		}
		
		FreeBuffers(lock);
		
		v4l2_close( camFD );
		camFD = -1;
		isOpen = false;
	}

	bool CameraDriver::IsStreaming() const
	{
		Lock lock(mutex);
		return isStreaming;
	}
	
	CameraCapabilities CameraDriver::ReadCapabilities() 
	{
		Lock lock(mutex);

		// Populate the capabilities struct using a V4L2 call
		v4l2_capability caps;
		zero_struct( caps );
		if( retry_ioctl( camFD, VIDIOC_QUERYCAP, &caps ) == -1 ) 
		{
			throw std::runtime_error("Could not read capabilities " + devicePath);
		}
		return CameraCapabilities(caps);
	}
	
	std::vector<OutputSpecification> CameraDriver::ReadOutputSpecifications()
	{
		Lock lock(mutex);
		
		std::vector<OutputSpecification> specs;
		
		// First get all the possible output formats for this camera
		std::vector<v4l2_fmtdesc> formats = EnumerateFormats( lock );
		
		BOOST_FOREACH( const v4l2_fmtdesc& format, formats ) 
		{
			std::vector<v4l2_frmsizeenum> sizes = EnumerateFrameSizes( format, lock );

			BOOST_FOREACH( const v4l2_frmsizeenum& size, sizes ) 
			{
				std::vector<v4l2_frmivalenum> intervals = EnumerateFrameIntervals( size, lock );

				BOOST_FOREACH( const v4l2_frmivalenum& interval, intervals ) 
				{
					specs.emplace_back(format, size, interval);
				}
			}
		}

		return specs;
	}
	
	std::vector<ControlSpecification> CameraDriver::ReadControlSpecifications() 
	{
		Lock lock(mutex);
		
		std::vector<ControlSpecification> specs;
		
		v4l2_queryctrl query;
		zero_struct( query );
		query.id = V4L2_CID_BASE - 1;
		while( true ) 
		{
			query.id = query.id | V4L2_CTRL_FLAG_NEXT_CTRL;
			if( retry_ioctl( camFD, VIDIOC_QUERYCTRL, &query ) == -1 ) 
			{
				if( errno == EINVAL ) { break; }
				throw std::runtime_error( "Could not query control." );
			}
			specs.emplace_back( query );
		}
		return specs;
	}
	
	int CameraDriver::ReadControl( unsigned int id )
	{
		Lock lock(mutex);
		
		v4l2_control query;
		zero_struct( query );
		query.id = id;
		
		if( retry_ioctl( camFD, VIDIOC_G_CTRL, &query ) == -1 )
		{
			throw std::runtime_error( "Could not read control." );
		}
		return query.value;
	}
	
	void CameraDriver::SetControl( unsigned int id, int value )
	{
		Lock lock(mutex);
		
		v4l2_control query;
		query.id = id;
		query.value = value;
		
		if( retry_ioctl( camFD, VIDIOC_S_CTRL, &query ) == -1 )
		{
			throw std::runtime_error( "Could not set control." );
		}
		
	}
	
	std::vector<v4l2_fmtdesc> CameraDriver::EnumerateFormats( Lock& lock ) 
	{
		CheckExternalLock( mutex, lock );
		
		std::vector<v4l2_fmtdesc> formats;
		unsigned int i = 0;
		while( true ) 
		{
			v4l2_fmtdesc format;
			zero_struct( format );
			format.index = i++;
			format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if( retry_ioctl( camFD, VIDIOC_ENUM_FMT, &format ) == -1 ) 
			{
				// EINVAL indicates we've enumerated all the formats
				if( errno == EINVAL) { break; }
				throw std::runtime_error( "Could not enumerate output formats " + devicePath );
			}
			formats.push_back( format );
		}
		return formats;
	}
	
	std::vector<v4l2_frmsizeenum> CameraDriver::EnumerateFrameSizes( const v4l2_fmtdesc& format,
																	 Lock& lock ) 
	{
		CheckExternalLock( mutex, lock );
		
		std::vector<v4l2_frmsizeenum> sizes;
		unsigned int i = 0;
		while( true ) 
		{
			v4l2_frmsizeenum size;
			memset( &size, 0, sizeof(v4l2_frmsizeenum) );
			size.index = i++;
			size.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			size.pixel_format = format.pixelformat;
			if( retry_ioctl( camFD, VIDIOC_ENUM_FRAMESIZES, &size ) == -1 ) 
			{
				// EINVAL indicates we've enumerated all the formats
				if( errno == EINVAL) { break; }
				throw std::runtime_error( "Could not enumerate output frame sizes " + devicePath );
			}
			sizes.push_back( size );
		}
		return sizes;
	}
	
	std::vector<v4l2_frmivalenum> CameraDriver::EnumerateFrameIntervals( const v4l2_frmsizeenum& size,
																		 Lock& lock ) 
	{
		CheckExternalLock( mutex, lock );
		
		std::vector<v4l2_frmivalenum> intervals;
		unsigned int i = 0;
		while( true ) 
		{
			v4l2_frmivalenum interval;
			memset( &interval, 0, sizeof(v4l2_frmivalenum) );
			interval.index = i++;
			interval.pixel_format = size.pixel_format;
			interval.width = size.discrete.width;
			interval.height = size.discrete.height;
			if( retry_ioctl( camFD, VIDIOC_ENUM_FRAMEINTERVALS, &interval ) == -1 ) 
			{
				// EINVAL indicates we've enumerated all the formats
				if( errno == EINVAL) { break; }
				throw std::runtime_error( "Could not enumerate output frame intervals " + devicePath );
			}
			intervals.push_back( interval );
		}
		return intervals;
	}

	OutputSpecification CameraDriver::ReadCurrentOutputSpecification() 
	{
		Lock lock(mutex);
		
		// Read the output format
		v4l2_format retFormat;
		zero_struct( retFormat );
		retFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if( retry_ioctl( camFD, VIDIOC_G_FMT, &retFormat ) == -1 ) 
		{
			throw std::runtime_error( "CameraDriver: Could not read format." );
		}
		
		// Read the framerate
		v4l2_streamparm retStreamParm;
		zero_struct( retStreamParm );
		retStreamParm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if( retry_ioctl( camFD, VIDIOC_G_PARM, &retStreamParm )  == -1 ) 
		{
			throw std::runtime_error("CameraDriver: Could not read streaming parameters.");
		}
		
		return OutputSpecification( "Current output format", retFormat, retStreamParm );
	}
	
	void CameraDriver::SetOutputSpecification( const OutputSpecification& spec ) 
	{
		Lock lock(mutex);
		
		// Set the output format
		v4l2_format retFormat = spec.GetFormatStruct();
		if( retry_ioctl( camFD, VIDIOC_S_FMT, &retFormat ) == -1 ) 
		{
			throw std::runtime_error( "CameraDriver: Could not set format." );
		}

		// Set the framerate
		v4l2_streamparm retStreamParm = spec.GetStreamParameterStruct();
		if( retry_ioctl( camFD, VIDIOC_S_PARM, &retStreamParm )  == -1 ) 
		{
			throw std::runtime_error("CameraDriver: Could not set streaming parameters.");
		}

		currentOutputSpec = OutputSpecification( spec.formatDescription,
												 retFormat, retStreamParm );
	}

	size_t CameraDriver::AllocateBuffers( size_t numBuffs ) 
	{
		Lock lock(mutex);
		Lock cLock(classMutex);

		// If buffers are already allocated, we have to first free them
		if( buffersAllocated )
		{
			FreeBuffers( lock );
		}
		
		// Double check that the registry is cleared
		if( bufferRegistry.size() != 0 ) 
		{
			throw std::runtime_error( "Unfreed buffers exist!" );
		}
		
		v4l2_requestbuffers reqbuf;
		memset( &reqbuf, 0, sizeof(v4l2_requestbuffers) );
		reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		reqbuf.memory = V4L2_MEMORY_MMAP;
		reqbuf.count = numBuffs;

		if( retry_ioctl( camFD, VIDIOC_REQBUFS, &reqbuf ) == -1 ) 
		{
			if(errno == EINVAL) 
			{
				throw std::runtime_error("CameraDriver: Device does not support streaming " + devicePath);
			} 
			else 
			{
				throw std::runtime_error("CameraDriver: Error allocating buffers.");
			}
		}

		if( reqbuf.count < numBuffs ) 
		{
			// TODO Warning?
		}

		for( size_t i = 0; i < reqbuf.count; i++ ) 
		{
			v4l2_buffer buffer;
			memset( &buffer, 0, sizeof(v4l2_buffer) );
			buffer.type = reqbuf.type;
			buffer.memory = V4L2_MEMORY_MMAP;
			buffer.index = i;
			if( retry_ioctl( camFD, VIDIOC_QUERYBUF, &buffer ) == -1 ) 
			{
				throw std::runtime_error("CameraDriver: Error querying buffer");
			}

			BufferInfo info;
			info.startAddress = v4l2_mmap( NULL, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, camFD,
							buffer.m.offset );
			info.buffer = buffer;
			info.isEnqueued = false;
			

			if( info.startAddress == MAP_FAILED ) 
			{
				FreeBuffers( lock );
				return 0;
			}

			bufferRegistry.push_back( info );

		}
		
		buffersAllocated = true;
		return reqbuf.count;

	}

	void CameraDriver::FreeBuffers( Lock& lock ) 
	{
		CheckExternalLock( mutex, lock );
		
		if( !buffersAllocated ) { return; }
		
		bool errored = false;
		BOOST_FOREACH( const BufferInfo& info, bufferRegistry ) 
		{
			if( info.isEnqueued ) 
			{
 				std::cout << "CameraDriver: Warning - freeing enqueued buffer." << std::endl;
			}
			int ret = v4l2_munmap( info.startAddress, info.buffer.length );
			if( ret == -1 ) 
			{
				errored = true;
			}
		}

		bufferRegistry.clear();
		if( errored )
		{
			throw std::runtime_error("CameraDriver:Error freeing buffer." );
		}
		buffersAllocated = false;
	}


	void CameraDriver::EnqueueBuffers( Lock& lock ) 
	{
		CheckExternalLock( mutex, lock );

		BOOST_FOREACH( BufferInfo& info, bufferRegistry ) 
		{
			if( !info.isEnqueued ) 
			{
				// NOTE QBUF sets the length field to the actual used length, 
				// but we need to remember the buffer length
				v4l2_buffer buffCopy = info.buffer; 
				if( retry_ioctl( camFD, VIDIOC_QBUF, &buffCopy ) == -1 ) 
				{
					throw std::runtime_error("CameraDriver: Error enqueueing buffer");
				}
				info.isEnqueued = true;
			}
		}
	}

	OutputSpecification CameraDriver::QueryCurrentOutputSpecification( Lock& lock ) 
	{
		CheckExternalLock( mutex, lock );
		v4l2_format format;
		zero_struct( format );
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if( -1 == retry_ioctl( camFD, VIDIOC_G_FMT, &format ) ) 
		{
// 			perror( "CameraDriver: Querying format" );
		};

		v4l2_streamparm streamparm;
		zero_struct( streamparm );
		streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		retry_ioctl( camFD, VIDIOC_G_PARM, &streamparm );

		OutputSpecification spec( "StatusQuery", format, streamparm );
		return spec;

	}

	void CameraDriver::SetStreaming( bool stream ) 
	{
		Lock lock(mutex);
		SetStreaming(stream, lock);
	}

	void CameraDriver::SetStreaming( bool stream, Lock& lock ) 
	{
		CheckExternalLock( mutex, lock );

		if( isStreaming == stream ) { return; }

		unsigned int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if( stream == true ) 
		{
			// Must enqueue buffers before starting streaming
			EnqueueBuffers( lock );

			if( retry_ioctl( camFD, VIDIOC_STREAMON, &type ) == -1 ) 
			{
				throw std::runtime_error("CameraDriver: Error turning on streaming.");
			}
		} 
		else 
		{
			if( retry_ioctl( camFD, VIDIOC_STREAMOFF, &type ) == -1 ) 
			{
				throw std::runtime_error("CameraDriver: Error turning off streaming.");
			}

			// Calling streamoff dequeues all queued buffers
			BOOST_FOREACH( BufferInfo& info, bufferRegistry ) 
			{
					info.isEnqueued = false;
			}
		}
		isStreaming = stream;
	}

	cv::Mat CameraDriver::GetFrame() 
	{
		Lock lock(mutex);

		imbuff = cv::Mat();
		if( !isOpen || !isStreaming ) 
		{
			return imbuff;
		}

		v4l2_buffer buffer;
		zero_struct( buffer );
		buffer.index = -1;
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;

		if( retry_ioctl( camFD, VIDIOC_DQBUF, &buffer, 1 ) == -1 ) 
		{
			if( errno == EAGAIN ) 
			{
				return imbuff;
			}
			throw std::runtime_error("CameraDriver: Error dequeueing buffer.");
		}

		// Set corresponding buffer registry info
		bufferRegistry[buffer.index].isEnqueued = false;

		void* bufferAddress = bufferRegistry[buffer.index].startAddress;

		// This operation does not copy data so we have to clone it!
		imbuff = cv::Mat( currentOutputSpec.frameSize.second, 
						  currentOutputSpec.frameSize.first,
						  CV_8UC3, bufferAddress );
		
 		cv::Mat image = imbuff.clone();
		
		if( retry_ioctl( camFD, VIDIOC_QBUF, &buffer ) == -1 ) 
		{
			throw std::runtime_error("CameraDriver: Error requeueing buffer.");
		}

		bufferRegistry[buffer.index].isEnqueued = true;

 		return image;
	}

	int CameraDriver::retry_ioctl( int fd, int request, void* argp, unsigned int maxRetries ) 
	{

		if( !isOpen )
		{
			throw std::runtime_error( "Cannot read/write to closed camera." );
		}
		
		for( unsigned int i = 0; i < maxRetries; i++ )
		{
			int retval = v4l2_ioctl( fd, request, argp );
			
			// If we get a valid return value, return it
			if( retval != -1 ) 
			{
				return retval;
			}
			// Else if the return valid is invalid and the device is not busy, 
			// the read has failed
			else if( retval == -1 && errno != EAGAIN ) 
			{
				return -1;
			}
		}
		
		// If we exceed the number of retries, return failure
		return -1;
		
	}
	
	void CameraDriver::CheckExternalLock( Mutex& m, Lock& lock ) 
	{
		if(!lock.owns_lock() || lock.mutex() != &m) {
			throw std::runtime_error( "Lock error: Wrong lock given!" );
		}
	}

}

