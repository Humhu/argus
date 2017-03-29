#pragma once

#include <linux/videodev2.h>
#include <iostream>
#include <tuple>
#include <vector>
#include <memory>

#include <opencv2/core/core.hpp>

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

#include "camplex/CameraTypes.h"

// TODO Enable setting controls
namespace camplex
{
	
	/*! \class CameraDriver CameraDriver.h
	* \brief A generic video capture wrapper around V4L2's interface.
	* It allows fine-grained querying and control of camera devices.
	* \note Read/write failures will throw a std::runtime_error.
	* \note libv4l2 does not seem to mutex buffer allocation/freeing, so those
	* calls should be protected if many cameras are being used. */
	class CameraDriver 
	{
	public:

		typedef std::shared_ptr<CameraDriver> Ptr;

		/*! \enum ReadMode CameraDriver.h
		* \brief ReadMode specifies if the device should block on accesses. */
		enum ReadMode { BLOCKING, NON_BLOCKING };

		CameraDriver();

		/*! \brief Releases all resources associated with the device and closes the descriptor. */
		~CameraDriver();

		/*! \brief Opens the device file descriptor. */
		void Open( const std::string& devPath, ReadMode m = BLOCKING );
		
		/*! \brief Returns whether the device is currently open. */
		bool IsOpen() const;

		/*! \brief Stops streaming and closes the device. */
		void Close();
		
		/*! \brief Returns whether the device is currently streaming. */
		bool IsStreaming() const;

		/*! \brief Reads capabilities struct for this device. Throws runtime_error
		 * if the device read fails. */
		CameraCapabilities ReadCapabilities();
		
		/*! \brief Reads all possible output specifications for this device. Throws
		 * runtime error if device read fails. */
		std::vector<OutputSpecification> ReadOutputSpecifications();
		
		/*! \brief Reads all possible control specifications for this device. Throws
		 * runtime error if device read fails. */
		std::vector<ControlSpecification> ReadControlSpecifications();

		/*! \brief Returns the currently active output specification for the device. */
		OutputSpecification ReadCurrentOutputSpecification();
		
		/*! \brief Set the output specification for the device. Throws runtime 
		 * error if setting fails. */
		void SetOutputSpecification( const OutputSpecification& spec );
		
		/*! \brief Read the value of a control. Throws runtime error if device
		 * read fails. */
		int ReadControl( unsigned int id );
		
		/*! \brief Set the value of a control. Throws runtime error if device
		 * write fails. */
		void SetControl( unsigned int id, int value );

		/*! \brief Set the camera streaming status. Throws runtime error if setting
		 * fails. */
		void SetStreaming( bool stream );

		/*! \brief Allocates the memory buffers for the camera driver. */
		size_t AllocateBuffers( size_t numBuffs );

		/*! \brief Retrieve this camera's device path. */
		std::string GetDevicePath() const;
		
		/*! \brief Retrieve the current read mode. */
		ReadMode GetReadMode() const;
		
		/*! \brief Attempts to get a filled frame from the camera. */
		cv::Mat GetFrame();

	protected:

		struct BufferInfo 
		{
			v4l2_buffer buffer;
			void* startAddress;
			bool isEnqueued;
		};

		typedef boost::mutex Mutex;
		typedef boost::unique_lock<Mutex> Lock;
		
		static Mutex classMutex;
		mutable Mutex mutex;
		
		int camFD;
		bool isOpen;
		bool isStreaming;
		bool buffersAllocated;
		std::string devicePath;
		ReadMode readMode;
		
		// Output buffer
		cv::Mat imbuff;
		
		OutputSpecification currentOutputSpec;

		// All the current buffer information structs
		std::vector<BufferInfo> bufferRegistry;

		std::vector<v4l2_fmtdesc> EnumerateFormats( Lock& lock );
		std::vector<v4l2_frmsizeenum> EnumerateFrameSizes( const v4l2_fmtdesc& format, Lock& lock  );
		std::vector<v4l2_frmivalenum> EnumerateFrameIntervals( const v4l2_frmsizeenum& size, Lock& lock  );

		void SetStreaming( bool stream, Lock& lock );

		/*! \brief Enqueues all unenqueued buffers. */
		void EnqueueBuffers( Lock& lock );

		/*! \brief Frees all buffers. */
		void FreeBuffers( Lock& lock );

		OutputSpecification QueryCurrentOutputSpecification( Lock& lock );

		/*! \brief Wraps v4l2_ioctl to retry on busy. Returns -1 on failure due to
		 * the ioctl failing or retries exceeded. */
		int retry_ioctl( int fd, int request, void* argp, unsigned int maxRetries=10 );

		void CheckExternalLock( Mutex& m, Lock& lock );
		
	};

	/*! \brief Helper function to initialize struct to all zero. Useful for many
	 V4L2 calls that require zero'd struct. */
	template <class T>
	void zero_struct(T& t) {
		memset( &t, 0, sizeof(T) );
	}
    
}
