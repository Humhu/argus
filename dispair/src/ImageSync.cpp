#include "dispair/ImageSync.h"

#include "argus_utils/utils/ParamUtils.h"

namespace argus
{
DisparityImageSync::DisparityImageSync( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: _imageTransport( nh )
{
	unsigned int imgQueueSize;
	std::string transport;
	GetParam( ph, "image_queue_size", imgQueueSize, (unsigned int) 3 );
	GetParam<std::string>( ph, "image_transport", transport, "raw" );

	_imageSub.subscribe( _imageTransport, "image", imgQueueSize, transport );
	_disparitySub.subscribe( nh, "disparity", imgQueueSize );
	_infoSub.subscribe( nh, "camera_info", imgQueueSize );

	unsigned int syncQueueSize;
	bool approximateSync;
	GetParam( ph, "sync_queue_size", syncQueueSize, (unsigned int) 5 );
	GetParam( ph, "approximate_sync", approximateSync, false );
	if( approximateSync )
	{
		_approxSync = std::make_shared<ApproximateSync>( ApproximatePolicy( syncQueueSize ),
		                                                 _imageSub,
		                                                 _disparitySub,
		                                                 _infoSub );
		_approxSync->registerCallback( boost::bind( &DisparityImageSync::DataCallback,
		                                            this, _1, _2, _3 ) );
	}
	else
	{
		_exactSync = std::make_shared<ExactSync>( ExactPolicy( syncQueueSize ),
		                                          _imageSub,
		                                          _disparitySub,
		                                          _infoSub );
		_exactSync->registerCallback( boost::bind( &DisparityImageSync::DataCallback,
		                                           this, _1, _2, _3 ) );
	}
}
}