#include "camplex/CameraTypes.h"

#include <stdexcept>

#include <cstring>

#include <libv4l2.h>

#include <sys/ioctl.h>
#include <sys/mman.h>

namespace camplex 
{

	FourCC::FourCC( unsigned int cc ) 
		: a( (cc & 0x000000FF) >> 0 ), b( (cc & 0x0000FF00) >> 8 ),
		c( (cc & 0x00FF0000) >> 16 ), d( (cc & 0xFF000000) >> 24 ), code( cc ) 
	{}

	FourCC::FourCC( unsigned char _a, unsigned char _b, unsigned char _c, unsigned char _d ) 
		: a( _a ), b( _b ), c( _c ), d( _d ), 
		code( (((unsigned int) a << 0) | ((unsigned int) b << 8)
			  | ((unsigned int) c << 16) | ((unsigned int) d << 24)) ) 
	{}

	std::ostream& operator<<( std::ostream& os, const FourCC& cc ) 
	{
		os << "(" << cc.a << ", " << cc.b << ", " << cc.c << ", " << cc.d << ")";
		return os;
	}

	Version::Version( unsigned char _maj, unsigned char _min, unsigned char _sub ) 
		: maj( _maj ), min( _min ), sub( _sub ) 
	{}

	std::ostream& operator<<( std::ostream& os, const Version& v ) 
	{
		os << v.maj << "." << v.min << "." << v.sub;
		return os;
	}

	bool FourCC::operator==( const FourCC& other ) const 
	{
		return code == other.code;
	}

	Fraction::Fraction() 
		: numerator(0), denominator(0) 
	{}

	Fraction::Fraction( int num, int den ) 
		: numerator(num), denominator(den) 
	{}

	std::ostream& operator<<( std::ostream& os, const Fraction& f ) 
	{
		os << f.numerator << " / " << f.denominator;
		return os;
	}
	
	CameraCapabilities::CameraCapabilities() 
		: driver( "invalid" ), card( "invalid" ), busInfo( "invalid" ),
		version( 0, 0, 0 ), supportsReadWrite(false), supportsStreaming(false),
		supportsTimePerFrame(false) 
	{}

	CameraCapabilities::CameraCapabilities( const v4l2_capability& caps ) 
		: driver( reinterpret_cast<const char*>(caps.driver) ),
		card( reinterpret_cast<const char*>(caps.card) ),
		busInfo( reinterpret_cast<const char*>(caps.bus_info) ),
		version( (caps.version >> 16) & 0xFF, (caps.version >> 8) & 0xFF, caps.version & 0xFF),
		supportsReadWrite( V4L2_CAP_READWRITE && caps.capabilities ),
		supportsStreaming( V4L2_CAP_STREAMING && caps.capabilities ),
		supportsTimePerFrame( V4L2_CAP_TIMEPERFRAME && caps.capabilities ) 
	{}

	std::ostream& operator<<( std::ostream& os, const CameraCapabilities& cap ) 
	{
		os << "Driver: " << cap.driver << std::endl;
		os << "Card: " << cap.card << std::endl;
		os << "Bus: " << cap.busInfo << std::endl;
		os << "Version: " << cap.version << std::endl;
		if( cap.supportsReadWrite ) {
			os << "Supports read/write operations." << std::endl;
		}
		if( cap.supportsStreaming ) {
			os << "Supports streaming operations." << std::endl;
		}
		if( cap.supportsTimePerFrame ) {
			os << "Supports output period specification." << std::endl;
		}
		return os;
	}

	OutputSpecification::OutputSpecification() 
		: formatDescription( "invalid" ), pixelFormat( 0 ),
		frameSize( 0, 0 ), framePeriod( 0, 0 ), bytesPerLine(0), bytesPerImage(0) 
	{}

	OutputSpecification::OutputSpecification( const v4l2_fmtdesc& fmt,
													const v4l2_frmsizeenum& sizing,
													const v4l2_frmivalenum& interval ) 
		: formatDescription( reinterpret_cast<const char*>(fmt.description) ),
		pixelFormat( fmt.pixelformat ),
		frameSize( sizing.discrete.width, sizing.discrete.height ),
		framePeriod( interval.discrete.numerator, interval.discrete.denominator ),
		bytesPerLine(0), bytesPerImage(0) 
	{}

	OutputSpecification::OutputSpecification( const std::string& description, const v4l2_format& form,
								const v4l2_streamparm& sparm ) 
		: formatDescription( description ), pixelFormat( form.fmt.pix.pixelformat ),
		frameSize( form.fmt.pix.width, form.fmt.pix.height ),
		framePeriod( sparm.parm.capture.timeperframe.numerator,
					sparm.parm.capture.timeperframe.denominator ),
		bytesPerLine( form.fmt.pix.bytesperline ), bytesPerImage( form.fmt.pix.sizeimage ) 
	{}

	v4l2_format OutputSpecification::GetFormatStruct() const 
	{
		v4l2_format form;
		memset( &form, 0, sizeof(v4l2_format) );
		form.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		form.fmt.pix.width = frameSize.first;
		form.fmt.pix.height = frameSize.second;
		form.fmt.pix.pixelformat = pixelFormat.code;
		form.fmt.pix.field = V4L2_FIELD_ANY;
		form.fmt.pix.bytesperline = bytesPerLine;
		form.fmt.pix.sizeimage = bytesPerImage;
		return form;
	}

	v4l2_streamparm OutputSpecification::GetStreamParameterStruct() const 
	{
		v4l2_streamparm streamParameters;
		memset( &streamParameters, 0, sizeof(v4l2_streamparm) );
		streamParameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		streamParameters.parm.capture.timeperframe.numerator = framePeriod.numerator;
		streamParameters.parm.capture.timeperframe.denominator = framePeriod.denominator;
		return streamParameters;
	}

	std::ostream& operator<<( std::ostream& os, const OutputSpecification& spec ) 
	{
		os << "Description: " << spec.formatDescription << std::endl;
		os << "Format: " << spec.pixelFormat << std::endl;
		os << "Frame size: " << spec.frameSize.first << " by " << spec.frameSize.second << std::endl;
		os << "Frame period: " << spec.framePeriod.numerator << "/" << spec.framePeriod.denominator << " seconds." << std::endl;
		os << "Bytes per line: " << spec.bytesPerLine << " image: " << spec.bytesPerImage << std::endl;
		return os;
	}

	ControlSpecification::ControlSpecification( const v4l2_queryctrl& ctrl ) 
		: id( ctrl.id ), type( ctrl.type ), name( reinterpret_cast<const char*>(ctrl.name) ),
		minVal( ctrl.minimum ), maxVal( ctrl.maximum ), stepSize( ctrl.step ),
		defaultVal( ctrl.default_value ), flags( ctrl.flags ),
		disabled( ctrl.flags & V4L2_CTRL_FLAG_DISABLED ),
		readOnly( ctrl.flags & V4L2_CTRL_FLAG_READ_ONLY ),
		writeOnly( ctrl.flags & V4L2_CTRL_FLAG_WRITE_ONLY ),
		isVolatile( ctrl.flags & V4L2_CTRL_FLAG_VOLATILE ) 
	{}

	std::string ControlSpecification::TypeToString( unsigned int c ) {
		switch ( c )
		{
			case V4L2_CTRL_TYPE_INTEGER:
				return "integer";
			case V4L2_CTRL_TYPE_BOOLEAN:
				return "boolean";
			case V4L2_CTRL_TYPE_MENU:
				return "menu";
			case V4L2_CTRL_TYPE_INTEGER_MENU:
				return "integer_menu";
			case V4L2_CTRL_TYPE_BITMASK:
				return "bitmask";
			case V4L2_CTRL_TYPE_BUTTON:
				return "button";
			case V4L2_CTRL_TYPE_INTEGER64:
				return "integer64";
			case V4L2_CTRL_TYPE_STRING:
				return "string";
			case V4L2_CTRL_TYPE_CTRL_CLASS:
				return "control_class";
// 			case V4L2_CTRL_TYPE_U8:
// 				return "unsigned8_array";
// 			case V4L2_CTRL_TYPE_U16:
// 				return "unsigned16_array";
			default:
				throw std::runtime_error( "Invalid control specification type." );
		}
	}
	
	std::ostream& operator<<( std::ostream& os, const ControlSpecification& spec ) 
	{
		os << "ID: " << spec.id << std::endl;
		os << "Type: " << ControlSpecification::TypeToString(spec.type) << std::endl;
		os << "Name: " << spec.name << std::endl;
		os << "Max: " << spec.maxVal << std::endl;
		os << "Min: " << spec.minVal << std::endl;
		os << "Default: " << spec.defaultVal << std::endl;
		if( spec.disabled ) 
		{
			os << "Is disabled" << std::endl;
		}
		if( spec.readOnly ) 
		{
			os << "Is read-only" << std::endl;
		}
		if( spec.writeOnly ) 
		{
			os << "Is write-only" << std::endl;
		}
		if( spec.isVolatile ) 
		{
			os << "Is volatile" << std::endl;
		}
		return os;
	}

}

