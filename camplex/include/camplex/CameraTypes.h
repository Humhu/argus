#pragma once

#include <linux/videodev2.h>

#include <iostream>

namespace camplex 
{

	/*! \struct FourCC Camera.h
	* \brief FourCC wraps the four character code representation of video codec and formats. */
	struct FourCC 
	{
		unsigned char a;
		unsigned char b;
		unsigned char c;
		unsigned char d;
		unsigned int code;

		FourCC(unsigned int cc);
		FourCC(unsigned char _a, unsigned char _b, unsigned char _c, unsigned char _d);

		bool operator==( const FourCC& other ) const;
	};

	/*! \struct Version Camera.h
	* \brief Version provides a major, minor, and subrelease encoding for releases. */
	struct Version 
	{
		unsigned int maj;
		unsigned int min;
		unsigned int sub;

		Version(unsigned char _maj, unsigned char _min, unsigned char _sub);
	};

	std::ostream& operator<<( std::ostream& os, const FourCC& cc );
	std::ostream& operator<<( std::ostream& os, const Version& v );

	/*! \struct Fraction CameraTypes.h
	 * \brief Fractions for framerates. */
	struct Fraction 
	{
		int numerator;
		int denominator;

		Fraction();
		Fraction( int num, int den );
	};

	std::ostream& operator<<( std::ostream& os, const Fraction& f );
	
	/*! \brief Width, height tuple. */
	typedef std::pair <size_t, size_t> FrameSize;

	/*! \struct CameraCapabilities CameraProperties.h
	* \brief Describes a camera's overall capabilities. */
	struct CameraCapabilities 
	{
		std::string driver;
		std::string card;
		std::string busInfo;
		Version version;
		bool supportsReadWrite;
		bool supportsStreaming;
		bool supportsTimePerFrame;

		CameraCapabilities();
		CameraCapabilities( const v4l2_capability& caps );
	};

	/*! \struct OutputSpecification CameraProperties.h
	* \brief Describes a camera output format. */
	struct OutputSpecification 
	{
		std::string formatDescription;
		FourCC pixelFormat;
		FrameSize frameSize;
		Fraction framePeriod;

		size_t bytesPerLine;
		size_t bytesPerImage;

		OutputSpecification();
		OutputSpecification( const v4l2_fmtdesc& fmt, const v4l2_frmsizeenum& sizing,
							const v4l2_frmivalenum& interval );
		OutputSpecification( const std::string& description, const v4l2_format& form,
							const v4l2_streamparm& sparm );

		v4l2_format GetFormatStruct() const;
		v4l2_streamparm GetStreamParameterStruct() const;
	};

	/*! \struct ControlSpecification CameraProperties.h
	 * \brief Describes a camera control property. */
	struct ControlSpecification 
	{
		unsigned int id;
		unsigned int type;
		std::string name;
		int minVal;
		int maxVal;
		int stepSize;
		int defaultVal;
		int flags;
		bool disabled;
		bool readOnly;
		bool writeOnly;
		bool isVolatile;

		ControlSpecification( const v4l2_queryctrl& ctrl );
		
		static std::string TypeToString( unsigned int type );
	};

	std::ostream& operator<<( std::ostream& os, const CameraCapabilities& cap );
	std::ostream& operator<<( std::ostream& os, const OutputSpecification& spec );
	std::ostream& operator<<( std::ostream& os, const ControlSpecification& spec );

}
