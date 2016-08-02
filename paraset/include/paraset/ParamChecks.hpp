#pragma once

#include <memory>
#include <string>

namespace argus
{

template <typename T>
struct ProjectionTraits
{
	static T min_increment();
};

template <typename T>
struct ParameterCheck
{
public:

	typedef std::shared_ptr<ParameterCheck> Ptr;

	ParameterCheck() {}

	virtual T Project( const T& val ) const = 0;
	virtual std::string GetDescription() const = 0;

};

template <typename T>
struct GreaterThan
: public ParameterCheck<T>
{
public:

	// NOTE inclusive means whether equalling the limit is OK
	GreaterThan( const T& lim )
	: _lim( lim ) {}

	T Project( const T& val ) const
	{
		if( val > _lim ) { return val; }
		return _lim + ProjectionTraits<T>::min_increment();
	}

	std::string GetDescription() const
	{
		return "Strictly greater than " + std::to_string( _lim );
	}

private:

	T _lim;
};

template <typename T>
struct GreaterThanOrEqual
: public ParameterCheck<T>
{
public:

	// NOTE inclusive means whether equalling the limit is OK
	GreaterThanOrEqual( const T& lim )
	: _lim( lim ) {}

	T Project( const T& val ) const
	{
		if( val >= _lim ) { return val; }
		return _lim;
	}

	std::string GetDescription() const
	{
		return "Greater than or equal to " + std::to_string( _lim );
	}

private:

	T _lim;
};

template <typename T>
struct LessThan
: public ParameterCheck<T>
{
public:

	// NOTE inclusive means whether equalling the limit is OK
	LessThan( const T& lim )
	: _lim( lim ) {}

	T Project( const T& val ) const
	{
		if( val < _lim ) { return val; }
		return _lim - ProjectionTraits<T>::min_increment();
	}

	std::string GetDescription() const
	{
		return "Strictly less than " + std::to_string( _lim );
	}

private:

	T _lim;
};


template <typename T>
struct LessThanOrEqual
: public ParameterCheck<T>
{
public:

	// NOTE inclusive means whether equalling the limit is OK
	LessThanOrEqual( const T& lim )
	: _lim( lim ) {}

	T Project( const T& val ) const
	{
		if( val <= _lim ) { return val; }
		return _lim;
	}

	std::string GetDescription() const
	{
		return "Less than or equal to " + std::to_string( _lim );
	}

private:

	T _lim;
};

}