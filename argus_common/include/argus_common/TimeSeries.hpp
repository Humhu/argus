#ifndef _TIME_SERIES_H_
#define _TIME_SERIES_H_

#include <vector>
#include <map>

#include "argus_common/ArgusTypes.h"

namespace argus_common {

	/*! \class TimeSeries TimeSeries.h
		* \brief Represents a series of items ordered by time. */
	template <class C>
	class TimeSeries {
	public:

		typedef std::shared_ptr< TimeSeries<C> > Ptr;
		typedef std::map< Time, C > ContentMap;
		struct Timepoint
		{
			Time time;
			C point;
			
			Timepoint() {}
			
			Timepoint( typename ContentMap::const_iterator item )
				: time( item->first ), point( item->second )
			{}
			
			Timepoint( typename ContentMap::const_reverse_iterator item )
				: time( item->first ), point( item->second )
			{}
		};

		typedef C ContentType;
		
		TimeSeries() {}

		Timepoint GetFirst() const 
		{
			return Timepoint( map.begin() );
		}

		Timepoint GetLast() const 
		{
			return Timepoint( map.rbegin() );
		}

		/*! \brief Adds a timepoint. */
		void AddTimepoint( Time time, C content ) 
		{
			map [time] = content;
		}
		
		Timepoint GetExact( Time t ) const
		{
			return Timepoint( map.find( t ) );
		}
		
		Timepoint GetClosestLower( Time t ) const 
		{
			typename ContentMap::const_iterator iter = map.lower_bound( t );
			iter--; // Have to subtract one because lower_bound gives lowest key _not_ before query
			return Timepoint( iter );
		}
		
		Timepoint GetClosestUpper( Time t ) const 
		{
			typename ContentMap::const_iterator iter = map.lower_bound( t );
			if( iter == map.end() ) {
				return Timepoint( map.begin() );
			}
			
			return Timepoint( iter );
		}

	private:
		
		ContentMap map;
		
	};

}

#endif
