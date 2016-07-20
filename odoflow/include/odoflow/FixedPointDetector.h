#pragma once

#include "odoflow/InterestPointDetector.h"
#include "paraset/ParameterManager.hpp"

namespace argus
{

class FixedPointDetector 
: public InterestPointDetector
{
public:

	typedef std::shared_ptr<FixedPointDetector> Ptr;
	
	FixedPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	/*! \brief Return fixed interest points. 
		* Image is used only for size scaling. */
	virtual InterestPoints FindInterestPoints( const cv::Mat& image );
	
	/*! \brief Set the points returned by this finder. Points
		* should be specified [x,y] with each element between [0,1]
		* representing relative coordinates in the image. */
	void SetPoints( const InterestPoints& pts );
	
private:
	
	typedef std::pair<unsigned int, unsigned int> GridSize;

	GridSize _cachedGridSize;
	InterestPoints _cachedGrid;

	ParameterManager<std::pair<unsigned int, unsigned int>> _gridSize;

	void UpdateInterestPoints();
	
};
	
} // end namespace argus
