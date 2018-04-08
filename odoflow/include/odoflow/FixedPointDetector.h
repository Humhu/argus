#pragma once

#include "odoflow/InterestPointDetector.h"
#include "paraset/ParameterManager.hpp"

namespace argus
{
/*! \brief Returns a fixed grid of interest points.
 */
class FixedPointDetector
	: public InterestPointDetector
{
public:

	typedef std::shared_ptr<FixedPointDetector> Ptr;

	FixedPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph );

	/*! \brief Return fixed interest points. Image dimensions
	 * are used for coordinate  scaling.
	 */
	virtual InterestPoints FindInterestPoints( const cv::Mat& image );

	/*! \brief Set the points returned by this finder. Points
	 * should be specified [x,y] with each element between [0,1]
	 * representing relative coordinates in the image.
	 * */
	void SetPoints( const InterestPoints& pts );

private:

	typedef std::pair<unsigned int, unsigned int> GridSize;

	GridSize _cachedGridSize;
	InterestPoints _cachedGrid;

	NumericParam _gridDim;

	void UpdateInterestPoints();
};
} // end namespace argus
