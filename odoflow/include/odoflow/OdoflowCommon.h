#pragma once

#include <opencv2/core.hpp>

#include "argus_utils/geometry/PoseSE2.h"
#include "camplex/CameraCalibration.h"

namespace argus
{

/*! \brief Typedef for describing a collection of image points.
*/
typedef cv::Point2d InterestPoint;
typedef std::vector<InterestPoint> InterestPoints;

typedef cv::Point2f InterestPointf;
typedef std::vector<InterestPointf> InterestPointsf;

/*! \brief Convenience method for printing a collection of image points. 
*/
std::ostream& operator<<( std::ostream& os, const InterestPoints& points );

InterestPointsf DowncastInterestPoints( const InterestPoints& in );
InterestPoints UpcastInterestPoints( const InterestPointsf& in );

// TODO Move into a utils package for open cv? Shared functionality with fiducials package

/*! \brief Undistorts points (performs lens correction)
*/
InterestPoints UndistortPoints( const InterestPoints& points,
                                const CameraCalibration& model );

/*! \brief Normalizes points from pixel coordinates to unit plane
*/
InterestPoints NormalizePoints( const InterestPoints& points,
                                const CameraCalibration& model );

/*! \brief Undistorts points and then converts to unit plane.
*/
InterestPoints UndistortAndNormalizePoints( const InterestPoints& points,
                                            const CameraCalibration& model );

/*! \brief Distort points in unit plane coordinates 
 */
InterestPoints DistortPoints( const InterestPoints& points,
                              const CameraCalibration& model );

/*! \brief Converts points from unit plane to pixel coordinates.
 */
InterestPoints UnnormalizePoints( const InterestPoints& points,
                                  const CameraCalibration& model );

/*! \brief Distort points in unit plane coordinates and converts
 * them to piel coordinates.
 */
InterestPoints DistortAndUnnormalizePoints( const InterestPoints& points,
                                            const CameraCalibration& model );

/*! \brief Applies a transform to image points.
 */
InterestPoints TransformPoints( const InterestPoints& points,
                                const PoseSE2& trans );
}
