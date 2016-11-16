#pragma once

#include "argus_utils/utils/LinalgTypes.h"

namespace argus
{

/*! \brief Information from a filter predict step used to learn
 * noise models. */
struct PredictInfo
{
	VectorType xpre; // State before predict step
	MatrixType Spre; // State covariance before predict step
	double dt; // The predict time step size
	MatrixType F; // Matrix used to propagate covariance
	MatrixType Q; // The Q matrix used in this step
};

/*! \brief Information from a filter update step used to learn 
 * noise models. */
struct UpdateInfo
{
	VectorType xpre; // State before update step
	MatrixType Spre; // State covariance before update
	VectorType innovation; // Observation prediction error
	VectorType observation;
	VectorType post_innovation;
	VectorType delta_x;
	MatrixType Spost;
	MatrixType H; // Matrix used to map state covariance to observation
	MatrixType R; // The R matrix used in this step
};

}