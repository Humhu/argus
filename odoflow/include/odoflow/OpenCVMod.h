#pragma once

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <Eigen/Dense>

#include "odoflow/InterestPointDetector.h"

namespace cv
{
	/*! \brief Modified from OpenCV 3.0's recoverPose to return triangulated
	 * points and take a distance threshold argument. */
	int RecoverPose( InputArray E, InputArray points1, InputArray points2,
	                 OutputArray R, OutputArray t, OutputArray triangulated,
	                 double focal = 1.0, Point2d pp = Point2d(0, 0),
	                 InputOutputArray mask = noArray(),
	                 double dist = 50.0 );
}

namespace argus
{
	
	/*! \brief Converts a Nx1x2 or 1xNx2 mat into a vector of points. */
	InterestPoints ParsePointMatrix( const cv::Mat& mat );
	
	/*! \brief Converts a vector of points into a Nx1x2 mat. */
	cv::Mat ParsePointVector( const InterestPoints& points );
	
	/*! \brief Copies the contents of an OpenCV matrix to an Eigen matrix.
	 * \note Eigen::Map fails when using OpenCV ROIs! */
	template<class D, int height, int width>
	Eigen::Map< Eigen::Matrix<D, height, width, Eigen::RowMajor> >
	MatToEigenMap( cv::Mat& mat )
	{
		typedef Eigen::Map< Eigen::Matrix<D, height, width, Eigen::RowMajor> > Map;
		Map map( mat.ptr<D>() );
		return map;
	}
	
	template<class D, int height, int width>
	Eigen::Matrix<D, height, width>
	MatToEigen( cv::Mat& mat )
	{
		Eigen::Matrix<D, height, width> ret;
		for( unsigned int i = 0; i < height; i++ )
		{
			for( unsigned int j = 0; j < width; j++ )
			{
				ret(i,j) = mat.at<D>(i,j);
			}
		}
		return ret;
	}
	
	/*! \brief Copies the contents of an OpenCV vector to an Eigen matrix. Note
	 * that this differs from MatToEigen since Eigen row-major errors for vectors */
	template<class D, int height>
	Eigen::Map< Eigen::Matrix<D, height, 1, Eigen::ColMajor> >
	VecToEigenMap( cv::Mat& mat )
	{
		typedef Eigen::Map< Eigen::Matrix<D, height, 1, Eigen::ColMajor> > Map;
		Map map( mat.ptr<D>() );
		return map;
	}
	
	/*! \brief Copies the contents of an OpenCV vector to an Eigen matrix. Note
	 * that this differs from MatToEigen since Eigen row-major errors for vectors */
	template<class D, int height>
	Eigen::Matrix<D, height, 1>
	VecToEigen( cv::Mat& mat )
	{
		Eigen::Matrix<D, height, 1> ret;
		for( unsigned int i = 0; i < height; i++ )
		{
			ret(i) = mat.at<D>(i);
		}
		return ret;
	}
	
}
