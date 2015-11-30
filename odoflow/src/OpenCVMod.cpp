#include "odoflow/OpenCVMod.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/utility.hpp>

#include <iostream>

int cv::RecoverPose( InputArray E, InputArray _points1, InputArray _points2, OutputArray _R,
                     OutputArray _t, OutputArray triangulated, double focal, 
					 Point2d pp, InputOutputArray _mask, double dist )
{

	Mat points1, points2;
    _points1.getMat().copyTo(points1);
    _points2.getMat().copyTo(points2);

    int npoints = points1.checkVector(2);
    CV_Assert( npoints >= 0 && points2.checkVector(2) == npoints &&
                              points1.type() == points2.type());

    if (points1.channels() > 1)
    {
        points1 = points1.reshape(1, npoints);
        points2 = points2.reshape(1, npoints);
    }
    points1.convertTo(points1, CV_64F);
    points2.convertTo(points2, CV_64F);

    points1.col(0) = (points1.col(0) - pp.x) / focal;
    points2.col(0) = (points2.col(0) - pp.x) / focal;
    points1.col(1) = (points1.col(1) - pp.y) / focal;
    points2.col(1) = (points2.col(1) - pp.y) / focal;

    points1 = points1.t();
    points2 = points2.t();

    Mat R1, R2, t;
    decomposeEssentialMat(E, R1, R2, t);
    Mat P0 = Mat::eye(3, 4, R1.type());
    Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), P3(3, 4, R1.type()), P4(3, 4, R1.type());
    P1(Range::all(), Range(0, 3)) = R1 * 1.0; P1.col(3) = t * 1.0;
    P2(Range::all(), Range(0, 3)) = R2 * 1.0; P2.col(3) = t * 1.0;
    P3(Range::all(), Range(0, 3)) = R1 * 1.0; P3.col(3) = -t * 1.0;
    P4(Range::all(), Range(0, 3)) = R2 * 1.0; P4.col(3) = -t * 1.0;

    // Do the cheirality check.
    // Notice here a threshold dist is used to filter
    // out far away points (i.e. infinite points) since
    // there depth may vary between postive and negtive.
//     double dist = 50.0;
    Mat X1, X2, X3, X4;
	Mat Q1, Q2, Q3, Q4;
    triangulatePoints(P0, P1, points1, points2, X1);
    Mat mask1 = X1.row(2).mul(X1.row(3)) > 0;
    X1.row(0) /= X1.row(3);
    X1.row(1) /= X1.row(3);
    X1.row(2) /= X1.row(3);
    X1.row(3) /= X1.row(3);
    mask1 = (X1.row(2) < dist) & mask1;
    Q1 = P1 * X1;
    mask1 = (Q1.row(2) > 0) & mask1;
    mask1 = (Q1.row(2) < dist) & mask1;

    triangulatePoints(P0, P2, points1, points2, X2);
    Mat mask2 = X2.row(2).mul(X2.row(3)) > 0;
    X2.row(0) /= X2.row(3);
    X2.row(1) /= X2.row(3);
    X2.row(2) /= X2.row(3);
    X2.row(3) /= X2.row(3);
    mask2 = (X2.row(2) < dist) & mask2;
    Q2 = P2 * X2;
    mask2 = (Q2.row(2) > 0) & mask2;
    mask2 = (Q2.row(2) < dist) & mask2;

    triangulatePoints(P0, P3, points1, points2, X3);
    Mat mask3 = X3.row(2).mul(X3.row(3)) > 0;
    X3.row(0) /= X3.row(3);
    X3.row(1) /= X3.row(3);
    X3.row(2) /= X3.row(3);
    X3.row(3) /= X3.row(3);
    mask3 = (X3.row(2) < dist) & mask3;
    Q3 = P3 * X3;
    mask3 = (Q3.row(2) > 0) & mask3;
    mask3 = (Q3.row(2) < dist) & mask3;

    triangulatePoints(P0, P4, points1, points2, X4);
    Mat mask4 = X4.row(2).mul(X4.row(3)) > 0;
    X4.row(0) /= X4.row(3);
    X4.row(1) /= X4.row(3);
    X4.row(2) /= X4.row(3);
    X4.row(3) /= X4.row(3);
    mask4 = (X4.row(2) < dist) & mask4;
    Q4 = P4 * X4;
    mask4 = (Q4.row(2) > 0) & mask4;
    mask4 = (Q4.row(2) < dist) & mask4;

    mask1 = mask1.t();
    mask2 = mask2.t();
    mask3 = mask3.t();
    mask4 = mask4.t();

    // If _mask is given, then use it to filter outliers.
    if (!_mask.empty())
    {
        Mat mask = _mask.getMat();
        CV_Assert(mask.size() == mask1.size());
        bitwise_and(mask, mask1, mask1);
        bitwise_and(mask, mask2, mask2);
        bitwise_and(mask, mask3, mask3);
        bitwise_and(mask, mask4, mask4);
    }
    if (_mask.empty() && _mask.needed())
    {
        _mask.create(mask1.size(), CV_8U);
    }

    CV_Assert(_R.needed() && _t.needed());
    _R.create(3, 3, R1.type());
    _t.create(3, 1, t.type());

    int good1 = countNonZero(mask1);
    int good2 = countNonZero(mask2);
    int good3 = countNonZero(mask3);
    int good4 = countNonZero(mask4);

    if (good1 >= good2 && good1 >= good3 && good1 >= good4)
    {
        R1.copyTo(_R);
        t.copyTo(_t);
        if (_mask.needed()) mask1.copyTo(_mask);
		Q1.copyTo(triangulated);
        return good1;
    }
    else if (good2 >= good1 && good2 >= good3 && good2 >= good4)
    {
        R2.copyTo(_R);
        t.copyTo(_t);
        if (_mask.needed()) mask2.copyTo(_mask);
		Q2.copyTo(triangulated);
        return good2;
    }
    else if (good3 >= good1 && good3 >= good2 && good3 >= good4)
    {
        t = -t;
        R1.copyTo(_R);
        t.copyTo(_t);
        if (_mask.needed()) mask3.copyTo(_mask);
		Q3.copyTo(triangulated);
        return good3;
    }
    else
    {
        t = -t;
        R2.copyTo(_R);
        t.copyTo(_t);
        if (_mask.needed()) mask4.copyTo(_mask);
		Q4.copyTo(triangulated);
        return good4;
    }
}

namespace odoflow
{
InterestPoints ParsePointMatrix( const cv::Mat& mat )
{
	InterestPoints points( mat.cols*mat.rows );
	if( mat.type() == CV_32FC2 )
	{
		cv::Vec2f p;
		if( mat.rows == 1 )
		{
			for( unsigned int i = 0; i < mat.cols; i++ )
			{
				p = mat.at<cv::Vec2f>(0,i);
				points[i] = InterestPoint( p[0], p[1] );
			}
		}
		else
		{
			for( unsigned int i = 0; i < mat.rows; i++ )
			{
				p = mat.at<cv::Vec2f>(i,0);
				points[i] = InterestPoint( p[0], p[1] );
			}
		}
	}
	else if( mat.type() == CV_64FC2 )
	{
		cv::Vec2d p;
		if( mat.rows == 1 )
		{
			for( unsigned int i = 0; i < mat.cols; i++ )
			{
				p = mat.at<cv::Vec2d>(0,i);
				points[i] = InterestPoint( p[0], p[1] );
			}
		}
		else
		{
			for( unsigned int i = 0; i < mat.rows; i++ )
			{
				p = mat.at<cv::Vec2d>(i,0);
				points[i] = InterestPoint( p[0], p[1] );
			}
		}
	}
	else
	{
		throw std::runtime_error( "Invalid mat type." );
	}
	return points;
}

cv::Mat ParsePointVector( const InterestPoints& points )
{
	
	cv::Mat mat( points.size(), 1, CV_64FC2 );
	for( unsigned int i = 0; i < points.size(); i++ )
	{
		cv::Vec2d p;
		p[0] = points[i].x;
		p[1] = points[i].y;
		mat.at<cv::Vec2d>(i,0) = p;
	}
	return mat;
}
}
