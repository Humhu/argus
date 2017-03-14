#pragma once

#include <tuple>
#include <ros/ros.h>
#include "visualization_msgs/Marker.h"

#include "manycal/ManycalCommon.h"

namespace argus
{

typedef visualization_msgs::Marker MarkerMsg;

// TODO Base class for all visualizers?
class Visualizer
{
public:

    Visualizer();
    void ReadParams( const ros::NodeHandle& nh );

    virtual void SetColor( double r, double g, double b );
    virtual void SetAlpha( double a );

    virtual void SetTextColor( double r, double g, double b );
    virtual void SetTextAlpha( double a );
    virtual void SetTextOffset( const PoseSE3& off );
    virtual void SetTextSize( double h );
    
    virtual void SetFrameID( const std::string& id );
    virtual void SetMarkerName( const std::string& n );
    virtual void SetShowName( bool e );

    // TODO
    //virtual std::vector<MarkerMsg> ToMarkers() = 0;

protected:

    double _r;
    double _g;
    double _b;
    double _alpha;

    double _rText;
    double _gText;
    double _bText;
    double _alphaText;
    double _hText;
    PoseSE3 _offsetText;

    std::string _markerName;
    std::string _frameID;
    bool _showText;

    MarkerMsg InitMarker() const;
    void AddNameMarker( const std::string& name,
                        const PoseSE3& pose,
                        std::vector<MarkerMsg>& markers ) const;
};

/*! \brief Visualizes pose messages as colored tri-axes RGB-XYZ */
class PoseVisualizer : public Visualizer
{
public:

    PoseVisualizer();

    void ReadParams( const ros::NodeHandle& nh );

    void SetLinewidth( double a );
    void SetAxesLength( double l );

    std::vector<MarkerMsg> ToMarkers( const PoseSE3& pose,
                                      const std::string& name = "" ) const;
    std::vector<MarkerMsg> ToMarkers( const std::vector<PoseSE3>& pose,
                                      const std::vector<std::string>& names = {} ) const;

private:

    double _linewidth;
    double _axesLength;

    void AddColors( std::vector<std_msgs::ColorRGBA>& colors ) const;
    void AddPosePoints( const PoseSE3& pose,
                        std::vector<geometry_msgs::Point>& points ) const;
};

/*! \brief Visualizes fiducials as points. */
class FiducialVisualizer : public PoseVisualizer
{
public:

    FiducialVisualizer();
    void ReadParams( const ros::NodeHandle& nh );

    void SetPointSize( double s );
    void SetShowAxes( bool b );

    std::vector<MarkerMsg> ToMarkers( const PoseSE3& pose,
                                      const Fiducial& fid,
                                      const std::string& name = "" ) const;
    std::vector<MarkerMsg> ToMarkers( const std::vector<PoseSE3>& poses,
                                      const std::vector<Fiducial>& fids,
                                      const std::vector<std::string>& names = {} ) const;

private:

    double _pointSize;
    bool _showAxes;

    void AddPoints( const Fiducial& fid,
                    std::vector<geometry_msgs::Point>& points ) const;
};

}