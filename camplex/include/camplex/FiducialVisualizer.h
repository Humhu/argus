#pragma once

#include "vizard/PoseVisualizer.h"

#include "camplex/FiducialCommon.h"

namespace argus
{

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