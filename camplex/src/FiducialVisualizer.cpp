#include "camplex/FiducialVisualizer.h"

#include "geometry_msgs/Point.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"

#include <boost/foreach.hpp>

namespace argus
{

geometry_msgs::Point trans_to_point( const Translation3Type& trans )
{
    geometry_msgs::Point p;
    p.x = trans.x();
    p.y = trans.y();
    p.z = trans.z();
    return p;
}

FiducialVisualizer::FiducialVisualizer()
{
    SetPointSize( 0.1 );
    SetShowAxes( true );
}

void FiducialVisualizer::ReadParams( const ros::NodeHandle& nh )
{
    PoseVisualizer::ReadParams( nh );
    double a;
    bool b;
    if( GetParam( nh, "point_size", a ) ) { SetPointSize( a ); }
    if( GetParam( nh, "show_axes", b ) ) { SetShowAxes( b ); }
}

void FiducialVisualizer::SetPointSize( double s )
{
    if( s < 0 )
    {
        throw std::invalid_argument( "Point size must be positive." );
    }
    _pointSize = s;
}

void FiducialVisualizer::SetShowAxes( bool b )
{
    _showAxes = b;
}

std::vector<MarkerMsg> FiducialVisualizer::ToMarkers( const PoseSE3& pose,
                                                      const Fiducial& fid,
                                                      const std::string& name ) const
{
    MarkerMsg marker = InitMarker();
    marker.id = VisualizerID::FID_ID;
    marker.scale.x = _pointSize;
    marker.scale.y = _pointSize;
    marker.type = MarkerMsg::POINTS;

    AddPoints( fid.Transform( pose ), marker.points );
    std::vector<MarkerMsg> markers;
    markers.push_back( marker );

    if( _showAxes )
    {
        std::vector<MarkerMsg> axesMarkers = PoseVisualizer::ToMarkers( pose, name );
        markers.insert( markers.end(), axesMarkers.begin(), axesMarkers.end() );
    }

    return markers;
}

std::vector<MarkerMsg> FiducialVisualizer::ToMarkers( const std::vector<PoseSE3>& poses,
                                                      const std::vector<Fiducial>& fids,
                                                      const std::vector<std::string>& names ) const
{
    if( poses.size() != fids.size() || (!names.empty() && names.size() != poses.size()) )
    {
        throw std::invalid_argument( "Must specify equal number of names, poses, and fiducials." );
    }

    std::vector<MarkerMsg> markers;
    if( poses.empty() )
    {
        return markers;
    }

    MarkerMsg marker = InitMarker();
    marker.id = VisualizerID::FID_ID;
    marker.scale.x = _pointSize;
    marker.scale.y = _pointSize;
    marker.type = MarkerMsg::POINTS;

    for( unsigned int i = 0; i < poses.size(); ++i )
    {
        const PoseSE3& pose = poses[i];
        const Fiducial& fid = fids[i];
        AddPoints( fid.Transform( pose ), marker.points );
        if( !names.empty() )
        {
            const std::string& name = names[i];
            AddNameMarker( name, pose, markers );
        }

    }
    markers.push_back( marker );

    if( _showAxes )
    {
        std::vector<MarkerMsg> axesMarkers = PoseVisualizer::ToMarkers( poses, names );
        markers.insert( markers.end(), axesMarkers.begin(), axesMarkers.end() );
    }

    return markers;
}

void FiducialVisualizer::AddPoints( const Fiducial& fid,
                                    std::vector<geometry_msgs::Point>& points ) const
{
    BOOST_FOREACH( const Translation3Type& trans, fid.points )
    {
        points.push_back( trans_to_point( trans ) );
    }
}

}
