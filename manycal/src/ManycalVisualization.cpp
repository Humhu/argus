#include "manycal/ManycalVisualization.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"

#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"

#include <boost/foreach.hpp>

#define POSE_ID (0)
#define LABEL_ID (1)
#define FID_ID (2)

namespace argus
{

geometry_msgs::Point vec3_to_point( const Eigen::Vector3d& vec )
{
    geometry_msgs::Point p;
    p.x = vec(0);
    p.y = vec(1);
    p.z = vec(2);
    return p;
}

geometry_msgs::Point trans_to_point( const Translation3Type& trans )
{
    geometry_msgs::Point p;
    p.x = trans.x();
    p.y = trans.y();
    p.z = trans.z();
    return p;
}

Visualizer::Visualizer() 
{
    SetAlpha( 1.0 );
    SetColor( 1.0, 1.0, 1.0 );
    
    SetTextAlpha( 1.0 );
    SetTextColor( 1.0, 1.0, 1.0 );
    SetTextOffset( PoseSE3( 0, 0, -0.1, 1, 0, 0, 0 ) );
    SetTextSize( 0.1 );
    
    SetFrameID( "" );
    SetMarkerName( "marker" );
    SetShowName( false );
}

void Visualizer::ReadParams( const ros::NodeHandle& nh )
{
    double a;
    double r, g, b;
    PoseSE3 off;
    std::string s;
    bool t;

    if( GetParam( nh, "alpha", a ) ) { SetAlpha( a ); }
    if( GetParam( nh, "red", r ) &&
        GetParam( nh, "green", g ) &&
        GetParam( nh, "blue", b ) ) { SetColor( r, g, b ); }
    
    if( GetParam( nh, "text_alpha", a ) ) { SetTextAlpha( a ); }
    if( GetParam( nh, "text_red", r ) &&
        GetParam( nh, "text_green", g ) &&
        GetParam( nh, "text_blue", b ) ) { SetTextColor( r, g, b ); }
    if( GetParam( nh, "text_offset", off ) ) { SetTextOffset( off ); }
    if( GetParam( nh, "text_size", a ) ) { SetTextSize( a ); }

    if( GetParam( nh, "frame_id", s ) ) { SetFrameID( s ); }
    if( GetParam( nh, "marker_name", s ) ) { SetMarkerName( s ); }
    if( GetParam( nh, "show_name", t ) ) { SetShowName( t ); }
}

void Visualizer::SetAlpha( double a )
{
    if( a < 0 || a > 1.0 )
    {
        throw std::invalid_argument( "Alpha must be between 0 and 1." );
    }
    _alpha = a;
}

void Visualizer::SetColor( double r, double g, double b )
{
    if( r < 0 || r > 1 || g < 0 || g > 1 || b < 0 || b > 1 )
    {
        throw std::invalid_argument( "Colors must be between 0 and 1" );
    }
    _r = r;
    _g = g;
    _b = b;
}

void Visualizer::SetTextAlpha( double a )
{
    if( a < 0 || a > 1.0 )
    {
        throw std::invalid_argument( "Alpha must be between 0 and 1." );
    }
    _alphaText = a;
}

void Visualizer::SetTextColor( double r, double g, double b )
{
    if( r < 0 || r > 1 || g < 0 || g > 1 || b < 0 || b > 1 )
    {
        throw std::invalid_argument( "Colors must be between 0 and 1" );
    }
    _rText = r;
    _gText = g;
    _bText = b;
}

void Visualizer::SetTextOffset( const PoseSE3& off )
{
    _offsetText = off;
}

void Visualizer::SetTextSize( double h )
{
    if( h < 0 )
    {
        throw std::invalid_argument( "Text size must be positive." );
    }
    _hText = h;
}

void Visualizer::SetFrameID( const std::string& id )
{
    _frameID = id;
}

void Visualizer::SetMarkerName( const std::string& n )
{
    _markerName = n;
}

void Visualizer::SetShowName( bool e )
{
    _showText = e;
}

MarkerMsg Visualizer::InitMarker() const
{
    MarkerMsg marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = _frameID;
    marker.action = MarkerMsg::ADD;
    marker.ns = _markerName;
    marker.color.r = _r;
    marker.color.g = _g;
    marker.color.b = _b;
    marker.color.a = _alpha;
    return marker;
}

void Visualizer::AddNameMarker( const std::string& name,
                                const PoseSE3& pose,
                                std::vector<MarkerMsg>& markers ) const
{
    if( !_showText || name.empty() ) { return; }

    MarkerMsg marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = _frameID;
    marker.action = MarkerMsg::ADD;
    marker.ns = name;
    marker.id = LABEL_ID;
    marker.type = MarkerMsg::TEXT_VIEW_FACING;
    marker.color.r = _rText;
    marker.color.g = _gText;
    marker.color.b = _bText;
    marker.color.a = _alphaText;
    marker.text = name;
    marker.scale.z = _hText;
    marker.pose = PoseToMsg( pose * _offsetText );
    markers.push_back( marker );
}

PoseVisualizer::PoseVisualizer()
{
     SetLinewidth( 1.0 );
     SetAlpha( 1.0 );
     SetAxesLength( 1.0 );
}

void PoseVisualizer::ReadParams( const ros::NodeHandle& nh )
{
    Visualizer::ReadParams( nh );
    double a;
    if( GetParam( nh, "linewidth", a ) ) { SetLinewidth( a ); }
    if( GetParam( nh, "axes_length", a ) ) { SetAxesLength( a ); }
}

void PoseVisualizer::SetLinewidth( double a )
{
    if( a < 0 )
    {
        throw std::invalid_argument( "Linewidth must be positive" );
    }
    _linewidth = a;
}

void PoseVisualizer::SetAxesLength( double l )
{
    if( l < 0 )
    {
        throw std::invalid_argument( "Axes length must be positive" );
    }
    _axesLength = l;
}

std::vector<MarkerMsg> PoseVisualizer::ToMarkers( const PoseSE3& pose,
                                                  const std::string& name ) const
{
    MarkerMsg marker = InitMarker();
    marker.id = POSE_ID;
    marker.type = MarkerMsg::LINE_LIST;
    marker.scale.x = _linewidth;

    AddColors( marker.colors );
    AddPosePoints( pose, marker.points );
    
    std::vector<MarkerMsg> markers;
    markers.push_back( marker );
    AddNameMarker( name, pose, markers );

    return markers;
}

std::vector<MarkerMsg> PoseVisualizer::ToMarkers( const std::vector<PoseSE3>& poses,
                                                  const std::vector<std::string>& names ) const
{
    if( !names.empty() && names.size() != poses.size() )
    {
        throw std::invalid_argument( "Must specify equal number of names and poses." );
    }

    std::vector<MarkerMsg> markers;
    if( poses.empty() ) { return markers; }

    MarkerMsg marker = InitMarker();
    marker.id = POSE_ID;
    marker.type = MarkerMsg::LINE_LIST;
    marker.scale.x = _linewidth;

    for( unsigned int i = 0; i < poses.size(); ++i )
    {
        const PoseSE3& pose = poses[i];
        AddColors( marker.colors );
        AddPosePoints( pose, marker.points );
        
        if( !names.empty() )
        {
            const std::string& name = names[i];
            AddNameMarker( name, pose, markers );
        }
    }
    
    markers.push_back( marker );
    return markers;
}

void PoseVisualizer::AddColors( std::vector<std_msgs::ColorRGBA>& colors ) const
{
    std_msgs::ColorRGBA r, g, b;
    r.r = 1; r.g = 0; r.b = 0;
    g.r = 0; g.g = 1; g.b = 0;
    b.r = 0; b.g = 0; b.b = 1;
    r.a = _alpha;
    g.a = _alpha;
    b.a = _alpha;    
    colors.push_back( r );
    colors.push_back( g );
    colors.push_back( b );
}

void PoseVisualizer::AddPosePoints( const PoseSE3& pose,
                                        std::vector<geometry_msgs::Point>& points ) const
{
    PoseSE3::Transform trans = pose.ToTransform();
    geometry_msgs::Point zero = vec3_to_point( trans * Eigen::Vector3d::Zero() );
    Eigen::Vector3d xTrans = trans * ( _axesLength * Eigen::Vector3d::UnitX() );
    Eigen::Vector3d yTrans = trans * ( _axesLength * Eigen::Vector3d::UnitY() );
    Eigen::Vector3d zTrans = trans * ( _axesLength * Eigen::Vector3d::UnitZ() );
    points.push_back( zero );
    points.push_back( vec3_to_point( xTrans ) );
    points.push_back( zero );    
    points.push_back( vec3_to_point( yTrans ) );
    points.push_back( zero );    
    points.push_back( vec3_to_point( zTrans ) );
}

FiducialVisualizer::FiducialVisualizer()
{
    SetPointSize( 0.1 );
    SetShowAxes( true );
}

void FiducialVisualizer::ReadParams( const ros::NodeHandle& nh )
{
    Visualizer::ReadParams( nh );
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
    marker.id = FID_ID;
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
    marker.id = FID_ID;
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
