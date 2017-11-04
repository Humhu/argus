#pragma once

#include ""

namespace argus
{

typedef std::vector<FiducialDetection> FiducialDetections;

class MultiArrayCalibrator
{
public:

    MultiArrayCalibrator();

    void Initialize( ros::NodeHandle& ph );

    void BufferOdometry( ??? );
    void BufferDetection( const FiducialDetections& detections );

private:



};

}