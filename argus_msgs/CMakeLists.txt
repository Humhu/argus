cmake_minimum_required(VERSION 2.8.3)
project(argus_msgs)

find_package(catkin REQUIRED COMPONENTS
                    message_generation
                    geometry_msgs
                    std_msgs
                    sensor_msgs
)

## Generate service files
add_message_files( FILES
                  CompactOdometry.msg
                  CompactOdometryArray.msg
                  CompactPoseWithCovariance.msg
                  CompactTwistWithCovariance.msg
                  EstimatePerformance.msg
                  FiducialDetection.msg
                  FilterStepInfo.msg
                  FilterUpdate.msg
                  FilterUpdateBounds.msg
                  FloatVectorStamped.msg
                  ImageFiducialDetections.msg
                  MatrixFloat64.msg
                  OdometryBounds.msg
                  Point2D.msg
                  RelativePose.msg
                  RelativePoseWithCovariance.msg
                  SymmetricFloat64.msg 
)

generate_messages( DEPENDENCIES 
                   std_msgs
                   sensor_msgs
)

catkin_package( CATKIN_DEPENDS message_runtime 
                               std_msgs 
                               sensor_msgs
)
