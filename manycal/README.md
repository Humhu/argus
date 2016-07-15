# manycal
Package for extrinsics/intrinsics calibration of sensor arrays.

## Matlab Dependencies:
* Ben Charrow's <code>matlab_rosbag</code> library: https://github.com/bcharrow/matlab_rosbag
* John Yao's <code>slam_test_suite</code>: git@gitlab.rasl.ri.cmu.edu:johnyao/slam_test_suite.git
* <code>YAMLMatlab_0.4.3</code> from yamlmatlab.googlecode.com/files/YAMLMatlab_0.4.3.zip

## Matlab Setup:
1. Edit <code>matlab/setpaths.m</code> to point to the locations of dependencies on your system, as well as bag file storage and output locations.
2. Edit <code>matlab/commondata.m</code> as necessary and run it to auto-generate the <code>launch/localizetags.launch</code> file.

## Get Positions of April Tag Corners from Bag File of Images:
1. Collect a bag file with rectified image and camera info messages where the camera passes over all april tags in the environment. Ensure that there are at least 2 tags visible in each camera frame.
2. Start roscore in a terminal.
3. In another terminal, run

        roslaunch manycal localizetags.launch

4. In a third terminal, rosbag play the bag file.
5. Go back to the second terminal. When the optimization is finished, the printous of 'Iteration...' should cease. At this point, Ctrl+C.
6. Dump the parameters to a yaml file

        rosparam dump nameoffile.yaml

7. Run <code>matlab/process_isam_results.m</code> to obtain the world frame positions of april tag corners. If no ground truth is available, you must edit the matlab script to arbitrarily set the world frame.
8. Ctrl+C roscore in the first terminal.
