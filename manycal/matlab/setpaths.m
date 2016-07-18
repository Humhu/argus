
% location of this package
self_path          = [getenv('HOME') '/sandbox/wind_adaptation/wet/src/argus/manycal/matlab/'];

% location where the matlab rosbag utility is installed
matlabrosbag_path  = [getenv('HOME') '/software/matlab_rosbag-0.3-linux64/'];

% location of YAMLMatlab_0.4.3
yamlmatlab_path    = [getenv('HOME') '/software/YAMLMatlab_0.4.3/'];

% location of slam_test_suite package
slamtestsuite_path = [getenv('HOME') '/MEGA/slam_test_suite/'];

% location where bag files are saved
bag_path           = [getenv('HOME') '/bagfiles/'];

% location to save ros launch file
launchfile_path    = [self_path '../launch/'];

% location to save tag map
tagmap_path        = [self_path '../../../april_localizer/config/'];

addpath(genpath(self_path));
addpath(genpath(slamtestsuite_path));
addpath(yamlmatlab_path);
addpath(matlabrosbag_path);
addpath(bag_path);
