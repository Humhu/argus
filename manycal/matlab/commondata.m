tag_size = 0.17;
tag_family = '36h11';
num_tags = 28;
batch_period = 10;
cam_name = 'mono';

data = ReadYaml([self_path '../../../april_localizer/config/raw_pinhole.yaml']);

K = [data.K11 0 data.K13;
     0 data.K22 data.K23;
     0 0 1];

% K = [346.2293 0 367.1517;
%      0 223.7621 242.8826;
%      0 0 1];

width = 752;
height = 480;

% location of vicon ground truth (optional)
groundtruthfile = [self_path 'tagcoords.mat'];
if exist(groundtruthfile,'file')
    load(groundtruthfile);
    num_tags = numel(tagcoords);
end
