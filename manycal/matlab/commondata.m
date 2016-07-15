tag_size = 0.17;
tag_family = '36h11';
num_tags = 17;
batch_period = 10;
cam_name = 'mono';
K = [346.2293 0 367.1517;
     0 223.7621 242.8826;
     0 0 1];
width = 752;
height = 480;

% location of vicon ground truth (optional)
groundtruthfile = [getenv('HOME') '/MEGA/ISER16/data/apriltagcoords.mat'];
if exist(groundtruthfile,'file')
   load(groundtruthfile);
end
