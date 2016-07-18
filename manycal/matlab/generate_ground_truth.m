clear;
close all;
clc;

% This script computes ground truth locations of april tag corners from a
% bag file that contains a vicon/Subject topic with the corner markers
% (assume that the corner markers are selected in order of increasing tag
% ID number, starting from 0, and the order of dots within each tag is
% clockwise starting from the northwest corner).

setpaths;

bag_name = 'danaus08_160717_handcarry_updown';
num_tags = 3;

bagfile = [bag_path bag_name '.bag'];
bag = ros.Bag(bagfile);
data = bag.readAll('/vicon/tripleapril');

tagcoords = cell(1,num_tags);

for cc = 1:4*num_tags
    tmp = cell2mat(cellfun(@(x) x.markers(cc).position, data, 'UniformOutput',false));
    tagcoords{ceil(cc/4)}(:,ceil(mod(cc-0.01,4))) = tmp(:,1);
end

% Switch up ordering of tag corners from CW starting at NW to CCW starting at SW
for tt = 1:numel(tagcoords)
   tagcoords{tt} = fliplr(tagcoords{tt}); 
end    

figure;
hold on;
for tt = 1:numel(tagcoords)
    plot3(tagcoords{tt}(1,[4 1:3]),tagcoords{tt}(2,[4 1:3]),tagcoords{tt}(3,[4 1:3]),'.-k');
    plot3(tagcoords{tt}(1,3:4),tagcoords{tt}(2,3:4),tagcoords{tt}(3,3:4),'-r');
    text(mean(tagcoords{tt}(1,:)),mean(tagcoords{tt}(2,:)),mean(tagcoords{tt}(3,:)),num2str(tt-1));
end
grid on; box on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');

save([getenv('HOME') '/MEGA/ISER16/data/apriltagcoords.mat'],'tagcoords');