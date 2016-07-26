clear;
close all;
clc;

% This script computes ground truth locations of selected april tag corners
% from a bag file that contains a vicon/Subject topic with the corner
% markers. The number of tags represented in the vicon model is LESS THAN
% the total number of tags, making this a PARTIAL ground truth.
% (assume that the corner markers are selected in order of increasing tag
% ID number, starting from 0, and the order of dots within each tag is
% counterclockwise starting from the southwest corner).

setpaths;

bag_name = 'danaus08_160725_linemap';

num_total_tags = 28;

tag_ids = [0 9 12 17];
num_tags = numel(tag_ids);

bagfile = [bag_path bag_name '.bag'];
bag = ros.Bag(bagfile);
data = bag.readAll('/vicon/quadrupleapril');

% tag ids with no ground truth remain empty
tagcoords = cell(1,num_total_tags);

for cc = 1:4*num_tags
    tmp = cell2mat(cellfun(@(x) x.markers(cc).position, data, 'UniformOutput',false));
    fprintf('cc = %d, ceil(cc/4) = %d \n', cc, ceil(cc/4));
    tagcoords{tag_ids(ceil(cc/4))+1}(:,ceil(mod(cc-0.01,4))) = tmp(:,1);
end

figure;
hold on;
for tt = (tag_ids+1)
    plot3(tagcoords{tt}(1,[4 1:3]),tagcoords{tt}(2,[4 1:3]),tagcoords{tt}(3,[4 1:3]),'.-k');
    plot3(tagcoords{tt}(1,3:4),tagcoords{tt}(2,3:4),tagcoords{tt}(3,3:4),'-r');
    text(mean(tagcoords{tt}(1,:)),mean(tagcoords{tt}(2,:)),mean(tagcoords{tt}(3,:)),num2str(tt-1));
end
grid on; box on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');

save([self_path 'tagcoords.mat']);
