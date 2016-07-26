close all;
clear;
clc;

setpaths;
commondata;

write_file = true;
use_ground_truth = true;

data = ReadYaml([self_path '../launch/dump.yaml']);

qmat = nan(4,num_tags);
pmat = nan(3,num_tags);

dimstr = 'xyz';

validind = 1:num_tags;

for tt = validind
    exdata = data.(['tag' num2str(tt-1)]).extrinsics;
    qmat(4,tt) = str2double(exdata.orientation.qw);
    for dd = 1:3
        qmat(dd,tt) = str2double(exdata.orientation.(['q' dimstr(dd)]));
        pmat(dd,tt) = str2double(exdata.position.(dimstr(dd)));
    end
end

a = tag_size/2;
rtagmap = nan(3,4,num_tags);

%% Draw Raw iSAM Output
figure;
hold on;
for tt = validind
    tmp = drawAprilTag(pmat(:,tt),QuatToR(qmat(:,tt)),tag_size,tt-1);
    rtagmap(:,:,tt) = tmp;
end
grid on; box on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Raw iSAM Output');

%% Assign an Arbitrary World Frame
% w is the GTSAM result reference frame
% v is the frame we want to fly with respect to
P = reshape(rtagmap(:,:,validind),[3, size(rtagmap,2)*numel(validind)]);

if exist('tagcoords','var') && use_ground_truth % ground truth data exists

    populatedind = find(~cellfun(@isempty,tagcoords));
    
    P = reshape(rtagmap(:,:,populatedind),[3, size(rtagmap,2)*numel(populatedind)]);
    Q = cell2mat(tagcoords(populatedind));
    
    [Rvw,Tvw] = getTransformBetweenCorrespondingPoints(P,Q);

    % transform all coordinates to this new world frame
    vtagmap = nan(size(rtagmap));

    for ii = 1:num_tags
        tmp = [Rvw Tvw; 0 0 0 1]*[rtagmap(:,:,ii); ones(1,4)];
        vtagmap(:,:,ii) = tmp(1:3,:);
    end

else % otherwise, arbitrarily assign a reference frame

    % first transform from humphrey's frame convention (w) to my frame
    % convention (u)
    Ruw = [0 0 1;
             0 1 0;
            -1 0 0];
    Tuw = mean(P,2);

    utagmap = nan(size(rtagmap));

    for ii = 1:num_tags
        tmp = [Ruw' -Ruw'*Tuw; 0 0 0 1]*[rtagmap(:,:,ii); ones(1,4)];
        utagmap(:,:,ii) = tmp(1:3,:);
    end

    % This transform makes it much easier to choose the final arbitrary
    % world frame (v)
    figure;
    hold on;
    for tt = validind
        plot3(utagmap(1,3:4,tt),utagmap(2,3:4,tt),utagmap(3,3:4,tt),'-r');
        plot3(utagmap(1,[4 1:3],tt),utagmap(2,[4 1:3],tt),utagmap(3,[4 1:3],tt),'.-k');
        text(mean(utagmap(1,:,tt)),mean(utagmap(2,:,tt)),mean(utagmap(3,:,tt)),num2str(tt-1));
    end
    grid on; box on; axis equal;
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title('iSAM Output in Intermediate Reference Frame');

    % form the rotation matrix Rvu = [e1 e2 e3]
    % let e3 be the normal of the least-squares plane
    U = reshape(utagmap(:,:,validind),[3 size(utagmap,2)*numel(validind)]);
    coeffs = fitLeastSquaresPlane(U);
    e3 = normc(coeffs(1:3)');

    % set the direction of the e1 and e2 axes while ensuring that they lie
    % in the least-squares plane
    theta = 0;
    e1 = normc([cos(theta) sin(theta) (e3(1)*cos(theta)+e3(2)*sin(theta))/e3(3)]');
    e2 = cross(e3,e1);
    Rvu = [e1 e2 e3];

    Tvu = [0.2; -0.2; 0.7];

    vtagmap = nan(size(utagmap));

    for ii = 1:num_tags
        tmp = [Rvu' Tvu; 0 0 0 1]*[utagmap(:,:,ii); ones(1,4)];
        vtagmap(:,:,ii) = tmp(1:3,:);
    end
end

%% Plot In World Frame
figure;
hold on;
for tt = validind
    plot3(vtagmap(1,3:4,tt),vtagmap(2,3:4,tt),vtagmap(3,3:4,tt),'-r');
    plot3(vtagmap(1,[4 1:3],tt),vtagmap(2,[4 1:3],tt),vtagmap(3,[4 1:3],tt),'.-k');
    text(mean(vtagmap(1,:,tt)),mean(vtagmap(2,:,tt)),mean(vtagmap(3,:,tt)),num2str(tt-1));
    if exist('tagcoords','var') && ~isempty(tagcoords{tt})
        plot3(tagcoords{tt}(1,3:4),tagcoords{tt}(2,3:4),tagcoords{tt}(3,3:4),'-m');
        plot3(tagcoords{tt}(1,[4 1:3]),tagcoords{tt}(2,[4 1:3]),tagcoords{tt}(3,[4 1:3]),'.-g');
        text(mean(tagcoords{tt}(1,:)),mean(tagcoords{tt}(2,:)),mean(tagcoords{tt}(3,:)),num2str(tt-1));
    end
end
grid on; box on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
if exist('tagcoords','var')
    title('iSAM (Black+Red) vs. Vicon (Green+Magenta)');
else
    title('iSAM Result In Arbitrary World Frame');
end
view([-40 20]);

%% Generate CSV File
% 5 columns, tagID (0-based), corner number (0-based), x, y, z
if write_file
    wfile = fopen([tagmap_path 'tagmap.txt'],'w');

    for tt = validind
        for ii = 1:4
            fprintf(wfile, '%3.5f , %3.5f , %3.5f , %3.5f , %3.5f \n',...
                    tt-1, ii-1, vtagmap(1,ii,tt), vtagmap(2,ii,tt), vtagmap(3,ii,tt));
            fprintf('%3.5f , %3.5f , %3.5f , %3.5f , %3.5f \n',...
                    tt-1, ii-1, vtagmap(1,ii,tt), vtagmap(2,ii,tt), vtagmap(3,ii,tt));
        end
    end
    fclose(wfile);
end
