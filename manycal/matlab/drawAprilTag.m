function [tagcoords, gvec] = drawAprilTag(pos,rot,tag_size,tag_id)
% gvec = drawAprilTag(pos,rot,tag_size,tag_id)
% This function draws an April Tag using plot3 assuming the following tag
% reference frame:
% origin is at center, x axis pointing at the viewer,
% y axis pointing right, z axis pointing up

a = tag_size/2;

gvec = gobjects(1,3);

tagcoords = nan(3,4);

tagcoords(:,1) = pos - rot(:,3)*a - rot(:,2)*a;
tagcoords(:,2) = pos - rot(:,3)*a + rot(:,2)*a;
tagcoords(:,3) = pos + rot(:,3)*a + rot(:,2)*a;
tagcoords(:,4) = pos + rot(:,3)*a - rot(:,2)*a;

gvec(1) = plot3(tagcoords(1,3:4),tagcoords(2,3:4),tagcoords(3,3:4),'-r');
gvec(2) = plot3(tagcoords(1,[4 1:3]),tagcoords(2,[4 1:3]),tagcoords(3,[4 1:3]),'-k');
gvec(3) = text(mean(tagcoords(1,:)),mean(tagcoords(2,:)),mean(tagcoords(3,:)),num2str(tag_id));

end
