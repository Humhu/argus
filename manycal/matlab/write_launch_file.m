clear;
clc;

setpaths;
commondata;

wf = fopen([launchfile_path 'localizetags.launch'],'w');

a = tag_size/2;

fprintf(wf, '<launch>\n');

for tt = 0:num_tags-1
    fprintf(wf, '<node pkg="lookup" type="lookup_registrar" name="tag%d_registrar">\n',tt);
    fprintf(wf, '    <rosparam>\n');
    fprintf(wf, '        target_name: apriltag_%s_id%d\n', tag_family, tt);
    fprintf(wf, '        target_namespace: tag%d\n',tt);
    fprintf(wf, '    </rosparam>\n');
    fprintf(wf, '</node>\n\n');
end

fprintf(wf, '<rosparam>\n');
for tt = 0:num_tags-1
    fprintf(wf, 'tag%d:\n',tt);
    fprintf(wf, '  frame_id: calibration_frame\n');
    if tt == 0
        fprintf(wf, '  extrinsics:\n');
        fprintf(wf, '    position:\n');
        fprintf(wf, '      x: 0\n');
        fprintf(wf, '      y: 0\n');
        fprintf(wf, '      z: 0\n');
        fprintf(wf, '    orientation:\n');
        fprintf(wf, '      yaw: 0\n');
        fprintf(wf, '      pitch: 0\n');
        fprintf(wf, '      roll: 0\n');
    end
    fprintf(wf, '  intrinsics:\n');
    fprintf(wf, '    points_x: [0, 0, 0, 0]\n');
    fprintf(wf, '    points_y: [%4.3f, %4.3f, %4.3f, %4.3f]\n', -a, a, a, -a);
    fprintf(wf, '    points_z: [%4.3f, %4.3f, %4.3f, %4.3f]\n\n', -a, -a, a, a);
end
fprintf(wf, '</rosparam>\n\n');

fprintf(wf, '<node pkg="atags" type="detector_node" name="detector">\n');

fprintf(wf, '  <remap from="image" to="%s/image_rect"/>\n', cam_name);
fprintf(wf, '  <remap from="~/detections_raw" to="detections_raw"/>\n');
fprintf(wf, '  <remap from="~/detections_processed" to="detections_processed"/>\n');
fprintf(wf, '  <rosparam>\n');
fprintf(wf, '    enable_undistortion: false\n');
fprintf(wf, '    enable_normalization: true\n');
fprintf(wf, '    tag_family: %s\n', tag_family);
fprintf(wf, '    fx: %4.3f\n', K(1,1));
fprintf(wf, '    fy: %4.3f\n', K(2,2));
fprintf(wf, '    cx: %4.3f\n', K(1,3));
fprintf(wf, '    cy: %4.3f\n', K(2,3));
fprintf(wf, '    width: %d\n', width);
fprintf(wf, '    height: %d\n', height);
fprintf(wf, '  </rosparam>\n');
fprintf(wf, '</node>\n\n');

fprintf(wf, '<node pkg="manycal" type="fiducial_array_calibrator_node" name="calibrator" output="screen">\n');
fprintf(wf, '  <remap from="detections" to="detections_processed"/>\n');
fprintf(wf, '    <rosparam>\n');
fprintf(wf, '      source_camera: %s\n', cam_name);
fprintf(wf, '      reference_frame: calibration_frame\n');
fprintf(wf, '      batch_period: %d\n', batch_period);
fprintf(wf, '    </rosparam>\n');
fprintf(wf, '</node>\n\n');

fprintf(wf, '</launch>\n');
fclose(wf);
