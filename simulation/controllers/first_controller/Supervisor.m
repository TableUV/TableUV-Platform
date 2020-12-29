%% ROBOT SETUP
% Getting the nodes
c = containers.Map;
robot_settings = { ROBOT_NAME, 'ROBOT_WHEEL_LEFT', 'ROBOT_WHEEL_RIGHT', ...
                  'ROBOT_BASE','ROBOT_ENC_LEFT',   'ROBOT_ENC_RIGHT',   ...
                  'ROBOT_TOF'};
for index = 1:length(robot_settings)
    setting = char(robot_settings(index));
    node = wb_supervisor_node_get_from_def(setting);
    if isNull(node)
      wb_console_print(sprintf('No DEF %s node found in the current world file', char(robot_settings(index))), WB_STDOUT);
      quit(1);
    end
    c(char(robot_settings(index))) = node;
end

% Robot True position
trans_field = wb_supervisor_node_get_field(c(ROBOT_NAME), 'translation');

% Robot left wheel
left_wheel_rad = wb_supervisor_node_get_field(c('ROBOT_WHEEL_LEFT'), 'radius');
right_wheel_rad = wb_supervisor_node_get_field(c('ROBOT_WHEEL_RIGHT'), 'radius');
wb_supervisor_field_set_sf_float(left_wheel_rad, WHEEL_RADIUS);
wb_supervisor_field_set_sf_float(right_wheel_rad, WHEEL_RADIUS);

% Robot base 
robot_base_rad = wb_supervisor_node_get_field(c('ROBOT_BASE'), 'radius');
robot_height = wb_supervisor_node_get_field(c('ROBOT_BASE'), 'height');
wb_supervisor_field_set_sf_float(robot_base_rad, ROBOT_BASE_RADIUS);
wb_supervisor_field_set_sf_float(robot_height, ROBOT_BASE_HEIGHT);

% Encoders
robot_left_enc_noise = wb_supervisor_node_get_field(c('ROBOT_ENC_LEFT'), 'noise');
robot_left_enc_res = wb_supervisor_node_get_field(c('ROBOT_ENC_LEFT'), 'resolution');
robot_right_enc_noise = wb_supervisor_node_get_field(c('ROBOT_ENC_RIGHT'), 'noise');
robot_right_enc_res = wb_supervisor_node_get_field(c('ROBOT_ENC_RIGHT'), 'resolution');
wb_supervisor_field_set_sf_float(robot_left_enc_noise, ENC_NOISE);
wb_supervisor_field_set_sf_float(robot_left_enc_res, ENC_RESOLUTION);
wb_supervisor_field_set_sf_float(robot_right_enc_noise, ENC_NOISE);
wb_supervisor_field_set_sf_float(robot_right_enc_res, ENC_RESOLUTION);

% Range Finder (ToF)
robot_tof_fov = wb_supervisor_node_get_field(c('ROBOT_TOF'), 'fieldOfView');
robot_tof_width = wb_supervisor_node_get_field(c('ROBOT_TOF'), 'width');
robot_tof_height = wb_supervisor_node_get_field(c('ROBOT_TOF'), 'height');
robot_tof_spherical = wb_supervisor_node_get_field(c('ROBOT_TOF'), 'spherical');
robot_tof_near = wb_supervisor_node_get_field(c('ROBOT_TOF'), 'near');
robot_tof_minR = wb_supervisor_node_get_field(c('ROBOT_TOF'), 'minRange');
robot_tof_maxR = wb_supervisor_node_get_field(c('ROBOT_TOF'), 'maxRange');
robot_tof_blur = wb_supervisor_node_get_field(c('ROBOT_TOF'), 'motionBlur');
robot_tof_noise = wb_supervisor_node_get_field(c('ROBOT_TOF'), 'noise');
robot_tof_res = wb_supervisor_node_get_field(c('ROBOT_TOF'), 'resolution');
wb_supervisor_field_set_sf_float(robot_tof_fov, TOF_FOV);
wb_supervisor_field_set_sf_int32(robot_tof_width, TOF_WIDTH);
wb_supervisor_field_set_sf_int32(robot_tof_height, TOF_HEIGHT);
wb_supervisor_field_set_sf_bool(robot_tof_spherical, TOF_IS_SPHERICAL);
wb_supervisor_field_set_sf_float(robot_tof_near, TOF_NEAR);
wb_supervisor_field_set_sf_float(robot_tof_minR, TOF_MIN_RANGE);
wb_supervisor_field_set_sf_float(robot_tof_maxR, TOF_MAX_RANGE);
wb_supervisor_field_set_sf_float(robot_tof_blur, TOF_MOTION_BLUR);
wb_supervisor_field_set_sf_float(robot_tof_noise, TOF_NOISE);
wb_supervisor_field_set_sf_float(robot_tof_res, TOF_RESOLUTION);

% IMU, Gyro

% IMU, Acc




%% TABLE SETUP
table = wb_supervisor_node_get_from_def('TABLE');
if isNull(table)
  wb_console_print('No DEF TABLE node found in the current world file', WB_STDOUT);
  quit(1);
end

table_size = wb_supervisor_node_get_field(table, 'size');
size = [TABLE_WIDTH 0.76 TABLE_HEIGHT];
wb_supervisor_field_set_sf_vec3f(table_size, size);


