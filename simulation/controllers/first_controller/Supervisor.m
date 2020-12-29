%% TABLE SETUP
table = wb_supervisor_node_get_from_def('TABLE');
table_size = wb_supervisor_node_get_field(table, 'size');
size = [TABLE_WIDTH 0.76 TABLE_HEIGHT];
wb_supervisor_field_set_sf_vec3f(table_size, size);

%% ROBOT SETUP
% Getting the robot
robot_node = wb_supervisor_node_get_from_def(ROBOT_NAME);
if robot_node == 0
  wb_console_print('No DEF MY_ROBOT node found in the current world file', WB_STDOUT);
  quit(1);
end
trans_field = wb_supervisor_node_get_field(robot_node, 'translation');

% LEFT, RIGHT MOTORS
robot_left_wheel = wb_supervisor_node_get_from_def('ROBOT_WHEEL_LEFT');
if robot_left_wheel == 0
  wb_console_print('No DEF ROBOT_WHEEL_LEFT node found in the current world file', WB_STDOUT);
  quit(1);
end 
left_wheel_rad = wb_supervisor_node_get_field(robot_left_wheel, 'radius');
wb_supervisor_field_set_sf_float(left_wheel_rad, WHEEL_RADIUS);

robot_right_wheel = wb_supervisor_node_get_from_def('ROBOT_WHEEL_RIGH');
if robot_right_wheel == 0
  wb_console_print('No DEF ROBOT_WHEEL_RIGHT node found in the current world file', WB_STDOUT);
  quit(1);
end 
right_wheel_rad = wb_supervisor_node_get_field(robot_right_wheel, 'radius');
wb_supervisor_field_set_sf_float(right_wheel_rad, WHEEL_RADIUS);

% LEFT, RIGHT ENCODERS
robot_left_encoder = wb_supervisor_node_get_from_def('ROBOT_WHEEL_LEFT');
if robot_left_encoder == 0
  wb_console_print('No DEF ROBOT_WHEEL_LEFT node found in the current world file', WB_STDOUT);
  quit(1);
end 

% GLOBAL GPS


% RANGE FINDER (TOF)


% IMU, GYRO


% IMU, ACC


