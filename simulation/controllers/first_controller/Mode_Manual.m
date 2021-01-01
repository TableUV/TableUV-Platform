% MAIN LOOP
wb_console_print('Starting Manual Mode!', WB_STDOUT);

%% INITIALIZATION
step = 1;
samples = 0;

% initial pose of the robot
position = wb_supervisor_field_get_sf_vec3f(trans_field);
orientation = wb_supervisor_field_get_sf_rotation(orien_field);
pose = [position(1) position(3) orientation(4)]'; % [x y theta]
pose_enc(:, step) = pose;
p(:, step) = pose;

% initializing values for encoder
prev_enc_left = 0;
prev_enc_right = 0;

while wb_robot_step(TIME_STEP) ~= -1
    step = step + 1;
    
    input = wb_keyboard_get_key();
    
    switch input 
        case 314
            % LEFT
            left_speed = -1.0;
            right_speed = 1.0;
        case 315
            % UP
            left_speed = 1.0;
            right_speed = 1.0;
        case 316
            % RIGHT
            left_speed = 1.0;
            right_speed = -1.0;
        case 317
            % DOWN
            left_speed = -1.0;
            right_speed = -1.0;
        otherwise
            % NOTHING 
            left_speed = 0;
            right_speed = 0;      
    end
    
    % set the velocity of the motors
    wb_motor_set_velocity(motor(1), left_speed*MAX_SPEED);
    wb_motor_set_velocity(motor(2), right_speed*MAX_SPEED);
    
    %% ENCODER 
    new_enc_left = wb_position_sensor_get_value(ps(1));
    new_enc_right = wb_position_sensor_get_value(ps(2));
    
    diff_enc_left = new_enc_left - prev_enc_left;
    diff_enc_right = new_enc_right - prev_enc_right;
    
    if abs(diff_enc_left) < 0.001
        diff_enc_left = 0;
    end
    
    if abs(diff_enc_right) < 0.001
        diff_enc_right = 0;
    end
    
    prev_enc_left = new_enc_left;
    prev_enc_right = new_enc_right;
    
    [x, z, theta] = enc2pose(diff_enc_left, diff_enc_right, pose_enc(1, step-1), pose_enc(2, step-1), pose_enc(3, step-1), ENC_UNIT, WHEEL_FROM_CENTER);
    pose_enc(:, step) = [x, z, theta]';
    
    
    %% TOF
    image = wb_range_finder_get_range_image(tof);
    
    %% IMU
    % IMU, Gyro
    roll_pitch_yaw_array = wb_inertial_unit_get_roll_pitch_yaw(imu_gyro);
    
    % IMU, Acce
    x_y_z_array = wb_accelerometer_get_values(imu_acc);
    
    %% PLOTTING
    % this is done repeatedly
    position = wb_supervisor_field_get_sf_vec3f(trans_field);
    orientation = wb_supervisor_field_get_sf_rotation(orien_field);
    
    p(1, step) = position(1);
    p(2, step) = position(3);
    p(3, step) = orientation(4);
    
    % plotting position of robot on a map
    plot(p(1, step), -p(2, step), 'ro');
    hold on;
    plot(pose_enc(1, step), -pose_enc(2, step), 'bx');
    hold on;
    axis([-0.8 0.8 -0.6 0.6]);
    rectangle('Position',[-TABLE_WIDTH/2 -TABLE_HEIGHT/2 TABLE_WIDTH TABLE_HEIGHT])
    hold off;
    
    wb_console_print(sprintf('TRUE position: %g %g\n', p(1, step), -p(3, step)), WB_STDOUT);
    wb_console_print(sprintf('ENC  position: %g %g\n', pose_enc(1, step), pose_enc(2, step)), WB_STDOUT);

    %% if your code plots some graphics, it needs to flushed like this:
    drawnow;
end

%% CLEAN UP
filename = "manual.mat";
save(filename)

% cleanup code goes here: write data to files, etc.