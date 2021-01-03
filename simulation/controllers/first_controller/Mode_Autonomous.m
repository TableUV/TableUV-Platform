% MAIN LOOP
wb_console_print('Starting Autonomous Mode!', WB_STDOUT);

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

% initial position and velocity for IMU
raw_imu_acc(:, step) = [0 0 0]';
imu_vel(1) = 0;
imu_dis(1) = 0;

while wb_robot_step(TIME_STEP) ~= -1
    step = step + 1;
    
    % setting motor speeds
    left_speed  = 1.0 * MAX_SPEED;
    right_speed = 2.0 * MAX_SPEED;
    
    for i=1:2
        if i == 1
            wb_motor_set_velocity(motor(i), left_speed);
        end
        if i == 2
            wb_motor_set_velocity(motor(i), right_speed);
        end
    end
    
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
    for i=1:3
        if abs(x_y_z_array(i)) < 0.0001
            x_y_z_array(i) = 0;
        end 
    end
    
    wb_console_print(sprintf('ACC values: %g %g %g\n', x_y_z_array(1), x_y_z_array(2), x_y_z_array(3)), WB_STDOUT);
    raw_imu_acc(:, step) = x_y_z_array';
    
    for i=2:step
        imu_vel(i) = imu_vel(i-1) + (raw_imu_acc(1, i)+raw_imu_acc(1, i-1))/2;
        imu_dis(i) = imu_dis(i-1) + imu_vel(i-1)+(imu_vel(i)+imu_vel(i-1))/2;
    end
    
    %% PLOTTING
    % Getting true position
    position = wb_supervisor_field_get_sf_vec3f(trans_field);
    orientation = wb_supervisor_field_get_sf_rotation(orien_field);
    
    p(1, step) = position(1);
    p(2, step) = position(3);
    p(3, step) = orientation(4);
    
    % Plotting IMU values 
    figure(1)
    subplot(3,1,1);
    plot(raw_imu_acc(1, 1:step));
    xlabel('Timestep');
    ylabel('Acceleration (m/s^2)');
    subplot(3,1,2);
    plot(imu_vel(1:step));
    xlabel('Timestep');
    ylabel('Velocity (??)');
    subplot(3,1,3);
    plot(imu_dis(1:step));
    xlabel('Timestep');
    ylabel('Distance (cm)');
    
    % Plotting position of robot on a map (true, odometry)
    figure(2)
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
% cleanup code goes here: write data to files, etc.