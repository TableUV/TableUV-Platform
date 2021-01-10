% MAIN LOOP
wb_console_print('Starting Manual Mode!', WB_STDOUT);

%% INITIALIZATION
step = 1;
samples = 0;

% initial pose of the robot
position = wb_supervisor_field_get_sf_vec3f(trans_field);
orientation = wb_supervisor_field_get_sf_rotation(orien_field);
init_pose = [position(1) position(3) orientation(4)]'; % [x z theta]
true_pose(:, step) = init_pose;

% initializing values for encoder
pose_enc(:, step) = init_pose;
prev_enc_left = 0;
prev_enc_right = 0;

% initial position and velocity for IMU
pose_imu(:, step) = init_pose;
prev_imu_acc_z = 0;
prev_imu_vel_z = 0;

% kf
[A,B,R,n] = motion_model(TIME_STEP);
prev_orient = orientation(4);

[C,D,Q,m] = measurement_model();

ssm.A = A;
ssm.B = B;
ssm.C = C;
ssm.D = D;
ssm.R = R;
ssm.Q = Q;
ssm.n = n;
ssm.m = m;

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
    
    %% ENCODER TO POSE USING KINEMATIC MODEL
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
    
    W = enc2wheelvel(diff_enc_left, diff_enc_right, TIME_STEP);
    mu = kinematicModel(pose_enc(:, step-1), W, TIME_STEP, WHEEL_RADIUS, WHEEL_FROM_CENTER);
    
    pose_enc(:, step) = mu;
    prev_enc_left = new_enc_left;
    prev_enc_right = new_enc_right;
    
    %% TOF
    image = wb_range_finder_get_range_image(tof);
    
    %% Kalman Filter
    orientation = wb_supervisor_field_get_sf_rotation(orien_field);
    
    U = wb_gyro_get_values(imu_gyro);
    u = U(2);
    pred_of_orient_with_gyro = A*prev_orient + B*u;
    
    Y = wb_inertial_unit_get_roll_pitch_yaw(imu_mag);
    y = Y(3);
    meas_of_orient_with_mag = C*y;
    
    prev_orient = pred_of_orient_with_gyro;
    
    wb_console_print(sprintf('ORIENT: %g %g %g\n', orientation(4), pred_of_orient_with_gyro, meas_of_orient_with_mag), WB_STDOUT);
    
    o_1(step) = orientation(4);
    o_2(step) = pred_of_orient_with_gyro;
    o_3(step) = meas_of_orient_with_mag;
    
    %% IMU    
    % IMU, Gyro
    roll_pitch_yaw_array = wb_gyro_get_values(imu_gyro);
    new_imu_gyro_y = roll_pitch_yaw_array(2);
    
    % IMU, Acce
    x_y_z_array = wb_accelerometer_get_values(imu_acc);
    new_imu_acc_z = x_y_z_array(3);
    
    vel_z = imu_acc2vel(new_imu_acc_z, prev_imu_vel_z, TIME_STEP);
    
    if abs(new_imu_gyro_y) < 0.001
        new_imu_gyro_y = 0;
    end
    
    if abs(vel_z) < 0.001
        vel_z = 0;
    end
    
    U = [vel_z new_imu_gyro_y]';
    mu = kinematicModel_vw(pose_imu(:, step-1), U, TIME_STEP);
    
    pose_imu(:, step) = mu;
    prev_imu_acc_z = new_imu_acc_z;
    prev_imu_vel_z = vel_z;
    
    %% PLOTTING
    % Getting true position
    position = wb_supervisor_field_get_sf_vec3f(trans_field);
    orientation = wb_supervisor_field_get_sf_rotation(orien_field);
    true_pose(1, step) = position(1);
    true_pose(2, step) = position(3);
    true_pose(3, step) = orientation(4);
    
    % Plotting position of robot on a map (true, odometry)
    figure(1)
    plot(true_pose(1, step), -true_pose(2, step), 'ro');
    hold on;
    plot(pose_enc(1, step), -pose_enc(2, step), 'bx');
    hold on;
    plot(pose_imu(1, step), -pose_imu(2, step), 'g.');
    hold on;
    axis([-0.8 0.8 -0.6 0.6]);
    rectangle('Position',[-TABLE_WIDTH/2 -TABLE_HEIGHT/2 TABLE_WIDTH TABLE_HEIGHT])
    text(0.35,0.55,'True Position','Color','red');
    text(0.35,0.50,'Odometry','Color','blue');
    text(0.35,0.45,'IMU Estimation','Color','green');
    hold off;

    % if your code plots some graphics, it needs to flushed like this:
    drawnow;
end

%% CLEAN UP
filename = "manual.mat";
save(filename)

% cleanup code goes here: write data to files, etc.