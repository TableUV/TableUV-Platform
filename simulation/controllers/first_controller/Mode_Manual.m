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
% prior
orient_mu = 0;
orient_S  = 0.1;
% motion model and measurement model
[A,B,R,n] = motion_model(TIME_STEP);
[C,D,Q,m] = measurement_model();
% store in state space model (ssm)
ssm.A = A;
ssm.B = B;
ssm.C = C;
ssm.D = D;
ssm.R = R;
ssm.Q = Q;
ssm.n = n;
ssm.m = m;
% kf initializations
orient_x(1) = orient_mu+sqrt(orient_S)*randn(1);
U = wb_gyro_get_values(imu_gyro);
gyro_u(1) = U(2);
pose_imu_kalman(:, step) = [position(1) position(3) orient_x(1)]';


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
    
    %% Kalman Filter
    % Select motion disturbance
    orient_e = normrnd(0, R);
    % Input for motion model
    U = wb_gyro_get_values(imu_gyro);
    gyro_u(step) = U(2);
    % Update state
    orient_x(step) = A*orient_x(step-1)+ B*gyro_u(step-1) + orient_e;
    
    % Take measurement
    % Select a measurement disturbance
    orient_d = normrnd(0, Q);
    % Determine measurement
    Y = wb_inertial_unit_get_roll_pitch_yaw(imu_mag);
    orient_z = Y(3);
    orient_y(step) = C*orient_z + orient_d;
    
    % Kalman Filter Estimation
    [orient_mu,orient_S,orient_mup,orient_Sp,K] = kalman_filter(ssm,orient_mu,orient_S,gyro_u(step-1),orient_y(step));
    
    % Store estimates
    orient_mup_S(step) = orient_mup;
    orient_mu_S(step) = orient_mu;
    orient_kalman(step) = K;   
   
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
    
    U_Kalman = [vel_z orient_mu_S(step)]';
    U = [vel_z new_imu_gyro_y]';
    
    mu = kinematicModel_vw(pose_imu(:, step-1), U, TIME_STEP);
    mu_kalman = kinematicModel_vw(pose_imu_kalman(:, step-1), U_Kalman, TIME_STEP);
    
    pose_imu(:, step) = mu;
    pose_imu_kalman(:, step) = mu_kalman;
    prev_imu_acc_z = new_imu_acc_z;
    prev_imu_vel_z = vel_z;
    
    
    
    %% TOF
    image = wb_range_finder_get_range_image(tof);
    
    %% PLOTTING
    % Getting true position
    position = wb_supervisor_field_get_sf_vec3f(trans_field);
    orientation = wb_supervisor_field_get_sf_rotation(orien_field);
    true_pose(1, step) = position(1);
    true_pose(2, step) = position(3);
    true_pose(3, step) = orientation(4);
    
    % Plotting position of robot on a map (true, odometry)
%     figure(1)
%     plot(true_pose(1, step), -true_pose(2, step), 'ro');
%     hold on;
%     plot(pose_enc(1, step), -pose_enc(2, step), 'bx');
%     hold on;
%     plot(pose_imu(1, step), -pose_imu(2, step), 'g.');
%     hold on;
%     plot(pose_imu_kalman(1, step), -pose_imu_kalman(2, step), 'c*');
%     hold on
%     axis([-0.8 0.8 -0.6 0.6]);
%     rectangle('Position',[-TABLE_WIDTH/2 -TABLE_HEIGHT/2 TABLE_WIDTH TABLE_HEIGHT])
%     text(0.35,0.55,'True Position','Color','red');
%     text(0.35,0.50,'Odometry','Color','blue');
%     text(0.35,0.45,'IMU Estimation','Color','green');
%     hold off;
    
    % Plotting compass showing current orientation of the robot
    figure(2)
    rad_enc = pose_enc(3, step);
    rad_enc = rad_enc+pi/2;
    u_enc = cos(rad_enc);
    v_enc = sin(rad_enc);
    
    rad_imu = pose_imu(3, step);
    rad_imu = rad_imu+pi/2;
    u_imu = cos(rad_imu);
    v_imu = sin(rad_imu);
    
    rad_kf = pose_imu_kalman(3, step);
    rad_kf = rad_kf+pi/2;
    u_kf = cos(rad_kf);
    v_kf = sin(rad_kf);
    
    compass(u_enc,v_enc,'r'); hold on;
    compass(u_kf,v_kf,'b');
    compass(u_imu,v_imu,'g'); hold off;

    % if your code plots some graphics, it needs to flushed like this:
    drawnow;
end

%% CLEAN UP
filename = "manual.mat";
save(filename)

% cleanup code goes here: write data to files, etc.