% MAIN LOOP
wb_console_print('Starting Manual Mode!', WB_STDOUT);

step = 0;
samples = 0;

while wb_robot_step(TIME_STEP) ~= -1
    step = step + 1;
    
    input = wb_keyboard_get_key();
    
    switch input 
        case 314
            % LEFT
            left_speed = 1.0;
            right_speed = -1.0;
        case 315
            % UP
            left_speed = 1.0;
            right_speed = 1.0;
        case 316
            % RIGHT
            left_speed = -1.0;
            right_speed = 1.0;
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
    wb_motor_set_velocity(motor(1), left_speed);
    wb_motor_set_velocity(motor(2), right_speed);
    
    %% ENCODER 
    value = wb_position_sensor_get_value(ps(1));
    type = wb_position_sensor_get_type(ps(1));
    
    %% GPS 
    x_y_z_array = wb_gps_get_values(gps);
    p(step,:) = x_y_z_array;
    % plotting position of robot on a map
    plot(p(step,1),-p(step,3),'ro');
    hold on;
    axis([-1 1 -0.8 0.8]);
    rectangle('Position',[-TABLE_WIDTH/2 -TABLE_HEIGHT/2 TABLE_WIDTH TABLE_HEIGHT])
    hold off;
    
    %% TOF
    image = wb_range_finder_get_range_image(tof);
    
    %% IMU
    % IMU, Gyro
    roll_pitch_yaw_array = wb_inertial_unit_get_roll_pitch_yaw(imu_gyro)
    
    % IMU, Acce
    x_y_z_array = wb_accelerometer_get_values(imu_acc)

    % if your code plots some graphics, it needs to flushed like this:
    drawnow;
end

%% CLEAN UP
% cleanup code goes here: write data to files, etc.