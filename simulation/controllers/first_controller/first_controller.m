% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
desktop;
keyboard;

% PARAMETERS
Parameters;
% DEVICE INITIALIZATION
Initialize;
% SUPERVISOR
Supervisor;

% MAIN LOOP
wb_console_print('Starting main loop!', WB_STDOUT);

step = 0;
samples = 0;

while wb_robot_step(TIME_STEP) ~= -1
    step = step + 1;
    
    % setting motor speeds
    left_speed  = 0.5 * MAX_SPEED;
    right_speed = 0.5 * MAX_SPEED;
    for i=1:2
        if i == 1
            wb_motor_set_velocity(motor(i), left_speed);
        end
        if i == 2
            wb_motor_set_velocity(motor(i), right_speed);
        end
    end

    % getting global robot position with GPS
    x_y_z_array = wb_gps_get_values(gps);
    p(step,:) = x_y_z_array;
    % plotting position of robot on a map
    plot(p(step,1),-p(step,3),'ro');
    hold on;
    axis([-1 1 -0.8 0.8]);
    rectangle('Position',[-TABLE_WIDTH/2 -TABLE_HEIGHT/2 TABLE_WIDTH TABLE_HEIGHT])
    hold off;
    
    % getting tof sensor readings
    image = wb_range_finder_get_range_image(tof);
    
    % getting encoder values 
    value = wb_position_sensor_get_value(ps(1));
    type = wb_position_sensor_get_type(ps(1));
    
    %% SUPERVISOR
    % this is done repeatedly
    values = wb_supervisor_field_get_sf_vec3f(trans_field);
    wb_console_print(sprintf('MY_ROBOT is at position: %g %g %g\n', values(1), values(2), values(3)), WB_STDOUT);

    % if your code plots some graphics, it needs to flushed like this:
    drawnow;
end

%% CLEAN UP
% cleanup code goes here: write data to files, etc.

