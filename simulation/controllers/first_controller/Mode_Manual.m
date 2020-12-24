% MAIN LOOP
wb_console_print('Starting Manual Mode!', WB_STDOUT);

step = 0;
samples = 0;

while wb_robot_step(TIME_STEP) ~= -1
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

    % if your code plots some graphics, it needs to flushed like this:
    drawnow;
end

%% CLEAN UP
% cleanup code goes here: write data to files, etc.