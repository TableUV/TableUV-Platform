% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
% desktop;
% keyboard;

% PARAMETERS
Parameters;
% DEVICE INITIALIZATION
Initialize;
% SUPERVISOR
Supervisor;

% GET INPUT
wb_console_print(sprintf(['Select controller type:\n', ...
    '1 - Autonomous\n', ...
    '2 - Manual Control\n', ...
    '3 - Exit\n'
    ]), WB_STDOUT);

while wb_robot_step(TIME_STEP) ~= -1
    input = wb_keyboard_get_key();
    switch input 
        case 49
            Mode_Autonomous
        case 50
            Mode_Manual
        case 51
            wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE)
            wb_console_print('Paused. Press Ctrl+Shift+T to reset.', WB_STDOUT);
            return
        case -1
            % do nothing
        otherwise
            wb_console_print('Invalid Input.', WB_STDOUT);
    end
end
