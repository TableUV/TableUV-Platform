function X_curr = kinematicModel_kalman(X_prev ,U ,TIME_STEP)
    % kinematicModel_vw
    % X_prev = [x z theta]'
    % U = [v_z theta]'
    
    dt = TIME_STEP * 0.001;
    
    X_curr = [ X_prev(1) + U(1)*sin(X_prev(3))*dt;
               X_prev(2) + U(1)*cos(X_prev(3))*dt;
               U(2) ];
    
end

