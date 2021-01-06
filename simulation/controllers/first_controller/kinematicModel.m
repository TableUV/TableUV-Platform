function X_curr = kinematicModel(X_prev,U,TIME_STEP,WHEEL_RADIUS,WHEEL_FROM_CENTER)
    %kinematicModel
    %X_prev = [x z theta]'
    
    r = WHEEL_RADIUS;
    l = WHEEL_FROM_CENTER;
    dt = TIME_STEP * 0.001;

    X_curr = [ X_prev(1) + (r*(U(1)+U(2)))*0.5*sin(X_prev(3))*dt;
               X_prev(2) + (r*(U(1)+U(2)))*0.5*cos(X_prev(3))*dt;
               X_prev(3) + (r*(U(2)-U(1)))/(2*l)*dt ];
end

