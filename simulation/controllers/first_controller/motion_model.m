function [A,B,R,n] = motion_model(dt)
    dt = dt * 0.001;
    
    A = 1;
    B = dt;
    R = 1;
    n = 1;
end

