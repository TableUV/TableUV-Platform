function [x,z,theta] = enc2pose(left_enc, right_enc, prev_x, prev_z, prev_theta, ENC_UNIT, WHEEL_FROM_CENTER)
    %enc2pose: Convert encoder values to robot pose
    
    left_dist = left_enc * ENC_UNIT;
    right_dist = right_enc * ENC_UNIT;
    center_dist = (left_dist + right_dist) / 2;
    
    if center_dist < 0.00001
        center_dist = 0
    end
    
    x = prev_x + center_dist*cos(prev_theta);
    z = prev_z + center_dist*sin(prev_theta);
    theta = prev_theta + (right_dist-left_dist)/(2*WHEEL_FROM_CENTER);
end

