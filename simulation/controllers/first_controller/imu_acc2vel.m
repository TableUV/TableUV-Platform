function vel_z = imu_acc2vel(acc_z, prev_vel_z, TIME_STEP)
    dt = TIME_STEP * 0.001;
    
    vel_z = prev_vel_z + acc_z*dt;
end

