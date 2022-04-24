function [qnew_norm] = filter_update2(a, w, qold, dt)
    gyroMeasError = 3.14159265358979 * (1 / 180); % gyroscope measurement error in rad/s (shown as 5 deg/s)
    beta = sqrt(3.0 / 4.0) * gyroMeasError; % compute beta

    % accelerometer measurements
    a_x = a(1);
    a_y = a(2);
    a_z = a(3);
    % gyroscope measurements in rad/s
    w_x = w(1);
    w_y = w(2);
    w_z = w(3);
    
    %normalise acceleration measurements
    normalise = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x = a_x/normalise;
    a_y = a_y/normalise;
    a_z = a_z/normalise;
    
    %objective function formulation
    d = [0,0,1]'; %desired g direction
    s = [a_x,a_y,a_z]'; %actual g direction
    f = magd_cost(qold,d,s); %cost function
    J = magd_jacob(qold,d); %jacobian of the cost
    
    %update part from gradient
    qupd = J'*f;
    qupd_norm = qupd/norm(qupd);
    %update part from angular velocity
    w = [0, w_x, w_y, w_z]';
    qw = quat_mult(qold,w)/2;
    
    %integrate change
    qint = qw - beta*qupd_norm;
    %updated quaternion
    qnew = qold + dt*qint;
    %normalised updated quaternion
    qnew_norm = qnew/norm(qnew);
end

