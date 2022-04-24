function [quat] = filter_update(a, w, qold)
    deltat = 0.002; % sampling period in seconds (shown as 2 ms - 500Hz)
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
    
    % estimated orientation quaternion elements with initial conditions
    SEq_1 = qold(1); 
    SEq_2 = qold(2);
    SEq_3 = qold(3);
    SEq_4 = qold(4); 
    
    % normalise; vector normalise
    %SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; % quaternion derrivative from gyroscopes elements
    %f_1, f_2, f_3; % objective function elements
    %J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; % objective function Jacobian elements
    %SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; % estimated direction of the gyroscope error
    % Axulirary variables to avoid reapeated calcualtions
    halfSEq_1 = 0.5 * SEq_1;
    halfSEq_2 = 0.5 * SEq_2;
    halfSEq_3 = 0.5 * SEq_3;
    halfSEq_4 = 0.5 * SEq_4;
    twoSEq_1 = 2.0 * SEq_1;
    twoSEq_2 = 2.0 * SEq_2;
    twoSEq_3 = 2.0 * SEq_3;

    % normalisealise the accelerometer measurement
    normalise = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x = a_x/normalise;
    a_y = a_y/normalise;
    a_z = a_z/normalise;
    
    % Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0 - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    J_11or24 = twoSEq_3; % J_11 negated in matrix multiplication
    J_12or23 = 2.0 * SEq_4;
    J_13or22 = twoSEq_1; % J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0 * J_14or21; % negated in matrix multiplication
    J_33 = 2.0 * J_11or24; % negated in matrix multiplication
    
    % Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
    
    % normalisealise the gradient
    normalise = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    if normalise
        SEqHatDot_1 = SEqHatDot_1 / normalise;
        SEqHatDot_2 = SEqHatDot_2 / normalise;
        SEqHatDot_3 = SEqHatDot_3 / normalise;
        SEqHatDot_4 = SEqHatDot_4 / normalise;

        % Compute the quaternion derrivative measured by gyroscopes
        SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
        SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
        SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
        SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

        % Compute then integrate the estimated quaternion derrivative
        SEq_1 = SEq_1 + (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
        SEq_2 = SEq_2 + (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
        SEq_3 = SEq_3 + (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
        SEq_4 = SEq_4 + (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

        % normalisealise quaternion
        normalise = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
        SEq_1 = SEq_1 / normalise;
        SEq_2 = SEq_2 / normalise;
        SEq_3 = SEq_3 / normalise;
        SEq_4 = SEq_4 / normalise;
    
        quat = [SEq_1;SEq_2;SEq_3;SEq_4];
    else
        quat = [1;0;0;0];
    end
end

