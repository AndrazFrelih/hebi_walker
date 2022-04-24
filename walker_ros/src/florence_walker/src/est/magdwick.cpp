#include "magdwick.hpp"

void filterUpdate(const Eigen::Vector3d & w,
                 const Eigen::Vector3d & a,
                 Eigen::Quaterniond & quat,
                 const Eigen::Quaterniond & quat_old, 
                 double dt)
{

    double beta = (sqrt(3.0/ 4.0) * (double)gyroMeasError);
    double a_x, a_y, a_z; // accelerometer measurements
    double w_x, w_y, w_z; // gyroscope measurements in rad/s
    a_x = a[0];
    a_y = a[1];
    a_z = a[2];
    w_x = w[0];
    w_y = w[1];
    w_z = w[2];
    
    double SEq_1 = quat_old.w(), SEq_2 = quat_old.x(), SEq_3 = quat_old.y(), SEq_4 = quat_old.z(); // estimated orientation quaternion elements with initial conditions
    
    // Local system variables
    double normalise; // vector normalise
    double SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
    double f_1, f_2, f_3; // objective function elements
    double J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    double SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    // Axulirary variables to avoid reapeated calcualtions
    double halfSEq_1 = 0.5f * SEq_1;
    double halfSEq_2 = 0.5f * SEq_2;
    double halfSEq_3 = 0.5f * SEq_3;
    double halfSEq_4 = 0.5f * SEq_4;
    double twoSEq_1 = 2.0f * SEq_1;
    double twoSEq_2 = 2.0f * SEq_2;
    double twoSEq_3 = 2.0f * SEq_3;

    // normalisealise the accelerometer measurement
    
    normalise = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= normalise;
    a_y /= normalise;
    a_z /= normalise;
    // Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    // Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
    // normalisealise the gradient
    normalise = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= normalise;
    SEqHatDot_2 /= normalise;
    SEqHatDot_3 /= normalise;
    SEqHatDot_4 /= normalise;
    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // Compute then integrate the estimated quaternion derrivative
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dt;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dt;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dt;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dt;
    // normalisealise quaternion
    normalise = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= normalise;
    SEq_2 /= normalise;
    SEq_3 /= normalise;
    SEq_4 /= normalise;
    
    quat.w()=SEq_1;
    quat.x()=SEq_2;
    quat.y()=SEq_3;
    quat.z()=SEq_4;
    // = Quaterniond(&SEq_1,&SEq_2,&SEq_3,&SEq_4);
}