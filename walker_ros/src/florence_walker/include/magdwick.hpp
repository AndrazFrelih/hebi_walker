// Code imported and adapted (Magdwick complementary filter)

// Math library required for `sqrt'
#include <cmath>
#include "definitions.hpp"


// System constants
#define gyroMeasError (3.14159265358979f * (5.0f / 180.0f)) // gyroscope measurement error in rad/s (shown as 5 deg/s)

// Prototype
void filterUpdate(const Eigen::Vector3d & w,
				 const Eigen::Vector3d & a,
				 Eigen::Quaterniond & quat,
				 const Eigen::Quaterniond & quat_old, 
				 double dt);