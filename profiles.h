#ifndef PROFILES_H
#define PROFILES_H

#ifdef __cplusplus
extern "C" {
#endif

// Function declarations
int s_profile(float current_position, float desired_position, float radians_per_step);  // S-curve profile function
int trapezoidal_profile(float current_position, float desired_position, float radians_per_step);  // Trapezoidal profile function

#ifdef __cplusplus
}
#endif

#endif // PROFILES_H
