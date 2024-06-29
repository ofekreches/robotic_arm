#include "profiles.h"
#include <math.h>
#include <Arduino.h>

// Trapezoidal profile function for smooth acceleration and deceleration
int trapezoidal_profile(float current_position, float desired_position, float radians_per_step) {
    // Desired speed and acceleration (these should be tuned to your specific application)
    const float max_speed = 1.0;  // Maximum speed in radians per second
    const float max_acceleration = 0.5;  // Maximum acceleration in radians per second squared

    // Calculate the distance to the desired position
    float distance_to_go = fabs(desired_position - current_position);

    // Calculate the maximum speed that can be reached over the remaining distance
    float speed = sqrt(2 * max_acceleration * distance_to_go);

    // Limit the speed to the maximum speed
    if (speed > max_speed) {
        speed = max_speed;
    }

    // Calculate the time between steps to achieve the desired speed
    float step_time = radians_per_step / speed;  // Time per step in seconds
    int delta_step_time = (int)(step_time * 1000000);  // Convert to microseconds

    return delta_step_time;
}

// S-curve profile function for smoother acceleration and deceleration
int s_profile(float current_position, float desired_position, float radians_per_step) {
    // Desired speed and acceleration (these should be tuned to your specific application)
    const float max_speed = 1.0;  // Maximum speed in radians per second
    const float max_acceleration = 0.5;  // Maximum acceleration in radians per second squared
    const float jerk = 0.1;  // Rate of change of acceleration

    // Calculate the distance to the desired position
    float distance_to_go = fabs(desired_position - current_position);

    // Calculate the remaining distance to apply S-curve profile
    float remaining_distance = distance_to_go;

    // Calculate the acceleration phase distance
    float acc_distance = (max_speed * max_speed) / (2 * max_acceleration);

    // Check if the distance to go is less than twice the acceleration distance
    if (distance_to_go <= 2 * acc_distance) {
        // Adjust the acceleration and deceleration phases for shorter distance
        float speed = sqrt(max_acceleration * remaining_distance);
        float step_time = radians_per_step / speed;  // Time per step in seconds
        return (int)(step_time * 1000000);  // Convert to microseconds
    }

    // Calculate the time between steps to achieve the desired speed
    float step_time = radians_per_step / max_speed;  // Time per step in seconds
    int delta_step_time = (int)(step_time * 1000000);  // Convert to microseconds

    return delta_step_time;
}
