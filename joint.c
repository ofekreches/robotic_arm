#include "joint.h"
#include <Arduino.h>

// Function to initialize a Joint struct
void init_joint(Joint_t* joint, StepperMotor_t* motor, Profile_t* s_profile, float gear_ratio) {
    joint->motor = *motor;
    joint->s_profile = *s_profile;
    joint->gear_ratio = gear_ratio;
    joint->radians_per_step = (2 * PI) / (joint->motor.steps_per_rev * gear_ratio);
    joint->current_position = 0.0;
    joint->desired_position = 0.0;
    joint->start_pos = 0.0; //TODO receive from absolute encoder
    joint->last_step_time = 0;
    joint->arrived_to_des_pos = false;
    joint->current_velocity = 0.0;  // Initialize current velocity
    joint->last_position = 0.0;  // Initialize last position
    joint->last_time = 0;  // Initialize last time
}

// Function to step the joint motor
void joint_step(Joint_t* joint, int delta_time) {
    unsigned long current_time = micros();
    if ((current_time - joint->last_step_time) >= delta_time) {
        if ((joint->desired_position - joint->current_position) > 0) {
            joint->motor.direction = FORWARD;
        } else if ((joint->desired_position - joint->current_position) < 0) {
            joint->motor.direction = BACKWARD;
        }
        joint->motor.perform_step_flag = true;
        joint->last_step_time = current_time;
    }
}

// Function to move the joint to the desired position
void move_joint_to_position(Joint_t* joint) {
    update_joint_state(joint);

    // Update the motion profile based on the current and desired positions only if desired position changes
    if (joint->desired_position != joint->s_profile.final_position) {
        update_profile(&joint->s_profile, joint->current_position, joint->desired_position);
    }

    if (fabs(joint->desired_position - joint->current_position) < joint->radians_per_step) { // if the delta position is smaller than the size of one step
        joint->arrived_to_des_pos = true;
    } else {
        joint->arrived_to_des_pos = false;
    }

    if (!joint->arrived_to_des_pos) {
        float desired_velocity = s_profile_velocity(&joint->s_profile, joint->current_position);
        joint->s_velocity = desired_velocity;
        int delta_step_time = (joint->radians_per_step / desired_velocity) * 1000000; // Convert to microseconds
        joint_step(joint, delta_step_time);
    }
}

// Function to update the current state of the joint
void update_joint_state(Joint_t* joint) {
    // Calculate the step position based on the step count and radians per step
    float step_position = joint->motor.step_count * joint->radians_per_step;
    joint->current_position = joint->start_pos + step_position; // the stepper motor does not know where it is when it turns on

    // Calculate the current velocity based on step position change over time
    unsigned long current_time = micros();

    // Check if the position has changed
    if (joint->current_position != joint->last_position) {
        joint->current_velocity = (joint->current_position - joint->last_position) / ((current_time - joint->last_time) / 1000000.0);  // in radians per second
        joint->last_position = joint->current_position;
        joint->last_time = current_time;
    } else if ((current_time - joint->last_time) > ZERO_VELOCITY_TIMEOUT) {
        // If the last position change was over ZERO_VELOCITY_TIMEOUT
        joint->current_velocity = 0.0;
    }
}
