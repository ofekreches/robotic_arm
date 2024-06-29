#include "joint.h"
#include <Arduino.h>

// Function to initialize a Joint struct
void init_joint(Joint_t* joint, StepperMotor_t* motor, float gear_ratio) {
    joint->motor = *motor;
    joint->gear_ratio = gear_ratio;
    joint->radians_per_step = (2 * PI) / (joint->motor.steps_per_rev * gear_ratio);
    joint->current_position = 0.0;
    joint->desired_position = 0.0;
    joint->start_pos = 0.0; //TODO receive from absolute encoder
    joint->last_step_time = 0;
    joint->arrived_to_des_pos = false;
}

// Function to step the joint motor
void joint_step(Joint_t* joint, int delta_time) {
    unsigned long current_time = micros();
    if ((current_time - joint->last_step_time) > delta_time) {
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
    update_joint_position(joint);

    if (fabs(joint->desired_position - joint->current_position) < joint->radians_per_step) { // if the delta position is smaller than the size of one step
        joint->arrived_to_des_pos = true;
    } else {
        joint->arrived_to_des_pos = false;
    }

    if (!joint->arrived_to_des_pos) {
        int delta_step_time = s_profile(joint->current_position, joint->desired_position, joint->radians_per_step);  // this function returns the delta time between steps to fulfill an s profile
        // int delta_step_time = 10;
        joint_step(joint, delta_step_time);
    }
}

// Function to update the current position of the joint
void update_joint_position(Joint_t* joint) {  // TODO: ADD ABSOLUTE ENCODER LOGIC
    float step_position = joint->motor.step_count * joint->radians_per_step;
    joint->current_position = joint->start_pos + step_position; // the stepper motor does not know where it is when it turns on
}
