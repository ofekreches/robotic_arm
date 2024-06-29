#ifndef JOINT_H
#define JOINT_H

#include "configuration.h"
#include "stepper_motor.h"
#include "profiles.h"

typedef struct {
    StepperMotor_t motor;
    float current_position;
    float desired_position;
    float gear_ratio;
    float radians_per_step;
    int joint_direction;
    float start_pos;
    unsigned long last_step_time;
    bool arrived_to_des_pos;
} Joint_t;

#ifdef __cplusplus
extern "C" {
#endif

// Function declarations
void init_joint(Joint_t* joint, StepperMotor_t* motor, float gear_ratio);
void move_joint_to_position(Joint_t* joint);
void update_joint_position(Joint_t* joint);
void joint_step(Joint_t* joint, int delta_time);

#ifdef __cplusplus
}
#endif

#endif // JOINT_H
