#include "stepper_motor.h"
#include "configuration.h"
#include <Arduino.h>

void init_stepper_motor(StepperMotor_t* motor, int ena_pin, int dir_pin, int pul_pin, int steps_per_rev) {
    motor->ena_pin = ena_pin;
    motor->dir_pin = dir_pin;
    motor->pul_pin = pul_pin;
    motor->steps_per_rev = steps_per_rev;
    motor->radian_per_step = 2 * PI / steps_per_rev;
    motor->last_step_time = 0;
    motor->step_state = false;
    motor->step_count = 0;
    motor->perform_step_flag = false;
    motor->step_function_frequency = 0;
    motor->last_call_time = micros(); // Initialize last call time

    // Set pin modes
    pinMode(motor->ena_pin, OUTPUT);
    pinMode(motor->dir_pin, OUTPUT);
    pinMode(motor->pul_pin, OUTPUT);

    // Disable motor initially
    disable_motor(motor);
}

// Function to enable the motor
void enable_motor(StepperMotor_t* motor) {
    digitalWrite(motor->ena_pin, LOW); 
}

// Function to disable the motor
void disable_motor(StepperMotor_t* motor) {
    digitalWrite(motor->ena_pin, HIGH);  
}

// Non-blocking function to step the stepper motor
void step_motor(StepperMotor_t* motor) {
    motor->step_function_counter++;
    unsigned long current_time = micros();

    // Calculate the elapsed time
    unsigned long elapsed_time = current_time - motor->last_call_time;
    
    // Check if elapsed time is greater than or equal to one second (1,000,000 microseconds)
    if (elapsed_time >= 1000000) {
        // Calculate the frequency of the step function being called
        motor->step_function_frequency = motor->step_function_counter; // Frequency in Hz
        motor->step_function_counter = 0; // Reset counter
        motor->last_call_time = current_time; // Reset the timer
    }

    if (motor->perform_step_flag) {
        // Check if it's time to toggle the step pin state
        if (current_time - motor->last_step_time >= MIN_STEP_TIME) {
            if (motor->step_state) {
                digitalWrite(motor->pul_pin, LOW);
                motor->step_state = false;
                compute_steps(motor, motor->direction);
                motor->perform_step_flag = false;
            } else {
                set_direction(motor, motor->direction);
                digitalWrite(motor->pul_pin, HIGH);
                motor->step_state = true;
            }
            motor->last_step_time = current_time;
        }
    }
}

void set_direction(StepperMotor_t* motor, StepDirection_e direction) {
    switch (direction) {
        case FORWARD:
            digitalWrite(motor->dir_pin, HIGH);
            break;
        case BACKWARD:
            digitalWrite(motor->dir_pin, LOW);
            break;
    }
}

void compute_steps(StepperMotor_t* motor, StepDirection_e direction) {
    switch (direction) {
        case FORWARD:
            motor->step_count++;
            break;
        case BACKWARD:
            motor->step_count--;
            break;
    }
}
