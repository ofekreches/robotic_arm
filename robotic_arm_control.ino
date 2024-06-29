#include <Arduino.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_task_wdt.h>
#include "configuration.h"
#include "stepper_motor.h"
#include "joint.h"
#include "profiles.h"
#include "arm.h"
#include "comm_controller.h"

// Motor objects
StepperMotor_t base_motor;
StepperMotor_t shoulder_motor_1;
StepperMotor_t shoulder_motor_2;
StepperMotor_t elbow_motor;
StepperMotor_t wrist_pitch_motor;
StepperMotor_t wrist_roll_motor;

// Joint objects
Joint_t base_joint;
Joint_t second_joint;
Joint_t third_joint;
Joint_t fourth_joint;
Joint_t fifth_joint;

// Arm object
Arm_t arm;

// Communication object
CommController comm;

// FreeRTOS objects
TaskHandle_t StepMotorTaskHandle;
TaskHandle_t UpdateJointTaskHandle;
TaskHandle_t CommunicationTaskHandle;
SemaphoreHandle_t StepMotorSemaphore;

// Timer handle for motor stepping
esp_timer_handle_t motor_timer;

void IRAM_ATTR onMotorTimer(void* arg) {
    xSemaphoreGiveFromISR(StepMotorSemaphore, NULL);
}

void stepMotorTask(void * parameter) {
    Arm_t *arm = (Arm_t *)parameter;

    for(;;) {
        if(xSemaphoreTake(StepMotorSemaphore, portMAX_DELAY)) {
            step_motor(&arm->base_joint.motor);
            step_motor(&arm->second_joint.motor);
            step_motor(&arm->third_joint.motor);
            step_motor(&arm->fourth_joint.motor);
            step_motor(&arm->fifth_joint.motor);
        }
    }
}

void updateJointTask(void * parameter) {
    Arm_t *arm = (Arm_t *)parameter;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(MOTOR_CONTROL_DT * 1000);
    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        move_joint_to_position(&arm->base_joint);
        move_joint_to_position(&arm->second_joint);
        move_joint_to_position(&arm->third_joint);
        move_joint_to_position(&arm->fourth_joint);
        move_joint_to_position(&arm->fifth_joint);
    }
}

void communicationTask(void * parameter) {
    Arm_t *arm = (Arm_t *)parameter;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(COMMUNICATION_DT * 1000);
    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        processDataToSend(&comm, arm);
        Serial.write(comm.TxData, SIZE_OF_TX_DATA);

        if (Serial.available() >= SIZE_OF_RX_DATA) {
            Serial.readBytes(comm.RxData, SIZE_OF_RX_DATA);
            int check = receiveData(&comm, arm);  
            if (check == 2) {
                Serial.flush(); 
            }
        }
    }
}

void setup() {
    BaseType_t taskCreated;

    // Initialize motors
    init_stepper_motor(&base_motor, BASE_ENA_PIN, BASE_DIR_PIN, BASE_PUL_PIN, BASE_MOTOR_STEPS_PER_REV);
    init_stepper_motor(&shoulder_motor_1, SHOULDER_ENA_PIN_1, SHOULDER_DIR_PIN_1, SHOULDER_PUL_PIN_1, SHOULDER_MOTOR_STEPS_PER_REV);
    init_stepper_motor(&shoulder_motor_2, SHOULDER_ENA_PIN_2, SHOULDER_DIR_PIN_2, SHOULDER_PUL_PIN_2, SHOULDER_MOTOR_STEPS_PER_REV);
    init_stepper_motor(&elbow_motor, ELBOW_ENA_PIN, ELBOW_DIR_PIN, ELBOW_PUL_PIN, ELBOW_MOTOR_STEPS_PER_REV);
    init_stepper_motor(&wrist_pitch_motor, WRIST_PITCH_ENA_PIN, WRIST_PITCH_DIR_PIN, WRIST_PUL_PIN, WRIST_PITCH_MOTOR_STEPS_PER_REV);
    init_stepper_motor(&wrist_roll_motor, WRIST_ROLL_ENA_PIN, WRIST_ROLL_DIR_PIN, WRIST_PUL_PIN, WRIST_ROLL_MOTOR_STEPS_PER_REV);

    // Initialize joints
    init_joint(&base_joint, &base_motor, BASE_GEAR_RATIO);
    init_joint(&second_joint, &shoulder_motor_1, SHOULDER_GEAR_RATIO);
    init_joint(&third_joint, &elbow_motor, ELBOW_GEAR_RATIO);
    init_joint(&fourth_joint, &wrist_pitch_motor, WRIST_PITCH_GEAR_RATIO);
    init_joint(&fifth_joint, &wrist_roll_motor, WRIST_ROLL_GEAR_RATIO);

    // Initialize arm
    init_arm(&arm, base_joint, second_joint, third_joint, fourth_joint, fifth_joint);

    // Initialize communication
    comm_controller_init(&comm);
    Serial.begin(SERIAL_BAUDRATE);

    StepMotorSemaphore = xSemaphoreCreateBinary();

    // Creating stepMotorTask
    taskCreated = xTaskCreatePinnedToCore(
        stepMotorTask,       
        "StepMotorTask",     
        4000,                   
        &arm,                   
        3,                      
        &StepMotorTaskHandle,
        MOTOR_CONTROL_CORE      
    );

    if (taskCreated != pdPASS) {
        Serial.println("StepMotorTask creation failed!");
    } else {
        Serial.println("StepMotorTask creation success!");
    }

    // Creating updateJointTask
    taskCreated = xTaskCreatePinnedToCore(
        updateJointTask,       
        "UpdateJointTask",     
        4000,                   
        &arm,                   
        2,                      
        &UpdateJointTaskHandle,
        MOTOR_CONTROL_CORE      
    );

    if (taskCreated != pdPASS) {
        Serial.println("UpdateJointTask creation failed!");
    } else {
        Serial.println("UpdateJointTask creation success!");
    }

    // Creating communicationTask
    taskCreated = xTaskCreatePinnedToCore(
        communicationTask,          
        "CommunicationTask",        
        3000,                       
        &arm,                       
        1,                          
        &CommunicationTaskHandle,   
        COMMUNICATION_CORE          
    );

    if (taskCreated != pdPASS) {
        Serial.println("CommunicationTask creation failed!");
    } else {
        Serial.println("CommunicationTask creation success!");
    }

    // Motor timer for stepMotorTask (2000 Hz)
    const esp_timer_create_args_t motor_timer_args = {
        .callback = &onMotorTimer,
        .arg = NULL,
        .name = "motor_timer"
    };
    esp_timer_create(&motor_timer_args, &motor_timer);
    esp_timer_start_periodic(motor_timer, 500); // 2000 Hz
}

void loop() {
    // Empty loop
}
