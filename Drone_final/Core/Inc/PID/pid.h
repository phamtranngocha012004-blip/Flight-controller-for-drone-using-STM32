/*
 * pid.h
 *
 *  Created on: Jan 16, 2026
 *      Author: phamt
 */

#ifndef INC_PID_PID_H_
#define INC_PID_PID_H_

/**********************************************************************************************
 * INCLUDE
 *********************************************************************************************/
#include "MPU_6050/mpu6050.h"

/**********************************************************************************************
 * VARIABLE
 *********************************************************************************************/

/* PID Structure */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float tichPhan;
    float saiSoTruoc;
    float doDoTruoc;
    float outMin;
    float outMax;
    float heSoLocD;
    float daoHamTruoc;
    uint8_t expo;
} PID_t;

/* External Variables */
extern PID_t pidGocRoll, pidGocPitch;
extern PID_t pidTocDoRoll, pidTocDoPitch;
extern uint16_t m1, m2, m3, m4;


#define THROTTLE_MIN  1000.0f
#define THROTTLE_MAX  1800.0f



/*
 * brief.....Initialize all PID controllers (Outer and Inner loops)
 * param.....None
 * Retval....None
 */
void PID_Init(void);

/*
 * brief.....Calculate non-linear error for exponential sensitivity
 * param.....e: raw error, expo: exponential factor
 * Retval....Mapped error value
 */
float nonlinear_error(float e, float expo);

/*
 * brief.....Update PID controller state and calculate output
 * param.....pid: pointer to PID struct, setpoint: target, measurement: feedback, dt: time step
 * Retval....Calculated control output
 */
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);

/*
 * brief.....Scale PID sensitivity based on current throttle
 * param.....throttle: current base throttle value
 * Retval....Scaling factor (0.1 to 1.2)
 */
float throttleScale(float throttle);

/*
 * brief.....Limit the input value within the THROTTLE_MIN and THROTTLE_MAX range
 * param.....duty: The input float value to be clamped
 * Retval....The limited value (clamped between MIN and MAX)
 */
float limit(float duty);

/*
 * brief.....Main flight control loop: Nested PID and Motor Mixer
 * param.....dt: time step, throttleCoBan: base throttle, setpointRoll/Pitch: target angles, yaw: yaw correction
 * Retval....None
 */
void vongDieuKhien(float dt, float throttleCoBan, float setpointRoll, float setpointPitch, float yaw);

/*
 * brief.....Reset PID integral and derivative states to zero
 * param.....pid: pointer to PID struct
 * Retval....None
 */
void PID_Reset(PID_t *pid);


#endif /* INC_PID_PID_H_ */
