/*
 * mpu6050.h
 *
 *  Created on: Jan 16, 2026
 *      Author: phamt
 */

#ifndef INC_MPU_6050_MPU6050_H_
#define INC_MPU_6050_MPU6050_H_


/**********************************************************************************************
 * INCLUDE
 *********************************************************************************************/
#include <stdint.h>
#include <math.h>
#include "i2c/i2c.h"


/**********************************************************************************************
 * VARIABLE
 *********************************************************************************************/

#define MPU_ADDR 0x68     // I2C Address of MPU6050
#define RTD      57.2957f // Radian to Degree conversion factor


/* ---------- MPU6050 Raw Data ---------- */
extern int16_t gx;                       // Raw Gyroscope X-axis data
extern int16_t gy;                       // Raw Gyroscope Y-axis data
extern int16_t gz;                       // Raw Gyroscope Z-axis data
extern int16_t ax;                       // Raw Accelerometer X-axis data
extern int16_t ay;                       // Raw Accelerometer Y-axis data
extern int16_t az;                       // Raw Accelerometer Z-axis data

/* ---------- MPU6050 Scaled Data ---------- */
extern float Ax;                             // Accelerometer X-axis (g)
extern float Ay;                             // Accelerometer Y-axis (g)
extern float Az;                             // Accelerometer Z-axis (g)
extern float Gx;                             // Gyroscope X-axis (deg/s)
extern float Gy;                             // Gyroscope Y-axis (deg/s)
extern float Gz;                             // Gyroscope Z-axis (deg/s)

/* ---------- Calculated Angles ---------- */
extern float roll_out;                       // Filtered Roll angle (degrees)
extern float pitch_out;                      // Filtered Pitch angle (degrees)

/**********************************************************************************************
 * API
 *********************************************************************************************/


/*
 * brief.....Initialize MPU6050 sensor via I2C
 * param.....None
 * Retval....None
 */
void MPU6050Init(void);

/*
 * brief.....Read Raw Gyroscope values and convert to float DPS
 * param.....None
 * Retval....None
 */
void MPU6050_Read_G(void);

/*
 * brief.....Read Raw Accelerometer values and convert to float G
 * param.....None
 * Retval....None
 */
void MPU_Read_A(void);

/*
 * brief.....Complementary filter to calculate stable Pitch and Roll
 * param.....Ax, Ay, Az: Accel data in g. Gx, Gy, Gz: Gyro data in DPS
 * Retval....None
 */
void Filter(float Ax, float Ay, float Az, float Gx, float Gy, float Gz);


#endif /* INC_MPU_6050_MPU6050_H_ */
