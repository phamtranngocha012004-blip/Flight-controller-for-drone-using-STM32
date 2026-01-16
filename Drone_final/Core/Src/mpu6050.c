

#include "MPU_6050/mpu6050.h"

/* ---------- MPU6050 Raw Data ---------- */
int16_t gx = 0;                       // Raw Gyroscope X-axis data
int16_t gy = 0;                       // Raw Gyroscope Y-axis data
int16_t gz = 0;                       // Raw Gyroscope Z-axis data
int16_t ax = 0;                       // Raw Accelerometer X-axis data
int16_t ay = 0;                       // Raw Accelerometer Y-axis data
int16_t az = 0;                       // Raw Accelerometer Z-axis data

/* ---------- MPU6050 Scaled Data ---------- */
float Ax;                             // Accelerometer X-axis (g)
float Ay;                             // Accelerometer Y-axis (g)
float Az;                             // Accelerometer Z-axis (g)
float Gx;                             // Gyroscope X-axis (deg/s)
float Gy;                             // Gyroscope Y-axis (deg/s)
float Gz;                             // Gyroscope Z-axis (deg/s)

/* ---------- Calculated Angles ---------- */
float roll_out;                       // Filtered Roll angle (degrees)
float pitch_out;                      // Filtered Pitch angle (degrees)


/**********************************************************************************************
 * API
 *********************************************************************************************/


/* ---------- MPU6050 Initialization ---------- */
void MPU6050Init(void)
{
    uint8_t check = 0;
    // Read WHO_AM_I register (0x75) to verify sensor identity
    i2c_ReadMulti(MPU_ADDR, 0x75, 1, &check);

    if (check == 0x68) // 0x68 is the default WHO_AM_I value for MPU6050
    {
        // PWR_MGMT_1 (0x6B): Wake up MPU6050 by setting SLEEP bit to 0
        i2c_WriteByte(MPU_ADDR, 0x6B, 0x00);

        // SMPLRT_DIV (0x19): Set sample rate divider to 7 (Sample Rate = 1kHz / (1 + 7) = 125Hz)
        i2c_WriteByte(MPU_ADDR, 0x19, 0x07);

        // GYRO_CONFIG (0x1B): Set Full Scale Range to ±250 °/s (0x00)
        i2c_WriteByte(MPU_ADDR, 0x1B, 0x00);

        // ACCEL_CONFIG (0x1C): Set Full Scale Range to ±2g (0x00)
        i2c_WriteByte(MPU_ADDR, 0x1C, 0x00);
    }
}

/* ---------- Read Gyroscope Data ---------- */
void MPU6050_Read_G()
{
    uint8_t dataG[6];
    // Read 6 bytes of Gyro data starting from GYRO_XOUT_H (0x43)
    i2c_ReadMulti(MPU_ADDR, 0x43, 6, dataG);

    // Combine high and low bytes into 16-bit signed integers
    gx = (int16_t)((dataG[0] << 8) | dataG[1]);
    gy = (int16_t)((dataG[2] << 8) | dataG[3]);
    gz = (int16_t)((dataG[4] << 8) | dataG[5]);

    // Convert raw values to Degrees Per Second (DPS)
    // Scale factor 131.0 for ±250 °/s range
    Gx = (float)gx / 131.0;
    Gy = (float)gy / 131.0;
    Gz = (float)gz / 131.0;
}

/* ---------- Read Accelerometer Data ---------- */
void MPU_Read_A()
{
    uint8_t dataA[6];
    // Read 6 bytes of Accel data starting from ACCEL_XOUT_H (0x3B)
    i2c_ReadMulti(MPU_ADDR, 0x3B, 6, dataA);

    // Combine high and low bytes into 16-bit signed integers
    ax = (int16_t)((dataA[0] << 8) | dataA[1]);
    ay = (int16_t)((dataA[2] << 8) | dataA[3]);
    az = (int16_t)((dataA[4] << 8) | dataA[5]);

    // Convert raw values to g-force (g)
    // Scale factor 16384.0 for ±2g range
    Ax = (float)ax / 16384.0;
    Ay = (float)ay / 16384.0;
    Az = (float)az / 16384.0;
}

/* ---------- Complementary Filter ---------- */
void Filter(float Ax, float Ay, float Az, float Gx, float Gy, float Gz)
{
    float pitchG;
    float rollG;
    float pitchA;
    float rollA;

    // Gyroscope integration (Angle = Previous_Angle + Angular_Velocity * dt)
    // dt = 10000us / 1000000.0s = 0.01s (10ms sampling time)
    pitchG = pitch_out + Gy * (10000 / 1000000.0f);
    rollG  = roll_out + Gx * (10000 / 1000000.0f);

    // Accelerometer angle calculation using trigonometry
    pitchA = atan2(Ay, sqrt(Ax * Ax + Az * Az)) * RTD; // RTD: Radian to Degree constant
    rollA  = atan2(Ax, sqrt(Ay * Ay + Az * Az)) * RTD;

    // Complementary Filter: 98% Gyro (stable short-term) + 2% Accel (stable long-term)
    pitch_out = 0.98 * pitchG + 0.02 * pitchA;
    roll_out  = 0.98 * rollG + 0.02 * rollA;
}




