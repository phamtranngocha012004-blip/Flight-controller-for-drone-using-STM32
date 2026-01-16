

#include "pid/pid.h"


/* ---------- PID Structure Definition ---------- */

/* ---------- PID Instances ---------- */
PID_t pidGocRoll, pidGocPitch;        // Outer loop PID (Angle/Level control)
PID_t pidTocDoRoll, pidTocDoPitch;    // Inner loop PID (Rate/Gyro control)

/* ---------- Motor PWM Values ---------- */
uint16_t m1;                          // Motor Front Left
uint16_t m2;                          // Motor Front Right
uint16_t m3;                          // Motor Rear Left
uint16_t m4;                          // Motor Rear Right


/* ---------- Initialize PID Coefficients ---------- */
void PID_Init(void)
{
    /* Outer Loop PID - Angle Control */
    pidGocRoll.Kp = 3.0f;
    pidGocRoll.Ki = 0.0f;
    pidGocRoll.Kd = 0.0f;

    pidGocRoll.tichPhan = 0;
    pidGocRoll.saiSoTruoc = 0;
    pidGocRoll.doDoTruoc = 0;

    pidGocRoll.outMin = -150.0f;   // Limit requested rotation rate (deg/s)
    pidGocRoll.outMax = 150.0f;

    pidGocRoll.heSoLocD = 0;
    pidGocRoll.daoHamTruoc = 0;
    pidGocRoll.expo = 1;

    /* Pitch outer loop uses same settings as Roll */
    pidGocPitch = pidGocRoll;

    /* Inner Loop PID - Rotation Rate Control */
    pidTocDoRoll.Kp = 0.12f;
    pidTocDoRoll.Ki = 0.02f;
    pidTocDoRoll.Kd = 0.003f;

    pidTocDoRoll.tichPhan = 0;
    pidTocDoRoll.saiSoTruoc = 0;
    pidTocDoRoll.doDoTruoc = 0;

    pidTocDoRoll.heSoLocD = 0.7f;
    pidTocDoRoll.daoHamTruoc = 0;
    pidTocDoRoll.expo = 0;

    pidTocDoPitch = pidTocDoRoll;
}

/* ---------- Exponential Error Mapping ---------- */
float nonlinear_error(float e, float expo)
{
    // Increases sensitivity as error grows larger
    return e * (1.0f + expo * fabs(e));
}

/* ---------- PID Calculation Update ---------- */
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
    float rawError = setpoint - measurement;
    float saiso;

    // Apply non-linear mapping if enabled
    if (pid->expo == 1)
    {
        saiso = nonlinear_error(rawError, 0.05f);
    } else {
        saiso = rawError;
    }

    // Integral calculation
    pid->tichPhan += saiso * dt;

    // Anti-windup: Clamp integral term to output limits
    float gioiHanTichPhan = fabs(pid->outMax);
    if (pid->tichPhan >  gioiHanTichPhan) pid->tichPhan =  gioiHanTichPhan;
    if (pid->tichPhan < -gioiHanTichPhan) pid->tichPhan = -gioiHanTichPhan;

    // Derivative calculation based on measurement (avoids derivative kick)
    float daoHamRaw = (measurement - pid->doDoTruoc) / dt;

    // Low-pass filter for the derivative term
    float daoHamLoc = pid->heSoLocD * pid->daoHamTruoc +
                      (1.0f - pid->heSoLocD) * daoHamRaw;

    pid->daoHamTruoc = daoHamLoc;
    pid->doDoTruoc = measurement;

    // Final PID output calculation
    float output =
        pid->Kp * saiso +
        pid->Ki * pid->tichPhan -
        pid->Kd * daoHamLoc;

    // Output saturation clamping
    if (output > pid->outMax) output = pid->outMax;
    if (output < pid->outMin) output = pid->outMin;

    pid->saiSoTruoc = saiso;
    return output;
}

/* ---------- Dynamic Scaling for PID Sensitivity ---------- */
float throttleScale(float throttle)
{
    float tMin = 1000.0f;
    float tMax = 1650.0f;

    if (throttle < tMin) throttle = tMin;
    if (throttle > tMax) throttle = tMax;

    /* Scales sensitivity based on throttle:
       Lower throttle -> higher sensitivity
       Higher throttle -> lower sensitivity (to prevent oscillations) */
    return 1.2f - (throttle - tMin) * (1.1f / (tMax - tMin));
}


/**
 * Limit the input value within the defined THROTTLE_MIN and THROTTLE_MAX range.
 */
float limit(float duty)
{
    float_t result; // Temporary variable to store the clamped value

    if (duty < THROTTLE_MIN)
    {
        // If input is below minimum threshold, set to THROTTLE_MIN
        result = THROTTLE_MIN;
    }
    else if (duty > THROTTLE_MAX)
    {
        // If input exceeds maximum threshold, set to THROTTLE_MAX
        result = THROTTLE_MAX;
    }
    else
    {
        // Input is within valid range
        result = duty;
    }

    return result;
}

/* ---------- Main Control Loop and Motor Mixer ---------- */
void vongDieuKhien(float dt,
                   float throttleCoBan,
                   float setpointRoll,
                   float setpointPitch,
                   float yaw)
{
    float gocRoll  = roll_out;
    float gocPitch = pitch_out;

    float tocDoRoll  = Gx;
    float tocDoPitch = Gy;

    float tocDoRoll_MongMuon;
    float tocDoPitch_MongMuon;

    float suaRoll;
    float suaPitch;

    float motorFRf, motorRRf, motorRLf, motorFLf;

    float error_pitch;
    float error_roll;

    float outMaxRoll;
    float outMaxPitch;

    // Calculate dynamic rate limits based on error and throttle scale
    error_pitch = fabs(setpointPitch - pitch_out);
    error_roll = fabs(setpointRoll - roll_out);
    outMaxRoll  = 10.0f + (error_roll  / 10.0f) * (200.0f - 10.0f) * throttleScale(throttleCoBan);
    outMaxPitch = 10.0f + (error_pitch / 10.0f) * (200.0f - 10.0f) * throttleScale(throttleCoBan);

    pidTocDoRoll.outMax  =  outMaxRoll;
    pidTocDoRoll.outMin  = -outMaxRoll;

    pidTocDoPitch.outMax =  outMaxPitch;
    pidTocDoPitch.outMin = -outMaxPitch;

    /* =================== OUTER LOOP: ANGLE PID =================== */
    tocDoRoll_MongMuon  = PID_Update(&pidGocRoll,  setpointRoll,  gocRoll,  dt);
    tocDoPitch_MongMuon = PID_Update(&pidGocPitch, setpointPitch, gocPitch, dt);

    /* =================== INNER LOOP: RATE PID =================== */
    suaRoll  = PID_Update(&pidTocDoRoll,  tocDoRoll_MongMuon,  tocDoRoll,  dt);
    suaPitch = PID_Update(&pidTocDoPitch, tocDoPitch_MongMuon, tocDoPitch, dt);

    /* ========================= MOTOR MIXER (X-FRAME) ========================= */
    motorFRf = throttleCoBan;
    motorRRf = throttleCoBan;
    motorRLf = throttleCoBan;
    motorFLf = throttleCoBan;

    /* ===== PITCH Correction ===== */
    motorFLf += suaPitch;   // Left side increase
    motorRLf += suaPitch;
    motorFRf -= suaPitch;   // Right side decrease
    motorRRf -= suaPitch;

    /* ===== ROLL Correction ===== */
    motorFLf += suaRoll;    // Front side increase
    motorFRf += suaRoll;
    motorRLf -= suaRoll;    // Rear side decrease
    motorRRf -= suaRoll;

    /* ===== YAW Correction ===== */
    motorFRf += yaw;        // M2 (CW) Increase
    motorRLf += yaw;        // M3 (CW) Increase
    motorFLf -= yaw;        // M1 (CCW) Decrease
    motorRRf -= yaw;        // M4 (CCW) Decrease

    // Clamp PWM values to valid hardware range
    motorFLf = limit(motorFLf);
    motorFRf = limit(motorFRf);
    motorRLf = limit(motorRLf);
    motorRRf = limit(motorRRf);

    m1 = (uint16_t)motorFLf;   // Front Left
    m2 = (uint16_t)motorFRf;   // Front Right
    m3 = (uint16_t)motorRLf;   // Rear Left
    m4 = (uint16_t)motorRRf;   // Rear Right
}

/* ---------- Reset PID States ---------- */
void PID_Reset(PID_t *pid)
{
    pid->tichPhan    = 0.0f;
    pid->saiSoTruoc  = 0.0f;
    pid->doDoTruoc   = 0.0f;
    pid->daoHamTruoc = 0.0f;
}


