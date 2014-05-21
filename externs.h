// Authored by Farrell Farahbod, last revised on 2014-05-20
// This file is released into the public domain

#include "stm32f0xx.h"

// Gyro
extern volatile int16_t gyroRateX;
extern volatile int16_t gyroRateY;
extern volatile int16_t gyroRateZ;
extern volatile int32_t gyroAngleX;
extern volatile int32_t gyroAngleY;
extern volatile int32_t gyroAngleZ;

// Accelerometer
extern volatile int16_t accelX;				// current samples
extern volatile int16_t accelY;
extern volatile int16_t accelZ;
extern volatile int16_t accelXavg;			// rolling averages
extern volatile int16_t accelYavg;
extern volatile int16_t accelZavg;
extern volatile int16_t accelXmad;			// mean absolute deviation of rolling averages
extern volatile int16_t accelYmad;
extern volatile int16_t accelZmad;
extern volatile int32_t accelGravityAngle;	// Calculated angle toward gravity

// Gimbal (0 = stick centered)
extern volatile int16_t gimbalX;
extern volatile int16_t gimbalY;

// Knobs
extern volatile int16_t knobLeft;
extern volatile int16_t knobMiddle;
extern volatile int16_t knobRight;

// Wheel encoders
extern volatile int32_t leftDrift;
extern volatile int32_t rightDrift;
extern volatile int32_t leftVelocity;
extern volatile int32_t rightVelocity;

// Motor speeds
extern volatile int16_t throttleL;
extern volatile int16_t throttleR;

// Errors
volatile int32_t proportionalError;
volatile int32_t integralError;
volatile int32_t derivativeError;
volatile int32_t pFactor;
volatile int32_t iFactor;
volatile int32_t dFactor;

// Time since power up
extern volatile uint32_t millis;
