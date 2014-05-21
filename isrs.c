// Authored by Farrell Farahbod, last revised on 2014-05-20
// This file is released into the public domain

#include "externs.h"
#include "f0lib/f0lib_timers.h"
#include "f0lib/f0lib_rf_cc2500.h"
#include "f0lib/f0lib_debug.h"
#include "f0lib/f0lib_rs232.h"
#include "f0lib/f0lib_accel_magn_lsm303dlhc.h"
#include "f0lib/f0lib_gyro_l3gd20.h"
#include <math.h>
#include <stdlib.h>

void updateMotors(void);

// ISR for External Interrupts 0 - 1
void EXTI0_1_IRQHandler() {
	if((EXTI->PR & (1<<0)) != 0) {	// interrupt occured on EXTI 0
		EXTI->PR = (1<<0);			// handling EXTI 0
	
		////////////////////////////////////////////////////////////////////////////////
		//                                                                            //
		//   EXTI 0:   Left wheel encoder                                             //
		//                                                                            //
		//   Pins:     PA0, PA1                                                       //
		//   Globals:  leftDrift, leftVelocity                                        //
		//                                                                            //
		//   Current and previous pin states and times are tracked to determine       //
		//      velocity and direction of travel for the left wheel.                  //
		//                                                                            //
		//   Forward motion (left wheel turning counter-clockwise) results in         //
		//      positive drift.                                                       //
		//                                                                            //
		////////////////////////////////////////////////////////////////////////////////
		debug_io(PC4, HIGH);

		static uint8_t prevA = 0;
		static uint8_t prevB = 0;
		static uint8_t currA = 0;
		static uint8_t currB = 0;
		static uint32_t prevTime = 0;
		static uint32_t currTime = 0;

		prevA = currA;
		prevB = currB;
		prevTime = currTime;
		currTime = millis;
		
		// update current state
		if((GPIOA->IDR & (1<<0)) == 0) currA = 0; else currA = 1;
		if((GPIOA->IDR & (1<<1)) == 0) currB = 0; else currB = 1;

		// only calculate drift and velocity if throttle/steering inputs are near zero
		#define DEADBAND 40
		if(gimbalY < DEADBAND && gimbalY > -DEADBAND && gimbalX < DEADBAND && gimbalX > -DEADBAND) {

			if((prevA == 0 && prevB == 1 && currA == 1 && currB == 0) || (prevA == 1 && prevB == 0 && currA == 0 && currB == 1)) {
				// drifting backward
				leftDrift--;
				leftVelocity = -4000 / (int32_t) (currTime - prevTime);
			}

			else if((prevA == 0 && prevB == 0 && currA == 1 && currB == 1) || (prevA == 1 && prevB == 1 && currA == 0 && currB == 0)) {
				// drifting forward
				leftDrift++;
				leftVelocity = 4000 / (int32_t) (currTime - prevTime);
			}
		} else {
			leftDrift = 0;
			leftVelocity = 0;
		}
		
		debug_io(PC4, LOW);
	} // end of EXTI 0
} // end of ISR


// ISR for ADC
void ADC1_COMP_IRQHandler() {

	////////////////////////////////////////////////////////////////////////////////
	//                                                                            //
	//   ADC:      Wired gimbal assembly (not used when RF remote is used.)       //
	//                                                                            //
	//   Pins:     PB0, PB1                                                       //
	//   Globals:  gimbalX, gimbalY                                               //
	//                                                                            //
	//   Averages the first n samples to calibrate the gimbal during power up.    //
	//                                                                            //
	////////////////////////////////////////////////////////////////////////////////
	debug_io(PC4, HIGH);
	
	static uint16_t i = 0;
	static uint32_t tempX, tempY = 0;		// raw ADC readings
	static uint16_t xOffset, yOffset;
	static uint8_t currentChannel = 0;		// x=1, y=0

	// calibrate the gimbal at power up: average the first n samples and store the offsets.
	#define ADC_SAMPLES 256
	if(i < ADC_SAMPLES) {
		// accumulate the first n samples
		if(currentChannel == 0) {
			tempY += ADC1->DR;
			currentChannel = 1;
			i++;
		} else if(currentChannel == 1) {
			tempX += ADC1->DR;
			currentChannel = 0;
			i++;
		}
	} else if(i == ADC_SAMPLES){
		// calculate the offsets
		xOffset = tempX / (ADC_SAMPLES / 2);
		yOffset = tempY / (ADC_SAMPLES / 2);
		tempY = ADC1->DR;
		currentChannel = 1;
		i++;
	} else {
		// normal ADC readings
		if(currentChannel == 0) {
			tempY = ADC1->DR;
			gimbalY = (tempY - yOffset) * -1;
			currentChannel = 1;
		} else if(currentChannel == 1) {
			tempX = ADC1->DR;
			gimbalX = tempX - xOffset;
			currentChannel = 0;
		}
	}
	debug_io(PC4, LOW);
} // end of ISR

// ISR for External Interrupts 4 - 15
void EXTI4_15_IRQHandler() {
	if((EXTI->PR & (1<<5)) != 0) {	// interrupt occured on EXTI 5
		EXTI->PR = (1<<5);			// handling EXTI 5

		////////////////////////////////////////////////////////////////////////////////
		//                                                                            //
		//   EXTI 5:   TI CC2500 RF module                                            //
		//                                                                            //
		//   Pins:     PA5, PA6, PA7, PF4, PF5                                        //
		//   Globals:  gimbalX, gimbalY, knobLeft, knobMiddle, knobRight              //
		//                                                                            //
		//   Checks if the CRC test passed, and if so, reads the data from the FIFO.  //
		//                                                                            //
		////////////////////////////////////////////////////////////////////////////////
		debug_io(PC4, HIGH);
		 
		int16_t gimX, gimY, knoL, knoM, knoR;
		gimX = gimY = knoL = knoM = knoR = 0;

		// ISR was called due to a falling edge on the GDO0 pin, indicating that a packet has been received
		// *HOWEVER* that does not mean the packet was valid (CRC test passed)
		// If the CRC check fails the RX FIFO will be automatically flushed by the CC2500.

		// Check the rxbytes register to see if the FIFO was flushed or has overflowed
		uint8_t rxbytes = cc2500_write_register(RXBYTES | READ_BYTE, 0x00);

		if((rxbytes & 0b10000000) != 0) { // bit7 set indicates rxfifo overflow
			cc2500_send_strobe(SFRX);
			cc2500_send_strobe(SRX);
		} else { // receive the bytes and print them out
			uint8_t i = 0;
			while(rxbytes-- > 0) {
				uint8_t currentByte = cc2500_write_register(FIFO | READ_BYTE, 0x00);

				if(i == 0) {
					// ignore 0x69 address byte
				} else if(i == 1) {
					// gimbalX byte0
					gimX = currentByte;
				} else if(i == 2) {
					// gimbalX byte1
					gimX |= currentByte << 8;
					gimbalX = gimX;
				} else if(i == 3) {
					// gimbalY byte0
					gimY = currentByte;
				} else if(i == 4) {
					// gimbalY byte1
					gimY |= currentByte << 8;
					gimbalY = gimY;
				} else if(i == 5) {
					// knobLeft byte0
					knoL = currentByte;
				} else if(i == 6) {
					// knobLeft byte1
					knoL |= currentByte << 8;
					knobLeft = knoL;
				} else if(i == 7) {
					// knobMiddle byte0
					knoM = currentByte;
				} else if(i == 8) {
					// knobMiddle byte1
					knoM |= currentByte << 8;
					knobMiddle = knoM;
				} else if(i == 9) {
					// knobRight byte0
					knoR = currentByte;
				} else if(i == 10) {
					// knobRight byte1
					knoR |= currentByte << 8;
					knobRight = knoR;
				} else if(i == 11) {
					// ignore 0x00 dummy byte
				}
				i++;
			}
			cc2500_send_strobe(SRX);
		}
		debug_io(PC4, LOW);
	} // end of EXTI 5

	if((EXTI->PR & (1<<9)) != 0) {	// interrupt occured on EXTI 9
		EXTI->PR = (1<<9);			// handling EXTI 9
		
		////////////////////////////////////////////////////////////////////////////////
		//                                                                            //
		//   EXTI 9:   STM LSM303DLHC Accelerometer                                   //
		//                                                                            //
		//   Pins:     PB6, PB7, PB9                                                  //
		//   Globals:  accelX, accelY, accelZ, accelXavg, accelYavg, accelZavg,       //
		//             accelGravityAngle                                              //
		//                                                                            //
		//   Reads the values, calculates rolling averages, calculates angle from     //
		//      gravity and trims gyroAngleY by using a complementary filter.         //
		//                                                                            //
		////////////////////////////////////////////////////////////////////////////////
		debug_io(PC4, HIGH);
		
		int16_t tempX, tempY, tempZ;
		
		// sample the accelerometer, and update globals
		accel_lsm303dlhc_get_xyz(&tempX, &tempY, &tempZ);
		tempZ -= 1050; // fix offset
		accelX = tempX;
		accelY = tempY;
		accelZ = tempZ;

		// use an n-sample rolling average, and update globals
		#define ACC_SAMPLES 16
		static uint8_t i = 0; // current index
		static int16_t samplesX[ACC_SAMPLES] = {0};
		static int16_t samplesY[ACC_SAMPLES] = {0};
		static int16_t samplesZ[ACC_SAMPLES] = {0};
		if(i == ACC_SAMPLES)
			i = 0;
		samplesX[i] = tempX;
		samplesY[i] = tempY;
		samplesZ[i] = tempZ;
		i++;
		
		uint8_t j = 0;
		int32_t sumX, sumY, sumZ;
		sumX = sumY = sumZ = 0;
		
		for(j = 0; j < ACC_SAMPLES; j++) {
			sumX += samplesX[j];
			sumY += samplesY[j];
			sumZ += samplesZ[j];
		}
		accelXavg = sumX / ACC_SAMPLES;
		accelYavg = sumY / ACC_SAMPLES;
		accelZavg = sumZ / ACC_SAMPLES;

		// calc mean absolute deviations
		int16_t xmad, ymad, zmad;
		xmad = ymad = zmad = 0;
		
		for(j = 0; j < ACC_SAMPLES; j++) {
			xmad += abs(samplesX[j] - accelXavg);
			ymad += abs(samplesY[j] - accelYavg);
			zmad += abs(samplesZ[j] - accelZavg);
		}
		accelXmad = xmad / ACC_SAMPLES;
		accelYmad = ymad / ACC_SAMPLES;
		accelZmad = zmad / ACC_SAMPLES;
		
		// calculate angle from gravity
		float radians = atan2f((float) accelZavg, (float) accelXavg);
		accelGravityAngle = (int32_t) (radians * 6548.089087);	// (radians * 57.295779513) converts rads to degs.
																// then *(1000/8.75) to get gyro units of 1 count = 8.75mDeg
		// trim gyroAngleY
		if(accelZmad < 800 && accelXmad < 800) {
			int32_t delta = gyroAngleY/190 - accelGravityAngle;
			gyroAngleY -= (delta / 32) * 190;
		}
		
		debug_io(PC4, LOW);
	} // end of EXTI 9

	if((EXTI->PR & (1<<10)) != 0) {	// interrupt occured on EXTI 10
		EXTI->PR = (1<<10);			// handling EXTI 10
		
		////////////////////////////////////////////////////////////////////////////////
		//                                                                            //
		//   EXTI 10:  STM L3GD20 Gyro                                                //
		//                                                                            //
		//   Pins:     PB6, PB7, PC10                                                 //
		//   Globals:  gyroRateX, gyroRateY, gyroRateZ,                               //
		//             gyroAngleX, gyroAngleY, gyroAngleZ                             //
		//                                                                            //
		//   Skips the first n sameples, calcs the offset from the next n samples,    //
		//      then takes normal samples, updates globals and calls updateMotors()   //
		//                                                                            //
		////////////////////////////////////////////////////////////////////////////////
		debug_io(PC4, HIGH);

		static uint16_t i = 0;
		static int16_t tempX, tempY, tempZ;
		static int32_t gyroOffsetX = 0;
		static int32_t gyroOffsetY = 0;
		static int32_t gyroOffsetZ = 0;

		#define GYRO_SKIP_SAMPLES 190	// skip first n samples
		#define GYRO_OFFSET_SAMPLES 128	// then use n samples to determine offset

		if(i < GYRO_SKIP_SAMPLES) {
			// read and ignore the first n samples
			gyro_l3gd20_get_xyz(&tempX, &tempY, &tempZ);
			i++;
		} else if(i < (GYRO_SKIP_SAMPLES + GYRO_OFFSET_SAMPLES)) {
			// take samples to determine offset
			gyro_l3gd20_get_xyz(&tempX, &tempY, &tempZ);
			gyroOffsetX += tempX;
			gyroOffsetY += tempY;
			gyroOffsetZ += tempZ;
			i++;
		} else if(i == (GYRO_SKIP_SAMPLES + GYRO_OFFSET_SAMPLES)) {
			// read and ignore the current sample
			gyro_l3gd20_get_xyz(&tempX, &tempY, &tempZ);
			
			// calculate offset (mean average of offset samples)
			gyroOffsetX /= GYRO_OFFSET_SAMPLES;
			gyroOffsetY /= GYRO_OFFSET_SAMPLES;
			gyroOffsetZ /= GYRO_OFFSET_SAMPLES;
			
			// turn on the green LED to indicate the gyro is calibrated
			gpio_high(PC1);
			
			i++;
		} else {
			// normal samples
			gyro_l3gd20_get_xyz(&tempX, &tempY, &tempZ);
			gyroRateX = tempX - gyroOffsetX;
			gyroRateY = tempY - gyroOffsetY;
			gyroRateZ = tempZ - gyroOffsetZ;

			gyroAngleX += (int32_t) gyroRateX;
			gyroAngleY += (int32_t) gyroRateY;
			gyroAngleZ += (int32_t) gyroRateZ;

			// mod angles to +/- 360 degrees
			gyroAngleX %= 41143 * 190;
			gyroAngleY %= 41143 * 190;
			gyroAngleZ %= 41143 * 190;

			updateMotors();
		}

		// if DRDY pin is still high, trigger this ISR again
		if((GPIOC->IDR & (1<<10)) != 0)
			EXTI->SWIER = (1 << 10);

		debug_io(PC4, LOW);
	} // end of EXTI 10

	if((EXTI->PR & (1<<13)) != 0) {	// interrupt occured on EXTI 13
		EXTI->PR = (1<<13);			// handling EXTI 13

		////////////////////////////////////////////////////////////////////////////////
		//                                                                            //
		//   EXTI 13:  Right wheel encoder                                            //
		//                                                                            //
		//   Pins:     PB13, PB14                                                     //
		//   Globals:  rightDrift, rightVelocity                                      //
		//                                                                            //
		//   Current and previous pin states and times are tracked to determine       //
		//      velocity and direction of travel for the left wheel.                  //
		//                                                                            //
		//   Forward motion (right wheel turning clockwise) results in positive       //
		//      drift.                                                                //
		//                                                                            //
		////////////////////////////////////////////////////////////////////////////////
		debug_io(PC4, HIGH);
		
		static uint8_t prevA = 0;
		static uint8_t prevB = 0;
		static uint8_t currA = 0;
		static uint8_t currB = 0;
		static uint32_t prevTime = 0;
		static uint32_t currTime = 0;

		prevA = currA;
		prevB = currB;
		prevTime = currTime;
		currTime = millis;

		// update current state
		if((GPIOB->IDR & (1<<13)) == 0) currA = 0; else currA = 1;
		if((GPIOB->IDR & (1<<14)) == 0) currB = 0; else currB = 1;

		// only calculate drift and velocity if throttle/steering inputs are near zero
		#define DEADBAND 40
		if(gimbalY < DEADBAND && gimbalY > -DEADBAND && gimbalX < DEADBAND && gimbalX > -DEADBAND) {

			if((prevA == 0 && prevB == 1 && currA == 1 && currB == 0) || (prevA == 1 && prevB == 0 && currA == 0 && currB == 1)) {
				// drifting backward
				rightDrift--;
				rightVelocity = -4000 / (int32_t) (currTime - prevTime);
			}

			else if((prevA == 0 && prevB == 0 && currA == 1 && currB == 1) || (prevA == 1 && prevB == 1 && currA == 0 && currB == 0)) {
				// drifting forward
				rightDrift++;
				rightVelocity = 4000 / (int32_t) (currTime - prevTime);
			}
		} else {
			rightDrift = 0;
			rightVelocity = 0;
		}
		
		debug_io(PC4, LOW);
	} // end of EXTI 13
}

// SysTick is called at 1kHz
void SysTick_Handler() {
	millis++;

	// update wheel velocities, bringing them toward zero
	if(millis % 4 == 0) {	
		if(leftVelocity > 0)
			leftVelocity--;
		else if(leftVelocity < 0)
			leftVelocity++;
		if(rightVelocity > 0)
			rightVelocity--;
		else if(rightVelocity < 0)
			rightVelocity++;
	}
}

// determine the proportional, integral and derivative errors
void calcPID() {
	static int32_t currAngle = 0;
	static int32_t prevAngle = 0;

	pFactor = -25 + (((int32_t) knobLeft - 2048) / 128);
	iFactor = 1 + ((int32_t) knobMiddle / 256);
	dFactor = 1 + ((int32_t) knobRight / 256);
	int32_t throttleFactor = 1;

	// When no throttle is applied, the robot should be perfectly vertical (angle = 0) if it has not drifted
	// When throttle is applied, the robot should be tilted proportionally.

	int32_t desiredAngle = (((int32_t) gimbalY) / throttleFactor) - (rightDrift * 16) - (rightDrift * abs(rightVelocity/64));
	int32_t currentAngle = -gyroAngleY / 190;
	
	proportionalError = (desiredAngle - currentAngle) / pFactor;

	prevAngle = currAngle;
	currAngle = proportionalError;
	
	derivativeError = (currAngle - prevAngle) / dFactor;

	integralError += proportionalError / iFactor;
	if(integralError > 200) integralError = 200;
	if(integralError < -200) integralError = -200;
}

// PWM the motors as needed
void updateMotors() {

	int32_t leftSpeed, rightSpeed;
	int32_t steeringFactor = (int32_t) (gimbalX / -200);

	calcPID();

	if(proportionalError > 120 || proportionalError < -120) // angle too extreme, disable motors
		leftSpeed = rightSpeed = 0;
	else {
		leftSpeed = proportionalError + integralError + derivativeError - steeringFactor;
		rightSpeed = proportionalError + integralError + derivativeError + steeringFactor;
	}
	
	// clip speeds to +/- 100
	if(leftSpeed > 100) leftSpeed = 100;
	else if(leftSpeed < -100) leftSpeed = -100;
	if(rightSpeed > 100) rightSpeed = 100;
	else if(rightSpeed < -100) rightSpeed = -100;

	// update the globals
	throttleL = leftSpeed;
	throttleR = rightSpeed;

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	////                                                                  ////
	////           BE CAREFUL WHEN MODIFYING THE CODE BELOW!              ////
	////       If both h-bridge inputs are high at the same time,         ////
	////            the power supply will be SHORT CIRCUITED.             ////
	////              THERE IS NO FUSE! THE FETS WILL BURN!               ////
	////                                                                  ////
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////

	if(leftSpeed == 0) { // motor stopped
		timer_pwm_value(TIM1, CH1, 0);
		timer_pwm_value(TIM1, CH2, 0);
	} else if(leftSpeed > 0) {
		timer_pwm_value(TIM1, CH2, 0);
		timer_pwm_value(TIM1, CH1, leftSpeed);
	} else if(leftSpeed < 0) {
		timer_pwm_value(TIM1, CH1, 0);
		timer_pwm_value(TIM1, CH2, leftSpeed * -1);
	}

	if(rightSpeed == 0) { // motor stopped
		timer_pwm_value(TIM1, CH3, 0);
		timer_pwm_value(TIM1, CH4, 0);
	} else if(rightSpeed > 0) {
		timer_pwm_value(TIM1, CH4, 0);
		timer_pwm_value(TIM1, CH3, rightSpeed);
	} else if(rightSpeed < 0) {
		timer_pwm_value(TIM1, CH3, 0);
		timer_pwm_value(TIM1, CH4, rightSpeed * -1);
	}

}
