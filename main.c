// Authored by Farrell Farahbod, last revised on 2014-05-20
// This file is released into the public domain

#include "stm32f0xx.h"
#include "f0lib/f0lib.h"

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//   STM L3GD20 Gyro (I2C Address = 0x6B)                                     //
//                                                                            //
//   Pins:           PB6  = SCK                                               //
//                   PB7  = SDA                                               //
//                   PC10 = DRDY                                              //
//                                                                            //
//   Configuration:  +/-250dps, 190Hz, "70 bandwidth"                         //
//                                                                            //
//   Notes:          1 count (rate)  = 8.75mdps                               //
//                   1 count (angle) = 8.75md                                 //
//                   +X              = Turning left (yaw)                     //
//                   +Y              = Falling backward (pitch)               //
//                   +Z              = Falling to the right (roll)            //
//                                                                            //
//   EXTI 10 ISR:    Ignores the first 190 samples,                           //
//                   Averages the next 128 samples to determine offsets,      //
//                   Updates the global variables accordingly,                //
//                   Calls updateMotors()                                     //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
volatile int16_t gyroRateX = 0;
volatile int16_t gyroRateY = 0;
volatile int16_t gyroRateZ = 0;
volatile int32_t gyroAngleX = 0;
volatile int32_t gyroAngleY = 2147000; // assume robot is powered up while on its back
volatile int32_t gyroAngleZ = 0;

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//   STM LSM303DLHC Accelerometer (I2C Address = 0x19)                        //
//                                                                            //
//   Pins:           PB6 = SCK                                                //
//                   PB7 = SDA                                                //
//                   PB9 = DRDY                                               //
//                                                                            //
//   Configuration:  +/-2G, 100Hz                                             //
//                                                                            //
//   Notes:          1 count = 0.061mG                                        //
//                   +X      = Moving upward                                  //
//                   +Y      = Strafing right                                 //
//                   +Z      = Moving forward                                 //
//                                                                            //
//   EXTI 9 ISR:     Corrects for offset in Z axis measurements,              //
//                   Updates the global variables accordingly,                //
//                   Uses mean of 16 samples for the rolling averages,        //
//                   Calculates the angle toward gravity, trims gyroAngleY    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
volatile int16_t accelX = 0;			// current samples
volatile int16_t accelY = 0;
volatile int16_t accelZ = 0;
volatile int16_t accelXavg = 0;			// rolling averages
volatile int16_t accelYavg = 0;
volatile int16_t accelZavg = 0;
volatile int16_t accelXmad = 0;			// mean absolute deviation of rolling averages
volatile int16_t accelYmad = 0;
volatile int16_t accelZmad = 0;
volatile int32_t accelGravityAngle = 0;	// calculated angle toward gravity

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//   State of the gimbal and knobs on the transmitter.                        //
//                                                                            //
//   If a transmitter will not be used, a gimbal can be attached directly:    //
//      X axis potentiometer = PB0                                            //
//      Y axis potentiometer = PB1                                            //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
volatile int16_t gimbalX = 0;
volatile int16_t gimbalY = 0;
volatile int16_t knobLeft = 0;
volatile int16_t knobMiddle = 0;
volatile int16_t knobRight = 0;

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//   Wheel positions are tracked with optical quadrature encoders.            //
//                                                                            //
//   Pins:           PA0  = Left Wheel A                                      //
//                   PA1  = Left Wheel B                                      //
//                   PB13 = Right Wheel A                                     //
//                   PB14 = Right Wheel B                                     //
//                                                                            //
//   Notes:          Each wheel has 40-step quadrature tracks.                //
//                                                                            //
//   EXTI 0,13 ISRs: Stores current and previous states of the A and B pins.  //
//                   Stores current and previous timestamps.                  //
//                   If no throttle or steering inputs are currently applied, //
//                     updates the drift/velocity globals.                    //
//                   If throttle or steering is applied, zeros drift/velocity.//
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
volatile int32_t leftDrift = 0;
volatile int32_t rightDrift = 0;
volatile int32_t leftVelocity = 0;
volatile int32_t rightVelocity = 0;

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//   Current throttle values for the left and right motors.                   //
//   updateMotors() writes to these globals to enable tracking via telemetry. //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
volatile int16_t throttleL = 0;
volatile int16_t throttleR = 0;

////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//   Current P, I and D errors and scaling factors.                           //
//   calcPID() writes to these globals to enable tracking via telemetry.      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
volatile int32_t proportionalError = 0;
volatile int32_t integralError = 0;
volatile int32_t derivativeError = 0;
volatile int32_t pFactor = 0;
volatile int32_t iFactor = 0;
volatile int32_t dFactor = 0;

// Time since power up
volatile uint32_t millis = 0;

int main() {
	// Call systick at 1kHz, it updates the millis variable
	SysTick_Config(SystemCoreClock / 1000);

	// LED1 (blue)
	gpio_setup(PC0, OUTPUT, PUSH_PULL, FIFTY_MHZ, NO_PULL, AF0);
	gpio_low(PC0);

	// LED2 (green)
	gpio_setup(PC1, OUTPUT, PUSH_PULL, FIFTY_MHZ, NO_PULL, AF0);
	gpio_low(PC1);

	// ISR-in-Progress output pin (for debugging use)
	gpio_setup(PC4, OUTPUT, PUSH_PULL, FIFTY_MHZ, NO_PULL, AF0);
	gpio_low(PC4);	

	// Setup the PWM timers for motor control via two h-bridges
	timer_pwm_setup(TIM1, 100, FOUR_CHANNELS, PA8, PA9, PA10, PA11); // 50us period (spec'd in half microseconds)
	timer_pwm_value(TIM1, CH1, 0);
	timer_pwm_value(TIM1, CH2, 0);
	timer_pwm_value(TIM1, CH3, 0);
	timer_pwm_value(TIM1, CH4, 0);
	
	// Setup RS232
	rs232_setup(USART2, PA2, 921600); // TX, 921600 Baud

	// Setup the gyro
	gyro_l3gd20_setup(I2C1, PB6, PB7, PC10); // SCK, SDA, DRDY as an EXTI
	
	// Setup the accelerometer
	accel_lsm303dlhc_setup(I2C1, PB6, PB7, PB9); // sck, sda, drdy

	// Setup the wheel encoders
	gpio_setup(PA0, INPUT, PUSH_PULL, FIFTY_MHZ, NO_PULL, AF0);		// left a
	gpio_setup(PA1, INPUT, PUSH_PULL, FIFTY_MHZ, NO_PULL, AF0);		// left b
	gpio_setup(PB13, INPUT, PUSH_PULL, FIFTY_MHZ, NO_PULL, AF0);	// right a
	gpio_setup(PB14, INPUT, PUSH_PULL, FIFTY_MHZ, NO_PULL, AF0);	// right b
	exti_setup(PA0, BOTH_EDGES);	// left wheel a
	exti_setup(PB13, BOTH_EDGES);	// right wheel a

	// Output HSI clock to PA8 (for debugging)
	//gpio_setup(PA8, AF, PUSH_PULL, FIFTY_MHZ, NO_PULL, AF0);	// PA8 as AF0 = MCO mode
	//RCC->CFGR |= RCC_CFGR_MCO_2 | RCC_CFGR_MCO_0;				// MCO = 101 = output the HSI

	// Setup the 2.4GHz RF module (only used when a gimbal is not attached)
	cc2500_setup(SPI1, PA5, PA6, PA7, PF4);	// clk, miso, mosi, cs
	exti_setup(PF5, FALLING_EDGE);			// GDO0 falls at end of packet
	cc2500_send_strobe(SRX);				// enter RX mode

	// Setup the ADC (only used when a gimbal is attached)
	//adc_setup(2, PB0, PB1);	

	// Turn on the blue LED to indicate setup procedure is complete
	gpio_high(PC0);

	// The ISRs (see isrs.c) handle all of the work. They update the globals and modify the motor speeds as needed.
	// No furthur action is required in main().

	// Constantly stream telemetry data via RS232
	while(1) {
		rs232_write_string(USART2, "\x1B[H"); // VT100 escape sequence to home the cursor

		rs232_print_graph(USART2, "RateX    ", gyroRateX);
		rs232_print_graph(USART2, "RateY    ", gyroRateY);
		rs232_print_graph(USART2, "RateZ    ", gyroRateZ);

		rs232_print_graph(USART2, "AngleX   ", gyroAngleX / 190);
		rs232_print_graph(USART2, "AngleY   ", gyroAngleY / 190);
		rs232_print_graph(USART2, "AngleZ   ", gyroAngleZ / 190);

		rs232_print_graph(USART2, "AccelX   ", accelX);
		rs232_print_graph(USART2, "AccelY   ", accelY);
		rs232_print_graph(USART2, "AccelZ   ", accelZ);

		rs232_print_graph(USART2, "AvgAcX   ", accelXavg);
		rs232_print_graph(USART2, "AvgAcY   ", accelYavg);
		rs232_print_graph(USART2, "AvgAcZ   ", accelZavg);

		rs232_print_graph(USART2, "AcXmad   ", accelXmad);
		rs232_print_graph(USART2, "AcYmad   ", accelYmad);
		rs232_print_graph(USART2, "AcZmad   ", accelZmad);

		rs232_print_graph(USART2, "Gravity  ", accelGravityAngle);

		rs232_print_graph(USART2, "GimbalX  ", gimbalX);
		rs232_print_graph(USART2, "GimbalY  ", gimbalY);

		rs232_print_graph(USART2, "KnobL    ", knobLeft);
		rs232_print_graph(USART2, "KnobM    ", knobMiddle);
		rs232_print_graph(USART2, "KnobR    ", knobRight);

		rs232_print_graph(USART2, "DriftL   ", leftDrift);
		rs232_print_graph(USART2, "DriftR   ", rightDrift);
		rs232_print_graph(USART2, "VelocL   ", leftVelocity);
		rs232_print_graph(USART2, "VelocR   ", rightVelocity);

		rs232_print_graph(USART2, "ThrottleL", throttleL);
		rs232_print_graph(USART2, "ThrottleR", throttleR);

		rs232_print_graph(USART2, "Millis   ", (int16_t) millis);

		rs232_print_graph(USART2, "PError   ", proportionalError);
		rs232_print_graph(USART2, "IError   ", integralError);
		rs232_print_graph(USART2, "DError   ", derivativeError);

		rs232_print_graph(USART2, "PFactor  ", pFactor);
		rs232_print_graph(USART2, "IFactor  ", iFactor);
		rs232_print_graph(USART2, "DFactor  ", dFactor);	
	}
}
