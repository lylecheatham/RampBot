#ifndef CONST_H
#define CONST_H

enum MotorNum { MotorA, MotorB };
#define STD_SPEED 50 // RPM
#define FIFO_RATE 50 //Sample Rate for IMU

/* Useful Ratios */
//#define PI 3.1415
//#define RAD_TO_DEG 57.3
#define RAD_TO_DEG_RND 57
//#define DEG_TO_RAD 0.0175

/* Mechanical Values */
#define D_O_WHEEL 50 //mm
#define D_I_WHEEL 30 //mm
#define RPM_TO_VO (0.26) //cm/s
#define RPM_TO_VI (0.16) //cm/s
#define WHEELBASE 180 //mm
#define COUNTS_REV 1203 //encoder counts per revolution

/* LEDs */
#define STD_LED 13

/* Motors */
#define M_STDBY 4  // Standby (pull high to use motors)

#define M_PWMA 5  // PWM Signals
#define M_PWMB 10

#define M_AIN1 7  // Two Direction Inputs
#define M_AIN2 6

#define M_BIN1 9  // Two Direction inputs
#define M_BIN2 8

#define M_AENC1 12  // Encoder
#define M_AENC2 11

#define M_BENC1 14  // Encoder
#define M_BENC2 15

/* IMU */
#define IMU_INT 17

/* Ultrasonic */

#define U_PING 2

/* Servo */

#define S_PULSE 3

#endif
