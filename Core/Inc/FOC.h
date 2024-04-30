/*
 * FOC.h
 *
 *  Created on: Feb 8, 2024
 *      Author: Ann
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#define ARM_MATH_CM4

#include "stm32g4xx_hal.h"
#include "main.h"
#include "arm_math.h"

#include <stdint.h>

#define PI_1_3  1.0471975512f
#define PI_2_3  2.0943951024f
#define PI_4_3  4.1887902048f
#define PI_5_3  5.235987756f
#define PI_2    6.2831853072f
#define PI_1_2  1.5707963267f
#define PI_3_2  4.7123889804f

#define D_SQRT3  0.5773502692f
#define D2_SQRT3 1.1547005384f

#define AS5600_Address       0x36проверять
#define ANGLE_REG            0x0E //first 4 bits and next byte - 8 bits

#define ANGLECOM  0xFFFF //0x3FFF  //14 bit
#define SETTINGS1 0x0018 // 10000000  ABI, DAE_COMP, NO_PWM, DIR=0
#define SETTINGS2 0x0019 // 00011000  no hysteresis, max resolution ABI

#define encConst          0.0003834952f  //2pi/2^14
#define curSenseConst     0.0016113281f // shunt resistor 10mOm = 0.01, gain =50, INA240A1, (max 5A - 2.5V) // 2 / 4096 * 3.3
#define PERIOD            0.00005f

#define ELEM_SWAP(a,b) { register float t=(a);(a)=(b);(b)=t; }

float normalizeAngle(float angle);
float saturate(float a, float amin, float amax);

void InitMotor(struct Motor* m, CORDIC_HandleTypeDef* hcordic);
void InitEncoder(struct Motor* m);
void processEncoderData(struct Motor* m);
void mainLoop(struct Motor* m);
void sendByUart(struct Motor* m, struct SentData* sentData);
void getByUart(struct Motor* m, struct RefData* refData);
// button
void changeEn(struct Motor* m);
void stopRun(struct Motor* m);
void changeSpeed(struct Motor* m);
void changeAngle(struct Motor* m);

#endif /* INC_FOC_H_ */
