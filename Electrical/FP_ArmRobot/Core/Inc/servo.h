/*
 * servo.h
 *
 *  Created on: Feb 8, 2024
 *      Author: rayha
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

// Clamp rotation angle between 0 and 180 degree
#define CLAMPDEG(R) (R < 0) ? 0 : (R > 180) ? 180 : R

// Servo structure
typedef struct {
	uint8_t rotDegree; // Angle
	uint32_t minCCR, maxCCR; // CCR limit
	__IO uint32_t *CCR; // Pointer to timer CCR
} Servo_t;

// Get functions
uint8_t Servo_getRotation(Servo_t *servo) {
	return servo->rotDegree;
}

uint32_t Servo_getMinCCR(Servo_t *servo) {
	return servo->minCCR;
}

uint32_t Servo_getMaxCCR(Servo_t *servo) {
	return servo->maxCCR;
}

uint32_t Servo_getCCR(Servo_t *servo) {
	return *(servo->CCR);
}

// Set functions
void Servo_setCCR(Servo_t *servo, uint32_t value) {
	uint32_t rescaled = (value * 180)
				/ (servo->maxCCR - servo->minCCR); // Linear interpolation
	servo->rotDegree = rescaled;

	*(servo->CCR) = value;
}

void Servo_setRotation(Servo_t *servo, uint8_t degree) {
	uint32_t rescaled = ((CLAMPDEG(degree) * (servo->maxCCR - servo->minCCR))
				/ (180)) + servo->minCCR; // Linear interpolation
	servo->rotDegree = degree;

	*(servo->CCR) = rescaled;
}

// Utility functions
void Servo_calibrate(Servo_t *servo, uint32_t newMin, uint32_t newMax) {
	servo->minCCR = newMin;
	servo->maxCCR = newMax;
	Servo_setRotation(servo, servo->rotDegree);
}

void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, TIM_TypeDef *TIM,
		uint8_t channel, uint32_t min, uint32_t max) {
	// Assign CCR pointer and start timer PWM
	switch (channel) {
	case 1:
		servo->CCR = &(TIM->CCR1);
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
		break;
	case 2:
		servo->CCR = &(TIM->CCR2);
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
		break;
	case 3:
		servo->CCR = &(TIM->CCR3);
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
		break;
	case 4:
		servo->CCR = &(TIM->CCR4);
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
		break;
	}

	servo->rotDegree = 0;
	servo->minCCR = min;
	servo->maxCCR = max;

	Servo_calibrate(servo, servo->minCCR, servo->maxCCR);
}

extern Servo_t dof1, dof2, dof3, dof4, grp1;

#endif /* INC_SERVO_H_ */
