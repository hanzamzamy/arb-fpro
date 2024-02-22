/*
 * cmd_handler.h
 *
 *  Created on: Feb 8, 2024
 *      Author: rayha
 */

#ifndef INC_CMD_HANDLER_H_
#define INC_CMD_HANDLER_H_

#include "main.h"

// Servo selector function
Servo_t* servoSelect(uint8_t ID) {
	// select by ID using pointer
	switch (ID) {
	case 0:
		return &grp1;
	case 1:
		return &dof1;
	case 2:
		return &dof2;
	case 3:
		return &dof3;
	case 4:
		return &dof4;
		break;
	default:
		return NULL;
	}
}

/*Multi-target Servo Rotation*/
void servoSet(uint8_t servoID, uint8_t rotDeg) {
	Servo_t *selServo = servoSelect(servoID);
	if (selServo) {
		Servo_setRotation(selServo, rotDeg);
	}
}

/*Multi-target Servo CCR Write*/
void servoWrite(uint8_t servoID, uint32_t value) {
	Servo_t *selServo = servoSelect(servoID);
	if (selServo) {
		Servo_setCCR(selServo, value);
	}
}

/*Multi-target Servo Calibration*/
void servoCal(uint8_t servoID, uint32_t min, uint32_t max) {
	Servo_t *selServo = servoSelect(servoID);
	if (selServo) {
		Servo_calibrate(selServo, min, max);
	}
}

/*Command Parser*/
void parseCommand(char *arg) {
	char cmd, tgt[6], *token;
	uint8_t parSize = 0, tgtSize;
	uint32_t params[10];

	// Get command
	token = strtok(arg, " ");
	if (token == NULL)
		return;
	cmd = token[0];

	// Get target
	token = strtok(NULL, " ");
	if (token == NULL)
		return;
	strncpy(tgt, token, 6);
	tgtSize = strlen(tgt);

	// Get parameters
	token = strtok(NULL, " ");
	while (token != NULL) {
		params[parSize] = atoi(token);
		token = strtok(NULL, " ");
		parSize++;
	}

	// Command execution branch
	switch (cmd) {
	case 'C': //  Calibrate servo
		if (2 * tgtSize == parSize) {
			for (uint8_t i = 0; i < tgtSize; i += 2) {
				servoCal(tgt[i] - '0', params[i], params[i + 1]);
			}
		}
		break;
	case 'R': // Set servo by CCR value
		if (tgtSize == parSize) {
			for (uint8_t i = 0; i < tgtSize; i++) {
				servoWrite(tgt[i] - '0', params[i]);
			}
		}
		break;
	case 'S': // Set servo by angle
		if (tgtSize == parSize) {
			for (uint8_t i = 0; i < tgtSize; i++) {
				servoSet(tgt[i] - '0', params[i]);
			}
		}
		break;
	case 'L': // Request log
		process.transmit = 1;
		break;
	// Autoscan feature. Not working
//	case 'F':
//		if (parSize == 5) {
//			process.scan = 1;
//			process.scanMode = params[0];
//			process.rangeTh = params[1];
//			process.minScan = params[2];
//			process.maxScan = params[3];
//			process.scanInc = params[4];
//			process.currentCCR = process.minScan;
//			process.timestamp = 0;
//			process.capturedCCR = 0;
//
//			Servo_setCCR(&dof1, process.currentCCR);
//		}
//		break;
	}
}

#endif /* INC_CMD_HANDLER_H_ */
