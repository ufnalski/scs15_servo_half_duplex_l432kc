/*
 * scs15_servo.h
 *
 *  Created on: Aug 13, 2024
 *      Author: user
 */

#ifndef INC_SCS15_SERVO_H_
#define INC_SCS15_SERVO_H_

#define SCS15_UART &huart1
#define BIG_ENDIAN_PERIPHERAL 1

// Commands
#define SCS15_READ_DATA_COMMAND 0x02
#define SCS15_WRITE_DATA_COMMAND 0x03

// Registers
#define SCS15_TARGET_POSITION_H_REG 0x2A
#define SCS15_BAUD_RATE_REG 0x06
#define SCS15_ACK_MODE_REG 0x08
#define SCS15_TEMPERATURE_REG 0x3F
#define SCS15_VOLTAGE_REG 0x3E
#define SCS15_MINIMUM_ANGLE_H_REG 0x09
#define SCS15_MOTOR_MODE_SPEED_H_REG 0x2C

#include "main.h"

void Host2Servo(uint8_t *DataL, uint8_t* DataH, uint16_t Data);
uint8_t SCS15Checksum(uint8_t * data);
uint8_t SCS15SetBaudrate(uint8_t scs15_id, uint32_t baudrate);

void SCS15SetSilentMode(uint8_t scs15_id);
void SCS15SetNonSilentMode(uint8_t scs15_id);

uint8_t SCS15ReadTemperature(uint8_t scs15_id, uint8_t *drive_temperature);
uint8_t SCS15ReadVoltage(uint8_t scs15_id, uint8_t *drive_voltage);

void SCS15SetMotorMode(uint8_t scs15_id);
void SCS15SetServoMode(uint8_t scs15_id);
void SCS15SetSpeed(uint8_t scs15_id, int16_t scs15_speed);

void SCS15TurnToPosition(uint8_t scs15_id, uint16_t scs15_position, uint16_t scs15_delay, uint16_t scs15_speed);

#endif /* INC_SCS15_SERVO_H_ */
