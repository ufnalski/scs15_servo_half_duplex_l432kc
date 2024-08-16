/*
 * scs15_servo.c
 *
 *  Created on: Aug 13, 2024
 *      Author: user
 */

#include "scs15_servo.h"
#include "usart.h"
#include <stdlib.h>

void Host2Servo(uint8_t *dataSentFirst, uint8_t *dataSendSecond, uint16_t data)
{
	if (BIG_ENDIAN_PERIPHERAL)
	{
		*dataSentFirst = (data >> 8);
		*dataSendSecond = (data & 0xFF);
	}
	else
	{
		*dataSendSecond = (data >> 8);
		*dataSentFirst = (data & 0xFF);
	}
}

uint8_t SCS15Checksum(uint8_t *scs15_data)
{
	uint8_t check_sum = 0;

	for (uint8_t i = 0; i <= scs15_data[3]; i++)
	{
		check_sum += scs15_data[i + 2];
	}

	return ~check_sum;
}

uint8_t SCS15SetBaudrate(uint8_t scs15_id, uint32_t baudrate)
{
	uint8_t txBuffer[8];

	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)
	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = scs15_id;
	txBuffer[3] = 0x04;
	txBuffer[4] = SCS15_WRITE_DATA_COMMAND;
	txBuffer[5] = SCS15_BAUD_RATE_REG;

	switch (baudrate)
	{
	case 1000000:
		txBuffer[6] = 0x00;
		break;
	case 500000:
		txBuffer[6] = 0x01;
		break;
	case 250000:
		txBuffer[6] = 0x02;
		break;
	case 128000:
		txBuffer[6] = 0x03;
		break;
	case 115200:
		txBuffer[6] = 0x04;
		break;
	case 76800:
		txBuffer[6] = 0x05;
		break;
	case 57600:
		txBuffer[6] = 0x06;
		break;
	case 38400:
		txBuffer[6] = 0x07;
		break;
	default:
		return 1;

	}

	txBuffer[7] = SCS15Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(SCS15_UART);
	HAL_UART_Transmit(SCS15_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_Delay(50);

	return 0;

}

void SCS15SetSilentMode(uint8_t scs15_id)
{
	uint8_t txBuffer[8];

	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)
	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = scs15_id;
	txBuffer[3] = 0x04;
	txBuffer[4] = SCS15_WRITE_DATA_COMMAND;
	txBuffer[5] = SCS15_ACK_MODE_REG;
	txBuffer[6] = 0x00;
	txBuffer[7] = SCS15Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(SCS15_UART);
	HAL_UART_Transmit(SCS15_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_Delay(50);
}

void SCS15SetNonSilentMode(uint8_t scs15_id)
{
	uint8_t txBuffer[8];

	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)
	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = scs15_id;
	txBuffer[3] = 0x04;
	txBuffer[4] = SCS15_WRITE_DATA_COMMAND;
	txBuffer[5] = SCS15_ACK_MODE_REG;
	txBuffer[6] = 0x01;
	txBuffer[7] = SCS15Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(SCS15_UART);
	HAL_UART_Transmit(SCS15_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_Delay(50);
}

uint8_t SCS15ReadTemperature(uint8_t scs15_id, uint8_t *drive_temperature)
{
	uint8_t rxBuffer[7] =
	{ 0 };
	uint8_t rx_data_length = 0;
	uint8_t rx_data_error = 0;

	uint8_t txBuffer[8];

	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)
	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = scs15_id;
	txBuffer[3] = 0x04;
	txBuffer[4] = SCS15_READ_DATA_COMMAND;
	txBuffer[5] = SCS15_TEMPERATURE_REG;
	txBuffer[6] = 0x01;
	txBuffer[7] = SCS15Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(SCS15_UART);
	HAL_UART_Transmit(SCS15_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_HalfDuplex_EnableReceiver(SCS15_UART);
	HAL_UART_Receive(SCS15_UART, rxBuffer, sizeof(rxBuffer), 100);

	if (rxBuffer[0] == 0xFF && rxBuffer[1] == 0xFF)
	{
		rx_data_length = rxBuffer[3] - 2;	//scs15 magic
		rx_data_error = rxBuffer[4];
	}
	else
	{
		//Error_Handler();
		return 1;
	}

	if (rx_data_error != 0)
	{
		//Error_Handler();
		return 2;
	}
	else if (rx_data_length == 0)
	{
		//Error_Handler();
		return 3;
	}
	else
	{
		*drive_temperature = rxBuffer[5];
		return 0;
	}
}

uint8_t SCS15ReadVoltage(uint8_t scs15_id, uint8_t *drive_voltage)
{
	uint8_t rxBuffer[7] =
	{ 0 };
	uint8_t rx_data_length = 0;
	uint8_t rx_data_error = 0;

	uint8_t txBuffer[8];

	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)
	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = scs15_id;
	txBuffer[3] = 0x04;
	txBuffer[4] = SCS15_READ_DATA_COMMAND;
	txBuffer[5] = SCS15_VOLTAGE_REG;
	txBuffer[6] = 0x01;
	txBuffer[7] = SCS15Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(SCS15_UART);
	HAL_UART_Transmit(SCS15_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_HalfDuplex_EnableReceiver(SCS15_UART);
	HAL_UART_Receive(SCS15_UART, rxBuffer, sizeof(rxBuffer), 100);

	if (rxBuffer[0] == 0xFF && rxBuffer[1] == 0xFF)
	{
		rx_data_length = rxBuffer[3] - 2;	//scs15 magic
		rx_data_error = rxBuffer[4];
	}
	else
	{
		//Error_Handler();
		return 1;
	}

	if (rx_data_error != 0)
	{
		//Error_Handler();
		return 2;
	}
	else if (rx_data_length == 0)
	{
		//Error_Handler();
		return 3;
	}
	else
	{
		*drive_voltage = rxBuffer[5];
		return 0;
	}
}

void SCS15TurnToPosition(uint8_t scs15_id, uint16_t scs15_position,
		uint16_t scs15_delay, uint16_t scs15_speed)
{
	uint8_t txBuffer[13];

	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)
	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = scs15_id;
	txBuffer[3] = 0x09;
	txBuffer[4] = SCS15_WRITE_DATA_COMMAND;
	txBuffer[5] = SCS15_TARGET_POSITION_H_REG;
	Host2Servo(txBuffer + 6, txBuffer + 7, scs15_position);
	Host2Servo(txBuffer + 8, txBuffer + 9, scs15_delay);
	Host2Servo(txBuffer + 10, txBuffer + 11, scs15_speed);

	txBuffer[12] = SCS15Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(SCS15_UART);
	HAL_UART_Transmit(SCS15_UART, txBuffer, sizeof(txBuffer), 100);
}

void SCS15SetMotorMode(uint8_t scs15_id)
{
	uint8_t txBuffer[11];
	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)

	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = scs15_id;
	txBuffer[3] = 0x07;
	txBuffer[4] = SCS15_WRITE_DATA_COMMAND;
	txBuffer[5] = SCS15_MINIMUM_ANGLE_H_REG;
	txBuffer[6] = 0x00;
	txBuffer[7] = 0x00;
	txBuffer[8] = 0x00;
	txBuffer[9] = 0x00;

	txBuffer[10] = SCS15Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(SCS15_UART);
	HAL_UART_Transmit(SCS15_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_Delay(50);
}

void SCS15SetSpeed(uint8_t scs15_id, int16_t scs15_speed)
{
	if (scs15_speed < -1024)
	{
		scs15_speed = -1024;
	}
	else if (scs15_speed > 1024)
	{
		scs15_speed = -1024;
	}
	else
	{
		;
	}

	uint8_t txBuffer[9];
	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)

	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = scs15_id;
	txBuffer[3] = 0x05;
	txBuffer[4] = SCS15_WRITE_DATA_COMMAND;
	txBuffer[5] = SCS15_MOTOR_MODE_SPEED_H_REG;

	Host2Servo(txBuffer + 6, txBuffer + 7,
			((uint16_t) (abs(scs15_speed)))
					| (((uint16_t) (scs15_speed < 0)) << 10));

	txBuffer[8] = SCS15Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(SCS15_UART);
	HAL_UART_Transmit(SCS15_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_Delay(50);
}

void SCS15SetServoMode(uint8_t scs15_id)
{
	uint8_t txBuffer[11];
	// frame: 0xFF, 0xFF, servo ID, message length (excluding servo ID), function, memory address, data bytes, checksum (begining from servo ID)

	txBuffer[0] = 0xFF;
	txBuffer[1] = 0xFF;
	txBuffer[2] = scs15_id;
	txBuffer[3] = 0x07;
	txBuffer[4] = SCS15_WRITE_DATA_COMMAND;
	txBuffer[5] = SCS15_MINIMUM_ANGLE_H_REG;
	txBuffer[6] = 0x00;
	txBuffer[7] = 0x00;
	txBuffer[8] = 0x03;
	txBuffer[9] = 0xFF;

	txBuffer[10] = SCS15Checksum(txBuffer);

	HAL_HalfDuplex_EnableTransmitter(SCS15_UART);
	HAL_UART_Transmit(SCS15_UART, txBuffer, sizeof(txBuffer), 100);

	HAL_Delay(50);
}
