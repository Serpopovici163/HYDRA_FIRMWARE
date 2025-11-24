#include "../Common/Inc/hydra_comm.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>

/*-----------------------------------------------------------
 * Private defines
 *-----------------------------------------------------------*/

// TCAN1146 Register Addresses (from datasheet)
#define TCAN_REG_DEVICE_CTRL  0x10
#define TCAN_REG_WAKE_CONFIG  0x03
#define TCAN_REG_WAKE_ID      0x04  // Start of 4-byte ID filter
#define TCAN_REG_WAKE_PAYLOAD 0x08  // Start of 8-byte payload filter

/*-----------------------------------------------------------
 * Private structs
 *-----------------------------------------------------------*/

typedef struct {
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef* cs_port;
	uint16_t cs_pin;
} TCAN1146_t;

/*-----------------------------------------------------------
 * Private variables
 *-----------------------------------------------------------*/

TCAN1146_t tcan1146 = {0};

/*-----------------------------------------------------------
 * Private Prototypes
 *-----------------------------------------------------------*/
void _spi_write(uint8_t addr, uint8_t data);
void _spi_read(uint8_t addr, uint8_t* data);

void TCAN1146_SetMode(TCAN_Mode_t mode);

//int _write(int file, char *ptr, int len)
//{
//    int DataIdx;
//
//    for (DataIdx = 0; DataIdx < len; DataIdx++)
//    {
//        ITM_SendChar(*ptr++);
//    }
//    return len;
//}

/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/

void hydra_comm_init(SPI_HandleTypeDef* TCAN1146_hspi, GPIO_TypeDef* TCAN1146_cs_port, uint16_t TCAN1146_cs_pin) {
	tcan1146.hspi = TCAN1146_hspi;
	tcan1146.cs_port = TCAN1146_cs_port;
	tcan1146.cs_pin = TCAN1146_cs_pin;

	_spi_write(0x1C, 0x80); //disable sleep wake error timer from shutting off TCAN1146 due to CAN bus inactivity
	TCAN1146_SetMode(TCAN_MODE_NORMAL);
}

/*-----------------------------------------------------------
 * Internal logic
 *-----------------------------------------------------------*/

//this function handles CS as GPIO since a few boards do it this way and I don't want to handle certain boards with hardware CS
//and others with software CS unless there's a real need to do it that way
void _spi_write(uint8_t addr, uint8_t data) {

	uint8_t tx[2] = {(addr << 1) | 0x01, data};  // Set write bit

	HAL_GPIO_WritePin(tcan1146.cs_port, tcan1146.cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(tcan1146.hspi, tx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(tcan1146.cs_port, tcan1146.cs_pin, GPIO_PIN_SET);
}

void _spi_read(uint8_t addr, uint8_t* data) {

	uint8_t tx[2] = {(addr << 1), 0};  // Set read bit
	uint8_t rx[2] = {0};

	HAL_GPIO_WritePin(tcan1146.cs_port, tcan1146.cs_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(tcan1146.hspi, tx, rx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(tcan1146.cs_port, tcan1146.cs_pin, GPIO_PIN_SET);

	*data = rx[1];
}

void TCAN1146_SetMode(TCAN_Mode_t mode) {
	uint8_t ctrl_reg = 0;

	//read current mode register
	_spi_read(TCAN_REG_DEVICE_CTRL, &ctrl_reg);

	//clear mode bits
	ctrl_reg &= ~0x07;

	//update mode bits
	switch (mode) {
		case TCAN_MODE_NORMAL:
			ctrl_reg |= 0x07;  // Normal mode
			break;
		case TCAN_MODE_STANDBY:
			ctrl_reg |= 0x04;  // Standby
			break;
		case TCAN_MODE_LISTEN_ONLY:
			ctrl_reg |= 0x05;  // Listen-Only
			break;
		case TCAN_MODE_SLEEP:
			ctrl_reg |= 0x01;  // Sleep
			break;
	}

	_spi_write(TCAN_REG_DEVICE_CTRL, ctrl_reg);

	//read current mode register
	//_spi_read(TCAN_REG_DEVICE_CTRL, &ctrl_reg);
}
