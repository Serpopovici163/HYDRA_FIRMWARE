#ifndef HYDRA_COMM_H
#define HYDRA_COMM_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

/*-----------------------------------------------------------
 * Public enums
 *-----------------------------------------------------------*/

typedef enum {
    TCAN_MODE_NORMAL,
    TCAN_MODE_STANDBY,
    TCAN_MODE_LISTEN_ONLY,
    TCAN_MODE_SLEEP
} TCAN_Mode_t;

/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/

void hydra_comm_init(SPI_HandleTypeDef* TCAN1146_hspi, GPIO_TypeDef* TCAN1146_cs_port, uint16_t TCAN1146_cs_pin);
void TCAN1146_SetMode(TCAN_Mode_t mode); //we only ever have one TCAN1146

#endif
