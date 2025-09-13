/**
  ******************************************************************************
  * @file    hydra_comm.h
  * @brief   Hardware abstraction and management for dual CAN FD buses.
  ******************************************************************************
  */
#pragma once

#include "hydra_core.h"
#include "FreeRTOS.h"
#include "queue.h"

#if defined(STM32G4xx)
    #include "stm32g4xx_hal.h" // Will be included by the G4 board's project
#elif defined(STM32H7xx)
    #include "stm32h7xx_hal.h" // Will be included by the H7 board's project
#endif

/* Callback function type for received messages */
typedef void (*hydra_comm_rx_callback_t)(uint32_t id, uint8_t* data, uint8_t len, CanBus bus);

/* Public API */
HydraStatus_t hydra_comm_init(void);
HydraStatus_t hydra_comm_send(uint32_t can_id, const uint8_t* data, uint8_t len, CanBus bus);
void hydra_comm_set_rx_callback(hydra_comm_rx_callback_t callback);
bool hydra_comm_is_bus_healthy(CanBus bus);
SystemState hydra_comm_get_system_degraded_state(void);

/* RTOS Task Functions */
void comm_tx_task(void *argument);
void comm_rx_task(void *argument);

#endif /* HYDRA_COMM_H */
