/**
  ******************************************************************************
  * @file    hydra_events.h
  * @brief   Central definition for FreeRTOS task synchronization events and queues.
  *          This is the shared contract between all modules for inter-task communication.
  ******************************************************************************
  */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "task.h"

/* Hydra core types */
#include "hydra_core.h"

/*-----------------------------------------------------------
 * EVENT GROUP BIT DEFINITIONS
 *-----------------------------------------------------------*/
/* Bits in the global system event group (xHydraEventGroup) */
#define HYDRA_EVENT_SYNC_PULSE         (1UL << 0)  ///< Bit 0: Time for synchronized LED flash
#define HYDRA_EVENT_CAN_RX             (1UL << 1)  ///< Bit 1: New CAN message received
#define HYDRA_EVENT_STATE_VOTE         (1UL << 2)  ///< Bit 2: State vote request needs processing
#define HYDRA_EVENT_HEARTBEAT          (1UL << 3)  ///< Bit 3: Time to send heartbeat
#define HYDRA_EVENT_ERROR_LOG          (1UL << 4)  ///< Bit 4: An error needs to be logged/broadcast
#define HYDRA_EVENT_SENSOR_READY       (1UL << 5)  ///< Bit 5: New sensor data ready for processing
#define HYDRA_EVENT_TELEMETRY_TX       (1UL << 6)  ///< Bit 6: Telemetry data ready for transmission
#define HYDRA_EVENT_SAFE_MODE          (1UL << 7)  ///< Bit 7: Enter safe mode immediately

/* Reserve bits 8-23 for application-specific events */
#define HYDRA_EVENT_APP_BASE           (1UL << 8)  ///< Base for application-specific events

/*-----------------------------------------------------------
 * QUEUE MESSAGE TYPE DEFINITIONS
 *-----------------------------------------------------------*/
/* Queue for passing received CAN messages from the ISR to the application task */
typedef struct {
    uint32_t can_id;            ///< CAN identifier of the received message
    uint8_t data[64];           ///< Data payload (max size for CAN FD)
    uint8_t data_len;           ///< Length of the data in bytes
    CanBus bus;                 ///< Which bus the message was received on
} HydraCanRxMessage_t;

/* Queue for sending CAN messages from any task to the comm_tx_task */
typedef struct {
    uint32_t can_id;            ///< CAN identifier to send
    uint8_t data[64];           ///< Data payload
    uint8_t data_len;           ///< Length of the data in bytes
    CanBus bus;                 ///< Which bus to send on (CAN_BUS_A or CAN_BUS_B)
    TickType_t timeout_ticks;   ///< How long to wait for TX to complete
} HydraCanTxMessage_t;

/* Queue for passing error codes and severity for logging/broadcast */
typedef struct {
    uint16_t error_code;        ///< Board-specific error code
    uint8_t severity;           ///< Error severity (e.g., LOG_INFO, LOG_WARNING, LOG_ERROR)
    uint32_t timestamp;         ///< Time when error occurred
    uint8_t source_alias;       ///< Alias of the board that generated the error
} HydraErrorMessage_t;

/* Queue for sensor data from acquisition tasks to processing tasks */
typedef struct {
    uint8_t sensor_type;        ///< Type of sensor (e.g., SENSOR_TYPE_IMU)
    uint8_t sensor_id;          ///< ID of the sensor on the board
    uint32_t timestamp;         ///< Sample timestamp
    float data[16];             ///< Flexible data array for sensor readings
} HydraSensorData_t;

/*-----------------------------------------------------------
 * GLOBAL EXTERN DECLARATIONS
 *-----------------------------------------------------------*/
/* The single, global event group for the application */
extern EventGroupHandle_t xHydraEventGroup;

/* The queues for inter-task communication */
extern QueueHandle_t xCanRxQueue;       ///< ISR -> comm_rx_task (HydraCanRxMessage_t)
extern QueueHandle_t xCanTxQueue;       ///< Any task -> comm_tx_task (HydraCanTxMessage_t)
extern QueueHandle_t xErrorLogQueue;    ///< Any task -> logger/telemetry task (HydraErrorMessage_t)
extern QueueHandle_t xSensorDataQueue;  ///< Sensor tasks -> processing tasks (HydraSensorData_t)

/*-----------------------------------------------------------
 * UTILITY MACROS
 *-----------------------------------------------------------*/
/**
 * @brief Sends a message to a queue with a critical section guard.
 * @param xQueue The queue handle.
 * @param pvItem Pointer to the item to send.
 * @param xTicksToWait Max time to wait.
 * @return pdTRUE if successful, pdFALSE otherwise.
 */
#define SAFE_QUEUE_SEND(xQueue, pvItem, xTicksToWait) \
    do { \
        taskENTER_CRITICAL(); \
        xQueueSend(xQueue, pvItem, xTicksToWait); \
        taskEXIT_CRITICAL(); \
    } while (0)

/**
 * @brief Sends a message to a queue from ISR with context yield.
 * @param xQueue The queue handle.
 * @param pvItem Pointer to the item to send.
 * @param pxHigherPriorityTaskWoken Pointer to flag indicating if a yield is needed.
 * @return pdTRUE if successful, pdFALSE otherwise.
 */
#define SAFE_QUEUE_SEND_FROM_ISR(xQueue, pvItem, pxHigherPriorityTaskWoken) \
    do { \
        UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR(); \
        xQueueSendFromISR(xQueue, pvItem, pxHigherPriorityTaskWoken); \
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus); \
    } while (0)

#ifdef __cplusplus
}
#endif
