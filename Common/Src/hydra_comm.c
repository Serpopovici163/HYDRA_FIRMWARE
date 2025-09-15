/**
  ******************************************************************************
  * @file    hydra_comm.c
  * @brief   Implementation of dual CAN FD bus management.
  ******************************************************************************
  */
#include "hydra_comm.h"
#include "hydra_events.h"
#include "string.h"

#if defined(STM32G4xx)
    #include "stm32g4xx_hal.h" // Will be included by the G4 board's project
#elif defined(STM32H7xx)
    #include "stm32h7xx_hal.h" // Will be included by the H7 board's project
#endif

/* Private variables */
static hydra_comm_rx_callback_t user_rx_callback = NULL;
static SystemState degraded_state = SYSTEM_STATE_NORMAL;

/* CAN Handle Definitions - These are defined in the board's main.c */
extern CAN_HandleTypeDef hcan1; // Assume hcan1 = BUS_A
extern CAN_HandleTypeDef hcan2; // Assume hcan2 = BUS_B

/* RTOS Handles */
static QueueHandle_t xCanTxQueue;
static TaskHandle_t xCommRxTaskHandle;

/* Bus Health Tracking */
typedef struct {
    uint32_t last_activity_ticks;
    bool is_healthy;
} BusHealth_t;

static BusHealth_t bus_health[2] = {0}; // Indexed by CanBus enum

/* Private function prototypes */
static void process_rx_message(CAN_HandleTypeDef *hcan, CanBus bus);
static void update_bus_health(CanBus bus, bool message_received);
static HAL_StatusTypeDef can_send_internal(CAN_HandleTypeDef *hcan, uint32_t can_id, const uint8_t* data, uint8_t len);

/**
  * @brief  Initializes the communication module, queues, and tasks.
  * @retval HydraStatus_t HYDRA_OK if successful, HYDRA_ERR otherwise.
  */
void hydra_comm_init(void) {
    // Initialize the TX queue
    xCanTxQueue = xQueueCreate(32, sizeof(HydraCanTxMessage_t));
    if (xCanTxQueue == NULL) {
        return HYDRA_ERR;
    }

    // Start the FreeRTOS tasks
    xTaskCreate(comm_tx_task, "COMM_TX", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(comm_rx_task, "COMM_RX", 1024, NULL, tskIDLE_PRIORITY + 4, &xCommRxTaskHandle);

    // Start the CAN peripherals and interrupts
    if (HAL_CAN_Start(&hcan1) != HAL_OK || HAL_CAN_Start(&hcan2) != HAL_OK) {
        return HYDRA_ERR;
    }

    // Activate CAN RX interrupts
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return HYDRA_ERR;
    }

    return HYDRA_OK;
}

/**
  * @brief  Sends a CAN message by queuing it for the TX task.
  * @param  can_id: The CAN identifier.
  * @param  data: Pointer to the data to send.
  * @param  len: Length of the data (0-64).
  * @param  bus: Which bus to send on.
  * @retval HydraStatus_t HYDRA_OK if queued successfully, HYDRA_ERR otherwise.
  */
void hydra_comm_send(uint32_t can_id, const uint8_t* data, uint8_t len, CanBus bus) {
    HydraCanTxMessage_t tx_msg;

    if (len > CANFD_MAX_DATA_LEN) {
        return HYDRA_ERR;
    }

    tx_msg.can_id = can_id;
    tx_msg.bus = bus;
    tx_msg.data_len = len;
    tx_msg.timeout_ticks = pdMS_TO_TICKS(100);
    memcpy(tx_msg.data, data, len);

    if (xQueueSend(xCanTxQueue, &tx_msg, 0) != pdPASS) {
        return HYDRA_ERR; // Queue is full
    }

    return HYDRA_OK;
}

/**
  * @brief  Sets the callback function for received messages.
  * @param  callback: Function to call when a message is received.
  */
void hydra_comm_set_rx_callback(hydra_comm_rx_callback_t callback) {
    user_rx_callback = callback;
}

/**
  * @brief  Checks the health status of a CAN bus.
  * @param  bus: The bus to check.
  * @retval bool True if the bus is considered healthy.
  */
bool hydra_comm_is_bus_healthy(CanBus bus) {
    uint32_t now = xTaskGetTickCount();
    // Consider bus healthy if we've seen activity in the last 2 seconds
    return ((now - bus_health[bus].last_activity_ticks) < pdMS_TO_TICKS(2000));
}

/**
  * @brief  Gets the current system degraded state due to bus failures.
  * @retval SystemState The current degraded state.
  */
SystemState hydra_comm_get_system_degraded_state(void) {
    return degraded_state;
}

/**
  * @brief  The TX task that processes messages from the queue and sends them.
  */
void comm_tx_task(void *argument) {
    HydraCanTxMessage_t tx_msg;

    for (;;) {
        if (xQueueReceive(xCanTxQueue, &tx_msg, portMAX_DELAY) == pdPASS) {
            CAN_HandleTypeDef *hcan = (tx_msg.bus == CAN_BUS_A) ? &hcan1 : &hcan2;

            if (can_send_internal(hcan, tx_msg.can_id, tx_msg.data, tx_msg.data_len) != HAL_OK) {
                // Log TX error, update bus health
                update_bus_health(tx_msg.bus, false);
            }
        }
    }
}

/**
  * @brief  The RX task that processes messages from the ISR queue.
  */
void comm_rx_task(void *argument) {
    HydraCanRxMessage_t rx_msg;

    for (;;) {
        // Wait for a message to be placed in the queue by the ISR
        if (xQueueReceive(xCanRxQueue, &rx_msg, portMAX_DELAY) == pdPASS) {
            // Update bus health - we received something!
            update_bus_health(rx_msg.bus, true);

            // Call the user's callback if set
            if (user_rx_callback != NULL) {
                user_rx_callback(rx_msg.can_id, rx_msg.data, rx_msg.data_len, rx_msg.bus);
            }
        }
    }
}

/**
  * @brief  Internal function to send a CAN message via HAL.
  */
static HAL_StatusTypeDef can_send_internal(CAN_HandleTypeDef *hcan, uint32_t can_id, const uint8_t* data, uint8_t len) {
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;

    tx_header.StdId = can_id;
    tx_header.ExtId = 0;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t*)data, &mailbox);
}

/**
  * @brief  Updates the health status of a bus.
  */
static void update_bus_health(CanBus bus, bool message_received) {
    uint32_t now = xTaskGetTickCount();
    bus_health[bus].last_activity_ticks = now;

    bool was_healthy = bus_health[bus].is_healthy;
    bus_health[bus].is_healthy = true; // Assume healthy if we're sending/receiving

    // Check if both buses are healthy to determine system state
    bool bus_a_ok = hydra_comm_is_bus_healthy(CAN_BUS_A);
    bool bus_b_ok = hydra_comm_is_bus_healthy(CAN_BUS_B);

    if (!bus_a_ok && !bus_b_ok) {
        degraded_state = SYSTEM_STATE_CRITICAL_FAILURE;
    } else if (!bus_a_ok || !bus_b_ok) {
        degraded_state = (bus == CAN_BUS_A) ? SYSTEM_STATE_DEGRADED_BUS_A_FAILED : SYSTEM_STATE_DEGRADED_BUS_B_FAILED;
    } else {
        degraded_state = SYSTEM_STATE_NORMAL;
    }

    // Optionally trigger an event if health status changed
    if (was_healthy != bus_health[bus].is_healthy) {
        xEventGroupSetBits(xHydraEventGroup, HYDRA_COMM_HEALTH_CHANGE_BIT);
    }
}

/**
  * @brief  CAN RX Interrupt Service Routine callback.
  *         This function is called by the HAL from the actual ISR.
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    HydraCanRxMessage_t rx_msg;
    CanBus bus = (hcan->Instance == hcan1.Instance) ? CAN_BUS_A : CAN_BUS_B;

    // Read the message from the hardware
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_msg.rx_header, rx_msg.data) == HAL_OK) {
        rx_msg.can_id = rx_msg.rx_header.StdId;
        rx_msg.data_len = rx_msg.rx_header.DLC;
        rx_msg.bus = bus;

        // Send the message to the RX task's queue (from ISR context)
        xQueueSendFromISR(xCanRxQueue, &rx_msg, &xHigherPriorityTaskWoken);
    }

    // Yield if necessary
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
