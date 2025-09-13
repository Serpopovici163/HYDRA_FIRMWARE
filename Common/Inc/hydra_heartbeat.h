#pragma once
#include "hydra_core.h"

#if defined(STM32G4xx)
    #include "stm32g4xx_hal.h" // Will be included by the G4 board's project
#elif defined(STM32H7xx)
    #include "stm32h7xx_hal.h" // Will be included by the H7 board's project
#endif

// Structure to hold LED GPIO configuration
typedef struct {
    GPIO_TypeDef* green_port;
    uint16_t green_pin;
    GPIO_TypeDef* red_port;
    uint16_t red_pin;
} HydraHeartbeatLedConfig_t;

// Structure to hold the status of another board
typedef struct {
    uint8_t alias;
    uint16_t status_code;
    uint32_t last_heard_ms; // Timestamp of last status update
    bool present;           // Flag if board is considered online
} BoardStatus_t;

typedef enum {
    HEARTBEAT_IDLE,
    HEARTBEAT_WAITING_FOR_RESPONSES
} HeartbeatState_t;

/* Time Synchronization Types */
typedef enum {
    TIME_SOURCE_INTERNAL = 0,    // Unsynchronized internal clock
    TIME_SOURCE_GPS,             // GNSS/GPS time
    TIME_SOURCE_ESTIMATED        // Dead-reckoned from last sync (by NAV)
} TimeSource_t;

typedef struct {
    uint32_t seconds;
    uint32_t milliseconds;
    TimeSource_t source;
    uint16_t confidence_ms; // Estimate of error (± value)
} HydraTime_t;

// Public API
void hydra_heartbeat_init(const HydraHeartbeatLedConfig_t *led_config);
void hydra_heartbeat_set_local_status(uint16_t status);
BoardStatus_t* hydra_heartbeat_get_status_table(uint8_t* count);
void hydra_heartbeat_set_time_source(TimeSource_t source, uint16_t confidence_ms);
HydraTime_t hydra_heartbeat_get_system_time(void);

// The main task for the heartbeat module
void heartbeat_task(void *argument);
