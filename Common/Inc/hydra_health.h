#pragma once
#include "hydra_core.h"

#if defined(STM32G4xx)
    #include "stm32g4xx_hal.h" // Will be included by the G4 board's project
#elif defined(STM32H7xx)
    #include "stm32h7xx_hal.h" // Will be included by the H7 board's project
#endif

/*-----------------------------------------------------------
 * Private enums
 *-----------------------------------------------------------*/

// Deepseek made these, might delete but I'll keep them if it makes the code more legible
typedef enum {
    HEARTBEAT_IDLE 						= 0x00,
    HEARTBEAT_WAITING_FOR_RESPONSES 	= 0x01
} HeartbeatState_t;

// Time source types, assigned such that the one with most confidence (GPS) is the greatest value so we can check if another board has a better time source by comparing the time source of another board with ours using >/<
typedef enum {
    TIME_SOURCE_INTERNAL 				= 0x01,		// Unsynchronized internal clock
    TIME_SOURCE_ESTIMATED				= 0x02,		// Dead-reckoned from last sync (by NAV)
	TIME_SOURCE_GPS						= 0x03		// GNSS/GPS time
} TimeSource_t;

// Since we use the same ID for all sync requests, we need a byte within the CAN frame to specify if we are the initiator or not
typedef enum {
	SYNC_TYPE_REQUEST 					= 0x01,
	SYNC_TYPE_RESPONSE 					= 0x02
} SyncType_t;

// Defines status of the board as a whole.
// IMPORTANT: only 3 bits are available in heartbeat message for these values
typedef enum {
	HYDRA_STATUS_OK 					= 0x0,
	HYDRA_STATUS_WARN 					= 0x1,
	HYDRA_STATUS_CRITICAL				= 0x2,
	HYDRA_STATUS_SAFE_MODE 				= 0x3,
	HYDRA_STATUS_INITIALIZING			= 0x4
} HydraGlobalStatus_t;

// Defines which subsystem has faulted if any
typedef enum {
	HYDRA_SUBSYS_FLT_NONE				= 0x0,
	HYDRA_SUBSYS_FLT_RTOS				= 0x1,
	HYDRA_SUBSYS_FLT_CAN				= 0x2,
	HYDRA_SUBSYS_FLT_SD					= 0x3,
	HYDRA_SUBSYS_FLT_INS				= 0x4,
	HYDRA_SUBSYS_FLT_PWR				= 0x5,
	HYDRA_SUBSYS_FLT_PYRO				= 0x6,
	HYDRA_SUBSYS_FLT_RADIO				= 0x7,
	HYDRA_SUBSYS_FLT_MEMORY				= 0x8,
	HYDRA_SUBSYS_FLT_CONFIG				= 0x9
} HydraSubsystemFault_t;

/*-----------------------------------------------------------
 * Private structs
 *-----------------------------------------------------------*/

// Structure to hold LED GPIO configuration since they change from board to board
typedef struct {
    GPIO_TypeDef* green_port;
    uint16_t green_pin;
    GPIO_TypeDef* red_port;
    uint16_t red_pin;
} HydraLedConfig_t;

// Structure to hold the status of another board
typedef struct {
    uint8_t alias;
    uint16_t status_code;
    uint32_t last_heard_ms; // Timestamp of last status update
    bool present;           // Flag if board is considered online
} BoardStatus_t;

// Struct to hold a time value since EPOCH
typedef struct {
    uint32_t seconds;
    uint32_t milliseconds;
    TimeSource_t source;
    uint16_t confidence_ms; // Estimate of error (± value)
} HydraTime_t;

/*-----------------------------------------------------------
 * CAN Frame Format
 *-----------------------------------------------------------*/

typedef struct {
	SyncType_t sync_type; //see enum above
	uint8_t alias; //alias of board that is sending this message
	uint16_t fault_code; //see enum above
	HydraTime_t hydra_time; //see struct above
} HydraHeartbeat_t;

/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/

void hydra_heartbeat_init(const HydraHeartbeatLedConfig_t *led_config);
void hydra_heartbeat_set_local_status(uint16_t status);
BoardStatus_t* hydra_heartbeat_get_status_table(uint8_t* count);
void hydra_heartbeat_set_time_source(TimeSource_t source, uint16_t confidence_ms);
HydraTime_t hydra_heartbeat_get_system_time(void);

// The main task for the heartbeat module
void heartbeat_task(void *argument);
