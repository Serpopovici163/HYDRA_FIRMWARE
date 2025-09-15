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

// Time source types, assigned such that the one with most confidence (GPS) is the lowest value so we can check if another board has a better time source by comparing the time source of another board with ours using >/<
typedef enum {
	TIME_SOURCE_GPS 					= 0x01,		// GNSS/GPS time
    TIME_SOURCE_ESTIMATED				= 0x02,		// Dead-reckoned from last GPS time sync
	TIME_SOURCE_INTERNAL				= 0x03		// Unsynchronized internal clock
} TimeSource_t;

// Since we use the same ID for all sync requests, we need a byte within the CAN frame to specify if we are the initiator or not
typedef enum {
	SYNC_TYPE_REQUEST 					= 0x01,
	SYNC_TYPE_RESPONSE 					= 0x02
} SyncType_t;

// Defines status of the board as a whole.
// IMPORTANT: only 3 bits are available in heartbeat message for these values
// Range: 0x0-0x7
typedef enum {
	HYDRA_STATUS_OK 					= 0x0,
	HYDRA_STATUS_WARN 					= 0x1,
	HYDRA_STATUS_CRITICAL				= 0x2,
	HYDRA_STATUS_SAFE_MODE 				= 0x3,
	HYDRA_STATUS_INITIALIZING			= 0x4
} HydraGlobalStatus_t;

// Defines which subsystem has faulted if any, ranked with 0x1 being highest priority fault (0x0 means no fault) and 0xF being lowest
// IMPORTANT: only 5 bits available in heartbeat message for these values
// RANGE: 0x0 - 0x1F
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
    uint16_t last_received_status_code;
    uint32_t last_heard_ms; // Timestamp of last status update
    uint32_t last_received_timestamp_ms;
    TimeSource_t last_received_timesource;
    bool is_supposed_to_be_present;           // Flag if board is supposed to be online (used when we turn off boards using the TCAN1146)
} BoardStatus_t;

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

void hydra_health_init(const HydraHeartbeatLedConfig_t *led_config);
void hydra_health_heartbeat_init(void);

uint32_t hydra_health_get_system_time_offset(void);
TimeSource_t hydra_health_get_system_time_source(void);
void hydra_health_set_time_source(TimeSource_t source);
static void update_local_time(uint32_t new_ms, TimeSource_t new_source);
BoardStatus_t* hydra_health_get_status_table(void);
uint8_t hydra_health_get_last_known_board_count(void);
uint16_t hydra_health_get_subsys_flt_flags(void);
void hydra_health_set_subsys_error(HydraSubsystemFault_t hydra_subsys, uint8_t subsys_error_code);

// The main task for the heartbeat module
void heartbeat_task(void *argument);
