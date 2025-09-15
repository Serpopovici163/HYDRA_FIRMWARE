#include "hydra_events.h"
#include "hydra_comm.h"
#include "hydra_identity.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hydra_health.h"
#include "hydra_core.h"

#if defined(STM32G4xx)
    #include "stm32g4xx_hal.h" // Will be included by the G4 board's project
#elif defined(STM32H7xx)
    #include "stm32h7xx_hal.h" // Will be included by the H7 board's project
#endif

/*-----------------------------------------------------------
 * Private variables
 *-----------------------------------------------------------*/

// Status related info
static uint16_t subsys_flt_flags 							= 0x0000; //Used to store any active errors. Any bit equal to 1 in here represents a subsys flt, the meaning of each bit index is defined by HydraSubsystemFault_t in hydrahealth.h
static uint8_t subsys_err_msg[32]; 									  //There are 32 possible subsystem IDs
static uint16_t local_status_code							= 0x0000; //This is what is transmitted during SYNC messages and only contains the highest priority error code

static BoardStatus_t status_table[MAX_BOARDS_IN_STACK-1]; //host is not included since most of these vars do not apply to the host
static HeartbeatState_t heartbeat_state 					= HEARTBEAT_IDLE;

// Sync message config
static TimerHandle_t sync_timer								= NULL; // Timer for triggering sync checks
static uint8_t local_sync_rank 								= 0; // Our position in the sync order (0 is first)
static uint8_t last_known_board_count 						= 0; // Last known total number of boards
static uint32_t response_window_start_time 					= 0; // If we initiate a sync message, this stores the time at which it was sent and determine when we stop waiting for remote boards to respond

// LED Configuration (stored after init)
static HydraLedConfig_t led_cfg;
static TimerHandle_t led_off_timer;

// Time Synchronization State
static TimeSource_t system_time_source						= TIME_SOURCE_INTERNAL; // Default, but will be updated if better time sources are available in system
static TimeSource_t local_time_source						= TIME_SOURCE_INTERNAL; // Default to assuming internal time source
static uint32_t local_time_offset							= 0; // Used to store offset between system time and local time. This is especially useful if a board reboots during ops since HAL_GetTick() will therefore be <<< system_time
																 // This offset will be used should the system ever rely on the host's internal clock due to a GPS shutdown event or similar. In that scenario, we will lose the higher quality GPS clock
																 // and will revert to some other board's local_time which will be HAL_GetTick()+local_time_offset since we don't want the timstamp to jump upon losing GPS.

/*-----------------------------------------------------------
 * Private function prototypes
 *-----------------------------------------------------------*/

// Sync stuff
static void send_sync_message(SyncType_t sync_type);
static void process_sync_message(uint32_t can_id, uint8_t* data, uint8_t len, CanBus bus);
static void sync_system_time(void);
uint32_t get_median(uint32_t *values, uint8_t size_of_values);
static void sort_uint32_array(uint32_t *array, uint8_t size);
static void sort_uint8_array(uint8_t *array, uint8_t size);
static void sync_timer_callback(TimerHandle_t xTimer);
static void update_sync_timer(void);
static void check_response_window(void);

// LED stuff
static LedColor determine_summary_color(void);
static void set_led_color(LedColor color);
static void flash_led_color(LedColor color);
static void led_off_timer_callback(TimerHandle_t xTimer);

/*-----------------------------------------------------------
 * Init functions
 *-----------------------------------------------------------*/

void hydra_health_init(const HydraLedConfig_t *led_config, TimeSource_t time_source_ability) {
    // Store the LED configuration
    if (led_config != NULL) {
        led_cfg = *led_config;
    }

    // Initialize LED GPIO pins to output, off
    if (led_cfg.green_port) {
        HAL_GPIO_WritePin(led_cfg.green_port, led_cfg.green_pin, GPIO_PIN_RESET);
    }
    if (led_cfg.red_port) {
        HAL_GPIO_WritePin(led_cfg.red_port, led_cfg.red_pin, GPIO_PIN_RESET);
    }

    local_time_source = time_source_ability;

    // Register the CAN callback
    hydra_comm_set_rx_callback(process_sync_message); //TODO: look in to how CAN Rx and Tx works

    // Create LED flash timer
    led_off_timer = xTimerCreate("LEDOffTimer", pdMS_TO_TICKS(LED_FLASH_DURATION_MS), pdFALSE, NULL, led_off_timer_callback);

    if (sync_timer && led_off_timer) {
        xTimerStart(sync_timer, 0);
    }

    // Initialize status table
    memset(status_table, 0, sizeof(status_table));
}

void hydra_health_heartbeat_init(void) {
	// Create sync/heartbeat timer
	sync_timer = xTimerCreate("SyncTimer", pdMS_TO_TICKS(1000), pdTRUE, NULL, sync_timer_callback);
}

/*-----------------------------------------------------------
 * Private var share functions
 *-----------------------------------------------------------*/

uint32_t hydra_health_get_system_time_offset(void) {
	return local_time_offset;
}

TimeSource_t hydra_health_get_system_time_source(void) {
	return local_time_source;
}

void hydra_health_set_time_source(TimeSource_t source) {
	local_time_source = source;
}

//used for the host to update its own time if it has a GPS and such
static void update_local_time(uint32_t new_ms, TimeSource_t new_source) {
	if (system_time_source != local_time_source) return; //check if we have the best time source available, otherwise we shouldn't be changing the time

	system_time_source = new_source;
    local_time_offset = new_ms - HAL_GetTick(); //consistently update offset such that if ever the system switches to the host's time, this time will be continuous to the previous time source
}

BoardStatus_t* hydra_health_get_status_table(void) {
    return status_table;
}

uint8_t hydra_health_get_last_known_board_count(void) {
	return last_known_board_count;
}

uint16_t hydra_health_get_subsys_flt_flags(void) {
	return subsys_flt_flags;
}

uint8_t hydra_health_get_subsys_flt_code(HydraSubsystemFault_t subsysID) {
	return subsys_err_msg[subsysID];
}

void hydra_health_set_subsys_error(HydraSubsystemFault_t hydra_subsys, uint8_t subsys_error_code) {
	subsys_err_msg[hydra_subsys] = subsys_error_code;
}
/*-----------------------------------------------------------
 * Task functions
 *-----------------------------------------------------------*/ //TODO: look into how tasks work

void hydra_health_heartbeat_task(void *argument) {
    hydra_heartbeat_init((HydraLedConfig_t *)argument);

    for (;;) {
        check_response_window();
        update_sync_timer();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*-----------------------------------------------------------
 * Sync message handling/computation
 *-----------------------------------------------------------*/

static void send_sync_message(SyncType_t sync_type) {
    uint8_t data[9];
    uint32_t local_time = HAL_GetTick() + local_time_offset;

    //Append message type, alias, and host status
    data[0] = message_type;
    data[1] = hydra_identity_get_alias();
    data[2] = (local_status_code >> 8) & 0xFF;
    data[3] = local_status_code & 0xFF;

    //Append our current time information
    data[4] = (local_time >> 24) & 0xFF;
    data[5] = (local_time >> 16) & 0xFF;
    data[6] = (local_time >> 8) & 0xFF;
    data[7] = local_time & 0xFF;
    data[8] = local_time_source;

    hydra_comm_send(CAN_ID_SYNC, data, sizeof(data), CAN_BUS_A);
}

static void process_sync_message(uint32_t can_id, uint8_t* data, uint8_t len, CanBus bus) {
    if (can_id != CAN_ID_SYNC || len < 9) return;

    uint8_t peer_count = hydra_identity_get_peer_count();

    for (uint8_t i = 0; i <= peer_count; i++) {
    	if (status_table[i].alias == data[1]) {
    		status_table[i].last_heard_ms = HAL_GetTick();
    		status_table[i].last_received_timesource = data[8];
    		status_table[i].last_received_timestamp_ms = (*(uint32_t*)(&data[4]));
    		status_table[i].last_received_status_code = (data[2] << 8) | data[3];
    	}
    }

    // If this is a sync initiation (not a response), respond
    if (data[0] == 0x01) { //data[0] is message type
        hydra_led_flash_green(); // Flash in unison with initiator
        send_sync_message(SYNC_TYPE_RESPONSE);
    }
}

static void sync_system_time(void) {
	uint8_t usable_time_indexes[MAX_BOARDS_IN_STACK-1]; //-1 since ignoring host time, stores indexes of status_table[] that contain timestamps deemed trustworthy
	uint8_t num_usable_indexes = 0; //stores the number of time sources that exist for the highest quality timesource type available
	uint8_t peer_count = hydra_identity_get_peer_count(); //done this way to not call the function multiple times

	TimeSource_t best_available_time_source_type = TIME_SOURCE_INTERNAL; //Default to worst one

	for (uint8_t i = 0; i < peer_count; i++) { //DON'T SHOVE TIME VALUES INTO A NEW ARRAY, JUST SAVE THE INDEX OF WHICH BOARDS HAVE USABLE TIMES, will be useful later
		if (status_table[i].last_received_timesource < best_available_time_source_type) best_available_time_source_type = status_table[i].last_received_timesource; //we found a better time source type, so set it as the new best time source type
	}

	for (uint8_t i = 0; i < peer_count; i++) {
		if (status_table[i].last_received_timesource == best_available_time_source_type) { //populate usable_received_times[] with any times we have that are of the best quality time source type available
			usable_time_indexes[num_usable_received_times] = i;
			num_usable_received_times++;
		}
	}

	if (local_time_source < best_available_time_source_type || num_usable_indexes == 0) { //we either have a better time source than everybody else, or there is nobody else so break
		system_time_source = local_time_source;
		break;
	} else if (local_time_source == best_available_time_source_type) { //our time source is equivalent to an external one that we've received
		//create an array of times including our own and set system time to the median of this array
		uint32_t time_array[usable_time_indexes+1];

		//add all eligibile received times to array
		for (uint8_t i = 0; i < num_usable_indexes; i++) {
			time_array[i] = status_table[usable_time_indexes[i]].last_received_timestamp_ms + HAL_GetTick() - status_table[usable_time_indexes[i]].last_heard_ms;
		}

		//add our own time to the array
		time_array[usable_time_indexes] = HAL_GetTick() + local_time_offset;

		local_time_offset = HAL_GetTick() - get_median(time_array, usable_time_indexes+1);

	} else { //we have an inferior time source than those received
		//create an array of times including our own and set system time to the median of this array
		uint32_t time_array[usable_time_indexes];

		//add all eligibile received times to array
		for (uint8_t i = 0; i < num_usable_indexes; i++) {
			time_array[i] = status_table[usable_time_indexes[i]].last_received_timestamp_ms + HAL_GetTick() - status_table[usable_time_indexes[i]].last_heard_ms;
		}

		local_time_offset = HAL_GetTick() - get_median(time_array, usable_time_indexes);
	}
	system_time_source = best_available_time_source_type;
}

/**
  * @brief  Calculates the median of an array of unsigned 32-bit integers.
  * @param  values: Pointer to the array of values.
  * @param  size_of_values: The number of elements in the array.
  * @retval uint32_t The median value of the array.
  *         For even-sized arrays, returns the average of the two middle values.
  *         For an empty array, returns 0.
  */
uint32_t get_median(uint32_t *values, uint8_t size_of_values) {
    // Handle edge cases
    if (values == NULL || size_of_values == 0) {
        return 0;
    }
    if (size_of_values == 1) {
        return values[0];
    }

    // Sort the array
    sort_uint32_array(values, size_of_values);

    // Check if the size is odd or even
    if (size_of_values % 2 == 1) {
        // Odd number of elements: return the middle element
        return sorted_values[size_of_values / 2];
    } else {
        // Even number of elements: return the average of the two middle elements
        uint32_t mid1 = sorted_values[(size_of_values / 2) - 1];
        uint32_t mid2 = sorted_values[size_of_values / 2];
        return (mid1 + mid2) / 2;
    }
}

// Helper function to sort an array (using a simple bubble sort)
static void sort_uint32_array(uint32_t *array, uint8_t size) {
    for (uint8_t i = 0; i < size - 1; i++) {
        for (uint8_t j = 0; j < size - i - 1; j++) {
            if (array[j] > array[j + 1]) {
                // Swap the elements
                uint32_t temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }
        }
    }
}

// Helper function to sort an array (using a simple bubble sort)
static void sort_uint8_array(uint8_t *array, uint8_t size) {
    for (uint8_t i = 0; i < size - 1; i++) {
        for (uint8_t j = 0; j < size - i - 1; j++) {
            if (array[j] > array[j + 1]) {
                // Swap the elements
                uint8_t temp = array[j];
                array[j] = array[j + 1];
                array[j + 1] = temp;
            }
        }
    }
}

static void sync_timer_callback(TimerHandle_t xTimer) { //TODO: implement time synchronization separation based on aliases
    // We are the initiator this time
    heartbeat_state = HEARTBEAT_WAITING_FOR_RESPONSES;
    response_window_start_time = HAL_GetTick();

    // Step 1: First Flash (Green) and send initiation
    hydra_led_flash_green();
    send_sync_message(0x01); // Type 0x01: Sync Initiation

    // We will check for responses in the main task or a dedicated function called periodically
}

static void update_sync_timer(void) {
	if (last_known_board_count != hydra_identity_get_peer_count()) {
		//board count has changed (this value can only increase, meaning that we must have discovered a new board)
		last_known_board_count = hydra_identity_get_peer_count();

		uint8_t peer_alias_list[last_known_board_count];
		uint8_t host_peer_index = 0;

		for (uint8_t i = 0; i < last_known_board_count; i++) {
			peer_alias_list[i] = hydra_identity_get_peer_info(i).alias;
		}

		sort_uint8_array(peer_alias_list, last_known_board_count);

		for (uint8_t i = 0; i < last_known_board_count; i++) {
			if (peer_alias_list[i] == hydra_identity_get_alias()) {
				host_peer_index = i;
				break;
			}
		}

		xTimerChangePeriod(sync_timer, pdMS_TO_TICKS(SYNC_INTERVAL_MS + SYNC_DISPERSION_MS * (host_peer_index / last_known_board_count)), 0);
	}
}

static void check_response_window(void) {
    if (heartbeat_state != HEARTBEAT_WAITING_FOR_RESPONSES) {
        return;
    }

    uint32_t current_time = HAL_GetTick();
    if (current_time - response_window_start_time > RESPONSE_WINDOW_MS) { // e.g., 250ms
        // Window is over. Analyze the results.
        LedColor summary_color = determine_summary_color();

        // Step 3: Second Flash (Summary)
        hydra_led_flash_color(summary_color);

        // Reset state
        heartbeat_state = HEARTBEAT_IDLE;
    }
}

/*-----------------------------------------------------------
 * LED functions
 *-----------------------------------------------------------*/

static LedColor determine_summary_color(void) {
    bool all_nominal = true;
    bool all_responded = true;

    for (uint8_t i = 0; i < known_board_count; i++) {
        // Check if this board is *expected* to respond (it's been seen before)
        if (status_table[i].present) {
            // Check if it responded during THIS sync window
            if (status_table[i].last_heard_ms >= response_window_start_time) {
                // It responded. Check its status.
                if (status_table[i].status_code != STATUS_NOMINAL) {
                    all_nominal = false;
                }
            } else {
                // It did NOT respond in time.
                all_responded = false;
                all_nominal = false; // A missing board is a critical fault
            }
        }
    }

    // Apply your decision logic
    if (!all_responded) {
        return LED_RED; // Option 3: Any board failed to reply
    } else if (!all_nominal) {
        return LED_YELLOW; // Option 2: Any board is not nominal (Red + Green = Yellow)
    } else {
        return LED_GREEN; // Option 1: All nominal
    }
}

static void set_led_color(LedColor color) {
    if (led_cfg.green_port == NULL || led_cfg.red_port == NULL) {
        return; // LED not configured
    }

    switch (color) {
        case LED_OFF:
            HAL_GPIO_WritePin(led_cfg.green_port, led_cfg.green_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(led_cfg.red_port, led_cfg.red_pin, GPIO_PIN_RESET);
            break;
        case LED_GREEN:
            HAL_GPIO_WritePin(led_cfg.green_port, led_cfg.green_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(led_cfg.red_port, led_cfg.red_pin, GPIO_PIN_RESET);
            break;
        case LED_RED:
            HAL_GPIO_WritePin(led_cfg.green_port, led_cfg.green_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(led_cfg.red_port, led_cfg.red_pin, GPIO_PIN_SET);
            break;
        case LED_YELLOW:
            HAL_GPIO_WritePin(led_cfg.green_port, led_cfg.green_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(led_cfg.red_port, led_cfg.red_pin, GPIO_PIN_SET);
            break;
    }
}

static void flash_led_color(LedColor color) {
    set_led_color(color);
    xTimerChangePeriod(led_off_timer, pdMS_TO_TICKS(LED_FLASH_DURATION_MS), 0);
}

static void led_off_timer_callback(TimerHandle_t xTimer) {
    set_led_color(LED_OFF);
}
