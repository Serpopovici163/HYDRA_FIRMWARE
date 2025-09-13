#include "hydra_heartbeat.h"
#include "hydra_events.h"
#include "hydra_comm.h"
#include "hydra_identity.h"
#include "FreeRTOS.h"
#include "task.h"

#if defined(STM32G4xx)
    #include "stm32g4xx_hal.h" // Will be included by the G4 board's project
#elif defined(STM32H7xx)
    #include "stm32h7xx_hal.h" // Will be included by the H7 board's project
#endif

// Private variables
static uint16_t local_status_code = 0;
static uint8_t known_board_count = 0;

static BoardStatus_t status_table[MAX_BOARDS_IN_STACK];
static HeartbeatState_t heartbeat_state = HEARTBEAT_IDLE;

static TimerHandle_t sync_timer;
static TimerHandle_t led_off_timer;

static uint32_t response_window_start_time = 0;
static uint8_t expected_responders = 0;

// LED Configuration (stored after init)
static HydraHeartbeatLedConfig_t led_cfg;

// Time Synchronization State
static HydraTime_t system_time;
static TimeSource_t local_time_source = TIME_SOURCE_INTERNAL;
static uint8_t local_time_confidence = 0xFF; // Worst confidence

/* Private function prototypes */
static void send_sync_message(uint8_t message_type);
static void process_sync_message(uint32_t can_id, uint8_t* data, uint8_t len, CanBus bus);
static void sync_timer_callback(TimerHandle_t xTimer);
static void led_off_timer_callback(TimerHandle_t xTimer);
static void check_response_window(void);

//LED stuff
static LedColor determine_summary_color(void);
static void set_led_color(LedColor color);

// Function to update our own time concept
static void update_system_time(uint32_t new_seconds, uint32_t new_ms, TimeSource_t new_source, uint8_t new_confidence);

void hydra_heartbeat_init(const HydraHeartbeatLedConfig_t *led_config) {
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

    // Register the CAN callback
    hydra_comm_set_rx_callback(process_sync_message);

    // Create timers
    sync_timer = xTimerCreate("SyncTimer", pdMS_TO_TICKS(1000), pdTRUE, NULL, sync_timer_callback);
    led_off_timer = xTimerCreate("LEDOffTimer", pdMS_TO_TICKS(LED_FLASH_DURATION_MS), pdFALSE, NULL, led_off_timer_callback);

    if (sync_timer && led_off_timer) {
        xTimerStart(sync_timer, 0);
    }

    // Initialize status table
    memset(status_table, 0, sizeof(status_table));
}

void heartbeat_task(void *argument) {
    hydra_heartbeat_init((HydraHeartbeatLedConfig_t *)argument);

    for (;;) {
        check_response_window();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void hydra_heartbeat_set_local_status(uint16_t status) {
    local_status_code = status;
}

BoardStatus_t* hydra_heartbeat_get_status_table(uint8_t* count) {
    if (count) *count = known_board_count;
    return status_table;
}

static void send_sync_message(uint8_t message_type) {
    uint8_t data[14]; // Expanded message

    data[0] = message_type;
    data[1] = hydra_identity_get_alias();
    data[2] = local_status_code & 0xFF;
    data[3] = (local_status_code >> 8) & 0xFF;

    //Append our current time information
    data[4] = (uint8_t)local_time_source;
    *(uint32_t*)(&data[5]) = __REV(system_time.seconds);
    *(uint32_t*)(&data[9]) = __REV(system_time.milliseconds);
    data[13] = local_time_confidence;

    hydra_comm_send(CAN_ID_SYNC, data, sizeof(data), CAN_BUS_A);
}

static void process_sync_message(uint32_t can_id, uint8_t* data, uint8_t len, CanBus bus) {
    if (can_id != CAN_ID_SYNC || len < 14) return;

    uint8_t message_type = data[0];
    uint8_t source_alias = data[1];
    uint16_t status_code = data[2] | (data[3] << 8);
    uint32_t current_time = HAL_GetTick();

    // NEW: Extract time data from the sync message
	TimeSource_t rx_time_source = (TimeSource_t)data[4];
	uint32_t rx_seconds = __REV(*(uint32_t*)(&data[5]));
	uint32_t rx_milliseconds = __REV(*(uint32_t*)(&data[9]));
	uint8_t rx_confidence = data[13];

	//  DETERMINISTIC TIME SELECTION LOGIC
	//  1. Always prefer a higher-quality source (GPS > ESTIMATED > INTERNAL)
	if (rx_time_source > system_time.source) {
		update_system_time(rx_seconds, rx_milliseconds, rx_time_source, rx_confidence);
	}
	//  2. If source quality is equal, prefer a more confident or more recent time
	else if (rx_time_source == system_time.source) {
		// For GPS, prefer lower confidence value (higher accuracy)
		if (rx_time_source == TIME_SOURCE_GPS) {
			if (rx_confidence < system_time.confidence_ms) {
				update_system_time(rx_seconds, rx_milliseconds, rx_time_source, rx_confidence);
			}
		}
		// For other sources, just take the largest (most recent) time value
		else if (rx_seconds > system_time.seconds ||
					(rx_seconds == system_time.seconds && rx_milliseconds > system_time.milliseconds)) {
			update_system_time(rx_seconds, rx_milliseconds, rx_time_source, rx_confidence);
		}
	}
	//  3. If the incoming source is lower quality, ignore its time.

    // CRITICAL: Update the entry with FRESH data
    status_table[i].status_code = status_code;
    status_table[i].last_heard_ms = current_time; // This timestamp is key
    status_table[i].present = true;

    // If this is a sync initiation (not a response), respond immediately
    if (message_type == 0x01) {
        hydra_led_flash_green(); // First flash (in unison with initiator)
        send_sync_message(0x02); // Type 0x02: Sync Response
    }
}

static void update_system_time(uint32_t new_seconds, uint32_t new_ms, TimeSource_t new_source, uint8_t new_confidence) {
    system_time.seconds = new_seconds;
    system_time.milliseconds = new_ms;
    system_time.source = new_source;
    system_time.confidence_ms = new_confidence;

    // If we are updating our time based on someone else, we are no longer a source.
    // Our next sync message will reflect this new, better time and its source.
    local_time_source = TIME_SOURCE_INTERNAL;
    local_time_confidence = 0xFF; // We are just a relay, we don't know the true confidence
}

static void sync_timer_callback(TimerHandle_t xTimer) {
    // We are the initiator this time
    heartbeat_state = HEARTBEAT_WAITING_FOR_RESPONSES;
    response_window_start_time = HAL_GetTick();

    // Step 1: First Flash (Green) and send initiation
    hydra_led_flash_green();
    send_sync_message(0x01); // Type 0x01: Sync Initiation

    // We will check for responses in the main task or a dedicated function called periodically
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

static LedColor determine_summary_color(void) {
    bool all_nominal = true;
    bool all_responded = true;

    for (int i = 0; i < known_board_count; i++) {
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
