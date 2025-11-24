#include <stdint.h>
#include "cmsis_os.h"
#include "power_telemetry_manager.h"

/*-----------------------------------------------------------
 * Private structs
 *-----------------------------------------------------------*/

/*-----------------------------------------------------------
 * Private variables
 *-----------------------------------------------------------*/
uint8_t tlm_msg_type_state[TLM_MSG_TYPE_NUM] = {0};

static osThreadId_t powerTelemetryTaskHandle;
const osThreadAttr_t powerTelemetryTaskAttr = {
		.name = "PowerTelemetryTask",
		.stack_size = 1024,
		.priority = (osPriority_t) osPriorityNormal
};

/*-----------------------------------------------------------
 * Private prototypes
 *-----------------------------------------------------------*/
static void PowerTelemetryTask(void *argument);

/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/
void power_telemetry_manager_init(void) {
	//start telemetry task
	//Begin state manager task
	powerTelemetryTaskHandle = osThreadNew(PowerTelemetryTask, NULL, &powerTelemetryTaskAttr);
}

void power_telemetry_manager_toggle_message(TelemetryMessageType_t msg_type, uint8_t enable) {
	//ensure enable is either 1 or 0
	if (enable >= 0x01)
		enable = 0x01;

	//update msg state
	tlm_msg_type_state[(uint8_t)msg_type];
}

/*-----------------------------------------------------------
 * Internal chocolate
 *-----------------------------------------------------------*/
//Task function
static void PowerTelemetryTask(void *argument) {
	for (;;) {
		if (tlm_msg_type_state[(uint8_t)TLM_MSG_PWR_SENSE]) {

		}
		if (tlm_msg_type_state[(uint8_t)TLM_MSG_BRD_STATE]) {

		}

		osDelay(100);
	}
}

void pack_12bit_values(const uint16_t *in, uint8_t *out, uint8_t count)
{
    uint32_t acc = 0;
    uint8_t bits = 0;
    uint8_t out_idx = 0;

    for (uint8_t i = 0; i < count; i++) {
        acc |= ((uint32_t)(in[i] & 0x0FFF)) << bits;
        bits += 12;

        while (bits >= 8) {
            out[out_idx++] = acc & 0xFF;
            acc >>= 8;
            bits -= 8;
        }
    }

    if (bits > 0)
        out[out_idx++] = acc & 0xFF;
}
