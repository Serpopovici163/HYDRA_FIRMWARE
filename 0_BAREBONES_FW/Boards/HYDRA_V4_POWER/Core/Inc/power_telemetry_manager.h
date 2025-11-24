/*-----------------------------------------------------------
 * Public enums
 *-----------------------------------------------------------*/

#define TLM_MSG_TYPE_NUM 2

typedef enum {
	TLM_MSG_PWR_SENSE 	= 0x00,
	TLM_MSG_BRD_STATE	= 0x01
} TelemetryMessageType_t;

/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/

void power_telemetry_manager_init(void);

void power_telemetry_manager_toggle_message(TelemetryMessageType_t msg_type, uint8_t enable);
