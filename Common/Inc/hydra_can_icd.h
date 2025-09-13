// State Management Messages
#define CAN_ID_STATE_REQUEST 0x90, // Request state transition
#define CAN_ID_STATE_VOTE 0x91, // Vote response
#define CAN_ID_STATE_COMMIT 0x92, // Final state commit

// UNIVERSAL SENSOR DATA MESSAGES - For any board (NAV, Recovery, etc.)
#define CAN_ID_RAW_IMU 0x320, // Raw IMU data from any board
#define CAN_ID_RAW_BARO 0x321, // Raw barometer data from any board
#define CAN_ID_RAW_GPS 0x322, // Raw GPS data
#define CAN_ID_FILTERED_STATE 0x323, // Filtered navigation state
