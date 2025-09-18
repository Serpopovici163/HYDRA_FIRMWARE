/**
  ******************************************************************************
  * @file    hydra_can_icd.h
  * @brief   List of all CAN_IDs and packet formats used by Hydra boards.
  ******************************************************************************
  */

/*-----------------------------------------------------------
 * AUTO-ENUMERATION PACKETS
 *-----------------------------------------------------------*/

#define CAN_ID_ALIAS_ANNOUNCE 			0x50
#define CAN_ID_ALIAS_REJECT 			0x51 //Do we need this? all boards follow the same logic and therefore don't need to be notified of a rejected alias

/*-----------------------------------------------------------
 * STATE MANAGEMENT PACKETS
 *-----------------------------------------------------------*/

// State Management Messages
#define CAN_ID_STATE_REQUEST 			0x90 // Request state transition
#define CAN_ID_STATE_VOTE 				0x91 // Vote response
#define CAN_ID_STATE_COMMIT				0x92 // Final state commit

/*-----------------------------------------------------------
 * HEARTBEAT/SYNC PACKETS
 *-----------------------------------------------------------*/

#define CAN_ID_SYNC 					0x100

/*-----------------------------------------------------------
 * GENERIC TELEMETRY PACKETS
 *-----------------------------------------------------------*/

//USE IDs 0x200-0x300

/*-----------------------------------------------------------
 * SENSOR DATA PACKETS
 *-----------------------------------------------------------*/

// UNIVERSAL SENSOR DATA MESSAGES - For any board (NAV, Recovery, etc.)
#define CAN_ID_RAW_IMU 					0x320 // Raw IMU data from any board
#define CAN_ID_RAW_BARO 				0x321 // Raw barometer data from any board
#define CAN_ID_RAW_GPS 					0x322 // Raw GPS data
#define CAN_ID_FILTERED_STATE 			0x323 // Filtered navigation state

typedef enum {
	SENSOR_TYPE_FILTERED 				= 0,
	SENSOR_TYPE_IMU0 					= 1,
	SENSOR_TYPE_IMU1 					= 2,
	SENSOR_TYPE_BARO0 					= 3,
	SENSOR_TYPE_BARO1 					= 4,
	SENSOR_TYPE_GPS 					= 5,
	SENSOR_TYPE_MAG 					= 6
} SensorType_t;

// Common header for all sensor data messages
typedef struct {
    uint8_t transmitter_alias; 			// The alias of the board sending this data
    SensorType_t sensor_type;         	// See enum statement above
    uint32_t timestamp;        			// Synchronized across stack
} SensorDataHeader_t;

// Specific message structs inherit the header
typedef struct {
    SensorDataHeader_t header; // <-- Common header
    float accel_x, accel_y, accel_z; // m/s²
    float gyro_x, gyro_y, gyro_z;    // rad/s
} RawIMU_t;

typedef struct {
    SensorDataHeader_t header; // <-- Common header
    float pressure;            // Pascals
    float temperature;         // Celsius
} RawBaro_t;

typedef struct {
    SensorDataHeader_t header; // <-- Common header
    double latitude;           // Degrees
    double longitude;          // Degrees
    float altitude;            // Meters
    uint8_t fix_type;          // GPS fix status
} RawGPS_t;

typedef struct {
    SensorDataHeader_t header; // <-- Common header
    float altitude;            // Meters MSL
    float velocity_z;          // m/s (Vertical velocity)
    float acceleration_z;      // m/s² (Vertical acceleration)
    // ... other fused state data ...
} FilteredState_t;
