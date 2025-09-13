#pragma once

#include <stdint.h>
#include <stdbool.h>

//System Configuration

#define MAX_BOARDS_IN_STACK 16			//arbitrary, technical limit is 255 but theoretical is much smaller since driven by CAN FD and CPU limitations

//System timings

#define SYNC_INTERVAL_MS 1000 			//amount of time between system-wide sync exchanges
#define SYNC_RESPONSE_TIMEOUT 250		//amount of time the initiator of a sync exchange will wait for other boards to respond

#define LED_BLINK_TIME 100				//amount of time an LED stays on for a blink

/* CAN FD Configuration */
#define CANFD_MAX_DATA_LEN 64

// CAN Bus Identifier - Used by comms module
typedef enum {
    CAN_BUS_A = 0,
    CAN_BUS_B = 1
} CanBus;

// Board Type - Used by identity, comms, heartbeat, etc.
typedef enum {
    HYDRA_POWER = 0x01,
    HYDRA_NAV   = 0x02,
    HYDRA_LINK  = 0x03,
    HYDRA_RECOV = 0x04,
} HydraBoardType;


// Common header for all sensor data messages
typedef struct {
    uint8_t transmitter_alias; // The alias of the board sending this data
    uint8_t sensor_id;         // ID of the sensor on THAT board (IMU_0, IMU_1, etc.)
    uint32_t timestamp;        // Synchronized across stack
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
