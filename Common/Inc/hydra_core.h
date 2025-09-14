#pragma once

#include <stdint.h>
#include <stdbool.h>

/*-----------------------------------------------------------
 * SYSTEM CONFIGURATION PARAMETERS
 *-----------------------------------------------------------*/

//System Configuration

#define MAX_BOARDS_IN_STACK 16			//arbitrary, technical limit is 255 but theoretical is much smaller since driven by CAN FD and CPU limitations

//System Timings

#define SYNC_INTERVAL_MS 1000 			//amount of time between system-wide sync exchanges
#define SYNC_RESPONSE_TIMEOUT 250		//amount of time the initiator of a sync exchange will wait for other boards to respond

#define LED_BLINK_TIME 100				//amount of time an LED stays on for a blink

//CAN FD Configuration

#define CANFD_MAX_DATA_LEN 64
#define CAN_BUS_A_SPEED 500000
#define CAN_BUS_B_SPEED 1000000

/*-----------------------------------------------------------
 * GENERAL COMMUNICATION DEFINES
 *-----------------------------------------------------------*/

// CAN Bus Identifier - Used by comms module

typedef enum {
    CAN_BUS_A = 0, 						//BUS_A is the one with TCAN1146 --> high reliability/time critical bus
    CAN_BUS_B = 1  						//BUS_B is the one with TCAN330  --> high speed data bus
} CanBus;

// Board Type - Used by identity, comms, heartbeat, etc.
typedef enum {
    HYDRA_POWER = 0x01,
    HYDRA_NAV   = 0x02,
    HYDRA_LINK  = 0x03,
    HYDRA_RECOV = 0x04,
} HydraBoardType;
