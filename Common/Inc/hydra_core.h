#pragma once

#include <stdint.h>
#include <stdbool.h>

/*-----------------------------------------------------------
 * SYSTEM CONFIGURATION PARAMETERS
 *-----------------------------------------------------------*/

//System Configuration

#define MAX_BOARDS_IN_STACK 16						//arbitrary, technical limit is 255 but theoretical is much smaller since driven by CAN FD and CPU limitations

//System Timings

#define SYNC_INTERVAL_MS 1000 						//amount of time between system-wide sync exchanges
#define SYNC_DISPERSION_MS 1000
#define SYNC_RESPONSE_TIMEOUT 250					//amount of time the initiator of a sync exchange will wait for other boards to respond

#define LED_BLINK_TIME 100							//amount of time an LED stays on for a blink

#define SYSTEM_TIME_ADJUSTMENT_DEADBAND_MS 25		//minimum amount of discrepancy (in ms) between a host's perceived local time and the best available system time before the host adjusts its local time

//State manager configuration

#define STATE_MGR_VOTE_RESPONSE_TIMEOUT 500 		//Max amount of time for boards to reply to a state change request before the request times out
#define STATE_MGR_MAX_STATE_REQ_ATTEMPTS 5 			//Max number of consecutive state change request attempts before the initiator acts without confirmation from other boards.

#define STATE_MGR_MAX_CONFIDENCE 63					//Maximum numerical confidence value for a state (limited by the number of bits reserved for this value in the sync packet)
#define STATE_MGR_VETO_CONFIDENCE 20				//Confidence value below which a board will veto a state request
#define STATE_MGR_APPROVE_CONFIDENCE 45				//Min confidence value required for a board to approve a request

#define STATE_MGR_MAX_VETOES 1						//Maximum allowable number of vetoes for which a state transition can succeed regardless of how many boards approve
#define STATE_MGR_MIN_APPROVALS 1					//Minimum number of approvals a board must receive to change states

//CAN FD Configuration

#define CANFD_MAX_DATA_LEN 64
#define CAN_BUS_A_SPEED 500000
#define CAN_BUS_B_SPEED 1000000

/*-----------------------------------------------------------
 * GENERAL COMMUNICATION DEFINES
 *-----------------------------------------------------------*/

// CAN Bus Identifier - Used by comms module

typedef enum {
    CAN_BUS_A = 0, 									//BUS_A is the one with TCAN1146 --> high reliability/time critical bus
    CAN_BUS_B = 1  									//BUS_B is the one with TCAN330  --> high speed data bus
} CanBus;

// Board Type - Used by identity, comms, heartbeat, etc.
typedef enum {
    HYDRA_POWER = 0x01,
    HYDRA_NAV   = 0x02,
    HYDRA_LINK  = 0x03,
    HYDRA_RECOV = 0x04,
} HydraBoardType;
