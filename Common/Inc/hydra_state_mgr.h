/**
  ******************************************************************************
  * @file    hydra_state_mgr.h
  * @brief   Header for decentralized state management with consensus voting.
  ******************************************************************************
  */
#pragma once

#include "hydra_core.h"

/*-----------------------------------------------------------
 * Public enums
 *-----------------------------------------------------------*/

#define HYDRA_STATE_MGR_NUM_STATES 		11 //number of states in the SystemState_t enum

// Defines possible states for system as a whole
// IMPORTANT: only 4 bits are available in sync message for these values
// Range: 0x0-0xF
typedef enum {
    // Pre-flight States
	SYSTEM_STATE_NOT_READY				= 0x0,
    SYSTEM_STATE_PREFLIGHT 				= 0x1,
    SYSTEM_STATE_ARMED					= 0x2,

    // Ascent States (can be re-entered for staging)
    SYSTEM_STATE_POWERED_ASCENT			= 0x3,
    SYSTEM_STATE_COASTING				= 0x4,

    // Event-based States (can only happen once)
    SYSTEM_STATE_APOGEE					= 0x5,
    SYSTEM_STATE_DROGUE_DEPLOY			= 0x6,
    SYSTEM_STATE_MAIN_DEPLOY			= 0x7,
    SYSTEM_STATE_LANDED					= 0x8,

    // Anytime States
    SYSTEM_STATE_SAFE					= 0x9,
    SYSTEM_STATE_ABORT					= 0xA,
} SystemState_t; //Update HYDRA_STATE_MGR_NUM_STATES if this gets changed!!!

/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/

void hydra_state_mgr_init(void);
SystemState_t hydra_state_mgr_get_current_state(void);
void hydra_state_mgr_update_confidence(SystemState_t state, uint8_t confidence); //important: confidence can only be 6 bits, max is 63

void process_state_message(uint8_t* data, uint8_t len);

/* The main state manager task */
void state_mgr_task(void *argument);
