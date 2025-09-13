/**
  ******************************************************************************
  * @file    hydra_state_mgr.h
  * @brief   Header for decentralized state management with consensus voting.
  ******************************************************************************
  */
#pragma once

#include "hydra_core.h"

typedef enum {
    // Pre-flight States
    SYSTEM_STATE_PREFLIGHT = 0,
    SYSTEM_STATE_ARMED,

    // Ascent States (can be re-entered for staging)
    SYSTEM_STATE_POWERED_ASCENT,
    SYSTEM_STATE_COASTING,

    // Event-based States (can only happen once)
    SYSTEM_STATE_APOGEE,
    SYSTEM_STATE_DROGUE_DEPLOY,
    SYSTEM_STATE_MAIN_DEPLOY,
    SYSTEM_STATE_LANDED,

    // Anytime States
    SYSTEM_STATE_SAFE,
    SYSTEM_STATE_CRITICAL_FAILURE,

    SYSTEM_STATE_COUNT
} SystemState;

/* State transition vote types */
typedef enum {
    VOTE_APPROVE = 0x00,
    VOTE_VETO    = 0x01,
    VOTE_ABSTAIN = 0x02
} StateVote_t;

/* Public API */
void hydra_state_mgr_init(void);
SystemState hydra_state_mgr_get_current_state(void);
void hydra_state_mgr_request_state(SystemState new_state);
void hydra_state_mgr_register_vote_callback(StateVote_t (*callback)(SystemState proposed_state));

/* The main state manager task */
void state_mgr_task(void *argument);
