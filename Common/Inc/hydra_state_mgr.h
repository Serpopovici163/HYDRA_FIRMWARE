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
} SystemState_t;

/* State transition vote types */
typedef enum {
    VOTE_APPROVE 						= 0x00,
    VOTE_VETO    						= 0x01,
    VOTE_ABSTAIN 						= 0x02
} StateVote_t;

/*-----------------------------------------------------------
 * CAN frame format
 *-----------------------------------------------------------*/

/* State transition request format */
typedef struct {
    uint8_t requester_alias;
    SystemState requested_state;
    uint8_t request_id;
    uint32_t timeout_ticks;
} StateRequest_t;

/* State transition response format */
typedef struct {
    uint8_t responder_alias;
    uint8_t request_id;
    StateVote_t state_vote;
} StateResponse_t;

/* State commit message format */
typedef struct {
    uint8_t requester_alias;
    uint8_t request_id;
    SystemState resulting_state;
} StateCommit_t;

/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/

void hydra_state_mgr_init(void);
SystemState_t hydra_state_mgr_get_current_state(void);
void hydra_state_mgr_request_state(SystemState new_state);
void hydra_state_mgr_register_vote_callback(StateVote_t (*callback)(SystemState proposed_state));

/* The main state manager task */
void state_mgr_task(void *argument);
