/**
  ******************************************************************************
  * @file    hydra_state_mgr.c
  * @brief   Implementation of decentralized state management with consensus.
  ******************************************************************************
  */

#include "hydra_state_mgr.h"
#include "hydra_can_icd.h"
#include "hydra_core.h"
#include "hydra_comm.h"
#include "hydra_identity.h"
#include "hydra_events.h"
#include "hydra_health.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#if defined(STM32G4xx)
    #include "stm32g4xx_hal.h" // Will be included by the G4 board's project
#elif defined(STM32H7xx)
    #include "stm32h7xx_hal.h" // Will be included by the H7 board's project
#endif

/*-----------------------------------------------------------
 * Private enums
 *-----------------------------------------------------------*/

/* State transition vote types */
typedef enum {
    VOTE_APPROVE 						= 0x00,
    VOTE_VETO    						= 0x01,
    VOTE_ABSTAIN 						= 0x02
} StateVote_t;

/*-----------------------------------------------------------
 * Private structs
 *-----------------------------------------------------------*/

/* State transition request format */
typedef struct {
    uint8_t requester_alias;
    SystemState_t requested_state;
    uint8_t private_votes_expected; //used to log how many boards were online when request was sent, not transmitted. Code will subtract from this number when it receives votes
    uint8_t private_approval_count; //used to log number of approvals received, not transmitted
    uint8_t private_veto_count; //used to log number of vetoes received, not transmitted
    uint32_t request_id; //request_id is currently just the output of the requester's HAL_GetTick() whenever the request is transmitted
} StateRequestInternal_t;

/* State transition request format */
typedef struct {
    uint8_t requester_alias;
    uint32_t request_id; //request_id is currently just the output of the requester's HAL_GetTick() whenever the request is transmitted
    SystemState_t requested_state;
} StateRequestExternal_t;

/*-----------------------------------------------------------
 * Private vars
 *-----------------------------------------------------------*/

static StateRequestInternal_t outgoing_request; //used to store this board's local request. Only one request can be initated at any given time
static StateRequestExternal_t incoming_request; //used to store an incoming request, should we not reply to it right away
static TimerHandle_t outgoing_request_timer;
static TimerHandle_t incoming_request_timer;

static uint8_t state_confidences[HYDRA_STATE_MGR_NUM_STATES]; //array to store system confidence in any particular state. All state confidences are continuously monitored by state_mgr
static SystemState_t current_state = SYSTEM_STATE_NOT_READY; //default to this on boot

TimerHandle_t outgoing_request_vote_timeout_timer;

/*-----------------------------------------------------------
 * Private function prototypes
 *-----------------------------------------------------------*/

void hydra_state_mgr_request_state_change(SystemState_t new_state);
StateVote_t hydra_state_mgr_determine_vote(SystemState_t state);
void hydra_state_mgr_vote();
void state_mgr_commit_state_change(SystemState_t state);
void hydra_state_mgr_vote_timeout_callback(TimerHandle_t timer);

/*-----------------------------------------------------------
 * Init functions
 *-----------------------------------------------------------*/

/**
  * @brief  Initializes the state manager module.
  */
void hydra_state_mgr_init(void) {
    // TODO: setup anything CAN related that we need to do

    // Create state vote timeout timer
	outgoing_request_vote_timeout_timer = xTimerCreate("StateVoteTimer", pdMS_TO_TICKS(STATE_MGR_VOTE_RESPONSE_TIMEOUT), pdFALSE, NULL, hydra_state_mgr_vote_timeout_callback);
}

/*-----------------------------------------------------------
 * External functions
 *-----------------------------------------------------------*/

SystemState_t hydra_state_mgr_get_current_state(void) {
	return current_state;
}

void hydra_state_mgr_update_confidence(SystemState state, uint8_t confidence) {
	state_confidences[state] = confidence;
}

/**
  * @brief  Processes incoming state-related CAN messages.
  */
void process_state_message(uint32_t can_id, uint8_t* data, uint8_t len) {
	switch (can_id) {
		case (uint32_t)CAN_ID_STATE_REQUEST: {
			if (len < 6) return;

			incoming_request.requester_alias = data[0];
			incoming_request.requested_state = (SystemState_t)data[1];
			incoming_request.request_id = (data[2] << 24) + (data[3] << 16) + (data[4] << 8) + data[5];

			// Get our vote for this state change
			hydra_state_mgr_vote();
			break;
		}

		case (uint32_t)CAN_ID_STATE_VOTE: {
			if (len < 3) return;

			uint32_t request_id = data[0];
			uint8_t voter_alias = data[1];
			StateVote_t vote = (StateVote_t)data[2];

			// Only process votes for the current pending request
			if (request_id != outgoing_request.request_id) return;

			// Count the vote
			switch (vote) {
				case VOTE_APPROVE:
					outgoing_request.private_approval_count++;
					break;
				case VOTE_VETO:
					outgoing_request.private_veto_count++;
					break;
			}

			outgoing_request.private_votes_expected--;

			// Check if we have received enough votes to make a decision
			if (outgoing_request.requested_state == SYSTEM_STATE_ABORT && outgoing_request.private_approval_count > 0) {
				//We requested to abort and at least one more board agrees so bypass voting process and abort
				xTimerStop(outgoing_request_vote_timeout_timer, 0);
				state_mgr_commit_state_change(outgoing_request.requested_state);
			} else if (outgoing_request.private_veto_count > STATE_MGR_MAX_VETOES) {
				// Veto received, cancel the request
				xTimerStop(outgoing_request_vote_timeout_timer, 0);
			} else if (outgoing_request.private_votes_expected == 0 && outgoing_request.private_approval_count >= STATE_MGR_MIN_APPROVALS) {
				// We've received all expected votes, make a call
				xTimerStop(outgoing_request_vote_timeout_timer, 0);
				state_mgr_commit_state_change(outgoing_request.requested_state);
			}
			break;
		}

		case (uint32_t)CAN_ID_STATE_COMMIT: {
			if (len < 1) return;

		}
	}
}

/*-----------------------------------------------------------
 * State management request functions
 *-----------------------------------------------------------*/

/**
  * @brief  Requests a transition to a new system state (initiates voting).
  * @param  new_state: The desired state to transition to.
  */
void hydra_state_mgr_request_state_change(SystemState_t new_state) { //todo: how do we deal with multiple boards sending state requests at the same time? --> deterministically pick one or the other (prioritize failsafe type requests), maybe logic allows for multiple concurrent requests with minimal changes
    // Create state transition request
	outgoing_request.requester_alias = hydra_identity_get_alias();
	outgoing_request.requested_state = new_state;
	outgoing_request.request_id = HAL_GetTick();

    // Reset vote counters
	outgoing_request.private_approval_count = 0;
	outgoing_request.private_veto_count = 0;
	outgoing_request.private_votes_expected = hydra_health_get_last_known_board_count(); // This would be set based on known boards from heartbeat

    //TODO: get_peer_list in hydra_identity always returns all boards even if dead but get_last_known_board_count only returns live members that have replied to heartbeats

    // Broadcast state transition request
    uint8_t data[6];
    data[0] = outgoing_request.requester_alias;
    data[1] = (outgoing_request.request_id >> 24) & 0xFF;
	data[2] = (outgoing_request.request_id >> 16) & 0xFF;
	data[3] = (outgoing_request.request_id >> 8) & 0xFF;
	data[4] = outgoing_request.request_id & 0xFF;
	data[5] = (uint8_t)outgoing_request.requested_state;

    hydra_comm_send((uint32_t)CAN_ID_STATE_REQUEST, data, 6, CAN_BUS_A);

    // Start vote timeout timer
    xTimerChangePeriod(outgoing_request_vote_timeout_timer, pdMS_TO_TICKS(STATE_MGR_VOTE_RESPONSE_TIMEOUT), 0);
}

/**
  * @brief  Figure out what our vote should be for a given state
  * @param  state: What state we should be analyzing
  */

StateVote_t hydra_state_mgr_determine_vote(SystemState_t state) {
	/*
	 * Very simple logic to begin with, might make it cooler later
	 * This logic relies on the confidence value
	 */
	if (state_confidences[state] < STATE_MGR_VETO_CONFIDENCE) return VOTE_VETO;
	else if (state_confidences[state] > STATE_MGR_APPROVE_CONFIDENCE) return VOTE_APPROVE;
	else return VOTE_ABSTAIN;
}

/**
  * @brief  Responds to an incoming state request with a vote
  * @param  vote: The desired vote to send.
  */
void hydra_state_mgr_vote() {
	uint8_t data[6];

	data[0] = hydra_identity_get_alias();
	data[1] = (incoming_request.request_id >> 24) & 0xFF;
	data[2] = (incoming_request.request_id >> 16) & 0xFF;
	data[3] = (incoming_request.request_id >> 8) & 0xFF;
	data[4] = incoming_request.request & 0xFF;
	data[5] = (uint8_t)hydra_state_mgr_determine_vote(incoming_request.requested_state);

	hydra_comm_send((uint32_t)CAN_ID_STATE_VOTE, data, sizeof(data), CAN_BUS_A);
}

/**
  * @brief  Commits a state to the system
  */
void state_mgr_commit_state_change(SystemState_t state) {
	//Update our own state
	current_state = state;

	//Let everybody know about the new state
	uint8_t data[6];
	data[0] = outgoing_request.requester_alias;
	data[1] = (outgoing_request.request_id << 24) && 0xFF;
	data[2] = (outgoing_request.request_id << 16) && 0xFF;
	data[3] = (outgoing_request.request_id << 8) && 0xFF;
	data[4] = outgoing_request.request_id && 0xFF;
	data[5] = outgoing_request.requested_state;

	hydra_comm_send((uint32_t)CAN_ID_STATE_COMMIT, data, sizeof(data), CAN_BUS_A);

	//Clear outgoing_request since its data is no longer relevant and we want to avoid potential future mistakes
	memset(&outgoing_request, 0, sizeof(outgoing_request));
}

/*-----------------------------------------------------------
 * Timer callbacks
 *-----------------------------------------------------------*/

void hydra_state_mgr_vote_timeout_callback(TimerHandle_t timer) {
	// We've waited for votes to come in long enough
	// If the system as a whole agrees with our state request, commit
	if (outgoing_request.private_approval_count >= STATE_MGR_MIN_APPROVALS) {
		state_mgr_commit_state_change(outgoing_request.requested_state);
	}
}

/*-----------------------------------------------------------
 * Task functions //TODO: do we want the tasks in here? Maybe each board should have its own task setup elsewhere
 *-----------------------------------------------------------*/

/**
  * @brief  The main state manager task function.
  */
void state_mgr_task(void *argument) {
    hydra_state_mgr_init();

    for (;;) {
        // Main task loop - can handle periodic state checks here
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
