/**
  ******************************************************************************
  * @file    hydra_state_mgr.c
  * @brief   Implementation of decentralized state management with consensus.
  ******************************************************************************
  */

#include "hydra_state_mgr.h"
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
 * Private structs
 *-----------------------------------------------------------*/

/* State transition request format */
typedef struct {
    uint8_t requester_alias;
    SystemState requested_state;
    uint8_t private_votes_received; //used to track how many remote boards have replied, not transmitted
    uint8_t private_expected_voters; //used to log how many boards were online when request was sent, not transmitted
    uint8_t private_approval_count; //used to log number of approvals received, not transmitted
    uint8_t private_veto_count; //used to log number of vetoes received, not transmitted
    uint32_t request_id; //request_id is currently just the output of the requester's HAL_GetTick() whenever the request is transmitted
    uint8_t timeout_ms; //amount of time allowed for board to reply. A reply must be sent within this timespan, if remote board is not confident, it should send VOTE_ABSTAIN
} StateRequestInternal_t;

/* State transition request format */
typedef struct {
    uint8_t requester_alias;
    SystemState requested_state;
    uint32_t request_id; //request_id is currently just the output of the requester's HAL_GetTick() whenever the request is transmitted
    uint8_t timeout_ms; //amount of time allowed for board to reply. A reply must be sent within this timespan, if remote board is not confident, it should send VOTE_ABSTAIN
} StateRequestExternal_t;

/* State commit message format */
typedef struct {
    uint8_t requester_alias;
    uint32_t request_id; //request_id is currently just the output of the requester's HAL_GetTick() whenever the request is transmitted
    SystemState resulting_state;
} StateCommit_t;

/* State transition response format */
typedef struct {
    uint8_t responder_alias;
    uint32_t request_id; //request_id is currently just the output of the requester's HAL_GetTick() whenever the request is transmitted
    StateVote_t state_vote;
} StateResponse_t;

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
 * Private vars
 *-----------------------------------------------------------*/

static StateRequestInternal_t outgoing_request; //used to store this board's local request. Only one request can be initated at any given time
static StateRequestExternal_t incoming_request; //used to store an incoming request, should we not reply to it right away
static TimerHandle_t outgoing_request_timer;
static TimerHandle_t incoming_request_timer;

static uint8_t state_confidences[HYDRA_STATE_MGR_NUM_STATES]; //array to store system confidence in any particular state. All state confidences are continuously monitored by state_mgr
static SystemState current_state = SYSTEM_STATE_NOT_READY; //default to this on boot

/*-----------------------------------------------------------
 * Private function prototypes
 *-----------------------------------------------------------*/

void hydra_state_mgr_request_state_change(SystemState_t new_state);

/*-----------------------------------------------------------
 * Init functions
 *-----------------------------------------------------------*/

/**
  * @brief  Initializes the state manager module.
  */
void hydra_state_mgr_init(void) {
    // TODO: setup anything CAN related that we need to do

    // Create state vote timeout timer
    state_vote_timer = xTimerCreate("StateVoteTimer", pdMS_TO_TICKS(STATE_MGR_VOTE_RESPONSE_TIMEOUT), pdFALSE, NULL, state_vote_timer_callback);
}

/*-----------------------------------------------------------
 * Private var share functions
 *-----------------------------------------------------------*/

SystemState_t hydra_state_mgr_get_current_state(void) {
	return current_state;
}

void hydra_state_mgr_update_confidence(SystemState state, uint8_t confidence) {
	state_confidences[state] = confidence;
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
	outgoing_request.timeout_ticks = xTaskGetTickCount() + pdMS_TO_TICKS(5000);

    // Reset vote counters
	outgoing_request.private_approval_count = 0;
	outgoing_request.private_veto_count = 0;
	outgoing_request.private_expected_voters = hydra_health_get_last_known_board_count(); // This would be set based on known boards from heartbeat

    //TODO: get_peer_list in hydra_identity always returns all boards even if dead but get_last_known_board_count only returns live members that have replied to heartbeats

    // Broadcast state transition request
    uint8_t data[7];
    data[0] = outgoing_request.requester_alias;
    data[1] = (uint8_t)outgoing_request.requested_state;
    data[2] = (outgoing_request.request_id >> 24) & 0xFF;
	data[3] = (outgoing_request.request_id >> 16) & 0xFF;
	data[4] = (outgoing_request.request_id >> 8) & 0xFF;
	data[5] = outgoing_request.request_id & 0xFF;
    data[6] = (uint8_t)STATE_MGR_VOTE_RESPONSE_TIMEOUT;

    hydra_comm_send(CAN_ID_STATE_REQUEST, data, sizeof(data), CAN_BUS_A);

    // Start vote timeout timer
    xTimerChangePeriod(state_vote_timer, pdMS_TO_TICKS(VOTE_RESPONSE_TIMEOUT), 0);
}

/**
  * @brief  Processes incoming state-related CAN messages.
  */
void process_state_message(uint32_t canID, uint8_t* data, uint8_t len) {
	switch (can_id) {
		case CAN_ID_STATE_REQUEST: {

			StateRequest_t request;
			request.requester_alias = data[0];
			request.requested_state = (SystemState)data[1];
			request.request_id = data[2];

			// Get our vote for this state change
			StateVote_t our_vote = VOTE_ABSTAIN;
			if (vote_callback != NULL) {
				our_vote = vote_callback(request.requested_state);
			}

			// Send our vote
			send_state_vote(our_vote, request.request_id);
			break;
		}

		case CAN_ID_STATE_VOTE: {
			if (len < 3) return;

			uint8_t request_id = data[0];
			uint8_t voter_alias = data[1];
			StateVote_t vote = (StateVote_t)data[2];

			// Only process votes for the current pending request
			if (request_id != pending_request.request_id) return;

			// Count the vote
			switch (vote) {
				case VOTE_APPROVE:
					vote_approvals++;
					break;
				case VOTE_VETO:
					vote_vetoes++;
					break;
				case VOTE_ABSTAIN:
				default:
					break;
			}

			// Check if we have consensus or veto
			if (vote_vetoes > 0) {
				// Veto received, cancel the request
				xTimerStop(state_vote_timer, 0);
				// TODO: Broadcast request cancellation
			} else if (vote_approvals >= expected_voters) {
				// Consensus reached, commit the change
				xTimerStop(state_vote_timer, 0);
				commit_state_change(pending_request.requested_state);
			}
			break;
		}

		case CAN_ID_STATE_COMMIT: {
			if (len < 1) return;
			SystemState new_state = (SystemState)data[0];
			commit_state_change(new_state);
			break;
		}
	}
}

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
