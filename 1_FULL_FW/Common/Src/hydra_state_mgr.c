/**
  ******************************************************************************
  * @file  	hydra_state_mgr.c
  * @brief  Implementation of decentralized confidence-based state machine with consensus.
  *
  * @logic	This module stores an array of confidences for each possible state of the vehicle. The only interaction
  * 		another part of the code can have with the state manager is updating the confidence for a particular state.
  *
  * 		The state manager regularly monitors the host's confidence in each state, and if:
  *
  * 			1. the host's confidence of a particular state exceeds STATE_MGR_APPROVE_CONFIDENCE, and
  * 			2. the host's confidence of every other state is lower than STATE_MGR_VETO_CONFIDENCE, and
  * 			3. the current_state differs from the state the host is most confident about, //TODO
  *
  * 		the state manager automatically sends a state change request to all other boards in the system.
  *
  * 		Whenever a state change request is received, the board checks its local confidence for the requested state
  * 		and replies as follows:
  *
  * 			1. if local confidence > STATE_MGR_APPROVE_CONFIDENCE 								--> VOTE_APPROVE
  * 				and every other state confidence < STATE_MGR_VETO_CONFIDENCE
  * 			2. if STATE_MGR_VETO_CONFIDENCE < local confidence < STATE_MGR_APPROVE_CONFIDENCE	--> VOTE_ABSTAIN
  * 			3. if local confidence < STATE_MGR_VETO_CONFIDENCE									--> VOTE_VETO
  *
  * 		The requester will make a call on the system state once it has either:
  *
  * 			1. received all the votes it expected to receive
  * 			2. STATE_MGR_VOTE_RESPONSE_TIMEOUT milliseconds have elapsed from when the request was initiated
  *
  * 		The requester checks if condition 1 is valid every time it receives a new vote, and uses a timer to execute
  * 		condition 2. The logic for making a call is at follows:
  *
  * 			1. If the requested state was SYSTEM_STATE_ABORT and at least one other board has approved, we abort
  * 			2. If at least STATE_MGR_MAX_VETOES boards have voted VOTE_VETO, the request is abandoned
  * 			3. If at least STATE_MGR_MIN_APPROVALS boards have voted VOTE_APPROVE and neither 1. or 2. was true,
  * 			   commit state
  *
  * 		If the request is dropped, no further communication happens from the requester (TODO: maybe change?).
  * 		If the system agrees to commit to the requested state, the requester will send a state commit message to
  * 		notify everybody of the updated state.
  *
  * @CAN frames:
  *
  * 		CAN_ID_STATE_REQUEST (6 bytes):
  * 			uint8_t --> alias
  * 			uint8_t --> requested_state (as per SystemState_t enum)
  * 			uint8_t --> time[4] <-- 4 item array since actually a 32bit int
  *
  * 			The time[4] is used to append the output of the requester's HAL_GetTick() function whenever a request is
  * 			generated. This is used to provide unique IDs for every request.
  *
  * 			For every subsequent vote or commit request, the time of the initial state change request is reused.
  *
  * 		CAN_ID_STATE_VOTE (6 bytes):
  * 			uint8_t --> alias
  * 			uint8_t --> vote (as per StateVote_t enum)
  * 			uint8_t --> time[4] <-- 4 item array since actually a 32bit int
  *
  * 		CAN_ID_STATE_COMMIT (6 bytes):
  * 			uint8_t --> alias
  * 			uint8_t --> new_state (as per SystemState_t enum)
  * 			uint8_t --> time[4] <-- 4 item array since actually a 32bit int
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



/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/

void hydra_state_mgr_init(uint8_t _has_position_sensors) {
	//If this board does not have positional sensors, set all flight-related states to 50% confidence so the board automatically abstains from voting process
	if (_has_position_sensors == 0) {
		//We don't have positional sensors
		hydra_state_update_confidence(SYSTEM_STATE_POWERED_ASCENT, (uint8_t)(STATE_MGR_MAX_CONFIDENCE/2));
		hydra_state_update_confidence(SYSTEM_STATE_COASTING, (uint8_t)(STATE_MGR_MAX_CONFIDENCE/2));
		hydra_state_update_confidence(SYSTEM_STATE_APOGEE, (uint8_t)(STATE_MGR_MAX_CONFIDENCE/2));
		hydra_state_update_confidence(SYSTEM_STATE_DROGUE_DEPLOY, (uint8_t)(STATE_MGR_MAX_CONFIDENCE/2));
		hydra_state_update_confidence(SYSTEM_STATE_MAIN_DEPLOY, (uint8_t)(STATE_MGR_MAX_CONFIDENCE/2));
		hydra_state_update_confidence(SYSTEM_STATE_LANDED, (uint8_t)(STATE_MGR_MAX_CONFIDENCE/2));
	}

    // TODO: setup anything CAN related that we need to do

    // Create state vote timeout timer
	outgoing_request_vote_timeout_timer = xTimerCreate("StateVoteTimer", pdMS_TO_TICKS(STATE_MGR_VOTE_RESPONSE_TIMEOUT), pdFALSE, NULL, hydra_state_mgr_vote_timeout_callback);
}

SystemState_t hydra_state_get_current_state(void) {
	return current_state;
}

void hydra_state_update_confidence(SystemState_t _state, uint8_t _confidence) {
	state_confidences[_state] = (_confidence > (uint8_t)STATE_MGR_MAX_CONFIDENCE) ? (uint8_t)STATE_MGR_MAX_CONFIDENCE : _confidence;

	//check if we should be sending a state change request
	SystemState_t new_system_state = _hydra_state_confidence_status();

	if (new_system_state != SYSTEM_STATE_NOT_CONFIDENT && new_system_state != current_state) {
		_hydra_state_send_state_change_request(new_system_state);
	}

}

void hydra_state_process_rx(uint32_t _can_id, uint8_t* _data, uint8_t _len) {
	switch (_can_id) {
		case (uint32_t)CAN_ID_STATE_REQUEST: {
			if (_len < 6) return;

			// Overwrite incoming_request and vote on the request.
			// No need to concern ourselves with overwriting an ongoing request since we always vote immediately upon receiving a request
			incoming_request.requester_alias = _data[0];
			incoming_request.requested_state = (SystemState_t)_data[1];
			incoming_request.request_id = (_data[2] << 24) + (_data[3] << 16) + (_data[4] << 8) + _data[5];

			// Send our vote for this state change
			_hydra_state_send_vote();
			break;
		}

		case (uint32_t)CAN_ID_STATE_VOTE: {
			if (_len < 3) return;

			uint8_t voter_alias = _data[0];
			StateVote_t vote = (StateVote_t)_data[1];
			uint32_t request_id = (_data[2] << 24) + (_data[3] << 16) + (_data[4] << 8) + _data[5];

			// Only process votes for the current pending request
			if (request_id != outgoing_request.request_id) return;

			// If this vote is for our pending request, count it
			switch (vote) {
				case VOTE_APPROVE:
					outgoing_request.private_approval_count++;
					break;
				case VOTE_VETO:
					outgoing_request.private_veto_count++;
					break;
			}

			// We can ignore and abstain votes, but we need to count that we received a vote
			outgoing_request.private_votes_expected--;

			//tally votes every time we get a new one, or when the vote timeout timer expires
			_hydra_state_tally_votes();
			break;
		}

		case (uint32_t)CAN_ID_STATE_COMMIT: {
			if (_len < 1) return;


		}
	}
}

/*-----------------------------------------------------------
 * Private functions
 *-----------------------------------------------------------*/

SystemState_t _hydra_state_confidence_status(void) {
	uint8_t low_confidence_tally = 0;
	SystemState_t highest_confidence_state = SYSTEM_STATE_NOT_READY; //default to this

	for (uint8_t i = 0; i < (uint8_t)HYDRA_STATE_MGR_NUM_STATES; i++) {
		if (state_confidences[i] < (uint8_t)STATE_MGR_VETO_CONFIDENCE) low_confidence_tally++;
		else if (state_confidences[i] > (uint8_t)STATE_MGR_APPROVE_CONFIDENCE) {
			if (state_confidences[i] == current_state) return;
			highest_confidence_state = (SystemState_t)i;
		}
	}

	//continue only if one state has high confidence and ALL others have low confidence
	if (low_confidence_tally != ((uint8_t)HYDRA_STATE_MGR_NUM_STATES-1)) return SYSTEM_STATE_NOT_CONFIDENT;

	return highest_confidence_state;
}

/**
  * @brief  Requests a transition to a new system state (initiates voting).
  * @param  new_state: The desired state to transition to.
  */
void _hydra_state_send_state_change_request(SystemState_t _new_state) {
    // Create state transition request
	outgoing_request.requester_alias = hydra_identity_get_alias();
	outgoing_request.requested_state = _new_state;
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

StateVote_t _hydra_state_determine_vote() {
	/*
	 * Very simple logic to begin with, might make it cooler later
	 * This logic relies on the confidence value
	 */

	if (state_confidences[incoming_request.requested_state] < (uint8_t)STATE_MGR_VETO_CONFIDENCE) {
		return VOTE_VETO;
	} else if (_hydra_state_confidence_status() == incoming_request.requested_state) {
		return VOTE_APPROVE;
	}

	//if we haven't reached a conclusion, abstain
	return VOTE_ABSTAIN;
}

/**
  * @brief  Responds to an incoming state request with a vote
  */
void _hydra_state_send_vote(void) {
	uint8_t data[6];

	data[0] = hydra_identity_get_alias();
	data[1] = (incoming_request.request_id >> 24) & 0xFF;
	data[2] = (incoming_request.request_id >> 16) & 0xFF;
	data[3] = (incoming_request.request_id >> 8) & 0xFF;
	data[4] = incoming_request.request_id & 0xFF;
	data[5] = (uint8_t)_hydra_state_tally_votes();

	hydra_comm_send((uint32_t)CAN_ID_STATE_VOTE, data, sizeof(data), CAN_BUS_A);
}

//if abort
xTimerStop(outgoing_request_vote_timeout_timer, 0);
_hydra_state_commit_state_change();

//if veto
xTimerStop(outgoing_request_vote_timeout_timer, 0);

//if ok
xTimerStop(outgoing_request_vote_timeout_timer, 0);
_hydra_state_commit_state_change();

SystemState_t _hydra_state_tally_votes(void) {

	// Check priority state changes first (abort and veto)
	if (outgoing_request.requested_state == SYSTEM_STATE_ABORT && outgoing_request.private_approval_count > 0) {
		//We requested to abort and at least one more board agrees so bypass voting process and abort
		return outgoing_request.requested_state;
	} else if (outgoing_request.private_veto_count > STATE_MGR_MAX_VETOES) {
		// Veto received, cancel the request
		return current_state;
	}

	//if we're not aborting or vetoing, check if everybody has voted and see if we have enough approvals to go through with state commit
	else if (outgoing_request.private_votes_expected == 0 && outgoing_request.private_approval_count >= STATE_MGR_MIN_APPROVALS) {
		// We've received all expected votes and the system approves
		return outgoing_request.requested_state;
	}
}

/**
  * @brief  Commits a state to the system
  */
void _hydra_state_commit_state_change(void) {
	//Update our own state
	current_state = outgoing_request.requested_state;

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

void _hydra_state_vote_timeout_callback(TimerHandle_t _timer) { //why passing timer?
	// We've waited for votes to come in long enough
	// If the system as a whole agrees with our state request, commit
	if (outgoing_request.private_approval_count >= STATE_MGR_MIN_APPROVALS) {
		state_mgr_commit_state_change(outgoing_request.requested_state);
	}
}
