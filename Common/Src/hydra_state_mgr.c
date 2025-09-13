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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* State transition request data */
typedef struct {
    uint8_t requester_alias;
    SystemState requested_state;
    uint8_t request_id;
    uint32_t timeout_ticks;
} StateRequest_t;

/* Private variables */
static SystemState current_state = SYSTEM_STATE_PREFLIGHT;
static StateVote_t (*vote_callback)(SystemState) = NULL;

/* State transition management */
static StateRequest_t pending_request;
static uint8_t vote_approvals = 0;
static uint8_t vote_vetoes = 0;
static uint8_t expected_voters = 0;
static TimerHandle_t state_vote_timer;

/* Queue for state-related CAN messages */
QueueHandle_t xStateRxQueue;

/* Private function prototypes */
static void process_state_message(uint32_t can_id, uint8_t* data, uint8_t len, CanBus bus);
static void state_vote_timer_callback(TimerHandle_t xTimer);
static void send_state_vote(StateVote_t vote, uint8_t request_id);
static void commit_state_change(SystemState new_state);

/**
  * @brief  Initializes the state manager module.
  */
void hydra_state_mgr_init(void) {
    // Register CAN callback for state messages
    hydra_comm_set_rx_callback(process_state_message);

    // Create queue for state messages if needed
    xStateRxQueue = xQueueCreate(10, sizeof(HydraCanRxMessage_t));

    // Create state vote timeout timer
    state_vote_timer = xTimerCreate("StateVoteTimer", pdMS_TO_TICKS(5000), pdFALSE, NULL, state_vote_timer_callback);
}

/**
  * @brief  Gets the current system state.
  * @retval Current SystemState.
  */
SystemState hydra_state_mgr_get_current_state(void) {
    return current_state;
}

/**
  * @brief  Registers a board-specific vote callback function.
  * @param  callback: Function pointer that returns a StateVote_t for a proposed state.
  */
void hydra_state_mgr_register_vote_callback(StateVote_t (*callback)(SystemState)) {
    vote_callback = callback;
}

/**
  * @brief  Requests a transition to a new system state (initiates voting).
  * @param  new_state: The desired state to transition to.
  */
void hydra_state_mgr_request_state(SystemState new_state) {
    // Create state transition request
    pending_request.requester_alias = hydra_identity_get_alias();
    pending_request.requested_state = new_state;
    pending_request.request_id++;
    pending_request.timeout_ticks = xTaskGetTickCount() + pdMS_TO_TICKS(5000);

    // Reset vote counters
    vote_approvals = 0;
    vote_vetoes = 0;
    expected_voters = 0; // This would be set based on known boards from heartbeat

    // Broadcast state transition request
    uint8_t data[4];
    data[0] = pending_request.requester_alias;
    data[1] = (uint8_t)pending_request.requested_state;
    data[2] = pending_request.request_id;
    data[3] = 0; // Reserved

    hydra_comm_send(CAN_ID_STATE_REQUEST, data, sizeof(data), CAN_BUS_A);

    // Start vote timeout timer
    xTimerChangePeriod(state_vote_timer, pdMS_TO_TICKS(5000), 0);
}

/**
  * @brief  Processes incoming state-related CAN messages.
  */
static void process_state_message(uint32_t can_id, uint8_t* data, uint8_t len, CanBus bus) {
    switch (can_id) {
        case CAN_ID_STATE_REQUEST: {
            if (len < 3) return;

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
  * @brief  Sends a state vote response.
  */
static void send_state_vote(StateVote_t vote, uint8_t request_id) {
    uint8_t data[3];
    data[0] = request_id;
    data[1] = hydra_identity_get_alias();
    data[2] = (uint8_t)vote;

    hydra_comm_send(CAN_ID_STATE_VOTE, data, sizeof(data), CAN_BUS_A);
}

/**
  * @brief  Commits a state change and broadcasts it to the stack.
  */
static void commit_state_change(SystemState new_state) {
    current_state = new_state;

    // Broadcast state commit message
    uint8_t data[1] = { (uint8_t)new_state };
    hydra_comm_send(CAN_ID_STATE_COMMIT, data, sizeof(data), CAN_BUS_A);

    // TODO: Notify other modules of state change (via event group)
}

/**
  * @brief  Callback for state vote timeout.
  */
static void state_vote_timer_callback(TimerHandle_t xTimer) {
    // Timeout reached, check if we can proceed without vetoes
    if (vote_vetoes == 0) {
        // No vetoes, commit the change despite missing some approvals
        commit_state_change(pending_request.requested_state);
    }
    // Else: veto was received, do nothing (request already cancelled)
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
