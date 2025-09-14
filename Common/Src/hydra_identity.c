/**
  ******************************************************************************
  * @file    hydra_identity.c
  * @brief   Implementation for board identity management.
  *          Supports multiple STM32 families (G4, H7, etc.).
  ******************************************************************************
  */
#include "hydra_identity.h"
#include "hydra_comm.h"
#include "hydra_can_icd.h"

#if defined(STM32G4xx)
    #include "stm32g4xx_hal.h" // Will be included by the G4 board's project
#elif defined(STM32H7xx)
    #include "stm32h7xx_hal.h" // Will be included by the H7 board's project
#endif

/* Private variables */
static HydraUUID uuid_list[MAX_BOARDS_IN_STACK]; //index 0 in this array will always be the host board. Anything beyond that is a discovered board
static uint8_t num_boards_discovered = 0;

/* Private function prototypes */
static uint8_t generate_deterministic_alias(HydraUUID *uuid);

/**
  * @brief  Gets the assigned 1-byte network alias for this board.
  * @retval uint8_t The alias. Returns 0 if hydra_identity_init hasn't completed.
  */
uint8_t hydra_identity_get_alias(void) {
    return uuid_list[0].alias;
}

uint8_t hydra_identity_get_peer_count(void) {
	return num_boards_discovered;
}

HydraUUID hydra_identity_get_peer_info(uint8_t peerIndex) {
	if (peerIndex > num_boards_discovered || peerIndex = 0) return 0;

	return uuid_list[peerIndex];
}

/**
  * @brief  Initializes the identity module. Reads UID, generates alias,
  *         and announces it on the CAN bus.
  */
void hydra_identity_init(void) {
    // 1. Read the STM32's unique ID
	// Based on https://community.st.com/t5/stm32-mcus/how-to-obtain-and-use-the-stm32-96-bit-uid/ta-p/621443
    uuid_list[0].uuid[0] = *(uint32_t *)UID_BASE;
    uuid_list[0].uuid[1] = *(uint32_t *)(UID_BASE + 4);
    uuid_list[0].uuid[2] = *(uint32_t *)(UID_BASE + 8);

    // 2. Set the board type (THIS MUST BE DEFINED PER BOARD)
    // This is the one board-specific thing. It must be defined in the board's code.
    uuid_list[0].board_type = HYDRA_BOARD_TYPE;

    // 3. Generate the deterministic alias from the UUID
    uuid_list[0].alias = 0x00; //set alias to something since generate_deterministic_alias() will use the value
    uuid_list[0].alias = generate_deterministic_alias(&uuid_list[0]);

    // 4. Announce ourselves on the CAN bus for auto-enumeration
    hydra_comm_send(CAN_ID_ALIAS_ANNOUNCE, uuid_list[0], sizeof(uuid_list[0]), CAN_BUS_A);
}

/**
  * @brief  Generates a deterministic 1-byte alias from a 13-byte UUID.
  *         Uses a simple XOR hash.
  * @param  uuid: Pointer to the HydraUUID structure.
  * @retval uint8_t The generated alias (never 0).
  */
static uint8_t generate_deterministic_alias(HydraUUID *uuid) {
    uint8_t alias = uuid->board_type; // Start with the board type

    // XOR all bytes of the UUID (both board_type and the 12-byte UID)
    for (int i = 0; i < sizeof(HydraUUID); i++) {
        alias ^= ((uint8_t*)uuid)[i];
    }

    // Ensure alias is never 0 (0 is reserved for "unassigned")
    return (alias == 0) ? 0x01 : alias;
}

/**
  * @brief  Processes an incoming UUID announcement from another board.
  *         Handles conflict detection and resolution for deterministic aliases.
  * @param  received_uuid: Pointer to the HydraUUID struct received from the CAN bus.
  */
void hydra_identity_process_announcement(const HydraUUID *received_uuid) {
    // -- 1. Check if this UUID is already in our list --
    for (int i = 0; i <= num_boards_discovered; i++) {
        if (memcmp(&uuid_list[i].uid, &received_uuid->uid, sizeof(received_uuid->uid)) == 0) {
            // UUID is already known, no need to process again
            return;
        }
    }

    // -- 2. Check if we have room to store this new board --
    if (num_boards_discovered >= (MAX_BOARDS_IN_STACK - 1)) {
        // TODO: Trigger an error - stack is full
        // hydra_health_set_fault(FAULT_STACK_FULL);
        return;
    }

    // -- 3. Check for alias conflict with self --
    if (received_uuid->alias == uuid_list[0].alias) {
        // Critical: Alias conflict detected!
        // Compare UUIDs to determine who should change
        int uuid_comparison = memcmp(&uuid_list[0].uid, &received_uuid->uid, sizeof(received_uuid->uid));

        if (uuid_comparison < 0) {
            // OUR UUID is numerically smaller -> WE must change
            uuid_list[0].alias++; // Increment our alias
            // TODO: Ensure alias doesn't overflow 0xFF, maybe wrap to 1
            if (uuid_list[0].alias == 0) uuid_list[0].alias = 1;

            // Re-broadcast our new identity
            uint8_t announcement[sizeof(HydraUUID)];
            memcpy(announcement, &uuid_list[0], sizeof(HydraUUID));
            hydra_comm_send(CAN_ID_ALIAS_ANNOUNCE, announcement, sizeof(announcement), CAN_BUS_A);

            return; // Exit after handling the conflict
        }
        else if (uuid_comparison > 0) {
            // THEIR UUID is smaller -> THEY must change
            // Send a rejection message targeting the received UUID
            uint8_t reject_data[1] = { received_uuid->alias };
            hydra_comm_send(CAN_ID_ALIAS_REJECT, reject_data, 1, CAN_BUS_A);

            return; // Exit, we expect them to resolve it
        }
        else {
            // UUIDs are identical! This should never happen with proper unique IDs.
            // TODO: Trigger a critical fault - non-unique UUIDs
            return;
        }
    }

    // -- 4. Check for alias conflict with other discovered boards --
    for (int i = 1; i <= num_boards_discovered; i++) {
        if (uuid_list[i].alias == received_uuid->alias) {
            // Conflict with another discovered board, reject this announcement
            uint8_t reject_data[1] = { received_uuid->alias };
            hydra_comm_send(CAN_ID_ALIAS_REJECT, reject_data, 1, CAN_BUS_A);
            return;
        }
    }

    // -- 5. If we get here, the announcement is valid and new --
    num_boards_discovered++;
    uuid_list[num_boards_discovered] = *received_uuid; // Store the new UUID

    // Optional: Log the new board discovery
    // HY_LOG("Discovered board: Type %d, Alias 0x%02X",
    //        received_uuid->board_type, received_uuid->alias);
}
