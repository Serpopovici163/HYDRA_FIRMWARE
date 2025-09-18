/**
  ******************************************************************************
  * @file  	hydra_identity.c
  * @brief  Implementation for board identity management.
  *         Supports multiple G4 and H7 MCU families
  * @logic	Upon board boot, hydra_identity_init() is called. This function generates a deterministic
  * 		alias for the host, and broadcasts a CAN_ID_ALIAS_ANNOUNCE packet to notify all others of its
  * 		chosen alias.
  *
  * 		Upon receiving a CAN_ID_ALIAS message, the host will check if this alias conflicts with its own,
  * 		and if not, store it locally in an array of HydraUUID structs. This struct serves to save the
  * 		96bit serial number of all boards, alongside board type and the associated alias.
  *
  * 		If multiple boards choose the same alias, whichever has the LOWEST 96bit serial number will win.
  * 		Losing board will increment its alias by one and CAN_ID_ALIAS_ANNOUNCE procedure again.
  *
  * @CAN	uint8_t --> alias
  * 		uint8_t --> board_type
  * 		uint8_t --> uuid[12] <-- 12 item arr since 3 item 32bit arr
  ******************************************************************************
  */


//implement alias removal/replacing if another board asserts its alias over a pre-existing one
#include "hydra_identity.h"
#include "hydra_comm.h"
#include "hydra_can_icd.h"

#if defined(STM32G4xx)
    #include "stm32g4xx_hal.h" // Will be included by the G4 board's project
#elif defined(STM32H7xx)
    #include "stm32h7xx_hal.h" // Will be included by the H7 board's project
#endif

/*-----------------------------------------------------------
 * Private enums
 *-----------------------------------------------------------*/

typedef enum {
	UUID_A_LOWER = 0x00,
	UUID_EQUAL = 0x01,
	UUID_B_LOWER = 0x02
} UUIDComparison;

/*-----------------------------------------------------------
 * Private structs
 *-----------------------------------------------------------*/

/* UUID Structure (13 bytes) */
typedef struct {
	uint8_t alias;
    uint8_t board_type;  // 1 byte (e.g., HYDRA_POWER)
    uint32_t uuid[3];     // 12 bytes (STM32 UID)
} HydraUUID;

/*-----------------------------------------------------------
 * Private variables
 *-----------------------------------------------------------*/

static HydraUUID uuid_list[MAX_BOARDS_IN_STACK]; //index 0 in this array will always be the host board. Anything beyond that is a discovered board
static uint8_t num_boards_discovered = 0;

/*-----------------------------------------------------------
 * Private prototypes
 *-----------------------------------------------------------*/

static uint8_t generate_deterministic_alias(HydraUUID *uuid);

/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/

/**
  * @brief  Initializes the identity module. Reads UID, generates alias,
  *         and announces it on the CAN bus.
  */
void hydra_identity_init(HydraBoardType host_board_type) {
    // 1. Read the STM32's unique ID
	// Based on https://community.st.com/t5/stm32-mcus/how-to-obtain-and-use-the-stm32-96-bit-uid/ta-p/621443
    uuid_list[0].uuid[0] = *(uint32_t *)UID_BASE;
    uuid_list[0].uuid[1] = *(uint32_t *)(UID_BASE + 4);
    uuid_list[0].uuid[2] = *(uint32_t *)(UID_BASE + 8);

    // 2. Set the board type (THIS MUST BE DEFINED PER BOARD)
    // This is the one board-specific thing. It must be defined in the board's code.
    uuid_list[0].board_type = host_board_type;

    // 3. Generate the deterministic alias from the UUID
    uuid_list[0].alias = 0x00; //set alias to something since generate_deterministic_alias() will use the value
    uuid_list[0].alias = generate_deterministic_alias(&uuid_list[0]);

    // 4. Announce ourselves on the CAN bus for auto-enumeration
    hydra_comm_send(CAN_ID_ALIAS_ANNOUNCE, uuid_list[0], sizeof(uuid_list[0]), CAN_BUS_A);
}

uint8_t hydra_identity_get_alias(void) {
    return uuid_list[0].alias;
}

uint8_t hydra_identity_get_peer_count(void) {
	return num_boards_discovered;
}

const uint8_t hydra_identity_get_peer_board_type(uint8_t peer_alias) {
	for (uint8_t i = 0; i < MAX_BOARDS_IN_STACK; i++) {
		if (uuid_list[i].alias == peer_alias) return uuid_list[i].board_type;
	}
	return 0x00;
}

/**
  * @brief  Processes an incoming UUID announcement from another board.
  *         Handles conflict detection and resolution for deterministic aliases.
  * @param  received_uuid: Pointer to the HydraUUID struct received from the CAN bus.
  */

//TODO: integrate automatic alias rejection for all boards. No specific board should have to reject the alias of another, they should all know which one is correct
void hydra_identity_process_rx(uint8_t can_id, uint8_t* data, uint8_t data_len) {
	if (can_id != CAN_ID_ALIAS_ANNOUNCE || data_len < 14) return; //we only care about a single CAN ID here

	//populate a HydraUUID object based on the data we received
	HydraUUID received_uuid;

	received_uuid.alias = data[0];
	received_uuid.board_type = data[1];
	received_uuid.uuid[0] = (data[2] << 24) + (data[3] << 16) + (data[4] << 8) + data[5];
	received_uuid.uuid[1] = (data[6] << 24) + (data[7] << 16) + (data[8] << 8) + data[9];
	received_uuid.uuid[2] = (data[10] << 24) + (data[11] << 16) + (data[12] << 8) + data[13];

    // -- 1. Check if this UUID is already in our list --

	uint8_t peer_index = _get_peer_index(received_uuid.alias);
	if (peer_index == MAX_BOARDS_IN_STACK) {
		//Board does not exist in our memory, add it
		_hydra_identity_register_new_board(received_uuid);
	} else {
		//Board has already been discovered --> Alias conflict
		//compare UUID of board that we already know of to the board whose request we received
		//first check if they are equal (could be a duplicate announce request)
		uint8_t uuid_comparison = _compare_uuids(received_uuid.uuid, uuid_list[peer_index]);

		if (uuid_comparison == UUID_EQUAL && received_uuid.board_type == uuid_list[peer_index].board_type) {
			//this board already exists, duplicate announce request
			return;
		} else if (uuid_comparison == UUID_A_LOWER) {
			//board we received has the lower alias, and therefore has priority. replace old one in our memory

			//first let's check if larger alias was our own, and update it
			if (peer_index == 0) {
				//increment our own alias and retry alias announce
				uuid[peer_index].alias++;
				_hydra_identity_announce_alias();

				//we don't want to replace element 0 of uuid_list[] so add a new board and place the data there
			}
			uuid_list[peer_index].alias = received_uuid.alias;
			uuid_list[peer_index].board_type = received_uuid.board_type;
			uuid_list[peer_index].uuid[0] = received_uuid.uuid[0];
			uuid_list[peer_index].uuid[1] = received_uuid.uuid[1];
			uuid_list[peer_index].uuid[2] = received_uuid.uuid[2];
		}
	}

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

/*-----------------------------------------------------------
 * Internal logic
 *-----------------------------------------------------------*/

/**
  * @brief  Generates a deterministic 1-byte alias from a 13-byte UUID.
  *         Uses a simple XOR hash.
  * @param  uuid: Pointer to the HydraUUID structure.
  * @retval uint8_t The generated alias (never 0).
  */
//WARNING: 96bit uuid length is HARDCODED
static uint8_t _hydra_identity_generate_deterministic_alias(HydraUUID _hydra_uuid) {
    uint8_t alias = _hydra_uuid.board_type; // Start with the board type

    //convert 3 element uint32_t array in hydra_uuid to a 12 element uint8_t array for hashing
    uint8_t uint8_uuid[12];

    for (uint8_t i = 0; i < sizeof(_hydra_uuid.uuid); i++) {
    	uint8_uuid[i*4] = (_hydra_uuid.uuid >> (96-8)-8*4*i) & 0xFF;
    	uint8_uuid[i*4+1] = (_hydra_uuid.uuid >> (96-16)-8*4*i) & 0xFF;
    	uint8_uuid[i*4+2] = (_hydra_uuid.uuid >> (96-24)-8*4*i) & 0xFF;
    	uint8_uuid[i*4+3] = (_hydra_uuid.uuid >> (96-32)-8*4*i) & 0xFF;
    }

    // XOR all bytes of the UUID (both board_type and the 12-byte UID)
    for (uint8_t i = 0; i < sizeof(uint8_uuid); i++) {
        alias ^= uint8_uuid[i];
    }

    // Ensure alias is never 0 (0 is reserved for "unassigned")
    return (alias == 0) ? 0x01 : alias;
}

void _hydra_identity_announce_alias() {

}

void _hydra_identity_register_new_board(HydraUUID _received_uuid) {
	uuid_list[num_boards_discovered].alias = _received_uuid.alias;
	uuid_list[num_boards_discovered].board_type = _received_uuid.board_type;
	uuid_list[num_boards_discovered].uuid[0] = _received_uuid.uuid[0];
	uuid_list[num_boards_discovered].uuid[1] = _received_uuid.uuid[1];
	uuid_list[num_boards_discovered].uuid[2] = _received_uuid.uuid[2];

	num_boards_discovered++;
}

uint8_t _hydra_identity_get_peer_index(uint8_t _alias) {
	for (uint8_t i = 0; i < MAX_BOARDS_IN_STACK; i++) {
		if (uuid_list[i].alias == peer_alias) return i;
	}

	return MAX_BOARDS_IN_STACK; //this is not a possible index
}

UUIDComparison _hydra_identity_compare_uuids(uint32_t* uuidA, uint32_t* uuidB) {
	if (siezof(uuidA) != 3 || sizeof(uuidB != 3)) return;

	//check most significant to least significant words
	for (uint8_t i = 0; i < sizeof(uuidA); i++) {
		if (uuidA[i] < uuidB[i]) return UUID_A_LOWER;
		else if (uuidA[i] > uuidB[i]) return UUID_B_LOWER;
	}

	return UUID_EQUAL;
}
