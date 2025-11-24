/**
  ******************************************************************************
  * @file  	hydra_identity.c
  * @brief  Implementation for board identity management.
  *         Supports multiple G4 and H7 MCU families
  *
  * @logic	Upon boot, hydra_identity_init() is called. This function generates a deterministic
  * 		alias for the host, and broadcasts a CAN_ID_ALIAS_ANNOUNCE packet to notify all others of its
  * 		chosen alias.
  *
  *			If multiple boards choose the same alias, whichever has the LOWEST 96bit serial number will win. The
  *			one with the higher serial number will then increment its own alias by 1 and resend a CAN_ID_ALIAS_ANNOUNCE
  *			message. This process repeats until the boards have all settled on an alias.
  *
  * 		Upon receiving a CAN_ID_ALIAS_ANNOUNCE message, the host will check if this alias conflicts with its own,
  * 		and if not, store it locally in an array of HydraUUID structs. This struct array serves to save the
  * 		96bit serial number, board type, and the associated alias of all boards which serves as a local
  * 		network map.
  *
  * 		Upon receiving a CAN_ID_ALIAS_ANNOUNCE message, if the alias does not interfere with the hosts but does
  * 		interfere with the alias of another board's who's announce message we've already received, this module
  * 		will check if the serial number of the previously received one is higher than that of the conflicting board
  * 		we just received, and overwrite the existing record if this the case.
  *
  * 		If the serial number of the board we just received is larger than that of the board we already had in memory,
  * 		the message is dropped.
  *
  * @CAN frames:
  *
  * 		CAN_ID_ALIAS_ANNOUNCE (14 bytes):
  * 			uint8_t --> alias
  * 			uint8_t --> board_type
  * 			uint8_t --> uuid[12] <-- 12 item arr since 3 item 32bit arr
  ******************************************************************************
  */

//implement alias removal/replacing if another board asserts its alias over a pre-existing one
#include "hydra_identity.h"
#include "hydra_comm.h"
#include "hydra_can_icd.h"

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
	HydraBoardType board_type;  // 1 byte (e.g., HYDRA_POWER)
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

static uint8_t _hydra_identity_generate_deterministic_alias(HydraUUID _hydra_uuid);
void _hydra_identity_announce_alias();
void _hydra_identity_register_new_board(HydraUUID _received_uuid);
uint8_t _hydra_identity_get_peer_index(uint8_t _alias);
UUIDComparison _hydra_identity_compare_uuids(uint32_t* uuidA, uint32_t* uuidB);

/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/

/**
  * @brief  Initializes the identity module. Reads UID, generates alias,
  *         and announces it on the CAN bus.
  */
void hydra_identity_init(HydraBoardType _host_board_type) {
    // 1. Read the STM32's unique ID
	// Based on https://community.st.com/t5/stm32-mcus/how-to-obtain-and-use-the-stm32-96-bit-uid/ta-p/621443
    uuid_list[0].uuid[0] = *(uint32_t *)UID_BASE;
    uuid_list[0].uuid[1] = *(uint32_t *)(UID_BASE + 4);
    uuid_list[0].uuid[2] = *(uint32_t *)(UID_BASE + 8);

    // 2. Set the board type (THIS MUST BE DEFINED PER BOARD)
    // This is the one board-specific thing. It must be defined in the board's code.
    uuid_list[0].board_type = _host_board_type;

    // 3. Generate the deterministic alias from the UUID
    uuid_list[0].alias = 0x00; //set alias to something since _hydra_identity_generate_deterministic_alias() will use the value
    uuid_list[0].alias = _hydra_identity_generate_deterministic_alias(&uuid_list[0]);

    // 4. Announce ourselves on the CAN bus for auto-enumeration
    _hydra_identity_announce_alias();
}

uint8_t hydra_identity_get_alias(void) {
    return uuid_list[0].alias;
}

//this returns a count of all boards, even if dead. hydra_health has a function to return live board indexes but identity doesn't keep track of any board's state
uint8_t hydra_identity_get_peer_count(void) {
	return num_boards_discovered;
}

const HydraBoardType hydra_identity_get_peer_board_type(uint8_t _peer_alias) {
	for (uint8_t i = 0; i < MAX_BOARDS_IN_STACK; i++) {
		if (uuid_list[i].alias == _peer_alias) return uuid_list[i].board_type;
	}

	//The requested alias does not exist
	return HYDRA_NONE;
}

/**
  * @brief  Processes an incoming UUID announcement from another board.
  *         Handles conflict detection and resolution for deterministic aliases.
  * @param  received_uuid: Pointer to the HydraUUID struct received from the CAN bus.
  */

//TODO: integrate automatic alias rejection for all boards. No specific board should have to reject the alias of another, they should all know which one is correct
void hydra_identity_process_rx(uint8_t _can_id, uint8_t* _data, uint8_t _data_len, CanBus _canbus) {
	if (_can_id != CAN_ID_ALIAS_ANNOUNCE || _data_len < 14) return; //we only care about a single CAN ID here

	//populate a HydraUUID object based on the data we received
	HydraUUID received_uuid;

	received_uuid.alias = data[0];
	received_uuid.board_type = data[1];
	received_uuid.uuid[0] = (data[2] << 24) + (data[3] << 16) + (data[4] << 8) + data[5];
	received_uuid.uuid[1] = (data[6] << 24) + (data[7] << 16) + (data[8] << 8) + data[9];
	received_uuid.uuid[2] = (data[10] << 24) + (data[11] << 16) + (data[12] << 8) + data[13];

    // -- 1. Check if this UUID is already in our list --

	uint8_t peer_index = _hydra_identity_get_peer_index(received_uuid.alias);
	if (peer_index == MAX_BOARDS_IN_STACK) {
		//UUID does not exist
		//check if stack is full
		if (num_boards_discovered < (MAX_BOARDS_IN_STACK - 1)) {
			//Register the new board
			_hydra_identity_register_new_board(received_uuid);
		}
		//otherwise, if the stack is full, we just drop the request
	} else {
		//Alias conflict
		//compare UUID of board that we already know of to the board whose request we received
		//first check if they are equal (could be a duplicate announce request)
		uint8_t uuid_comparison = _hydra_identity_compare_uuids(received_uuid.uuid, uuid_list[peer_index].uuid);

		if (uuid_comparison == UUID_EQUAL && received_uuid.board_type == uuid_list[peer_index].board_type) {
			//this board already exists, duplicate announce request
			return;
		} else if (uuid_comparison == UUID_A_LOWER) {
			//board we received has the lower uuid, and therefore has priority. replace old one in our memory

			//first let's check if this alias conflicts with our own, and update our own if it does
			if (peer_index == 0) {
				//increment our own alias and retry alias announce
				uuid[peer_index].alias++; //TODO: does uint8_t roll over if I do 255++? IF not, implement this
				_hydra_identity_announce_alias();

				//we don't want to replace element 0 of uuid_list[] so add a new board and place the data there
				_hydra_identity_register_new_board(received_uuid);
			} else {
				//Since there is a conflict but it's not with our own alias, we are not responsible for rejecting the alias //TODO: do we want alias reject messages? shouldn't need them
				uuid_list[peer_index].alias = received_uuid.alias;
				uuid_list[peer_index].board_type = received_uuid.board_type;
				uuid_list[peer_index].uuid[0] = received_uuid.uuid[0];
				uuid_list[peer_index].uuid[1] = received_uuid.uuid[1];
				uuid_list[peer_index].uuid[2] = received_uuid.uuid[2];
			}
		} //no need to check for UUID_B_LOWER, as that board will have to change its alias
	}
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
	uint8_t data[14];

	data[0] = uuid_list[0].alias;
	data[1] = uuid_list[0].board_type;
	for (uint8_t i = 0; i < sizeof(_hydra_uuid.uuid); i++) {
		data[2+i*4] = (uuid_list.uuid >> (96-8)-8*4*i) & 0xFF;
		data[2+i*4+1] = (uuid_list.uuid >> (96-16)-8*4*i) & 0xFF;
		data[2+i*4+2] = (uuid_list.uuid >> (96-24)-8*4*i) & 0xFF;
		data[2+i*4+3] = (uuid_list.uuid >> (96-32)-8*4*i) & 0xFF;
	}

	hydra_comm_send((uint32_t)CAN_ID_ALIAS_ANNOUNCE, data, sizeof(data), CAN_BUS_A);
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

	return MAX_BOARDS_IN_STACK; //this is not a possible index, meaning that board was not found in uuid_list[]
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
