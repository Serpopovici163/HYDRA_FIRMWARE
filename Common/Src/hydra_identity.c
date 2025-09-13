/**
  ******************************************************************************
  * @file    hydra_identity.c
  * @brief   Implementation for board identity management.
  *          Supports multiple STM32 families (G4, H7, etc.).
  ******************************************************************************
  */
#include "hydra_identity.h"
#include "hydra_comm.h"

#if defined(STM32G4xx)
    #include "stm32g4xx_hal.h" // Will be included by the G4 board's project
#elif defined(STM32H7xx)
    #include "stm32h7xx_hal.h" // Will be included by the H7 board's project
#endif

/* Private variables */
static HydraUUID my_uuid;
static uint8_t my_alias = 0; // 0 signifies not yet assigned

/* Private function prototypes */
static void read_stm32_uid(uint8_t *uid_buffer);
static uint8_t generate_deterministic_alias(HydraUUID *uuid);

/**
  * @brief  Gets the unique UUID for this board.
  * @retval HydraUUID structure containing board type and STM32 UID.
  */
HydraUUID hydra_identity_get_uuid(void) {
    return my_uuid;
}

/**
  * @brief  Gets the assigned 1-byte network alias for this board.
  * @retval uint8_t The alias. Returns 0 if hydra_identity_init hasn't completed.
  */
uint8_t hydra_identity_get_alias(void) {
    return my_alias;
}

/**
  * @brief  Initializes the identity module. Reads UID, generates alias,
  *         and announces it on the CAN bus.
  */
void hydra_identity_init(void) {
    // 1. Read the STM32's unique ID
    read_stm32_uid(my_uuid.uid);

    // 2. Set the board type (THIS MUST BE DEFINED PER BOARD)
    // This is the one board-specific thing. It must be defined in the board's code.
    my_uuid.board_type = HYDRA_BOARD_TYPE;

    // 3. Generate the deterministic alias from the UUID
    my_alias = generate_deterministic_alias(&my_uuid);

    // 4. (Optional) Announce ourselves on the CAN bus for auto-enumeration
    uint8_t announcement[13];
    announcement[0] = my_alias;
    memcpy(&announcement[1], &my_uuid, sizeof(HydraUUID));
    hydra_comm_send(CAN_ID_ALIAS_ANNOUNCE, announcement, sizeof(announcement), CAN_BUS_A);
}

/**
  * @brief  Reads the 96-bit unique device identifier from the STM32.
  *         This function is family-agnostic.
  * @param  uid_buffer: Pointer to a 12-byte array to store the UID.
  */
static void read_stm32_uid(uint8_t *uid_buffer) {
    uint32_t uid_addr;

    // Determine the UID address based on the MCU's predefined macros
#if defined(STM32G4xx)
    uid_addr = UID_BASE; // STM32G4 HAL defines UID_BASE in stm32g4xx_hal.h
#elif defined(STM32H7xx)
    uid_addr = UID_BASE; // STM32H7 HAL defines UID_BASE in stm32h7xx_hal.h
    // For some H7 lines, it might be:
    // uid_addr = 0x1FF1E800UL; // For STM32H7x3
#endif

    // Copy the UID from the predefined address
    uint32_t *uid_ptr = (uint32_t*)uid_addr;
    memcpy(uid_buffer, uid_ptr, 12); // 96 bits = 12 bytes
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
