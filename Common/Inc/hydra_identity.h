/**
  ******************************************************************************
  * @file    hydra_identity.h
  * @brief   Module for handling board UUID and deterministic alias.
  ******************************************************************************
  */
#pragma once

#include "hydra_core.h" // For HydraBoardType

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
 * Public API
 *-----------------------------------------------------------*/

void hydra_identity_init(void);
uint8_t hydra_identity_get_alias(void);

const uint8_t hydra_identity_get_peer_count();
const HydraUUID hydra_identity_get_peer_info(uint8_t peerIndex);

// Auto-enumeration message processing function (called from hydra_comm's callback)
void hydra_identity_process_announcement(const HydraUUID *received_uuid);
