/**
  ******************************************************************************
  * @file    hydra_identity.h
  * @brief   Module for handling board UUID and deterministic alias.
  ******************************************************************************
  */
#pragma once

#include "hydra_core.h" // For HydraBoardType

/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/

//Init functions
void hydra_identity_init(void);

// Private var share functions
uint8_t hydra_identity_get_alias(void);
const uint8_t hydra_identity_get_peer_count();
const uint8_t hydra_identity_get_peer_board_type(uint8_t alias);

// CAN processing command
void hydra_identity_process_rx(uint8_t can_id, uint8_t* data, uint8_t data_len);
