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
void hydra_identity_init(HydraBoardType _host_board_type);

// Private var share functions
uint8_t hydra_identity_get_alias(void);
const uint8_t hydra_identity_get_peer_count(void);
const uint8_t hydra_identity_get_peer_board_type(uint8_t _peer_alias);

// CAN processing command
void hydra_identity_process_rx(uint8_t _can_id, uint8_t* _data, uint8_t _data_len, CanBus _canbus);
