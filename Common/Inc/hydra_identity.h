/**
  ******************************************************************************
  * @file    hydra_identity.h
  * @brief   Module for handling board UUID and deterministic alias.
  ******************************************************************************
  */
#pragma once

#include "hydra_core.h" // For HydraBoardType

/* UUID Structure (13 bytes) */
typedef struct {
    uint8_t board_type;  // 1 byte (e.g., HYDRA_POWER)
    uint8_t uid[12];     // 12 bytes (STM32 UID)
} HydraUUID;

/* Public API */
HydraUUID hydra_identity_get_uuid(void);
uint8_t hydra_identity_get_alias(void);
void hydra_identity_init(void);
