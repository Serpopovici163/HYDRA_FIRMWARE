/**
  ******************************************************************************
  * @file    hydra_common.h
  * @brief   Master include file for the Hydra Avionics Common Library.
  *          Include this file to get access to the entire Hydra common API.
  ******************************************************************************
  */

#pragma once

/* Include all public headers for the Hydra common modules */
#include "hydra_core.h"         /* Core system types and definitions */
#include "hydra_can_icd.h"      /* CAN Interface Control Document (IDs) */
#include "hydra_comm.h"         /* CAN communication layer */
#include "hydra_identity.h"     /* UUID and alias management */
#include "hydra_heartbeat.h"    /* Synchronized heartbeat and status system */
#include "hydra_watchdog.h"     /* Hardware watchdog management */
#include "hydra_state_mgr.h"    /* Decentralized state machine */
