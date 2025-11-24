/**
  ******************************************************************************
  * @file    hydra_watchdog.h
  * @brief   Watchdog module for system health monitoring and fault recovery.
  ******************************************************************************
  */
#pragma once

#include "hydra_core.h"

/* Watchdog kick priorities - higher numbers are more critical */
typedef enum {
    WATCHDOG_PRIO_IDLE = 0,   // Background tasks
    WATCHDOG_PRIO_HEARTBEAT,  // Heartbeat task
    WATCHDOG_PRIO_COMM,       // Communication tasks
    WATCHDOG_PRIO_SENSOR,     // Sensor reading tasks
    WATCHDOG_PRIO_CRITICAL,   // Control and state machine tasks
    WATCHDOG_NUM_PRIORITIES   // Must be last
} WatchdogPriority_t;

/* Public API */
void hydra_watchdog_init(void);
void hydra_watchdog_kick(WatchdogPriority_t priority);
void hydra_watchdog_task(void *argument);
