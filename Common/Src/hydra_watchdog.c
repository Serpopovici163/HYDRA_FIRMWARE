/**
  ******************************************************************************
  * @file    hydra_watchdog.c
  * @brief   Implementation of the independent watchdog and task monitoring.
  ******************************************************************************
  */
#include "hydra_watchdog.h"
#include "hydra_events.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32g4xx_hal.h"
#include "stm32h7xx_hal.h"

/* Watchdog Configuration */
#define IWDG_TIMEOUT_MS 2000  // 2 second hardware watchdog timeout
#define TASK_CHECK_PERIOD_MS 1000 // Check task states every 1 second

/* Private variables */
static IWDG_HandleTypeDef hiwdg;
static uint32_t last_kick_time[WATCHDOG_NUM_PRIORITIES];
static TaskStatus_t *task_stats;
static uint32_t task_count;
static uint32_t last_task_check_time = 0;

/**
  * @brief  Initializes the independent watchdog (IWDG) and monitoring structures.
  */
void hydra_watchdog_init(void) {
    // 1. Initialize the Hardware Watchdog (IWDG)
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256; // Adjust for your clock and desired timeout
    hiwdg.Init.Reload = (IWDG_TIMEOUT_MS * 40) / 256; // LSI ~32kHz, /256 prescaler -> ~125 Hz -> 8ms/count
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;

    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        // Initialization Error - maybe trigger a safe mode?
        Error_Handler();
    }

    // 2. Initialize the software kick timers
    for (int i = 0; i < WATCHDOG_NUM_PRIORITIES; i++) {
        last_kick_time[i] = 0;
    }

    // 3. Get the number of tasks for later monitoring
    task_count = uxTaskGetNumberOfTasks();
    task_stats = pvPortMalloc(task_count * sizeof(TaskStatus_t));

    // 4. Kick the watchdog immediately to start
    hydra_watchdog_kick(WATCHDOG_PRIO_IDLE);
}

/**
  * @brief  "Kicks" the watchdog, indicating a part of the system is healthy.
  *         Also updates the last kick time for the specified priority level.
  * @param  priority: The priority level of the component doing the kicking.
  */
void hydra_watchdog_kick(WatchdogPriority_t priority) {
    // Update the last kick time for this priority level
    last_kick_time[priority] = xTaskGetTickCount();

    // Actually refresh the hardware watchdog
    HAL_IWDG_Refresh(&hiwdg);
}

/**
  * @brief  The main watchdog task that monitors system health.
  */
void hydra_watchdog_task(void *argument) {
    hydra_watchdog_init();
    uint32_t current_ticks;

    for (;;) {
        current_ticks = xTaskGetTickCount();

        // 1. Check for stalled priority levels
        for (int i = 0; i < WATCHDOG_NUM_PRIORITIES; i++) {
            // If a critical priority hasn't kicked in too long, it's a fault
            uint32_t max_allowed_stall = pdMS_TO_TICKS( (i+1) * 1000 ); // Higher priorities get shorter timeouts
            if (current_ticks - last_kick_time[i] > max_allowed_stall) {
                // CRITICAL: A vital task has stalled! Log error and maybe trigger safe state.
                // Since we can't reset (we're the watchdog), set a fatal error flag.
                // Other tasks should monitor this and enter safe mode.
                trigger_system_fault(FAULT_TASK_STALL, i);
                // We continue running to keep kicking the HW watchdog and allow for possible recovery
            }
        }

        // 2. Periodically check task states for more detailed analysis (less frequent)
        if (current_ticks - last_task_check_time > pdMS_TO_TICKS(TASK_CHECK_PERIOD_MS)) {
            last_task_check_time = current_ticks;

            // Get a snapshot of all task states
            uxTaskGetSystemState(task_stats, task_count, NULL);

            // Check for tasks that are in the "Blocked" state for too long
            // This could indicate a deadlock (e.g., waiting on a mutex that will never be released)
            for (int i = 0; i < task_count; i++) {
                if (task_stats[i].eCurrentState == eBlocked) {
                    // You could track how long each task has been blocked and flag it
                }
            }
        }

        // 3. This task itself must kick the watchdog
        hydra_watchdog_kick(WATCHDOG_PRIO_CRITICAL);

        // 4. Sleep until next check
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
}

// ... In other task files, they regularly kick the watchdog ...
