#include "power_state_manager.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdio.h>

/*-----------------------------------------------------------
 * Private structs
 *-----------------------------------------------------------*/

/* Struct to hold local board state */
typedef struct {
	//general state bits
	uint16_t board_state_bits; //See BookStack and .h file for details

	//v and i sense
	uint16_t raw_adc[NUM_ADC_CHANNELS];
	uint16_t scaled_adc[NUM_ADC_CHANNELS];
} PowerBoardState_t;

/*-----------------------------------------------------------
 * Private variables
 *-----------------------------------------------------------*/
PowerBoardState_t powerBoardState;

static osMutexId_t powerBoardStateMutex;
const osMutexAttr_t powerBoardStateMutexAttr = {
		.name = "PowerBoardStateMutex"
};

static osThreadId_t powerStateTaskHandle;
const osThreadAttr_t powerStateTaskAttr = {
		.name = "PowerStateTask",
		.stack_size = 1024,
		.priority = (osPriority_t) osPriorityNormal
};

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc5;

/*-----------------------------------------------------------
 * Private prototypes
 *-----------------------------------------------------------*/
static void PowerStateTask(void *argument);

/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/

void power_state_manager_init(void) {
	//Init board_state
	powerBoardState.board_state_bits = 0b1011000000000000; //See.h file for meaning

	for (uint8_t i = 0; i < NUM_ADC_CHANNELS; i++) {
		powerBoardState.raw_adc[i] = 0;
		powerBoardState.scaled_adc[i] = 0;
	}

	//Init Mutex for power_board_state struct
	powerBoardStateMutex = osMutexNew(&powerBoardStateMutexAttr);

	//Begin state manager task
	powerStateTaskHandle = osThreadNew(PowerStateTask, NULL, &powerStateTaskAttr);
}

void power_state_manager_store_raw_adc_values(volatile uint16_t *adc_buf, ADC_HandleTypeDef *hadc) {

	osMutexAcquire(powerBoardStateMutex, osWaitForever);

	if (hadc->Instance == ADC1) {
		powerBoardState.raw_adc[ADC_INDEX_M3_CURRENT] = adc_buf[0];
		powerBoardState.raw_adc[ADC_INDEX_MCU_TEMP] = adc_buf[1];
	} else if (hadc->Instance == ADC2) {
		powerBoardState.raw_adc[ADC_INDEX_M1_CURRENT] = adc_buf[0];
		powerBoardState.raw_adc[ADC_INDEX_INT_5V_VOLTAGE] = adc_buf[1];
		powerBoardState.raw_adc[ADC_INDEX_HYDRA_3V3_VOLTAGE] = adc_buf[2];
		powerBoardState.raw_adc[ADC_INDEX_INT_3V3_VOLTAGE] = adc_buf[3];
		powerBoardState.raw_adc[ADC_INDEX_M2_CURRENT] = adc_buf[4];
		powerBoardState.raw_adc[ADC_INDEX_M2_VOLTAGE] = adc_buf[5];
		powerBoardState.raw_adc[ADC_INDEX_M1_VOLTAGE] = adc_buf[6];
		powerBoardState.raw_adc[ADC_INDEX_HYDRA_5V_VOLTAGE] = adc_buf[7];
		powerBoardState.raw_adc[ADC_INDEX_INT_VCC_VOLTAGE] = adc_buf[8];
		powerBoardState.raw_adc[ADC_INDEX_HYDRA_VCC_CURRENT] = adc_buf[9];
	} else if (hadc->Instance == ADC5) {
		powerBoardState.raw_adc[ADC_INDEX_HYDRA_5V_CURRENT] = adc_buf[0];
		powerBoardState.raw_adc[ADC_INDEX_HYDRA_3V3_CURRENT] = adc_buf[1];
	}

	osMutexRelease(powerBoardStateMutex);
}

uint16_t getBoardStateBits(uint16_t *value) {
	if (osMutexAcquire(powerBoardStateMutex, osWaitForever) == osOK) {
		*value = powerBoardState.board_state_bits;
		osMutexRelease(powerBoardStateMutex);
	}
}

uint16_t getRawADCValue(uint16_t *value, PowerBoardStateADCIndex_t requestedIndex) {
	if (osMutexAcquire(powerBoardStateMutex, osWaitForever) == osOK) {
		*value = powerBoardState.raw_adc[requestedIndex];
		osMutexRelease(powerBoardStateMutex);
	}
}

uint16_t getScaledADCValue(uint16_t *value, PowerBoardStateADCIndex_t requestedIndex) {
	if (osMutexAcquire(powerBoardStateMutex, osWaitForever) == osOK) {
		*value = powerBoardState.scaled_adc[requestedIndex];
		osMutexRelease(powerBoardStateMutex);
	}
}

/*-----------------------------------------------------------
 * Internal logic
 *-----------------------------------------------------------*/

//Task function
static void PowerStateTask(void *argument) {
	for (;;) {

		//update GPIO bit vars
		uint16_t board_state_bits_temp =
				(HAL_GPIO_ReadPin(OUT_FORCE_ON_GPIO_Port, OUT_FORCE_ON_Pin) << 0) |
				(0x00 << 1) | //todo: add bit 1
				((~HAL_GPIO_ReadPin(OUT_M1_INH_GPIO_Port, OUT_M1_INH_Pin) & 0x01) << 2) | //invert
				((~HAL_GPIO_ReadPin(OUT_M2_INH_GPIO_Port, OUT_M2_INH_Pin) & 0x01) << 3) | //invert
				(HAL_GPIO_ReadPin(OUT_M3_EN_GPIO_Port, OUT_M3_EN_Pin) << 4) |
				(HAL_GPIO_ReadPin(INP_M1_OC_FLT_GPIO_Port, INP_M1_OC_FLT_Pin) << 5) | //invert
				(0x00 << 6) | //does not exist
				(0x00 << 7) | //does not exist
				((~HAL_GPIO_ReadPin(INP_VCC_LS_PG_GPIO_Port, INP_VCC_LS_PG_Pin) & 0x01) << 8) | //invert
				(HAL_GPIO_ReadPin(OUT_5V_REG_EN_GPIO_Port, OUT_5V_REG_EN_Pin) << 9) |
				((~HAL_GPIO_ReadPin(INP_5V_LS_PG_GPIO_Port, INP_5V_LS_PG_Pin) & 0x01) << 10) | //invert
				(HAL_GPIO_ReadPin(OUT_3V3_REG_EN_GPIO_Port, OUT_3V3_REG_EN_Pin) << 11) |
				((~HAL_GPIO_ReadPin(INP_3V3_LS_PG_GPIO_Port, INP_3V3_LS_PG_Pin) & 0x01) << 12) | //invert
				((~HAL_GPIO_ReadPin(INP_GEN_PG_GPIO_Port, INP_GEN_PG_Pin) & 0x01) << 13) | //invert
				(HAL_GPIO_ReadPin(OUT_EMAG_GPIO_Port, OUT_EMAG_Pin) << 14) |
				(0x00 << 15);

		osMutexAcquire(powerBoardStateMutex, osWaitForever);

		powerBoardState.board_state_bits = board_state_bits_temp;

		//TODO: scale as needed, currently no scaling is involved

		osMutexRelease(powerBoardStateMutex);

		printf("powerstatetask\n");

		osDelay(100);
	}
}
