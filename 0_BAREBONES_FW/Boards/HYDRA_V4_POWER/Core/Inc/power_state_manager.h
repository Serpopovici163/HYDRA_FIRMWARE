#include <stdint.h>
#include "stm32g4xx_hal.h"

/*-----------------------------------------------------------
 * Public enums
 *-----------------------------------------------------------*/

typedef enum {
	BIT_INDEX_SFTY_SW_FORCE_ON 		= 0x00,
	BIT_INDEX_CELL_SENSE_FLT		= 0x01,
	BIT_INDEX_M1_STATE				= 0x02,
	BIT_INDEX_M2_STATE				= 0x03,
	BIT_INDEX_M3_STATE				= 0x04,
	BIT_INDEX_M1_FLT				= 0x05,
	BIT_INDEX_M2_FLT				= 0x06, //DOES NOT EXIST
	BIT_INDEX_M3_FLT				= 0x07, //DOES NOT EXIST
	BIT_INDEX_VCC_LS_PG				= 0x08,
	BIT_INDEX_HYDRA_5V_EN			= 0x09,
	BIT_INDEX_HYDRA_5V_LS_PG		= 0x0A,
	BIT_INDEX_HYDRA_3V3_EN			= 0x0B,
	BIT_INDEX_HYDRA_3V3_LS_PG		= 0x0C,
	BIT_INDEX_GEN_PG				= 0x0D,
	BIT_INDEX_EMAG_STATE			= 0x0E,
	BIT_INDEX_RESERVED				= 0x0F  //UNUSED AT THE MOMENT
} PowerBoardStateBitIndex_t;

typedef enum {
	ADC_INDEX_MCU_TEMP				= 0x00,
	ADC_INDEX_M1_VOLTAGE 			= 0x01,
	ADC_INDEX_M1_CURRENT			= 0x02,
	ADC_INDEX_M2_VOLTAGE			= 0x03,
	ADC_INDEX_M2_CURRENT			= 0x04,
	ADC_INDEX_M3_VOLTAGE			= 0x05, //DOES NOT EXIST
	ADC_INDEX_M3_CURRENT			= 0x06,
	ADC_INDEX_HYDRA_VCC_VOLTAGE		= 0x07, //DOES NOT EXIST
	ADC_INDEX_HYDRA_VCC_CURRENT		= 0x08,
	ADC_INDEX_HYDRA_5V_VOLTAGE		= 0x09,
	ADC_INDEX_HYDRA_5V_CURRENT		= 0x0A,
	ADC_INDEX_HYDRA_3V3_VOLTAGE		= 0x0B,
	ADC_INDEX_HYDRA_3V3_CURRENT		= 0x0C,
	ADC_INDEX_INT_VCC_VOLTAGE		= 0x0D,
	ADC_INDEX_INT_5V_VOLTAGE		= 0x0E,
	ADC_INDEX_INT_3V3_VOLTAGE		= 0x0F
} PowerBoardStateADCIndex_t;

#define NUM_ADC_CHANNELS 0x0F+1 //max val from PowerBoardStateADCIndex_t enum + 1

/*-----------------------------------------------------------
 * Public API
 *-----------------------------------------------------------*/

void power_state_manager_init(void);

void power_state_manager_store_raw_adc_values(volatile uint16_t *adc_buf, ADC_HandleTypeDef *hadc);

uint16_t getBoardStateBits(uint16_t *value);
uint16_t getRawADCValue(uint16_t *value, PowerBoardStateADCIndex_t requestedIndex);
uint16_t getScaledADCValue(uint16_t *value, PowerBoardStateADCIndex_t requestedIndex);
