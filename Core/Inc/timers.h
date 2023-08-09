#ifndef INC_TIMERS_H_
#define INC_TIMERS_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"

extern TIM_HandleTypeDef fM;
extern TIM_HandleTypeDef SH;
extern TIM_HandleTypeDef ICG;
extern TIM_HandleTypeDef DATA;
extern TIM_HandleTypeDef DELAY;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef husart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

void MX_fM_Init(void);
void MX_SH_Init(void);
void MX_ICG_Init(void);
void MX_DATA_Init(void);
void MX_DELAY_Init(void);

void ADC_Init(void);
void DMA_ADC_Init(void);
void DMA_USART1_TX_Init(void);
void USART1_Init(void);

void stop_timers(void);
void start_timers(void);

void start_data_timer(void);
void stop_data_timer(void);

void start_delay_timer(void);
void stop_delay_timer(void);


#endif /* INC_TIMERS_H_ */
