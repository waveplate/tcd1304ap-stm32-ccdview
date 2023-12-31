#include "consts.h"
#include "timers.h"
#include "main.h"

// exposure time (exposure*3.7ms)
// min exposure for 2mhz fM = 2 = 7.388ms
volatile uint8_t exposure = 20;

// tx_busy when USART1 TX in use
volatile uint8_t tx_busy = 0;

// average multiple exposures before sending
volatile uint8_t avg = 1;

// fM (TIM3-CH3)   PB0 - A3
// SH (TIM2-CH2)   PA1 - A1
// ICG (TIM5-CH1)  PA0 - A0
// OS (ADC1-in)    PC0 - A5

// USART1 (TX)     PA9  - D8
// USART1 (RX)     PA10 - D2

// B1 button       PC13 (starts/stops timers)

// DATA (TIM4-CH4) PB9 - D14 (only for debug)

int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	USART1_Init();

	MX_fM_Init();
	HAL_TIM_Base_Start(&fM);
	HAL_TIM_PWM_Start(&fM, TIM_CHANNEL_3);

	MX_SH_Init();
	MX_ICG_Init();

	MX_DATA_Init();
	HAL_TIM_Base_Start(&DATA);

	ADC_Init();
	DMA_ADC_Init();
	DMA_USART1_TX_Init();

	start_timers();

	while (1)
	{

	}
}

int write_data(UART_HandleTypeDef *husart, volatile uint32_t *buf)
{
	if(tx_busy == 1) return -1;

	uint8_t data[2*NUM_PIXELS];

	// convert each 12-bit value to two bytes for TX
	for (int i = 0; i < NUM_PIXELS; i++)
	{
	  data[2 * i] = (buf[i] >> 8) & 0xFF;
	  data[2 * i + 1] = buf[i] & 0xFF;
	}

	// only send if previous TX not in progress
	if(tx_busy == 1) return -1;

	tx_busy = 1;
	HAL_UART_Transmit_DMA(husart, (uint8_t*)data, 2*NUM_PIXELS);

	return 2*NUM_PIXELS;
}

void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_TIM5_CLK_ENABLE();

	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	// PB0 - fM
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PA1 - SH
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PA0 - ICG
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PB9 - DATA timer (for debug, this can be commented out)
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// PC0 - ADC1-10
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// PC13 - B1 button
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// USART1 on pins PA9 (TX) and PA10 (RX)
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// ICG interrupt
	HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM5_IRQn);

	// DMA buffer full interrupt
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	// USART TX complete interrupt
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

	// DMA USART1 interrupt
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

	// push button interrupt
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void)
{
	// dummy function, do something here if you want
	while(1)
	{
	}
}
