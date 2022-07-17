#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "stdio.h"
#include "main.h"

#define ADC_ISR_EVENT 0x0001

extern osThreadId SampleTaskHandle;

static uint16_t analog_buffer[2];

static uint32_t timer;

void sample_task(ADC_HandleTypeDef *hadc)
{
  while (12) {
    timer = __HAL_TIM_GET_COUNTER(&htim2);
    HAL_ADC_Start_DMA(hadc, (uint32_t *)analog_buffer, 2);

    osSignalWait(ADC_ISR_EVENT, osWaitForever);
    printf("ADC conversion time: %ld uSec\n", __HAL_TIM_GET_COUNTER(&htim2) - timer);

    osDelay(1000);
    printf("adc0: %d, adc1: %d\n", analog_buffer[0], analog_buffer[1]);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  osSignalSet(SampleTaskHandle, ADC_ISR_EVENT);
}
