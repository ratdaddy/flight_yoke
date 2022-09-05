#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "stdio.h"
#include "main.h"

#define ADC_ISR_EVENT 0x0001

extern osThreadId SampleTaskHandle;

static uint16_t analog_buffer[2];

static uint32_t timer;

#define BUFFER_SIZE 32

int8_t x_value, y_value;
static uint32_t x_accum, y_accum;
static uint16_t x_value_buffer[BUFFER_SIZE], y_value_buffer[BUFFER_SIZE];
static size_t x_value_buffer_idx, y_value_buffer_idx;

void sample_task(ADC_HandleTypeDef *hadc)
{
  x_accum = 0;
  y_accum = 0;

  x_value_buffer_idx = 0;
  y_value_buffer_idx = 0;

  for (size_t i = 0; i < BUFFER_SIZE; i++) {
    x_value_buffer[i] = 0;
    y_value_buffer[i] = 0;
  }

  while (12) {
    timer = __HAL_TIM_GET_COUNTER(&htim2);
    HAL_ADC_Start_DMA(hadc, (uint32_t *)analog_buffer, 2);

    osSignalWait(ADC_ISR_EVENT, osWaitForever);

    uint16_t value = analog_buffer[0];

    x_accum += value;
    x_accum -= x_value_buffer[x_value_buffer_idx];
    x_value_buffer[x_value_buffer_idx++] = value;
    x_value_buffer_idx %= BUFFER_SIZE;

    x_value = ((x_accum / BUFFER_SIZE) >> 4) - 128;

    value = analog_buffer[1];

    y_accum += value;
    y_accum -= y_value_buffer[y_value_buffer_idx];
    y_value_buffer[y_value_buffer_idx++] = value;
    y_value_buffer_idx %= BUFFER_SIZE;

    y_value = ((y_accum / BUFFER_SIZE) >> 4) - 128;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  osSignalSet(SampleTaskHandle, ADC_ISR_EVENT);
}
