#include "cmsis_os.h"
#include <stdlib.h>

#if defined(STM32F303xE)
#include "stm32f3xx_hal.h"
#elif defined(STM32F411xE)
#include "stm32f4xx_hal.h"
#endif

#include "write.h"
#include "main.h"

#define WRITE_ISR_EVENT 0x0001

static size_t count_newlines(uint8_t *str, size_t len);

static UART_HandleTypeDef *write_huart;
extern osMessageQId WriteQueueHandle;
extern osThreadId WriteTaskHandle;

#ifdef DMA_MUTEX
extern osMutexId DMA1_MutexHandle;
#endif

void write_task(void *huart)
{
  osEvent write_message;
  WriteBuffer *write_buffer;

  write_huart = huart;

  while (12) {
    write_message = osMessageGet(WriteQueueHandle, osWaitForever);
    write_buffer = write_message.value.p;

#ifdef DMA_MUTEX
    osMutexWait(DMA1_MutexHandle, osWaitForever);
#endif

    HAL_UART_Transmit_IT(write_huart, write_buffer->buffer, write_buffer->length);

    osSignalWait(WRITE_ISR_EVENT, osWaitForever);

#ifdef DMA_MUTEX
    osMutexRelease(DMA1_MutexHandle);
#endif

    free(write_buffer->buffer);
    free(write_buffer);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  osSignalSet(WriteTaskHandle, WRITE_ISR_EVENT);
}

int _write(int fd, uint8_t *ptr, size_t len)
{
  WriteBuffer *write_buffer;
  size_t newlines;

  newlines = count_newlines(ptr, len);

  write_buffer = malloc(sizeof(WriteBuffer));
  write_buffer->length = len + newlines;
  write_buffer->buffer = malloc(write_buffer->length);

  size_t i = 0, j = 0;
  for (; i < len; i++, j++) {
    if (ptr[i] == '\n') {
      write_buffer->buffer[j++] = '\r';
    }
    write_buffer->buffer[j] = ptr[i];
  }

  osMessagePut(WriteQueueHandle, (uint32_t)write_buffer, osWaitForever);

  return len;
}

static size_t count_newlines(uint8_t *str, size_t len)
{
  size_t count = 0;

  for (size_t i = 0; i < len; i++) {
    if (str[i] == '\n') {
      count++;
    }
  }

  return count;
}
