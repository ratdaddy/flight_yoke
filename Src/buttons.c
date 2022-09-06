#include "main.h"

#define NUM_BUTTONS 16

struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} buttons[NUM_BUTTONS] = {
  { .port = RIGHT1_GPIO_Port, .pin = RIGHT1_Pin },
  { .port = RIGHT2_GPIO_Port, .pin = RIGHT2_Pin },
  { .port = RIGHT3_GPIO_Port, .pin = RIGHT3_Pin },
  { .port = RIGHT4_GPIO_Port, .pin = RIGHT4_Pin },
  { .port = RIGHT5_GPIO_Port, .pin = RIGHT5_Pin },
  { .port = RIGHT6_GPIO_Port, .pin = RIGHT6_Pin },
  { .port = HAT1_GPIO_Port, .pin = HAT1_Pin },
  { .port = HAT2_GPIO_Port, .pin = HAT2_Pin },
  { .port = HAT3_GPIO_Port, .pin = HAT3_Pin },
  { .port = HAT4_GPIO_Port, .pin = HAT4_Pin },
  { .port = HAT5_GPIO_Port, .pin = HAT5_Pin },
  { .port = LEFT1_GPIO_Port, .pin = LEFT1_Pin },
  { .port = LEFT2_GPIO_Port, .pin = LEFT2_Pin },
  { .port = LEFT3_GPIO_Port, .pin = LEFT3_Pin },
  { .port = LEFT4_GPIO_Port, .pin = LEFT4_Pin },
  { .port = LEFT5_GPIO_Port, .pin = LEFT5_Pin },
};

uint16_t read_buttons()
{
  uint16_t result = 0;

  for (size_t i = 0; i < NUM_BUTTONS; i++) {
    if (!HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin)) {
      result |= 1 << i;
    }
  }
  //printf("reading first button: %d\n", HAL_GPIO_ReadPin(RIGHT1_GPIO_Port, RIGHT1_Pin));
  return result;
}
