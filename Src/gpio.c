/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "motor.h"
#include "cmsis_os.h"

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7|USARTx_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_3|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY3_Pin KEY4_Pin KEY5_Pin KEY1_Pin
                           KEY2_Pin */
  GPIO_InitStruct.Pin = KEY3_Pin|KEY4_Pin|KEY5_Pin|KEY1_Pin
                          |KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Beep_Pin */
  GPIO_InitStruct.Pin = Beep_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Beep_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF1 PF2 PF11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : LimitSwitch5_Pin */
  GPIO_InitStruct.Pin = LimitSwitch5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LimitSwitch5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LimitSwitch4_Pin */
  GPIO_InitStruct.Pin = LimitSwitch4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LimitSwitch4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PH7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : USARTx_CTRL_Pin */
  GPIO_InitStruct.Pin = USARTx_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USARTx_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LimitSwitch1_Pin LimitSwitch2_Pin LimitSwitch3_Pin */
  GPIO_InitStruct.Pin = LimitSwitch1_Pin|LimitSwitch2_Pin|LimitSwitch3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : LimitSwitch6_Pin LimitSwitch7_Pin LimitSwitch8_Pin */
  GPIO_InitStruct.Pin = LimitSwitch6_Pin|LimitSwitch7_Pin|LimitSwitch8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD3 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_3|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

static int keyPressedRaw(GPIO_TypeDef* port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 0 : 1;
}

int KEY1_pressed(void)
{
  return keyPressedRaw(GPIOE, KEY1_Pin);
}
int KEY2_pressed(void)
{
  return keyPressedRaw(GPIOE, KEY2_Pin);
}
int KEY3_pressed(void)
{
  return keyPressedRaw(GPIOE, KEY3_Pin);
}
int KEY4_pressed(void)
{
  return keyPressedRaw(GPIOE, KEY4_Pin);
}
int KEY5_pressed(void)
{
  return keyPressedRaw(GPIOE, KEY5_Pin);
}

/*
 * NC wiring convention:
 *   pin LOW  -> switch chain is closed (normal)  -> limitSwitch = 1
 *   pin HIGH -> switch opened / triggered         -> limitSwitch = 0
 */
static int readLimitSwitchNc(GPIO_TypeDef* gpioPort, uint16_t gpioPin)
{
    if (HAL_GPIO_ReadPin(gpioPort, gpioPin) == GPIO_PIN_RESET)
    {
        osDelay(5);
        if (HAL_GPIO_ReadPin(gpioPort, gpioPin) == GPIO_PIN_RESET)
        {
            return 1;
        }
    }
    return 0;
}
void CheckInsideLimitSwitches(void)
{ 
    limitSwitches[0] = readLimitSwitchNc(GPIOH, LimitSwitch1_Pin);
    limitSwitches[1] = readLimitSwitchNc(GPIOH, LimitSwitch2_Pin);
    limitSwitches[2] = readLimitSwitchNc(GPIOH, LimitSwitch3_Pin);
    limitSwitches[3] = readLimitSwitchNc(GPIOA, LimitSwitch4_Pin);
} 
/* USER CODE END 2 */
