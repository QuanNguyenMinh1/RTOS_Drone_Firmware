/*
 * bldc.h
 *
 *  Created on: Oct 31, 2025
 *      Author: Quan
 */

#ifndef USERCODE_BLDC_BLDC_H_
#define USERCODE_BLDC_BLDC_H_

#include "tim.h"

void ESC_Init(TIM_HandleTypeDef *htim);
void ESC_CalibAll(TIM_HandleTypeDef *htim);
void ESC_Write(TIM_HandleTypeDef *htim, uint8_t motor_id, uint16_t throttle);
void ESC_Write_All(TIM_HandleTypeDef *htim, uint16_t throttle);
void ESC_Arm(TIM_HandleTypeDef *htim, uint8_t motor_id);
void ESC_Arm_All(TIM_HandleTypeDef *htim);

#endif /* USERCODE_BLDC_BLDC_H_ */
