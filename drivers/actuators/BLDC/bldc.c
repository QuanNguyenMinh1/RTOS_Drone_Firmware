/*
 * bldc.c
 *
 *  Created on: Oct 31, 2025
 *      Author: Quan
 */

#include "../UserCode/BLDC/bldc.h"

void ESC_Init(TIM_HandleTypeDef *htim)
{
	  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
}

// Set throttle = 1000–2000 (us) cho esc, tức là calibration
void ESC_CalibAll(TIM_HandleTypeDef *htim)
{
    // Gửi tín hiệu GA MAX (2000us -> CCR = 2000)
    // ESC sẽ phát ra âm báo MAX.
    ESC_Write_All(htim, 2000);

    // Chờ 7 giây để ESC nhận tín hiệu MAX (âm báo đầu tiên)
    HAL_Delay(7000);


    // Gửi tín hiệu GA MIN (1000us -> CCR = 1000)
    // ESC sẽ phát ra âm báo xác nhận "Calibration Complete" (âm dài).
    ESC_Write_All(htim, 1000);

    // Chờ 8 giây để ESC lưu tín hiệu MIN
    HAL_Delay(8000);

    // KẾT THÚC HIỆU CHUẨN
}

void ESC_Write(TIM_HandleTypeDef *htim, uint8_t motor_id, uint16_t throttle)
{
    if (throttle < 1000) throttle = 1000;
    if (throttle > 2000) throttle = 2000;

    switch (motor_id) {
		case 1:
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, throttle);
			break;
		case 2:
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, throttle);
			break;
		case 3:
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, throttle);
			break;
		case 4:
			__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, throttle);
			break;
		default:
			break;
	}
}

void ESC_Write_All(TIM_HandleTypeDef *htim, uint16_t throttle)
{
    if (throttle < 1000) throttle = 1000;
    if (throttle > 2000) throttle = 2000;

	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, throttle);

	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, throttle);

	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, throttle);

	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, throttle);
}

void ESC_Arm(TIM_HandleTypeDef *htim, uint8_t motor_id)
{
    // gửi tín hiệu thấp trong 3 giây
//    for (int i=0; i<300; i++)
//    {
//        switch (motor_id) {
//    		case 1:
//    	        ESC_Write(htim, 1, 1000);
//    			break;
//    		case 2:
//    	        ESC_Write(htim, 2, 1000);
//    			break;
//    		case 3:
//    	        ESC_Write(htim, 3, 1000);
//    			break;
//    		case 4:
//    	        ESC_Write(htim, 4, 1000);
//    			break;
//    		default:
//    			break;
//    	}
//        HAL_Delay(10);
//    }
	    // Gửi tín hiệu throttle thấp (1000 µs) trong 3 giây
	    uint16_t throttle = 1000;
	    uint32_t start = HAL_GetTick();

	    while (HAL_GetTick() - start < 3000)
	    {
	        ESC_Write(htim, 1, throttle);
	        ESC_Write(htim, 2, throttle);
	        ESC_Write(htim, 3, throttle);
	        ESC_Write(htim, 4, throttle);
	        HAL_Delay(20); // gửi đều, không cần quá nhanh
	    }

}

void ESC_Arm_All(TIM_HandleTypeDef *htim)
{
    for (int i=0; i<300; i++)
    {
		ESC_Write(htim, 1, 1000);
		ESC_Write(htim, 2, 1000);
		ESC_Write(htim, 3, 1000);
		ESC_Write(htim, 4, 1000);

        HAL_Delay(10);
    }
}
