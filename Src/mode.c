#include "mode.h"
#include "filter.h"
#include "gpio.h"
#include "interface.h"
#include "stdint.h"
#include "tim.h"
#include "filter.h"
#include "motion.h"
#include "control.h"
#include "spi.h"
#include "flash.h"
#include "maze_info.h"
#include "motion_plan.h"

#define TRUE 1
#define FALSE 0
#define ON 1
#define OFF 0

extern enc_t enc;

extern sensor_t sen_l;
extern sensor_t sen_front;
extern sensor_t sen_r;

extern uint16_t flag_motion_end;
extern uint8_t flag_motor;

extern maze_t maze;

void Mode_Run(unsigned char flag_search);

/****************************************************************************************
 * outline  : wright mode 
 * argument : mode nomber
 * return   : void
********************************************************************************************/
void Mode_Mouse(int8_t mode)
{
    switch (mode)
    {
    case 0:
        Mode_Run(FALSE);
        break;
    case 1:
        Mode_Run(TRUE);
        break;
    case 2:
        HAL_Delay(5000);
        gyro_offset_calc_reset();
        HAL_Delay(2000);

        flag_motor = TRUE;

        Straight_half_accel();
        while (flag_motion_end == FALSE)
        {
        }
        Motion_Left();
        Straight_half_stop();
        while (flag_motion_end == FALSE)
        {
        }
        flag_motor = FALSE;
        break;
    case 3:
        HAL_Delay(5000);
        gyro_offset_calc_reset();
        HAL_Delay(2000);

        flag_motor = TRUE;

        Straight_half_accel();
        while (flag_motion_end == FALSE)
        {
        }
        Motion_Right();
        Straight_half_stop();
        while (flag_motion_end == FALSE)
        {
        }
        flag_motor = FALSE;
        break;
    case 4:
        HAL_Delay(5000);
        gyro_offset_calc_reset();
        HAL_Delay(2000);

        flag_motor = TRUE;

        Straight_half_accel();
        while (flag_motion_end == FALSE)
        {
        }
        Motion_Straight();
        Straight_half_stop();
        while (flag_motion_end == FALSE)
        {
        }
        flag_motor = FALSE;
        break;
    case 5:
        loadMaze();
        Maze_Printf(maze);
        break;
    case 6:
        Mode_Sensor_Check();
        break;
    default:
        break;
    }
}

/****************************************************************************************
 * outline  : return mode nomber
 * argument : void
 * return   : mode nomber 
********************************************************************************************/
int8_t Mode_Select(void)
{
    int8_t mode = 0;
    flag_motion_end = TRUE;
    flag_motor = FALSE;

    while (1)
    {
        if (enc.distance > 15 || enc.distance < -15)
        {
            if (enc.distance > 15)
            {
                mode++;
            }
            else if (enc.distance < -15)
            {
                mode--;
            }
            if (mode > 6)
            {
                mode = 0;
            }
            else if (mode < 0)
            {
                mode = 6;
            }
            enc.distance = 0;
            Output_Buzzer(170 - 10 * mode);
        }
        if (Push() == ON)
        {
            Output_Buzzer(HZ_C_H);
            HAL_Delay(500);
            break;
        }
        LED_Control(mode);
        printf("%d\r", mode);
    }
    return mode;
}

void Mode_Sensor_Check(void)
{
    adcStart();

    while (1)
    {
        if (sen_r.is_wall == TRUE)
        {
            HAL_GPIO_WritePin(led0_GPIO_Port, led0_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(led0_GPIO_Port, led0_Pin, GPIO_PIN_SET);
        }
        if (sen_l.is_wall == TRUE)
        {
            HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
        }
        if (sen_front.is_wall == TRUE)
        {
            HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
        }
        printf("l:%d f:%d r:%d \r", sen_l.now, sen_front.now, sen_r.now);

        if (Push() == ON)
        {
            Output_Buzzer(HZ_C_H);
            HAL_Delay(500);
            break;
        }
    }
}

void Mode_Run(unsigned char flag_search)
{
    if (flag_search == FALSE)
    {
        Plan_Adachi();
        HAL_Delay(500);
        writeMaze();
        HAL_Delay(1000);
        Plan_AllSearch();
        HAL_Delay(500);
        writeMaze();
    }
    else if(flag_search==TRUE)
    {
        loadMaze();
        Plan_Root();
        Plan_Fast();
    }
}