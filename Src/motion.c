#include "motion.h"
#include "control.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

#define TRUE 1
#define FALSE 0

//straight
#define ACCEL 4000       //[mm/s^2]
#define MAX_VELOCITY 400 //[mm/s]
//fast
#define FAST_VELOCITY 500    //[mm/s]
#define FASTEST_VELOCITY 700 //[mm/s]

//yawrate
#define Y_ACCEL 2000       //[degree/s^2]//2000
#define Y_MAX_VELOCITY 380 //[degree/s]
//slalom
#define S_ACCEL 4000
#define S_VELOCITY 440
#define OFFSET_IN 17
#define OFFSET_OUT 15 //10

extern volatile unsigned char flag_motion_end;
extern enc_t enc;
extern unsigned char flag_wall;
extern float enc_sum;
extern float y_sum;

void Straight_half_accel(void)
{
    Straight_Calc_fb(90, 0, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
}

void Straight_half_stop(void)
{
    Straight_Calc_fb(90 - enc.offset, MAX_VELOCITY, 0, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
}

void Straight_const(void)
{
    Straight_Calc_fb(180 - enc.offset, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
}

void LeftTurn(void)
{
    Straight_Calc_fb(0, 0, 0, 0, 0);
    Yawrate_Calc_fb(90, 0, 0, Y_MAX_VELOCITY, Y_ACCEL);
}

void RightTurn(void)
{
    Straight_Calc_fb(0, 0, 0, 0, 0);
    Yawrate_Calc_fb(-90, 0, 0, Y_MAX_VELOCITY, Y_ACCEL);
}

void U_Turn(void)
{
    Straight_Calc_fb(0, 0, 0, 0, 0);
    Yawrate_Calc_fb(180, 0, 0, Y_MAX_VELOCITY, Y_ACCEL);
}

void Motion_Start(void)
{
    /*
    Straight_Calc_fb(-55,0,0,200,2000);
    Yawrate_Calc_fb(0,0,0,0,0);
    while(flag_motion_end==FALSE){}
    HAL_Delay(500);
    */
    enc_sum = 0;
    y_sum = 0;
    Straight_Calc_fb(137, 0, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
}

void Motion_Straight(void)
{
    flag_wall = TRUE;
    Straight_const();
    while (flag_motion_end == FALSE)
    {
    }
}

void Motion_Left(void)
{
    flag_wall = TRUE;
    Straight_half_stop();
    while (flag_motion_end == FALSE)
    {
    }
    HAL_Delay(1000);
    flag_wall = FALSE;
    LeftTurn();
    while (flag_motion_end == FALSE)
    {
    }
    HAL_Delay(1000);
    flag_wall = TRUE;
    Straight_half_accel();
    while (flag_motion_end == FALSE)
    {
    }
}

void Motion_Right(void)
{
    flag_wall = TRUE;
    Straight_half_stop();
    while (flag_motion_end == FALSE)
    {
    }
    HAL_Delay(1000);
    flag_wall = FALSE;
    RightTurn();
    while (flag_motion_end == FALSE)
    {
    }
    HAL_Delay(1000);
    flag_wall = TRUE;
    Straight_half_accel();
    while (flag_motion_end == FALSE)
    {
    }
}

void Motion_Uturn(void)
{
    flag_wall = TRUE;
    Straight_half_stop();
    while (flag_motion_end == FALSE)
    {
    }
    HAL_Delay(500);
    flag_wall = FALSE;
    U_Turn();
    while (flag_motion_end == FALSE)
    {
    }
    HAL_Delay(500);
    flag_wall = TRUE;
    Straight_half_accel();
    while (flag_motion_end == FALSE)
    {
    }
}

void Motion_Kabeate(void)
{
    //U turn
    flag_wall = TRUE;
    Straight_half_stop();
    while (flag_motion_end == FALSE)
    {
    }
    //front wall control
    //while(sen_front.adc > sen_front.reference-20);
    HAL_Delay(1000);
    flag_wall = FALSE;
    U_Turn();
    while (flag_motion_end == FALSE)
    {
    }
    HAL_Delay(500);
    //kabeate
    Straight_Calc_fb(-47, 0, 0, 200, 2000);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    enc_sum = 0;
    y_sum = 0;
    HAL_Delay(500);
    Straight_Calc_fb(137, 0, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
}

void Motion_Goal(void)
{
    enc.offset = 0;
    flag_wall = TRUE;
    Straight_half_stop();
    while (flag_motion_end == FALSE)
    {
    }
}

void Motion_Restart(void)
{
    U_Turn();
    while (flag_motion_end == FALSE)
    {
    }
    HAL_Delay(500);
    //kabeate
    Straight_Calc_fb(-47, 0, 0, 200, 2000);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    enc_sum = 0;
    y_sum = 0;
    HAL_Delay(500);
    Straight_Calc_fb(137, 0, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
}

void Motion_SlalomLeft(void)
{
    flag_wall = FALSE;
    //offset
    Straight_Calc_fb(OFFSET_IN - 1 - enc.offset, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    //slalom
    Straight_Calc_fb(154, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(90, 0, 0, S_VELOCITY, S_ACCEL);
    while (flag_motion_end == FALSE)
    {
    }
    //out straight
    Straight_Calc_fb(OFFSET_OUT, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
}

void Motion_SlalomRight(void)
{
    flag_wall = FALSE;
    //offset
    Straight_Calc_fb(OFFSET_IN - enc.offset, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    //slalom
    //flag_wall = FALSE;
    Straight_Calc_fb(154, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(-90, 0, 0, S_VELOCITY, S_ACCEL);
    while (flag_motion_end == FALSE)
    {
    }
    //out straight
    //flag_wall = TRUE;
    Straight_Calc_fb(OFFSET_OUT, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
}

//ゲインは500未満
void Motion_StartFast(unsigned char step)
{
    enc_sum = 0;
    y_sum = 0;
    flag_wall = TRUE;
    Straight_Calc_fb(137 + 180 * step, 0, MAX_VELOCITY, MAX_VELOCITY + 150 * step, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
}

void Motion_StraightFast(unsigned char step)
{
    flag_wall = TRUE;
    Straight_Calc_fb(180 * step - enc.offset, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY + 150 * step, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
}

void Motion_GoalFast(unsigned char step)
{
    flag_wall = TRUE;
    Straight_Calc_fb(90 + 180 * step - enc.offset, MAX_VELOCITY, 0, MAX_VELOCITY + (150 * step), ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
}