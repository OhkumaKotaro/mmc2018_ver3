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
#define FAST_ACCEL 6000    //[mm/s]

//yawrate
#define Y_ACCEL 2000       //[degree/s^2]//2000
#define Y_MAX_VELOCITY 380 //[degree/s]
//slalom
#define S_ACCEL 5000
#define S_VELOCITY 500
#define OFFSET_IN 29
#define OFFSET_OUT 21

extern volatile unsigned char flag_motion_end;
extern enc_t enc;
extern unsigned char flag_wall;
extern volatile unsigned char flag_front_wall;
extern float enc_sum;
extern float y_sum;
extern float enc_before;
extern float y_before;

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
    enc_sum = 0;
    enc_before=0;
    y_sum = 0;
    y_before=0;
    Straight_Calc_fb(130, 0, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_Straight(void)
{
    Straight_const();
    while (flag_motion_end == FALSE)
    {
    }
    enc.offset = 0;
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
    Straight_half_stop();
    while (flag_motion_end == FALSE)
    {
    }
    HAL_Delay(500);
    U_Turn();
    while (flag_motion_end == FALSE)
    {
    }
    HAL_Delay(500);
    Straight_half_accel();
    while (flag_motion_end == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_Kabeate(void)
{
    //U turn
    Straight_half_stop();
    while (flag_motion_end == FALSE)
    {
    }
    //front wall control
    HAL_Delay(500);
    flag_front_wall=TRUE;
    while(flag_front_wall==TRUE);
    HAL_Delay(500);
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
    enc_before=0;
    y_sum = 0;
    y_before=0;
    HAL_Delay(500);
    Straight_Calc_fb(130, 0, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_Goal(void)
{
    Straight_half_stop();
    while (flag_motion_end == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_Restart(void)
{
    HAL_Delay(500);
    flag_front_wall=TRUE;
    while(flag_front_wall);
    HAL_Delay(500);
    U_Turn();
    while (flag_motion_end == FALSE)
    {
    }
    flag_wall = FALSE;
    HAL_Delay(500);
    //kabeate
    Straight_Calc_fb(-47, 0, 0, 200, 2000);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    flag_wall = TRUE;
    enc_sum = 0;
    enc_before=0;
    y_sum = 0;
    y_before=0;
    HAL_Delay(500);
    Straight_Calc_fb(137, 0, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_SlalomLeft(void)
{
    //offset
    Straight_Calc_fb(OFFSET_IN - enc.offset, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    //slalom
    Straight_Calc_fb(112, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
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
    enc.offset = 0;
}

void Motion_SlalomRight(void)
{
    //offset
    Straight_Calc_fb(OFFSET_IN - 4 - enc.offset, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    //slalom
    Straight_Calc_fb(112, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(-90, 0, 0, S_VELOCITY, S_ACCEL);
    while (flag_motion_end == FALSE)
    {
    }
    //out straight
    Straight_Calc_fb(OFFSET_OUT + 3, MAX_VELOCITY, MAX_VELOCITY, MAX_VELOCITY, ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    enc.offset = 0;
}

//ゲインは500未満
void Motion_StartFast(unsigned char step)
{
    enc_sum = 0;
    enc_before=0;
    y_sum = 0;
    y_before=0;
    unsigned int velo=MAX_VELOCITY + 150 * step;
    if(velo>2500){
        velo=2500;
    }
    enc_sum = 0;
    y_sum = 0;
    Straight_Calc_fb(137 + 180 * step, 0, MAX_VELOCITY, velo, FAST_ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_StraightFast(unsigned char step)
{
    unsigned int velo=MAX_VELOCITY + 150 * step;
    if(velo>2500){
        velo=2500;
    }
    Straight_Calc_fb(180 * step - enc.offset, MAX_VELOCITY, MAX_VELOCITY, velo, FAST_ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_GoalFast(unsigned char step)
{
    unsigned int velo=MAX_VELOCITY + 150 * step;
    if(velo>2500){
        velo=2500;
    }
    Straight_Calc_fb(90 + 180 * step - enc.offset, MAX_VELOCITY, 0, velo, FAST_ACCEL);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while (flag_motion_end == FALSE)
    {
    }
    enc.offset = 0;
}

void Motion_Enkai(void){
    Straight_Calc_fb(0,0, 0,0,0);
    Yawrate_Calc_fb(0, 0, 0, 0, 0);
    while(flag_motion_end==FALSE){

    }
}