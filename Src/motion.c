#include "motion.h"
#include "control.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

#define TRUE 1
#define FALSE 0

//straight
#define ACCEL 4000 //[mm/s^2]
#define MAX_VELOCITY 400 //[mm/s]
//fast
#define FAST_VELOCITY 500 //[mm/s]
#define FASTEST_VELOCITY 700 //[mm/s]

//yawrate
#define Y_ACCEL 2000 //[degree/s^2]//2000
#define Y_MAX_VELOCITY 380//[degree/s]
//slalom
#define S_ACCEL 2100
#define S_VELOCITY 400
#define OFFSET 5

extern volatile unsigned char flag_motion_end;
extern enc_t enc;


void Straight_half_accel(void){
    Straight_Calc_fb(90,0,MAX_VELOCITY,MAX_VELOCITY,ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
}

void Straight_half_stop(void){
    Straight_Calc_fb(90-enc.offset,MAX_VELOCITY,0,MAX_VELOCITY,ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
}

void Straight_const(void){
    Straight_Calc_fb(180-enc.offset,MAX_VELOCITY,MAX_VELOCITY,MAX_VELOCITY,ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
}

void LeftTurn(void){
    Straight_Calc_fb(0,0,0,0,0);
    Yawrate_Calc_fb(90,0,0,Y_MAX_VELOCITY,Y_ACCEL);
}

void RightTurn(void){
    Straight_Calc_fb(0,0,0,0,0);
    Yawrate_Calc_fb(-90,0,0,Y_MAX_VELOCITY,Y_ACCEL);
}

void U_Turn(void){
    Straight_Calc_fb(0,0,0,0,0);
    Yawrate_Calc_fb(180,0,0,Y_MAX_VELOCITY,Y_ACCEL);
}

void Motion_Start(void){
    /*
    Straight_Calc_fb(-55,0,0,200,2000);
    Yawrate_Calc_fb(0,0,0,0,0);
    while(flag_motion_end==FALSE){}
    HAL_Delay(500);
    */
    Straight_Calc_fb(137,0,MAX_VELOCITY,MAX_VELOCITY,ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
    while(flag_motion_end==FALSE){}
}


void Motion_Straight(void){
    Straight_const();
    while(flag_motion_end==FALSE){}
}

void Motion_Left(void){
    Straight_half_stop();
    while(flag_motion_end==FALSE){}
    /*
    if(sen_front.is_wall == TRUE){
        flag.fr_wall = TRUE;
    }
    HAL_Delay(1000);
    flag.fr_wall = FALSE;
    */
    HAL_Delay(1000);
    LeftTurn();
    while(flag_motion_end==FALSE){}
    HAL_Delay(1000);
    Straight_half_accel();
    while(flag_motion_end==FALSE){}
}

void Motion_Right(void){
    Straight_half_stop();
    while(flag_motion_end==FALSE){}
    /*
    if(sen_front.is_wall == TRUE){
        flag.fr_wall = TRUE;
    }
    HAL_Delay(1000);
    flag.fr_wall = FALSE;
    */
    HAL_Delay(1000);
    RightTurn();
    HAL_Delay(1000);
    while(flag_motion_end==FALSE){}
    Straight_half_accel();
    while(flag_motion_end==FALSE){}
}

void Motion_Uturn(void){
    Straight_half_stop();
    while(flag_motion_end==FALSE){}
    HAL_Delay(500);
    U_Turn();
    while(flag_motion_end==FALSE){}
    HAL_Delay(500);
    Straight_half_accel();
    while(flag_motion_end==FALSE){}
}

void Motion_Kabeate(void){
    //U turn 
    Straight_half_stop();
    while(flag_motion_end==FALSE){}
    //front wall control
    //while(sen_front.adc > sen_front.reference-20);
    HAL_Delay(1000);
    U_Turn();
    while(flag_motion_end==FALSE){}
    HAL_Delay(500);
    //kabeate
    Straight_Calc_fb(-47,0,0,200,2000);
    Yawrate_Calc_fb(0,0,0,0,0);
    while(flag_motion_end==FALSE){}
    HAL_Delay(500);
    Straight_Calc_fb(137,0,MAX_VELOCITY,MAX_VELOCITY,ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
    while(flag_motion_end==FALSE){}
}

void Motion_Goal(void){
    enc.offset = 0;
    Straight_half_stop();
    while(flag_motion_end==FALSE){}
}

void Motion_FastStraight(unsigned char block){
    Straight_Calc_fb(90,MAX_VELOCITY,0,MAX_VELOCITY,ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
    Straight_Calc_fb(90,MAX_VELOCITY,0,MAX_VELOCITY,ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
}

void Motion_Restart(void){
    U_Turn();
    while(flag_motion_end==FALSE){}
    HAL_Delay(500);
    //kabeate
    Straight_Calc_fb(-47,0,0,200,2000);
    Yawrate_Calc_fb(0,0,0,0,0);
    while(flag_motion_end==FALSE){}
    HAL_Delay(500);
    Straight_Calc_fb(137,0,MAX_VELOCITY,MAX_VELOCITY,ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
    while(flag_motion_end==FALSE){}
}

//float buff_enc_offset;
void Motion_SlalomLeft(void){
    //offset
    //buff_enc_offset=enc.offset;
    Straight_Calc_fb(OFFSET-enc.offset,MAX_VELOCITY,MAX_VELOCITY,MAX_VELOCITY,ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
    while(flag_motion_end==FALSE){}
    //slalom
    Straight_Calc_fb(180,MAX_VELOCITY,MAX_VELOCITY,MAX_VELOCITY,ACCEL);
    Yawrate_Calc_fb(90,0,0,S_VELOCITY,S_ACCEL);
    while(flag_motion_end==FALSE){}
    //out straight
    Straight_Calc_fb(OFFSET,MAX_VELOCITY,MAX_VELOCITY,MAX_VELOCITY,ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
    while(flag_motion_end==FALSE){}
}

//ゲインは500未満
void Motion_StartFast(unsigned char step){
    Straight_Calc_fb(137+180*step,0,MAX_VELOCITY,MAX_VELOCITY+150*step,ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
    while(flag_motion_end==FALSE){}
}

void Motion_StraightFast(unsigned char step){
    Straight_Calc_fb(180*step-enc.offset,MAX_VELOCITY,MAX_VELOCITY,MAX_VELOCITY+150*step,ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
    while(flag_motion_end==FALSE){}
}

void Motion_GoalFast(unsigned char step){
    Straight_Calc_fb(90+180*step-enc.offset,MAX_VELOCITY,0,MAX_VELOCITY+(150*step),ACCEL);
    Yawrate_Calc_fb(0,0,0,0,0);
    while(flag_motion_end==FALSE){}
}