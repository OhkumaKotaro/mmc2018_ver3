#include "control.h"
#include "tim.h"
#include "stdint.h"
#include "filter.h"
#include "spi.h"
#include "interface.h"

#define TRUE 1
#define FALSE 0

#define ON 1
#define OFF

#define dt 0.001

typedef struct
{
    int16_t velocity;
    int16_t accel;
    int8_t dir;
} target_t;
target_t straight_tgt;
target_t yawrate_tgt;

typedef struct
{
    uint16_t up;
    uint16_t cons;
    uint16_t down;
} trapezoid_tim_t;
trapezoid_tim_t straight_tim;
trapezoid_tim_t yawrate_tim;

extern enc_t enc;
extern gyro_t gyro;

extern sensor_t sen_l;
extern sensor_t sen_fl;
extern sensor_t sen_front;
extern sensor_t sen_fr;
extern sensor_t sen_r;

float enc_sum;
float enc_before;

float y_sum;
float y_before;

float fwall_error_s = 0;
float fwall_error_y = 0;

volatile unsigned char flag_motion_end;
uint8_t flag_motor;
unsigned char flag_wall;
volatile unsigned char flag_front_wall;

unsigned char add_l = 0, add_r = 0;

/****************************************************************************************
 * outline  : PID control
 * argument : 
 * return   : control value
********************************************************************************************/
float PID_value(float target, float measured, float *sum, float *old, float Kp, float Ki, float Kd)
{
    float error;
    float p, i, d;

    error = target - measured;
    p = Kp * error;

    *sum += error * dt;
    i = *sum * Ki;

    d = Kd * (error - *old);
    *old = error;

    return (p + i + d);
}

/****************************************************************************************
 * outline  : calcurate accele distance
 * argument : distance[mm],v_start[mm/s],v_end[mm/s]
 * return   : void
********************************************************************************************/
void Straight_Calc_fb(int16_t distant, int16_t v_start, int16_t v_end, uint16_t v_max, uint16_t accel)
{
    float t1, t2, t3;
    float constant_L;

    enc_before = 0;
    y_before = 0;

    straight_tgt.velocity = v_start;
    straight_tgt.accel = accel;

    if (distant < 0)
    {
        distant = -1 * distant;
        straight_tgt.dir = -1;
    }
    else if (distant == 0)
    {
        straight_tgt.dir = 0;
        accel = 1;
        v_max = 1;
    }
    else if (distant > 0)
    {
        straight_tgt.dir = 1;
    }

    t1 = (float)(v_max - v_start) / accel;
    t3 = (float)(v_max - v_end) / accel;

    constant_L = (float)distant - (v_max + v_start) * t1 / 2 - (v_max + v_end) * t3 / 2;

    t2 = constant_L / v_max;

    t1 *= 1000;
    t2 *= 1000;
    t3 *= 1000;

    straight_tim.up = t1;
    straight_tim.cons = t2;
    straight_tim.down = t3;

    flag_motion_end = FALSE;

    //printf("%d %d %d\r\n",accel_T,constant_T,decrease_T);
}

/****************************************************************************************
 * outline  : calcurate trapezoid accel
 * argument : degree[degree],v_start[degree/s],v_end[degree/s]
 * return   : void
********************************************************************************************/
void Yawrate_Calc_fb(int16_t degree, int16_t v_start, int16_t v_end, int16_t v_max, int16_t accel)
{
    float t1 = 0, t2 = 0, t3 = 0;
    float constant_L;

    enc_before = 0;
    y_before = 0;

    yawrate_tgt.velocity = v_start;
    yawrate_tgt.accel = accel;

    if (degree < 0)
    {
        degree = -1 * degree;
        yawrate_tgt.dir = -1;
    }
    else if (degree == 0)
    {
        yawrate_tgt.dir = 0;
        accel = 1;
        v_max = 1;
    }
    else if (degree > 0)
    {
        yawrate_tgt.dir = 1;
    }

    t1 = (float)(v_max - v_start) / accel;
    t3 = (float)(v_max - v_end) / accel;

    constant_L = (float)degree - (v_max + v_start) * t1 / 2 - (v_max + v_end) * t3 / 2;

    t2 = constant_L / v_max;

    t1 *= 1000;
    t2 *= 1000;
    t3 *= 1000;

    yawrate_tim.up = t1;
    yawrate_tim.cons = t2;
    yawrate_tim.down = t3;

    flag_motion_end = FALSE;
}

/****************************************************************************************
 * outline  : output pwm for trapezoid accele straight by feadbuck control
 * argument : void
 * return   : void
********************************************************************************************/
float Control_encoder(void)
{
    float target = 0;
    if (straight_tgt.dir == 0)
    {
    }
    else
    {
        if (straight_tim.up > 0)
        {
            straight_tgt.velocity += straight_tgt.accel * dt;
            straight_tim.up--;
        }
        else if (straight_tim.cons > 0)
        {
            straight_tim.cons--;
        }
        else if (straight_tim.down > 0)
        {
            straight_tgt.velocity -= straight_tgt.accel * dt;
            straight_tim.down--;
        }
        else
        {
            flag_motion_end = TRUE;
        }
    }

    target = (float)straight_tgt.dir * straight_tgt.velocity + fwall_error_s;

    return PID_value(target, enc.velocity, &enc_sum, &enc_before, 4.2f, 10.0f, 0);
}

float Control_Side_Wall(void)
{
    float wall_dif = 0;
    float kp = 0.3f;

    if (sen_l.now > sen_l.threshold + add_l && sen_r.now > sen_r.threshold + add_r)
    {
        LED_Control(12);
        if (sen_l.diff_1ms > -10 && sen_r.diff_1ms > -10)
        {
            if (sen_l.diff_1ms < 10 && sen_r.diff_1ms < 10)
            {
                wall_dif = kp * ((sen_l.now - sen_l.reference) - (sen_r.now - sen_r.reference));
                add_l = 0;
                add_r = 0;
            }
        }
        else
        {
            add_l = 150;
            add_r = 150;
        }
    }
    else if (sen_l.now > sen_l.threshold + add_l)
    {
        LED_Control(4);
        if (sen_l.diff_1ms > -10 && sen_l.diff_1ms < 10)
        {
            wall_dif = 2 * kp * (sen_l.now - sen_l.reference);
            add_l=0;
        }
        else
        {
            add_l = 150;
        }
    }
    else if (sen_r.now > sen_r.threshold + add_r)
    {
        LED_Control(8);
        if (sen_r.diff_1ms > -10 && sen_r.diff_1ms < 10)
        {
            wall_dif = -2 * kp * (sen_r.now - sen_r.reference);
            add_r = 0;
        }
        else
        {
            add_r = 150;
        }
    }
    else
    {
        LED_Control(0);
    }

    if (flag_wall != TRUE)
    {
        wall_dif = 0;
        LED_Control(0);
    }

    if (yawrate_tgt.dir != 0 || straight_tgt.velocity < 400)
    {
        wall_dif = 0;
        LED_Control(0);
    }
    return wall_dif;
}

/****************************************************************************************
 * outline  : call 1ms (control yawrate by feadbuck)
 * argument : void
 * return   : void
********************************************************************************************/
float Control_gyro(void)
{
    float target = 0;

    if (yawrate_tgt.dir == 0)
    {
        yawrate_tgt.velocity = 0;
    }
    else if (yawrate_tim.up > 0)
    {
        yawrate_tgt.velocity += yawrate_tgt.accel * dt;
        yawrate_tim.up--;
    }
    else if (yawrate_tim.cons > 0)
    {
        yawrate_tim.cons--;
    }
    else if (yawrate_tim.down > 0)
    {
        yawrate_tgt.velocity -= yawrate_tgt.accel * dt;
        yawrate_tim.down--;
    }
    else
    {
        yawrate_tgt.velocity = 0;
        flag_motion_end = TRUE;
    }

    target = (float)yawrate_tgt.dir * yawrate_tgt.velocity - Control_Side_Wall() + fwall_error_y;

    return PID_value(target, gyro.velocity, &y_sum, &y_before, 0.98f, 18.0f, 4.5f); //2.0f, 20.0f, 4.0f//0.92f, 17.0f, 4.0f
}

void Control_Front_Wall(void)
{
    float error_s = 0;
    float error_y = 0;

    if (straight_tgt.velocity == 0 && yawrate_tgt.velocity == 0)
    {
        if (sen_front.is_wall == TRUE)
        {
            error_s = sen_front.reference - sen_front.now;
            error_y = sen_fl.now - sen_fl.reference - (sen_fr.now - sen_fr.reference);
            fwall_error_s = error_s * 2.0f;
            fwall_error_y = error_y * 3.0f;
            if (-3 < error_s && error_s < 3)
            {
                if (-3 < error_y && error_y < 3)
                {
                    fwall_error_s = 0;
                    fwall_error_y = 0;
                    enc_sum = 0;
                    enc_before = 0;
                    y_sum = 0;
                    y_before = 0;
                    flag_front_wall = FALSE;
                }
            }
        }
    }
    if (flag_front_wall == FALSE)
    {
        fwall_error_s = 0;
        fwall_error_y = 0;
    }
}

void Control_pwm(void)
{
    if (flag_motor == TRUE)
    {
        int16_t straight_pid = 0;
        int16_t yawrate_pid = 0;

        Control_Front_Wall();
        straight_pid = (int16_t)Control_encoder();
        yawrate_pid = (int16_t)Control_gyro();

        Motor_pwm(straight_pid - yawrate_pid, straight_pid + yawrate_pid);
    }
    else
    {
        Motor_pwm(0, 0);
    }
}

void Control_EmergencyStop(unsigned short diff_yaw)
{
    int buff_yaw = yawrate_tgt.dir * yawrate_tgt.velocity - Control_Side_Wall() - gyro.velocity;
    //yawrate
    if (buff_yaw > diff_yaw || buff_yaw < -diff_yaw)
    {
        flag_motor = FALSE;
    }
}