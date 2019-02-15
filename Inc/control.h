/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROL_H
#define __CONTROL_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"


float PID_value(float target,float measured,float *sum,float *old,float Kp,float Ki,float Kd);
void Straight_Calc_fb(int16_t distant, int16_t v_start, int16_t v_end, uint16_t v_max, uint16_t accel);
void Yawrate_Calc_fb(int16_t degree,int16_t v_start,int16_t v_end,int16_t v_max,int16_t accel);
void Straight_SysTic_fb(void);
void Yawrate_SysTic_fb(void);
void Control_pwm(void);


#ifdef __cplusplus
}
#endif
#endif /*__CONTROL_H */