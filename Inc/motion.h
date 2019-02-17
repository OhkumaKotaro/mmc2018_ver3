/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __motion_H
#define __motion_H
#ifdef __cplusplus
 extern "C" {
#endif

void Straight_half_accel(void);
void Straight_half_stop(void);
void Straight_const(void);
void LeftTurn(void);
void RightTurn(void);
void U_Turn(void);
void Motion_Start(void);
void Motion_Straight(void);
void Motion_Left(void);
void Motion_Right(void);
void Motion_Uturn(void);
void Motion_Kabeate(void);
void Motion_Goal(void);
void Motion_Restart(void);
void Motion_StartFast(unsigned char step);
void Motion_StraightFast(unsigned char step);
void Motion_GoalFast(unsigned char step);

#ifdef __cplusplus
}
#endif
#endif /*__motion_H */
