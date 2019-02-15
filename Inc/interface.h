/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __interface_H
#define __interface_H
#ifdef __cplusplus
 extern "C" {
#endif

/***************define**********************/

//buzzer
#define HZ_C 153
#define HZ_D 136
#define HZ_E 121
#define HZ_F 114
#define HZ_G 102
#define HZ_A 91
#define HZ_B 81
#define HZ_C_H 76
#define HZ_NORMAL 10

/***************prottype*******************/
//buzzer
void Output_Buzzer(unsigned char Hz);
//push
unsigned char Push(void);
//LED
void All_LED_ON(void);
void All_LED_OFF(void);
void LED_Control(unsigned char pattern);

#ifdef __cplusplus
}
#endif
#endif /*__interface_H */