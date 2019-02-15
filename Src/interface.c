#include "interface.h"
#include "gpio.h"
#include "tim.h"


/****************************************************************************************
 * outline  : turn on buzzer 0.3sec
 * argument : Hz
 * return   : void
********************************************************************************************/
void Output_Buzzer(unsigned char Hz){
  Buzzer_pwm(Hz,300);
  HAL_Delay(100);
  Buzzer_pwm(HZ_NORMAL,0);
}

/****************************************************************************************
 * outline  : push -> ON , else -> OFF
 * argument : void
 * return   : ON or OFF
********************************************************************************************/
unsigned char Push(void){
  unsigned char push;
  if(HAL_GPIO_ReadPin(push0_GPIO_Port,push0_Pin) == GPIO_PIN_RESET){
    push=1;
  }
  if(HAL_GPIO_ReadPin(push0_GPIO_Port,push0_Pin) == GPIO_PIN_SET){
    push = 0;
  }
  return push;
}

/****************************************************************************************
 * outline  : turn on all led
 * argument : void
 * return   : void
********************************************************************************************/
void All_LED_ON(void)
{
  HAL_GPIO_WritePin(led0_GPIO_Port,led0_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(led3_GPIO_Port,led3_Pin,GPIO_PIN_RESET);
}

/****************************************************************************************
 * outline  : turn off all led
 * argument : void
 * return   : void
********************************************************************************************/
void All_LED_OFF(void)
{
  HAL_GPIO_WritePin(led0_GPIO_Port,led0_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(led3_GPIO_Port,led3_Pin,GPIO_PIN_SET);
}

void LED_Control(unsigned char pattern){
  if(pattern == 0) All_LED_OFF();
  if((pattern & 0b0001) == 0b0001) HAL_GPIO_WritePin(led0_GPIO_Port,led0_Pin,GPIO_PIN_RESET);
  else                             HAL_GPIO_WritePin(led0_GPIO_Port,led0_Pin,GPIO_PIN_SET);
  if((pattern & 0b0010) == 0b0010) HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,GPIO_PIN_RESET);
  else                             HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,GPIO_PIN_SET);
  if((pattern & 0b0100) == 0b0100) HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,GPIO_PIN_RESET);
  else                             HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,GPIO_PIN_SET);
  if((pattern & 0b1000) == 0b1000) HAL_GPIO_WritePin(led3_GPIO_Port,led3_Pin,GPIO_PIN_RESET);
  else                             HAL_GPIO_WritePin(led3_GPIO_Port,led3_Pin,GPIO_PIN_SET);
}
