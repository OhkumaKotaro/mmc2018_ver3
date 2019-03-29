#include "filter.h"
#include "adc.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "main.h"
#include "gpio.h"
#include <stdint.h>
#include "filter.h"
#include "interface.h"


#define OFF_VALUE 0
#define LEFT_VALUE 1
#define RIGHT_VALUE 2
#define FINISH_CONVERT 3

#define ADC_CONVERT_DATA_SIZE ((uint32_t)  4)

#define TRUE 1
#define FALSE 0


sensor_t sen_l;
sensor_t sen_fl;
sensor_t sen_front;
sensor_t sen_fr;
sensor_t sen_r;

uint16_t ADCBuff[ADC_CONVERT_DATA_SIZE];
uint16_t ADCOffData[ADC_CONVERT_DATA_SIZE];
uint16_t ADCOntData[ADC_CONVERT_DATA_SIZE];
int16_t adc_counter;

void setSensorConstant(void)
{
  sen_l.reference = 578;
  sen_l.threshold = 472;

  sen_fl.reference = 786;

  sen_front.reference = 793;
  sen_front.threshold = 610;

  sen_fr.reference = 803;

  sen_r.reference = 670;
  sen_r.threshold = 570;
}

void update_sensor_data(void)
{

  sen_front.now = (sen_fl.now + sen_fr.now) / 2;

  if (sen_front.now < sen_front.threshold)
  {
    sen_front.is_wall = FALSE;
  }
  else
  {
    sen_front.is_wall = TRUE;
  }

  if (sen_l.now < sen_l.threshold)
  {
    sen_l.is_wall = FALSE;
  }
  else
  {
    sen_l.is_wall = TRUE;
  }

  if (sen_r.now < sen_r.threshold)
  {
    sen_r.is_wall = FALSE;
  }
  else
  {
    sen_r.is_wall = TRUE;
  }
}

void adcStart(void)
{
  adc_counter = 0;
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCBuff, ADC_CONVERT_DATA_SIZE);
}

void adcCheckConvert(void)
{
  if (adc_counter == FINISH_CONVERT)
  {
    update_sensor_data();
    adc_counter = 0;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCBuff, ADC_CONVERT_DATA_SIZE);

    sen_l.diff_1ms = sen_l.now - sen_l.befor_1ms;
    sen_l.befor_1ms = sen_l.now;

    sen_r.diff_1ms = sen_r.now - sen_r.befor_1ms;
    sen_r.befor_1ms = sen_r.now;
  }
}

// DMA の変換式を記載
void getADSensor(void)
{
  volatile unsigned char i;
  switch (adc_counter)
  {
  case OFF_VALUE:
    HAL_ADC_Stop_DMA(&hadc1);
    ADCOffData[0] = ADCBuff[0];
    ADCOffData[1] = ADCBuff[1];
    ADCOffData[2] = ADCBuff[2];
    ADCOffData[3] = ADCBuff[3];

    HAL_GPIO_WritePin(paluse2_GPIO_Port, paluse2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(paluse3_GPIO_Port, paluse3_Pin, GPIO_PIN_SET);
    
    for (i = 0; i < 200; i++)
    {
    }
    
    adc_counter = LEFT_VALUE;

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCBuff,ADC_CONVERT_DATA_SIZE);
    break;

  case LEFT_VALUE:
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_GPIO_WritePin(paluse2_GPIO_Port, paluse2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(paluse3_GPIO_Port, paluse3_Pin, GPIO_PIN_RESET);

    ADCOntData[2] = ADCBuff[2];
    ADCOntData[3] = ADCBuff[3];

    //sen_l.diff = sen_l.now - (ADCOntData[2] - ADCOffData[2]);
    sen_l.now = ADCOntData[2] - ADCOffData[2];

    //sen_fl.diff = sen_fl.now - (ADCOntData[3] - ADCOffData[3]);
    sen_fl.now = ADCOntData[3] - ADCOffData[3];

    HAL_GPIO_WritePin(paluse0_GPIO_Port, paluse0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(paluse1_GPIO_Port, paluse1_Pin, GPIO_PIN_SET);

    
    for (i = 0; i < 200; i++)
    {
    }
    
    adc_counter = RIGHT_VALUE;

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADCBuff,ADC_CONVERT_DATA_SIZE);
    break;

  case RIGHT_VALUE:
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_GPIO_WritePin(paluse0_GPIO_Port, paluse0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(paluse1_GPIO_Port, paluse1_Pin, GPIO_PIN_RESET);

    ADCOntData[0] = ADCBuff[0];
    ADCOntData[1] = ADCBuff[1];

    //sen_fr.diff = sen_fr.now - (ADCOntData[0] - ADCOffData[0]);
    sen_fr.now = ADCOntData[0] - ADCOffData[0];

    //sen_r.diff = sen_r.now - (ADCOntData[1] - ADCOffData[1]);
    sen_r.now = ADCOntData[1] - ADCOffData[1];
    
    for (i = 0; i < 200; i++)
    {
    }
    
    adc_counter = FINISH_CONVERT;
    break;

  default:
    break;
  }
}

float Filter_GetBatt(void)
{
  float batt = 0;
  for (int i = 0; i < 10; i++)
  {
    HAL_ADC_Start(&hadc2); // ad convert start
    while (HAL_ADC_PollForConversion(&hadc2, 50) != HAL_OK)
    {
    } // trans
    batt += HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
  }
  batt = batt / 10.0f / 4095.0f * 133.0f / 33.0f * 3.3f;
  //finish
  while(1){
    LED_Control((unsigned char)batt);
    if(Push()==1){
      Output_Buzzer(HZ_C_H);
      HAL_Delay(500);
      break;
    }
  }
  return batt;
}
