/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
//mpu6500
#define WHO_AM_I      0x75
#define Certain       0x70//return from "WHO_AM_I"
#define CONFIG        0x1A
#define GYRO_CONFIG   0x1B
#define PWR_MGMT_1    0x6B
#define GYRO_OUT_Z_L  0x48
#define GYRO_OUT_Z_H  0x47
#define SETTING       0x80  //0b1000 0000 8bitの上位bitを立てると

#define GYRO_FACTOR  16.4f

#define TRUE 0
#define FALSE 1

#define dt 0.001f

gyro_t gyro;
uint8_t flag_gyro_calc;
/* USER CODE END 0 */

SPI_HandleTypeDef hspi2;

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/********************************************************************
 * Overview : spi read register
 * Argument : register
 * Return : 1byte data
********************************************************************/
uint8_t read_byte(uint8_t reg){
  uint8_t ret,val;
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_RESET);
  ret = reg|SETTING ;
  //HAL_SPI_TransmitReceive(&hspi2,&ret,&val,1,400); //not work
  HAL_SPI_Transmit(&hspi2,&ret,1,100);
  HAL_SPI_Receive(&hspi2,&val,1,100);
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_SET);
  return val;
}


/****************************************************************
 * Overview : shift 8bit and spi read register
 * Argument : register
 * Return : 2byte data (shift 8bit)
 ************************************************************/
int16_t read_shift_byte(uint8_t reg){
  uint8_t address,val_1;
  int16_t val_2;
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_RESET);
  address = reg | SETTING ;
  //HAL_SPI_TransmitReceive(&hspi2,&address,&val_1,1,100); //not work
  HAL_SPI_Transmit(&hspi2,&address,1,100);
  HAL_SPI_Receive(&hspi2,&val_1,1,100);
  val_2 = (int16_t)(val_1 << 8);
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_SET);
  return val_2;
}


/**************************************************************
 * Overview : spi write 1byte
 * Argument : register
 * Return : 
 *************************************************************/
void write_byte( uint8_t reg,uint8_t val){
  uint8_t ret;
  ret = reg & 0x7F;
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2,&ret,1,100);
  HAL_SPI_Transmit(&hspi2,&val,1,100);
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_SET);
}


/****************************************************************
 * Overview : set up mpu6500
 * argument : void
 * return   : void
 ****************************************************************/
void set_mpu6500(void){
  uint8_t val = 0x00;
  while(val!=Certain){
    val = read_byte(WHO_AM_I);
  }
  printf("\r\nI am 0x%x\r\n",val );
  write_byte(PWR_MGMT_1,0x00);
  write_byte(CONFIG,0x00);
  write_byte(GYRO_CONFIG,0x18);
}


/*****************************************************************
 * Overview : get gyro value
 * argument : void
 * Return   : degree (2000 deg/sec)
 ****************************************************************/
int16_t get_gyro(void){
  int16_t gyro_z;
  gyro_z = (int16_t)(read_shift_byte(GYRO_OUT_Z_H) | read_byte(GYRO_OUT_Z_L));
  gyro_z -= (int16_t)gyro.offset;
  return gyro_z;
}

/*****************************************************************
 * Overview : calculate gyro offset
 * argument : void
 * Return   : void
 ****************************************************************/
void gyro_offset_calc(void){
  int16_t gyro_z;

  gyro_z = (int16_t)(read_shift_byte(GYRO_OUT_Z_H) | read_byte(GYRO_OUT_Z_L));

  if(gyro.offset_cnt<1000){
    gyro.offset += gyro_z;
    gyro.offset_cnt++;
  }else{
    gyro.offset /= 1000;
    flag_gyro_calc = TRUE;
  }
}

void Update_gyro(void){
  if(flag_gyro_calc == TRUE){
    gyro.velocity = (float)get_gyro()/GYRO_FACTOR;
    gyro.degree += (float)get_gyro()/GYRO_FACTOR * dt;
  }else{
    gyro_offset_calc();
  }
}
/*****************************************************************
 * Overview : reset gyro offset variable
 * argument : void
 * Return   : void
 ****************************************************************/
void gyro_offset_calc_reset(void){
  gyro.offset = 0;
  gyro.offset_cnt = 0;
  flag_gyro_calc = FALSE;

  gyro.degree = 0;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
