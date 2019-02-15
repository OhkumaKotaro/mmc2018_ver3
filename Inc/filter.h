/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __filter_H
#define __filter_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"

  // sensor data
  typedef struct
  {
    int16_t now;
    int16_t befor_1ms;
    int16_t reference; // 真ん中のときのセンサー値
    int16_t threshold; // 閾値
    int16_t diff;      // 差分
    int16_t diff_1ms;  // 1msec前
    uint8_t is_wall;   // 壁があるかどうか判断
  } sensor_t;
  
  void setSensorConstant(void);
  void adcStart(void);

  void Filter_SetReference(void);
  void adcCheckConvert(void);
  void update_sensor_data(void);
  void getADSensor(void);
  float Filter_GetBatt(void);

#ifdef __cplusplus
}
#endif
#endif /*__filter_H */