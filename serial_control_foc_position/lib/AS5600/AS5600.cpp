//在灯哥开源基础上修改
// 原仓库地址：https://github.com/ToanTech/DengFOC_Lib/blob/main/DengFOC



#include "Wire.h" 
#include "AS5600.h"
#include <Arduino.h> 

#define SCL_PIN GPIO_NUM_4
#define SDA_PIN GPIO_NUM_3

int _raw_ang_hi = 0x0c;
int _raw_ang_lo = 0x0d;
int _ams5600_Address = 0x36;
int ledtime = 0;
int32_t full_rotations=0; // full rotation tracking;
float angle_prev=0; 

void AS5600_Init() {
  Wire.begin(SDA_PIN,SCL_PIN, 400000UL);
  delay(1000);
}


word readTwoBytes(int in_adr_hi, int in_adr_lo)
{
  word retVal = -1;
 
  /* 读低位 */
  Wire.beginTransmission(_ams5600_Address);
  Wire.write(in_adr_lo);
  Wire.endTransmission();
  Wire.requestFrom(_ams5600_Address, 1);
  while(Wire.available() == 0);
  int low = Wire.read();
 
  /* 读高位 */  
  Wire.beginTransmission(_ams5600_Address);
  Wire.write(in_adr_hi);
  Wire.endTransmission();
  Wire.requestFrom(_ams5600_Address, 1);
  while(Wire.available() == 0);
  int high = Wire.read();
  
  retVal = (high << 8) | low;
  
  return retVal;
}

word getRawAngle()
{
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo);
}

float getAngle_Without_track()
{
  return getRawAngle()*0.08789* PI / 180;    //得到弧度制的角度
}

float getAngle()
{
    float val = getAngle_Without_track();
    float d_angle = val - angle_prev;
    //计算旋转的总圈数
    //通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
    if(abs(d_angle) > (0.8f*6.28318530718f) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
    return (float)full_rotations * 6.28318530718f + angle_prev;
    
}