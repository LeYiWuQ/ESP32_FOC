#include <Wire.h>
#include "AS5600.h"

#define SDA_PIN GPIO_NUM_3
#define SCL_PIN GPIO_NUM_4

// 创建一个传感器对象（假设电机编号为 1）
Sensor_AS5600 as5600_sensor(1);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("AS5600 测试启动...");

  // 初始化传感器（使用默认 I2C Wire）
  as5600_sensor.Sensor_init(&Wire);

  Serial.println("初始化完成。开始读取数据...");
}

void loop() {
  // 更新传感器数据
  as5600_sensor.Sensor_update();

  // 获取角度（弧度）和机械角度
  float mechanicalAngle = as5600_sensor.getMechanicalAngle(); // 当前圈内角度（0~2π）
  float totalAngle = as5600_sensor.getAngle();                 // 总角度（包括圈数）
  float velocity = as5600_sensor.getVelocity();                // 角速度（弧度/秒）

  // 打印到串口
  Serial.print("机械角度(rad): ");
  Serial.print(mechanicalAngle, 4);
  Serial.print("\t总角度(rad): ");
  Serial.print(totalAngle, 4);
  Serial.print("\t角速度(rad/s): ");
  Serial.println(velocity, 4);

  delay(100);  // 每100ms刷新一次
}
