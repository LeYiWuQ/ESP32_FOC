#include <Wire.h>
#include "AS5600.h"

#define USE_FLOAT_PRINTF 

#define PR 7 // Pole pairs
#define DIR 1 // Rotation direction

#define PWMA_PIN  10
#define PWMB_PIN  11
#define PWMC_PIN  12

static float constrain_limit(float limit_value, float low_value, float high_value);
static float normalizeAngle(float angle);
static float electricalAngel( );
static void set_pwm(float Ua, float Ub, float Uc);
static void setPhaseVoltage(float Uq, float Ud, float electrical_angle);
// static void velocity_Openloop(float target_velocity);
static void serialEvent();
static void parseSerialCommand();
void target_Angle();

static float Voltage_Power_Supply = 12; // Power
static float target_velocity = 0.0;

static String serial_input = "";
static bool new_command_available = false;

static float zero_electric_angle = 0; // Electrical angle offset
static float target_angle = 0; // Target angle in radians

static float r_vol = 0;


void setup() {
  // put your setup code here, to run once:
  pinMode(48,OUTPUT);


  //Set to output mode
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(PWMC_PIN, OUTPUT);

  //Initialize the PWM channel
  ledcAttachPin(PWMA_PIN, 0);
  ledcAttachPin(PWMB_PIN, 1);
  ledcAttachPin(PWMC_PIN, 2);

  //Configure the PWM channel so that iTime_s output frequency is 30 kHz and the duty cycle resolution is 8 biTime_s (0 to 255)
  ledcSetup(0, 30000, 8);
  ledcSetup(1, 30000, 8);
  ledcSetup(2, 30000, 8);

  // setPhaseVoltage(0, 0, 3 * PI/2); //Initial PWM output to 0V

  Serial.begin(9600);

  AS5600_Init();
  setPhaseVoltage(3, 0,3*PI/2);
  zero_electric_angle = electricalAngel();
  setPhaseVoltage(0, 0,3*PI/2);


}

void loop() {
  serialEvent();
    if(new_command_available) {
    parseSerialCommand();
    new_command_available = false;
    serial_input = "";  // Clear the input buffer
  }
  static float kp = 0.1, ki = 0.000, kd = 0.0;  // ki = 0.055 , ki = 0.0055, kd = 0.0
  static float output = 0;
  static float prev_error = 0;

  static float integral = 0;

  float sensor_angle = getAngle();
  float error =  target_angle - sensor_angle;
  integral += error;
  float derivative = error - prev_error;
  prev_error = error;
  output = kp * error + ki * integral + kd * derivative;
  float voltage = constrain_limit(output * 180.0f / PI, -6.0f, 6.0f);

  setPhaseVoltage(voltage, 0, electricalAngel()); 

  Serial.printf(" %.2f, %.2f,  %.2f\n", sensor_angle, target_angle, voltage);
  delay(10);
}

/**
 * @brief Serial port event handling function
 * @param 
 * @retval none
 */
static void serialEvent() {
  while(Serial.available()) {
    char inChar = (char)Serial.read();
    
    if(inChar == '\n' || inChar == '\r') {
      if(serial_input.length() > 0) {
        new_command_available = true;
        serial_input.trim();
      }
    } else {
      serial_input += inChar;
    }
  }
}

/**
 * @brief Parse serial port commands
 * @param 
 * @retval none
 */
static void parseSerialCommand() {
  if(serial_input.length() == 0) return;
  
  // Remove leading and trailing spaces
  serial_input.trim();
  
  if(serial_input.length() == 0) return;
  
  // Parsing command
  if(serial_input.charAt(0) == 'P' || serial_input.charAt(0) == 'p') {
    // Speed control command: V speed
    int space_index = serial_input.indexOf(' ');
    if(space_index > 0) {
      String speed_str = serial_input.substring(space_index + 1);
      float new_speed = speed_str.toFloat();
      
      // Limit the speed range, which can be adjusted according to actual needs.
      if(new_speed < -20.0) new_speed = -20.0;
      if(new_speed > 20.0) new_speed = 20.0;
      
      target_angle = new_speed;
    
    } else {
      Serial.println("Invalid command format. Use: P position");
    }
  }
}



/**
 * @brief limit_value functiom
 * @param limit_value, low_value, high_value
 * @retval limit_value
 */
static float constrain_limit(float limit_value, float low_value, float high_value)
{
    if(limit_value < low_value) return low_value;
    if(limit_value > high_value) return high_value;
    return limit_value;
}

/**
 * @brief Angle normalization function
 * @param angle
 * @retval real_angle
 */
static float normalizeAngle(float angle)
{
  float real_angle = fmod(angle, 2* PI);

  if(real_angle <0 ){
    real_angle = (real_angle+ 2*PI);
  }

  return real_angle;
}

/**
 * @brief Solve the electrical angle function
 * @param 
 * @retval
 */

static float electricalAngel(){
  return normalizeAngle((DIR * PR) * getAngle_Without_track() - zero_electric_angle);
}

/**
 * @brief PWM function
 * @param Ua, Ub, Uc
 * @retval none
 */
static void set_pwm(float Ua, float Ub, float Uc){
  static float  dc_a = 0 ,dc_b = 0 , dc_c = 0;
  // Normalize the voltage to the range of 0.0f to -1.0f.
  dc_a = constrain_limit(Ua / Voltage_Power_Supply, 0.0f, 1.0f);
  dc_b = constrain_limit(Ub / Voltage_Power_Supply, 0.0f, 1.0f);
  dc_c = constrain_limit(Uc / Voltage_Power_Supply, 0.0f, 1.0f);

  ledcWrite(0, dc_a *255);
  ledcWrite(1, dc_b *255);
  ledcWrite(2, dc_c *255);
}

/**
 * @brief  setPhaseVoltage function
 * @param Uq, Ud, electrical_angle
 * @retval none
 */
static void setPhaseVoltage(float Uq, float Ud, float electrical_angle){

  static float Ualpha = 0, Ubeta = 0;  // Park inverse transformation param
  static float Ua = 0 , Ub = 0 , Uc = 0; //Clark inverse transformation param
  static float zero_electic_angle = 0 ;

  electrical_angle = normalizeAngle(electrical_angle + zero_electic_angle);

  //Park inverse transformation
  Ualpha = -Uq * sin(electrical_angle);
  Ubeta =  Uq *cos(electrical_angle);

  //Clark inverse transformation
  Ua = Ualpha + Voltage_Power_Supply/2; //Shift from [-5, 5]V to [0, 10]
  Ub = ( sqrt(3) * Ubeta - Ualpha ) / 2 + Voltage_Power_Supply / 2;
  Uc = ( Ualpha - sqrt(3) * Ubeta) / 2 + Voltage_Power_Supply /2;

  set_pwm(Ua, Ub, Uc);
}

// float PID_control(float target, float current){
//   static float kp = 0.1;
//   static float ki = 0.01;
//   static float kd = 0.005;

//   static float integral = 0;
//   static float previous_error = 0;

//   float error = target - current;
//   integral += error;
//   float derivative = error - previous_error;

//   float output = kp * error + ki * integral + kd * derivative;

//   previous_error = error;

//   return output;
// }

// void target_Angle()
// {
//   static float kp =  -0.133, ki = 0.001, kd = 0.01;
//   static float output = 4;
//   static float prev_error = 0;
//   static float target_angle = 0; // Target angle in radians

//   static float integral = 0;

//   float sensor_angle = getAngle();
//   float error =  DIR *(target_angle - sensor_angle);
//   integral += error;
//   float derivative = error - prev_error;
//   prev_error = error;
//   output = kp * error +  kd * derivative;
//   float voltage = constrain_limit(output * 180.0f / PI, -6.0f, 6.0f);

//   setPhaseVoltage(voltage, 0, electricalAngel()); 
// }

// void target_Angle() {
//   static float target_angle = 0; 
//   Serial.println(getAngle());
//   float Sensor_Angle=getAngle();
//   float Kp=-0.133;
//   setPhaseVoltage(constrain(Kp*DIR*(target_angle-Sensor_Angle)*180/PI,-6,6),0,electricalAngel());

// }