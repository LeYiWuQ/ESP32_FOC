#include <Arduino.h>

#define target_value  10

#define PWMA_PIN  10
#define PWMB_PIN  11
#define PWMC_PIN  12

static float constrain_limit(float limit_value, float low_value, float high_value);
static float normalizeAngle(float angle);
static float electicalAngel(float shaft_angle, int pole_pairs);
static void set_pwm(float Ua, float Ub, float Uc);
static void setPhaseVoltage(float Uq, float Ud, float electrical_angle);
static void velocity_Openloop(float target_velocity);

static float Voltage_Power_Supply = 12.6; // Power


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

  Serial.begin(9600);
  Serial.println("Complete PWM initialization \r\n");

  delay(3000);

}

void loop() {
  // put your main code here, to run repeatedly:
 velocity_Openloop(3);
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
 * @param shaft_angle, pole_pairs  // 2804 - pole_pairs = 7
 * @retval
 */

static float electicalAngel(float shaft_angle, int pole_pairs){
    return (shaft_angle * pole_pairs);
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
  Ua = Ualpha + Voltage_Power_Supply/2;
  Ub = ( sqrt(3) * Ubeta - Ualpha )  + Voltage_Power_Supply / 2;
  Uc = ( Ualpha - sqrt(3) * Ubeta) / 2 + Voltage_Power_Supply /2;

  set_pwm(Ua, Ub, Uc);

  //park 
}

/**
 * @brief open_loop function
 * @param target_velocity
 * @retval none
 */
static void velocity_Openloop(float target_velocity){
  //set time param
  static float shaft_angle = 0, open_loop_timedtamp = 0; 
  unsigned long now_us = micros();

  //get time (s)
  float Time_s = (now_us - open_loop_timedtamp ) * 1e-6f;

  //fix time
  if(Time_s <=0 || Time_s > 0.5f){
    Time_s = 1e-3f;
  }

  // determine the shaft angle
  shaft_angle = normalizeAngle(shaft_angle + target_velocity * Time_s);

  //
  float Uq_real = Voltage_Power_Supply / 3;
  
  //
  setPhaseVoltage(Uq_real, 0, electicalAngel(shaft_angle, 7));

  //
  open_loop_timedtamp = now_us;
}