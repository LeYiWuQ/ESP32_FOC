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
static void serialEvent();
static void parseSerialCommand();

static float Voltage_Power_Supply = 12; // Power
static float target_velocity = 0.0;

static String serial_input = "";
static bool new_command_available = false;


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
  delay(100);
  Serial.println("Complete PWM initialization");
  Serial.println("Commands: 'V speed' - Set velocity (e.g., 'V 5')");

  delay(3000);

}

void loop() {
  serialEvent();
  // put your main code here, to run repeatedly:
  // If there are new commands, they will be parsed and processed.
  if(new_command_available) {
    parseSerialCommand();
    new_command_available = false;
    serial_input = "";  // Clear the input buffer
  }
  
  // Implement open-loop control
  velocity_Openloop(target_velocity);
  
  delay(1);
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
      //Command has ended. New command is marked.
      if(serial_input.length() > 0) {
        new_command_available = true;
      }
    } else {
      serial_input += inChar;  // Cumulative characters
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
  if(serial_input.charAt(0) == 'V' || serial_input.charAt(0) == 'v') {
    // Speed control command: V speed
    int space_index = serial_input.indexOf(' ');
    if(space_index > 0) {
      String speed_str = serial_input.substring(space_index + 1);
      float new_speed = speed_str.toFloat();
      
      // Limit the speed range, which can be adjusted according to actual needs.
      if(new_speed < -12.0) new_speed = -12.0;
      if(new_speed > 12.0) new_speed = 12.0;
      
      target_velocity = new_speed;
      
      Serial.print("Speed set to: ");
      Serial.println(target_velocity);
    } else {
      Serial.println("Invalid command format. Use: V speed");
    }
  }
  else if(serial_input.charAt(0) == 'S' || serial_input.charAt(0) == 's') {
    // Query status command
    Serial.print("Current speed: ");
    Serial.println(target_velocity);
    Serial.print("Supply voltage: ");
    Serial.println(Voltage_Power_Supply);
  }
  else if(serial_input.charAt(0) == 'H' || serial_input.charAt(0) == 'h') {
    // Help command
    Serial.println("Available commands:");
    Serial.println("V speed - Set target velocity (e.g., 'V 5')");
    Serial.println("S - Query status");
    Serial.println("H - Show help");
  }
  else {
    Serial.println("Unknown command. Type 'H' for help.");
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
  Ua = Ualpha + Voltage_Power_Supply/2; //Shift from [-5, 5]V to [0, 10]
  Ub = ( sqrt(3) * Ubeta - Ualpha ) / 2 + Voltage_Power_Supply / 2;
  Uc = ( Ualpha - sqrt(3) * Ubeta) / 2 + Voltage_Power_Supply /2;

  set_pwm(Ua, Ub, Uc);
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