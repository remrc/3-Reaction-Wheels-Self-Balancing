#include "ESP32.h"
#include <Wire.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("3-Wheel-Robot"); // Bluetooth device name
  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);
  
  pinMode(DIR1, OUTPUT);
  ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  Motor1_control(0);
  
  pinMode(DIR2, OUTPUT);
  ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  Motor2_control(0);
  
  pinMode(DIR3, OUTPUT);
  ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM3, PWM3_CH);
  Motor3_control(0);
  delay(3000);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  angle_setup();
}

void loop() {

  currentT = millis();

  if (currentT - previousT_1 >= loop_time) {
    Tuning(); 
    angle_calc();
    if (abs(robot_angleX) < 8 || abs(robot_angleY) < 8) {  
      Gyro_amount = 0.996; 
    } else 
      Gyro_amount = 0.1;
    
	if (vertical) {
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0; // Convert to deg/s
      gyroY = GyY / 131.0; // Convert to deg/s
      
      int pwm_X = constrain(K1 * angleX + K2 * gyroX + K3 * motor_speed_X, -255, 255);
      int pwm_Y = constrain(K1 * angleY + K2 * gyroY + K3 * motor_speed_Y, -255, 255);
      motor_speed_X += pwm_X; 
      motor_speed_Y += pwm_Y;
      XY_to_threeWay(-pwm_X, pwm_Y);
    } else {
      XY_to_threeWay(0, 0);
      digitalWrite(BRAKE, LOW);
      motor_speed_X = 0;
      motor_speed_Y = 0;
    }
    previousT_1 = currentT;
  }
  
  if (currentT - previousT_2 >= 150) {    
    battVoltage((double)analogRead(VBAT) / 206); // need to measure
    previousT_2 = currentT;
  }
}

