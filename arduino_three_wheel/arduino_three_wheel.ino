#include "arduino_three_wheel.h"
#include <EEPROM.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);

  TCCR1A = 0b00000001; 
  TCCR1B = 0b00001010;
  TCCR2B = 0b00000010;
  TCCR2A = 0b00000011;

  pinMode(DIR_1, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(DIR_3, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  
  Motor1_control(0);
  Motor2_control(0);
  Motor3_control(0);

  EEPROM.get(0, offsets);
  if (offsets.ID == 33) calibrated = true;
    else calibrated = false;
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
    if (vertical && calibrated) {
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0; // Convert to deg/s
      gyroY = GyY / 131.0; // Convert to deg/s
      
      int pwm_X = constrain(K1 * angleX + K2 * gyroX + K3 * motor_speed_pwmX, -255, 255);
      int pwm_Y = constrain(K1 * angleY + K2 * gyroY + K3 * motor_speed_pwmY, -255, 255);
 
      if (!calibrating) {
        motor_speed_pwmX += pwm_X; 
        motor_speed_pwmY += pwm_Y;
        XY_to_threeWay(-pwm_X, pwm_Y);
      } else {
         XY_to_threeWay(0, 0);
         digitalWrite(BRAKE, LOW);
      }
    } else {
      XY_to_threeWay(0, 0);
      digitalWrite(BRAKE, LOW);
      motor_speed_pwmX = 0;
      motor_speed_pwmY = 0;
    }
    previousT_1 = currentT;
  }
  
  if (currentT - previousT_2 >= 2000) {    
    battVoltage((double)analogRead(VBAT) / bat_divider); // kube buvo 300
    if (!calibrated && !calibrating) {
      Serial.println("first you need to calibrate the balancing point...");
    }
    previousT_2 = currentT;
  }
  
}

