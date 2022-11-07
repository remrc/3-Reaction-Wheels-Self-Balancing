void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void angle_setup() {
  Wire.begin();
  delay(100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay(100);
  Serial.println("Calibrating gyros...");

  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    GyY_offset_sum += GyY;
    delay(5);
  }
  GyX_offset = GyX_offset_sum >> 10;
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyX offset value = "); Serial.println(GyX_offset);
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  delay(80);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
}

void angle_calc() {

  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);                       
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);  
  AcX = Wire.read() << 8 | Wire.read();   // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();   // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();   // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);  
  GyX = Wire.read() << 8 | Wire.read();   // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();   // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)

  AcX += AcX_offset;
  AcY += AcY_offset;
  AcZ += AcZ_offset;
  GyX -= GyX_offset;
  GyY -= GyY_offset;

  robot_angleY += GyY * loop_time / 1000 / 65.536;  
  Acc_angleY = -atan2(AcX, AcZ) * 57.2958;               // angle from acc. values    * 57.2958 (deg/rad)
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  robot_angleX += GyX * loop_time / 1000 / 65.536;  
  Acc_angleX = atan2(AcY, AcZ) * 57.2958;                // angle from acc. values    * 57.2958 (deg/rad)
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  angleX = robot_angleX - offsets.X;
  angleY = robot_angleY - offsets.Y;
  
  //Serial.print("AngleX: "); Serial.print(angleX); Serial.print(" AngleY: "); Serial.println(angleY); 
  
  if (abs(angleX) > 8 || abs(angleY) > 8) vertical = false;
  if (abs(angleX) < 0.4 && abs(angleY) < 0.4) vertical = true;
}

void XY_to_threeWay(float pwm_X, float pwm_Y) {
  
  int16_t m1 = round(0.5 * pwm_X - 0.75 * pwm_Y); 
  int16_t m2 = round(0.5 * pwm_X + 0.75 * pwm_Y);
  int16_t m3 = pwm_X;  

  Motor1_control(-m1);
  Motor2_control(-m2);
  Motor3_control(m3);
  
}

void battVoltage(double voltage) {
  //Serial.print("batt: "); Serial.println(voltage); //debug
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void Motor1_control(int sp) {
  if (sp > 0) digitalWrite(DIR_1, LOW);
    else digitalWrite(DIR_1, HIGH);
  analogWrite(PWM_1, 255 - abs(sp));
}

void Motor2_control(int sp) {
  if (sp > 0) digitalWrite(DIR_2, LOW);
    else digitalWrite(DIR_2, HIGH);
  analogWrite(PWM_2, 255 - abs(sp));
}

void Motor3_control(int sp) {
  if (sp > 0) digitalWrite(DIR_3, LOW);
    else digitalWrite(DIR_3, HIGH);
  analogWrite(PWM_3, 255 - abs(sp));
}

int Tuning() {
  if (!Serial.available())  return 0;
  delay(2);
  char param = Serial.read();               // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                 // get command byte
  Serial.flush();
  switch (param) {
    case 'p':
      if (cmd == '+')    K1 += 1;
      if (cmd == '-')    K1 -= 1;
      printValues();
      break;
    case 'i':
      if (cmd == '+')    K2 += 0.05;
      if (cmd == '-')    K2 -= 0.05;
      printValues();
      break;
    case 's':
      if (cmd == '+')    K3 += 0.005;
      if (cmd == '-')    K3 -= 0.005;
      printValues();
      break;  
    case 'b':
      if (cmd == '+')    bat_divider += 1;
      if (cmd == '-')    bat_divider -= 1;
      printValues();
      break;
    case 'c':
      if (cmd == '+' && !calibrating) {
        calibrating = true;
         Serial.println("calibrating on");
      }
      if (cmd == '-' && calibrating)  {
        Serial.print("X: "); Serial.print(robot_angleX); Serial.print(" Y: "); Serial.println(robot_angleY);
        if (abs(robot_angleX) < 15 && abs(robot_angleY) < 15) {
          offsets.ID = 33;
          offsets.X = robot_angleX;
          offsets.Y = robot_angleY;
          EEPROM.put(0, offsets);
          calibrated = true;
          calibrating = false;
          Serial.println("calibrating off");
          digitalWrite(BUZZER, HIGH);
          delay(70);
          digitalWrite(BUZZER, LOW);
        } else {
          Serial.println("The angles are wrong!!!");
          digitalWrite(BUZZER, HIGH);
          delay(50);
          digitalWrite(BUZZER, LOW);
          delay(70);
          digitalWrite(BUZZER, HIGH);
          delay(50);
          digitalWrite(BUZZER, LOW);
        }
      }
      break;                
   }
}

void printValues() {
  Serial.print("P: "); Serial.print(K1);
  Serial.print(" I: "); Serial.print(K2);
  Serial.print(" S: "); Serial.println(K3,4);
  Serial.print("Bat_divider: "); Serial.println(bat_divider);
}
