#define PWM_1         3
#define DIR_1         4

#define PWM_2         10
#define DIR_2         2

#define PWM_3         9
#define DIR_3         7

#define BRAKE         8
#define BUZZER        12
#define VBAT          A7

#define MPU6050 0x68           // Device address
#define ACCEL_CONFIG 0x1C      // Accelerometer configuration address
#define GYRO_CONFIG 0x1B       // Gyro configuration address
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

//Sensor output scaling
#define accSens 0              // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1             // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s
#define Gyro_amount 0.996     

bool vertical = false;
bool calibrating = false;
bool calibrated = false;

float K1 = 135;
float K2 = 6.0;
float K3 = 0.05;
int loop_time = 11;

struct OffsetsObj {
  int ID;
  float X;
  float Y;
};

OffsetsObj offsets;

int16_t  AcX, AcY, AcZ, GyY, gyroY, GyX, gyroX;

int16_t  AcX_offset = -940; 
int16_t  AcY_offset = -200;      
int16_t  AcZ_offset = 1800;
int16_t  GyX_offset = 0;
int16_t  GyY_offset = 0;
int32_t  GyX_offset_sum = 0;
int32_t  GyY_offset_sum = 0;

float robot_angleX, robot_angleY, angleX, angleY;
float Acc_angleX, Acc_angleY;      
int32_t motor_speed_pwmX;  
int32_t motor_speed_pwmY;  

int bat_divider = 57;

long currentT, previousT_1, previousT_2 = 0;  

