#define BUZZER      27
#define VBAT        34
#define INT_LED     2

#define BRAKE       26

#define DIR2        15
#define PWM2        25
#define PWM2_CH     0

#define DIR3        5
#define PWM3        18
#define PWM3_CH     2

#define DIR1        4
#define PWM1        32
#define PWM1_CH     1

#define TIMER_BIT  8
#define BASE_FREQ  20000

#define MPU6050 0x68           // Device address
#define ACCEL_CONFIG 0x1C      // Accelerometer configuration address
#define GYRO_CONFIG 0x1B       // Gyro configuration address
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

//Sensor output scaling
#define accSens 0              // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1             // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

#define EEPROM_SIZE 32

float Gyro_amount = 0.1;    

bool vertical = false;
bool calibrating = false;
bool calibrated = false;

float K1 = 135;
float K2 = 6.0;
float K3 = 0.05;
int loop_time = 10;

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
int32_t motor_speed_X; 
int32_t motor_speed_Y;   

long currentT, previousT_1, previousT_2 = 0;
