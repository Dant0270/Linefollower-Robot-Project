#ifndef CONFIG_H
#define CONFIG_H

//---------------Thông số phần cứng---------------
const float WheelDiameter = 24.665f;
const float CountPerRev = 138.0f;
const float RotationBias = 0.0f;
const float RealWidthMm = 64.25f;
const float SkidFactor = 1.2495f;
const float RobotWidthMM = (RealWidthMm * SkidFactor);
const float MouseRadius = (RobotWidthMM * 0.5f); 

const float MmPerCount = (WheelDiameter * PI) / CountPerRev;
const float MmPerCountLeft =  MmPerCount;
const float MmPerCountRight = MmPerCount;
const float DegPerMmDiffrence = (1.0f / RobotWidthMM * (180 /  PI)); 
const float RadianPerDegree =  PI / 180.0f ;
const float DegreePerRadian = 180.0f / PI;

const float LOOP_FREQUENCY = 1000.0f;
const float LOOP_INTERVAL = (1.0 / LOOP_FREQUENCY);

//---------------MODE---------------
#define SW1 PA1
#define SW2 PA2
#define SW3 PA3
#define Button PB8

//---------------LED---------------
#define LED1 PB10
#define LED2 PB1
#define LED3 PB0
#define LED4 PC13

//---------------MOTOR---------------
#define PWMA PA8
#define AIN1 PB13
#define AIN2 PB12
#define PWMB PA11
#define BIN1 PB15
#define BIN2 PB14

//---------------ENCODER---------------
#define MotorLeftPolarity (1) 
#define MotorRightPolarity (1)
#define ENCRA PA15
#define ENCRB PB3
#define ENCLA PB5
#define ENCLB PB4

//---------------SENSOR---------------
#define XSHUTL PA6
#define XSHUTR PA5
#define XSHUTFL PA7
#define XSHUTFR PA4

const float STEERING_KP = 0.09f;
const float STEERING_KD = 0.25f; 
const float STEERING_ADJUST_LIMIT = 3.5f;  

const float HighSpeed_STEERING_KP = 0.0001f;           
const float HighSpeed_STEERING_KD = 0.005f;           
const float HighSpeed_STEERING_ADJUST_LIMIT = 0.5f;
 
float WALL_TARGET_L = 119.0f; 
float WALL_TARGET_R = 119.0f; 
const float LEFT_THRESHOLD = 190.0f; 
const float RIGHT_THRESHOLD = 190.0f;
const float FRONT_THRESHOLD = 600.0f; 
const int FRONT_WALL_RELIABILITY_LIMIT  = 0.0f; 
const float FRONT_TARGET = 90.0f; 

//----------------Motors--------------
const float ConstVolts = 7.40; 
const float MaxMotorVolts = 6.0;
const int   MotorMaxPwm= 255;

const float Fwd_Km = 880.38f;
const float Fwd_Tm = 0.257f;
const float Fwd_Zeta = 1.2f;
const float Fwd_Td = Fwd_Tm;
const float Rot_Km = 1216.67f;
const float Rot_Tm = 0.279f;
const float Rot_Zeta = 1.2f;
const float Rot_Td = Rot_Tm;

const float SpeedFF = 1.0f / Fwd_Km;
const float AccFF = Fwd_Tm / Fwd_Km;
const float BiasFF = 1.530;

extern float HighSpeed_Fwd_KP = 2.5f; 
extern float HighSpeed_Fwd_KD = 5.0f; 

extern float HighSpeed_Rot_KP = 0.4f; 
extern float HighSpeed_Rot_KD = 10.0f;

const float Fwd_KP = 7.5; //5*16 * Fwd_Tm / (Fwd_Km * Fwd_Zeta * Fwd_Zeta * Fwd_Td * Fwd_Td);
const float Fwd_KD = 10.0; //5*LOOP_FREQUENCY * (8 * Fwd_Tm - Fwd_Td) / (Fwd_Km * Fwd_Td);

const float Rot_KP = 1.0f; // Lực ép mũi xe về 0 độ khi đi thẳng
const float Rot_KD = 8.0f; // Chống lắc, giữ đầu xe đầm chắc

//const float Rot_KP = 5*16 * Rot_Tm / (Rot_Km * Rot_Zeta * Rot_Zeta * Rot_Td * Rot_Td);
//const float Rot_KD = 2*LOOP_FREQUENCY * (8 * Rot_Tm - Rot_Td) / (Rot_Km * Rot_Td);

// --- CẬP NHẬT PID QUAY ĐỂ CHỐNG LẮC MPU ---
const float Turn_Rot_KP = 4.0f; 
const float Turn_Rot_KD = 10.0f; 

const int SEARCH_SPEED = 230;   
const int SEARCH_ACCELERATION = 3000;
const int SEARCH_TURN_SPEED = 230; 

//----------------------Thông số map và turn smooth---------------------- 
const float FULL_CELL = 180.0f; 
const float HALF_CELL = 90.0f;
const float BACK_WALL_TO_CENTER = 45.0f; 
const float FRONT_REFERENCE = 80.0f; 
const float EXTRA_WALL_ADJUST = 0.0f; 

// --- CẬP NHẬT THÔNG SỐ QUAY TẠI CHỖ ĐỂ BÁM SÀN TỐT ---
const float OMEGA_SPIN_TURN = 250.0f;
const float ALPHA_SPIN_TURN = 1500.0f;

struct TurnParameters {
  int turn_id;
  float speed;
  float entry_offset;
  float exit_offset;
  float angle;
  float omega;
  float alpha;
  float trigger;
};

const TurnParameters turn_params[6] = {
  {0, 330.0, 35.0, 35.0,  90.0, 800.0, 8000.0, 325.0},  
  {1, 330.0, 35.0, 35.0, -90.0, 800.0, 8000.0, 325.0},
  {2, 230.0, 63.0, 25.0, 90.0, 400.0, 3000.0, 325.0},  
  {3, 230.0, 63.0, 25.0, -90.0, 400.0, 3000.0, 325.0}, 
  {4, 230.0, 60.0, 30.0, 90.0, 400.0, 3000.0, 325.0},  
  {5, 230.0, 60.0, 30.0, -90.0, 400.0, 3000.0, 325.0}  
};

//----------------------Thuật tán----------------------
#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16

#define LOWER_X_GOAL 7
#define LOWER_Y_GOAL 7
#define UPPER_X_GOAL 8
#define UPPER_Y_GOAL 8

#define STARTING_X 0
#define STARTING_Y 0

const int TURN_SCORE = 1;
const int TILE_SCORE = 1;
const int STREAK_SCORE = 0;
const int STREAK_MULTIPLIER = 0;

const unsigned char RESET_AT_CENTER = 0;
const unsigned char STAY_AT_CENTER = 0;

#define OUT_OF_BOUNDS -2
#define NOT_YET_SET -1

// --- HỆ TỌA ĐỘ VẬT LÝ ĐÃ CHUẨN HÓA ---
const float AXIS_TO_FRONT = 52.5f;
const float BACK_TO_AXIS = 39.1f;       
const float SENSING_POSITION = 140.0f;   
const float CHECK_WALL_POS = 57.5f;     
const float TURN_POS = 127.5f;          
const float DIST_CHECK_TO_TURN = 55.0f; 

#endif
