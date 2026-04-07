#include "config.h"
#include "I2C.h"
#include "sensor.h"
#include "switches.h"
#include "motors.h"
#include "encoders.h"
#include "systick.h"
#include "motion.h"
#include "maze.h"
#include "mouse.h"

int currentMode = 0;
int lastMode = -1;
bool isLocked = false;

I2C i2c;
Sensors sensor;
Encoders encoders;
Motors motors;
Motion motion;
Profile forward;
Profile rotation;
Systick systick;
Mouse mouse;
Maze maze;

void setup() {
  Serial.begin(9600);
  i2c.begin();
  sensor.enable();
  SetupMode();
  encoders.begin();
  motors.begin();   
  systick.begin();
}

void testWallFollow3Cells() {
  Serial.println("Bat dau...");
  sensor.WaitStart(); 
  blinkAllLeds();// Rút tay ra nhanh nhé để xe không rung

  sensor.enable();
  sensor.SetSteeringMode(STEERING_OFF); 
  
  // ==========================================
  // THÊM 3 DÒNG NÀY ĐỂ XÓA MỌI KÝ ỨC SAI LỆCH CỦA MPU & PID
  // ==========================================
  motion.ResetDriveSystem(); 
  motors.ResetControll();  // Reset lại bộ tích phân PID động cơ
  // ==========================================

  motion.StartMove(1000.0f , SEARCH_SPEED, 0, SEARCH_ACCELERATION);
  
  while(!motion.MoveFinished()) {
    // Không cần gọi SetSteeringMode trong này nữa, gọi 1 lần ở trên là đủ
    i2c.ReadSensor();
    i2c.ReadMPU(); 
    sensor.update(); 
    delay(2);
  }

  sensor.SetSteeringMode(STEERING_OFF);
  motors.ResetControll();               
  motion.stop();
  Serial.println("Da hoan thanh va dung im!");
}

void loop() {
  i2c.ReadMPU();
  int readMode = readSwitchMode();
  if(readMode != lastMode){
    currentMode = readMode; delay(100);
    Serial.print("Mode:   "); Serial.println(currentMode);
    showModeLed(currentMode);
    lastMode = currentMode;
  }

  if(checkButton()){
    blinkLed();
    isLocked = true;
  }
  
  if(isLocked){
    switch(currentMode){
      case 0: { 
        Serial.println("--- DEBUG: FULL SENSOR SET (L, R, FL, FR) ---");
        sensor.enable(); 
        while(isLocked) {
          i2c.ReadSensor();
          sensor.update();
          
          Serial.print("L:"); Serial.print(sensor.lss_val); 
          Serial.print(" R:"); Serial.print(sensor.rss_val);
          Serial.print(" | FL:"); Serial.print(sensor.lfs_val); 
          Serial.print(" FR:"); Serial.print(sensor.rfs_val);
          Serial.print(" | Sum:"); Serial.print(sensor.GetFrontSum());   
          Serial.print(" Diff:"); Serial.print(sensor.GetFrontDiff());
          Serial.print(" | Err:"); Serial.print(sensor.GetCrossTrackError(), 2);
          Serial.print(" Adj:"); Serial.println(sensor.GetSteerinFeedback(), 3);

          if(checkButton()) { isLocked = false; blinkLed(); break; }
          delay(50);
        }
        sensor.disable();
        break;
      }
      
      case 1: {
        Serial.println("Mode 1: Che cam bien de quay 90");
        while(isLocked) {
          i2c.ReadSensor();
          i2c.ReadMPU();
          if(i2c.GetSenL() < 80) { 
            motion.spinTurn(90.0f, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
            while(i2c.GetSenL() < 120) { i2c.ReadSensor(); i2c.ReadMPU(); delay(2); } // Giảm delay từ 10->2 và thêm MPU
          } 
          else if(i2c.GetSenR() < 80) { 
            motion.spinTurn(-90.0f, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
            while(i2c.GetSenR() < 120) { i2c.ReadSensor(); i2c.ReadMPU(); delay(2); }
          }
          if(checkButton()) { isLocked = false; break; }
          delay(10);
        }
        break;
      }

      case 2: {
        Serial.println("Mode 2: Che cam bien de quay 180");
        while(isLocked) {
          i2c.ReadSensor();
          i2c.ReadMPU();
          if(i2c.GetSenL() < 80 || i2c.GetSenR() < 80) {
            motion.spinTurn(3600.0f, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
            while(i2c.GetSenL() < 120 || i2c.GetSenR() < 120) { i2c.ReadSensor(); i2c.ReadMPU(); delay(2); }
          }
          if(checkButton()) { isLocked = false; break; }
          delay(10);
        }
        break;
      }
        
      case 3: {
        testWallFollow3Cells() ;
        break;
      }
        
      case 4: {
        Serial.println("=== TEST CHI RE MUOT (QUET TAY LIEN TUC) ===");
        sensor.enable();
        while(isLocked) {
          i2c.ReadSensor();
          i2c.ReadMPU();
          int turn_id = -1;
          if(i2c.GetSenL() < 80) { 
            turn_id = 2;
            while(i2c.GetSenL() < 120) { i2c.ReadSensor(); i2c.ReadMPU(); delay(2); } 
            delay(200);
          } 
          else if(i2c.GetSenR() < 80) { 
            turn_id = 3;
            while(i2c.GetSenR() < 120) { i2c.ReadSensor(); i2c.ReadMPU(); delay(2); }
            delay(200);
          }

          if (turn_id != -1) {
            motion.ResetDriveSystem();
            sensor.disable();
            
            // 1. Bắt đầu rẽ mượt với tốc độ cao
            forward.setSpeed(turn_params[turn_id].speed);
            motion.turn(turn_params[turn_id].angle, turn_params[turn_id].omega, turn_params[turn_id].alpha);
            
            while(!motion.TurnFinished()) { i2c.ReadMPU(); delay(2); } 
            
            motion.StartMove(60.0f, forward.Speed(), 0, SEARCH_ACCELERATION); 
            
            while(!motion.MoveFinished()) { i2c.ReadMPU(); delay(2); }
            
            motors.stop();
            forward.setSpeed(0);
            sensor.enable();
          }

          if(checkButton()) { isLocked = false; motors.stop(); break; }
          delay(10);
        }
        break;
      }
      
      case 5:
        mouse.case1_searchToGoal();
        isLocked = false;
        break;

      case 6:
        mouse.case2_speedRun();
        isLocked = false;
        break;

      case 7: 
        mouse.case3_searchAndReturn();
        isLocked = false;
        break;   
    }
    return;
  }
}
