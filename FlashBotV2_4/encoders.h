#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>
#include "config.h"
#include <HardwareTimer.h> 

void leftA_ISR(); void leftB_ISR(); // ngắt trái
void rightA_ISR(); void rightB_ISR(); // ngắt phải

class Encoders {
  private:
    volatile float RobotDistance;
    volatile float RobotAngle;

    float FwdChange = 0; // khoảng cách thay đổi
    float RotChange = 0; // góc quay thay đổi

    int32_t LeftCounter = 0; // đếm xung trái
    int32_t RightCounter = 0; // đếm xung phải

    int32_t LeftCounterRaw = 0; // đếm xung trái
    int32_t RightCounterRaw = 0; // đếm xung phải

  public:
    Encoders(){ 
      reset();
    }

    void begin() {
      pinMode(ENCLA, INPUT_PULLUP); pinMode(ENCLB, INPUT_PULLUP);
      pinMode(ENCRA, INPUT_PULLUP); pinMode(ENCRB, INPUT_PULLUP);
      
      attachInterrupt(digitalPinToInterrupt(ENCLA), leftA_ISR, CHANGE);
      attachInterrupt(digitalPinToInterrupt(ENCLB), leftB_ISR, CHANGE);
      attachInterrupt(digitalPinToInterrupt(ENCRA), rightA_ISR, CHANGE);
      attachInterrupt(digitalPinToInterrupt(ENCRB), rightB_ISR, CHANGE);
    }

    void reset(){
      noInterrupts();
      LeftCounter = 0;
      RightCounter = 0;
      RobotDistance = 0;
      RobotAngle = 0;
      interrupts();
    }
    
    void handleLeft(bool isA) {
      bool a = (GPIOB->IDR & GPIO_IDR_IDR_5); 
      bool b = (GPIOB->IDR & GPIO_IDR_IDR_4); 
      if (isA){ 
        (a == b) ? LeftCounter ++ : LeftCounter --; 
        } else{ 
          (a != b) ? LeftCounter ++ : LeftCounter --; 
        }
    }

    void handleRight(bool isA) {
      bool a = (GPIOA->IDR & GPIO_IDR_IDR_15); 
      bool b = (GPIOB->IDR & GPIO_IDR_IDR_3);  
      if (isA){ 
        (a == b) ? RightCounter ++ : RightCounter --; 
      } else{ 
        (a != b) ? RightCounter ++ : RightCounter --; 
      }
    }

    

    void update() {
      int LeftDelta = 0;
      int RightDelta = 0;
      
      noInterrupts();
      LeftDelta = LeftCounter;
      RightDelta = RightCounter;
      LeftCounterRaw += LeftDelta ;
      RightCounterRaw += RightDelta;
      LeftCounter = 0;
      RightCounter = 0;
      interrupts();
      
      float LeftChange = LeftDelta * MmPerCountLeft;
      float RightChange = RightDelta * MmPerCountRight;
      FwdChange = (LeftChange + RightChange) * 0.5000;
      RobotDistance += FwdChange;
      RotChange = (RightChange - LeftChange) * DegPerMmDiffrence;
      RobotAngle += RotChange;      
    }

   
    float robot_distance() {
        float distance;
        noInterrupts();
        distance = RobotDistance;
        interrupts();
        return distance;
    }

    float robot_speed() {
        float speed;
        noInterrupts();
        speed = LOOP_FREQUENCY * FwdChange;
        interrupts();
        return speed;
    }

    float robot_omega() {
        float omega;
        noInterrupts();
        omega = LOOP_FREQUENCY * RotChange;
        interrupts();
        return omega;
    }

    float robot_fwd_change(){
        float val;
        noInterrupts();
        val = FwdChange;
        interrupts();
        return val;
    }

    float robot_rot_change() {
        float val;
        noInterrupts();
        val =RotChange;
        interrupts();
        return val;
    }

    float robot_angle() {
        float angle;
        noInterrupts();
        angle = RobotAngle;
        interrupts();
        return angle;
    }


     void printDebug() {
      static unsigned long t = 0;
      if (millis() - t > 20) { 
        t = millis();
        Serial.print("Left Count:  "); Serial.print(LeftCounterRaw);
        Serial.print("  Right Count:  "); Serial.print(RightCounterRaw);
        Serial.print("  Dist:  ");  Serial.print(robot_distance(), 1);
        Serial.print("  Spd:  ");   Serial.print(robot_speed(), 1);
        Serial.print("  Ang:  ");   Serial.print(robot_angle(), 1);
        Serial.print("  Omg:  ");   Serial.print(robot_omega(), 1);     
        Serial.print("  Fwd:  ");  Serial.print(robot_fwd_change(), 3);
        Serial.print("  Rot:  ");  Serial.println(robot_rot_change(), 3);
      }
    }
};  

extern Encoders encoders;

void leftA_ISR(){ 
  encoders.handleLeft(true); 
}

void leftB_ISR(){ 
  encoders.handleLeft(false); 
}

void rightA_ISR(){ 
  encoders.handleRight(true); 
}

void rightB_ISR(){ 
  encoders.handleRight(false); 
}


#endif
