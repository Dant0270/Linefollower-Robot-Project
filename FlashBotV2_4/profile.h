#ifndef PROFILE_H
#define PROFILE_H

#include "config.h"

class Profile;

extern Profile forward;
extern Profile rotation;

class Profile{
  private:
   volatile uint16_t M_State = PS_IDLE;
   volatile float speed = 0;
   volatile float position = 0;
   int16_t sign = 1;
   float Acceleration = 0;
   float OneOverAcc = 1;
   float TargetSpeed = 0;
   float FinalSpeed = 0;
   float FinalPosition = 0;

  public:
   enum State : uint16_t {
    PS_IDLE = 0,
    PS_ACCELERATING = 1,
    PS_BRAKING = 2,
    PS_FINISHED = 3,
   };

   void reset(){
    noInterrupts();
    position = 0;
    speed = 0;
    TargetSpeed = 0;
    M_State = PS_IDLE;
    interrupts();
   }

   bool IsFinished(){
    return M_State == PS_FINISHED;
   }

   void start(float dist, float TopSpeed, float Final_Speed, float acceleration){
    sign = (dist < 0) ? -1 : +1;
    if(dist < 0){
      dist = - dist;
    }
    if(dist < 1.0){
      M_State = PS_FINISHED;
      return;
    }
    if(Final_Speed > TopSpeed){
      Final_Speed = TopSpeed;
    }

    position = 0;
    FinalPosition = dist;
    TargetSpeed = sign * fabsf(TopSpeed);
    FinalSpeed =  sign * fabsf(Final_Speed);
    Acceleration = fabsf(acceleration);
    if(Acceleration >= 1){
      OneOverAcc = 1.0f / Acceleration;
    } else {
      OneOverAcc = 1.0f;
    }
    M_State = PS_ACCELERATING;
   }

   void move(float dist, float TopSpeed, float Final_Speed, float acceleration){
    start(dist, TopSpeed, Final_Speed, acceleration);
    WaitUntilFinished();
   }

   void stop(){
    noInterrupts();
    TargetSpeed = 0;
    interrupts();
    finish();
   }

   void finish(){
    noInterrupts();
    speed = TargetSpeed;
    M_State= PS_FINISHED;
    interrupts();
   }

   void WaitUntilFinished(){
    while(M_State != PS_FINISHED){
      i2c.ReadMPU();
      delay(2);
    }
   }
   
   void SetState(State state){
    M_State = state;
   }

   float GetBreakingDist(){
    return fabsf(speed * speed - FinalSpeed * FinalSpeed) * 0.5 * OneOverAcc;
   }

   float Position(){
    float pos;
    noInterrupts();
    pos = position;
    interrupts();
    return pos;
   }

   float Speed(){
    float SPEED;
    noInterrupts();
    SPEED = speed;
    interrupts();
    return SPEED;
   }

   float ACCELERATION(){
    float acc;
    noInterrupts();
    acc = Acceleration;
    interrupts();
    return acc;
   }

   void setSpeed(float SPEED){
    noInterrupts();
    speed = SPEED;
    interrupts();
   }

   void setTargetSpeed(float SPEED){
    noInterrupts();
    TargetSpeed = SPEED;
    interrupts();
   }

   void AdjustPosition(float adjustment){
    noInterrupts();
    position += adjustment;
    interrupts();
   }

   void setPosition(float POSITION){
    noInterrupts();
    position = POSITION;
    interrupts();
   }

   void update(){
    if(M_State == PS_IDLE){
      return;
    }
    float DeltaV = Acceleration * LOOP_INTERVAL;
    float remaining = fabsf(FinalPosition) - fabsf(position);
    if(M_State == PS_ACCELERATING){
      if(remaining < GetBreakingDist()){
        M_State = PS_BRAKING;
        if(FinalSpeed == 0){
          TargetSpeed = sign * 15.0f;
        }else {
          TargetSpeed = FinalSpeed;
        };
      }
    }

    if(speed < TargetSpeed){
      speed += DeltaV;
      if(speed > TargetSpeed){
        speed = TargetSpeed;
      }
    }
    if(speed > TargetSpeed){
      speed -= DeltaV;
      if(speed < TargetSpeed){
        speed = TargetSpeed;
      }
    }
    position += speed * LOOP_INTERVAL;
    if(M_State != PS_FINISHED && remaining < 1.0f){
      M_State = PS_FINISHED;
      TargetSpeed = FinalSpeed;
    }
   }   
};

#endif
