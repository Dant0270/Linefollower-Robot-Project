#ifndef MOTION_H
#define MOTION_H

#include "motors.h"
#include "profile.h"

class Motion{
  public:
   void ResetDriveSystem(){
    motors.stop();
    motors.DisableControll();
    encoders.reset();
    forward.reset();
    rotation.reset();
    motors.ResetControll();
    motors.EnableControll();
   }

   void stop(){
    motors.stop();
   }

   void DisableDrive(){
    motors.DisableControll();
   }

   float position(){
    return forward.Position();
   }

   float velocity(){
    return forward.Speed();
  }

  float acceleration(){
    return forward.ACCELERATION();
  }

  void SetTargetVelocity(float velocity){
   forward.setTargetSpeed(velocity);
  }

  float angle() {
    return rotation.Position();
  }

  float omega() {
    return rotation.Speed();
  }

  float alpha() {
    return rotation.ACCELERATION();
  }

  void StartMove(float dist, float TopSpeed, float Final_Speed, float acceleration){
    forward.start(dist, TopSpeed, Final_Speed, acceleration);
  }

  bool MoveFinished(){
    return forward.IsFinished();
  }

  void move(float dist, float TopSpeed, float Final_Speed, float acceleration){
    forward.move(dist, TopSpeed, Final_Speed, acceleration);
  }

  void StartTurn(float dist, float TopSpeed, float Final_Speed, float acceleration){
    rotation.start(dist, TopSpeed, Final_Speed, acceleration);
  }

  bool TurnFinished(){
    return rotation.IsFinished();
  }

  void turn(float dist, float TopSpeed, float Final_Speed, float acceleration){
    rotation.move(dist, TopSpeed, Final_Speed, acceleration);
  }

  void update(){
    forward.update();
    rotation.update();
  }

  void SetPosition(float pos){
    forward.setPosition(pos);
  }

  void adjust_forward_position(float delta) {
    forward.AdjustPosition(delta);
  } 

  void turn(float angle, float omega, float alpha){
    rotation.reset();
    rotation.start(angle, omega, 0, alpha); 
  }

  void spinTurn(float angle, float omega, float alpha){
    forward.setTargetSpeed(0);
    while(forward.Speed() != 0){
      i2c.ReadMPU(); 
      delay(2);
    }
    
    rotation.reset();
    rotation.start(angle, omega, 0, alpha); 
    
    while(!rotation.IsFinished()){
      i2c.ReadMPU();
      delay(2);
    }

    for(int i = 0; i < 100; i++) { 
      i2c.ReadMPU();
      delay(2);
    }
  };

  void stop_at(float position) {
    float remaining = position - forward.Position();
    forward.move(remaining, forward.Speed(), 0, forward.ACCELERATION());
  }

  void stop_after(float distance) {
    forward.move(distance, forward.Speed(), 0, forward.ACCELERATION());
  }

  void wait_until_position(float position) {
    while (forward.Position() < position) {
      i2c.ReadMPU();
      delay(2);
    }
  }

  void SetAngle(float target_angle) {
        rotation.setPosition(target_angle); 
  }

  void wait_until_distance(float distance) {
    float target = forward.Position() + distance;
    wait_until_position(target);
  }   
};

extern Motion motion;

#endif
