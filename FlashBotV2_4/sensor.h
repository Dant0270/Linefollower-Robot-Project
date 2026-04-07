#ifndef SENSOR_H
#define SENSOR_H

#include "config.h"
#include "I2C.h"

enum SteeringMode {
  STEER_NORMAL,
  STEER_LEFT_WALL,
  STEER_RIGHT_WALL,
  STEERING_OFF,
};

const uint16_t NoStart = 0;
const uint16_t LeftStart = 1;
const uint16_t RightStart = 2;

class Sensors;
extern Sensors sensor;

class Sensors {
  private: 
    volatile float LastSteeringError = 0; 
    volatile bool Active = false;
    volatile float CrossTrackError = 0;
    volatile float SteeringAdjustment = 0;
    volatile int FrontSum = 0;
    volatile int FrontDiff = 0;
    
  public:
    volatile int lfs_val, lss_val, rss_val, rfs_val;
    volatile bool SeeFrontWall;
    volatile bool SeeLeftWall;
    volatile bool SeeRightWall;

  public:
    uint16_t SteeringMode = STEER_NORMAL;  
    
    int GetFrontSum(){ return int(FrontSum); }
    int GetFrontDiff(){ return int(FrontDiff); }
    float GetSteerinFeedback(){ return SteeringAdjustment; }
    float GetCrossTrackError(){ return CrossTrackError; }

    float Calculate_Steering_Adjustment(float current_velocity = 0.0f){
      float pTerm, dTerm, raw_adjustment, adjustment;
      
      if (fabs(current_velocity) >= 1000.0f) {
          pTerm = HighSpeed_STEERING_KP * CrossTrackError;
          dTerm = HighSpeed_STEERING_KD * (CrossTrackError - LastSteeringError);
          raw_adjustment = pTerm + dTerm;
          adjustment = constrain(raw_adjustment, -HighSpeed_STEERING_ADJUST_LIMIT, HighSpeed_STEERING_ADJUST_LIMIT);
      } else {
          pTerm = STEERING_KP * CrossTrackError;
          dTerm = STEERING_KD * (CrossTrackError - LastSteeringError);
          raw_adjustment = pTerm + dTerm;
          adjustment = constrain(raw_adjustment, -STEERING_ADJUST_LIMIT, STEERING_ADJUST_LIMIT);
      }
      
      LastSteeringError = CrossTrackError;
      SteeringAdjustment = adjustment;
      return adjustment;
    }

    void SetSteeringMode(uint16_t mode){
      LastSteeringError = CrossTrackError;
      SteeringAdjustment = 0;
      SteeringMode  = mode;
    }

    void enable(){ i2c.enable(); Active = true; }
    void disable(){ i2c.disable(); Active = false; }

   void update() {
      if(!Active) { CrossTrackError = SteeringAdjustment = 0; return; }

      lss_val = i2c.GetSenL(); rss_val = i2c.GetSenR();
      lfs_val = i2c.GetSenFL(); rfs_val = i2c.GetSenFR();

      SeeLeftWall = (lss_val < LEFT_THRESHOLD); 
      SeeRightWall = (rss_val < RIGHT_THRESHOLD);
      FrontSum = lfs_val + rfs_val; 
      FrontDiff = lfs_val - rfs_val;
      SeeFrontWall = (FrontSum < FRONT_THRESHOLD);
      
      float err = 0;

      const float SAFE_ZONE = 12.0f; 

      if(SteeringMode == STEER_NORMAL) {
        bool tooCloseLeft = SeeLeftWall && (lss_val < WALL_TARGET_L - SAFE_ZONE);
        bool tooCloseRight = SeeRightWall && (rss_val < WALL_TARGET_R - SAFE_ZONE);

        if (tooCloseLeft) err = lss_val - (WALL_TARGET_L - SAFE_ZONE); 
        else if (tooCloseRight) err = (WALL_TARGET_R - SAFE_ZONE) - rss_val;
      }

      
      
      if(FrontSum < FRONT_WALL_RELIABILITY_LIMIT) err = 0;

      static float filtered_err = 0;
      filtered_err = 0.6f * filtered_err + 0.4f * err;

      CrossTrackError = constrain(filtered_err, -20.0f, 20.0f);
      Calculate_Steering_Adjustment();      
    }

     bool occluded_left(){ return i2c.GetSenFR() > 100 && i2c.GetSenFL() < 100; }
     bool occluded_right(){ return i2c.GetSenFR() <100 && i2c.GetSenFL() > 100; }

     uint16_t WaitStart(){
      int state = 0;
      enable();
      uint16_t choice = NoStart;
      while(choice == NoStart){
        i2c.ReadSensor();
        int count = 0;
        while(occluded_left()){
          count++; delay(20); i2c.ReadSensor();
        }
        if(count > 3){ choice = LeftStart; break; }
        
        count = 0;
        while(occluded_right()){
          count++; delay(20); i2c.ReadSensor();
        }
        if(count > 3){ choice = RightStart; break; }        
      }
      disable();
      return choice;
     }
};
#endif
