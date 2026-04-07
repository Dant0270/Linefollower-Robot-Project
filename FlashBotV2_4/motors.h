#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"
#include "encoders.h"
#include "I2C.h" 

class Motors;
extern Motors motors;

class Motors{
  private: 
   bool ControllerOuputEnable = true;
   bool FeedforwardEnable = true;
   float PreviousFwdError = 0;
   float PreviousRotError = 0;
   float FwdError = 0;
   float RotError = 0;
   float Velocity = 0;
   float Omega = 0;
   
   float LeftMotorVolts = 0;
   float RightMotorVolts = 0;

   float TargetHeading = 0.0f;
   float ActualHeading = 0.0f;

  public:
   void EnableControll(){
    ControllerOuputEnable = true;
   }

   void DisableControll(){
    ControllerOuputEnable = false;
   }

  void ResetControll(){
    PreviousFwdError = 0;
    PreviousRotError = 0;
    FwdError = 0;
    RotError = 0;
    ActualHeading = i2c.GetAngleZ(); 
    TargetHeading = ActualHeading; 
   }

   void stop(){
    SetLeftMotorVolts(0);
    SetRightMotorVolts(0);
   }

   void begin(){
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT); 
    pinMode(BIN2, OUTPUT);
    analogWriteFrequency(31250); 
    stop();
   }

//   float PositonControll(){
//    float increment = Velocity * LOOP_INTERVAL;
//    FwdError += increment - encoders.robot_fwd_change();
//    float diff = FwdError - PreviousFwdError;
//    PreviousFwdError = FwdError;  
//    
//    return Fwd_KP * FwdError + Fwd_KD * diff;
//   }
//
//  float AngleControll(float SteeringAdjustment){
//    float increment = Omega * LOOP_INTERVAL;
//    TargetHeading += increment;
//    
//    // --- 1. SENSOR FUSION (LAI GHÉP ENCODER + MPU) ---
//    float enc_change = encoders.robot_rot_change(); 
//    ActualHeading += enc_change; // Cộng dồn góc siêu mượt từ Encoder
//    
//    // Bộ lọc bù: Tin tưởng 98% vào Encoder để giữ êm ái, kéo 2% về MPU để chống trượt
//    ActualHeading = (ActualHeading * 0.98f) + (i2c.GetAngleZ() * 0.02f);
//    
//    // --- 2. HÒA GIẢI MPU VÀ CẢM BIẾN TƯỜNG ---
//    // Nếu xe đi thẳng (Omega ~ 0) VÀ cảm biến tường đang ép xe ra giữa (SteeringAdjustment != 0)
//    // -> Ép góc mục tiêu trôi theo góc thực tế để MPU không gồng lại cảm biến nữa.
//    if (fabs(Omega) < 0.1f && fabs(SteeringAdjustment) > 0.1f) {
//        TargetHeading = ActualHeading;
//    }
//
//    // --- 3. TÍNH TOÁN PID ---
//    RotError = TargetHeading - ActualHeading;
//    RotError = constrain(RotError, -180.0f, 180.0f);
//    
//    float diff = RotError - PreviousRotError;
//    PreviousRotError = RotError;
//    
//    if (fabs(Omega) > 0.1f || fabs(Velocity) < 1.0f) {
//        return Turn_Rot_KP * RotError + Turn_Rot_KD * diff; 
//    } else {
//        return Rot_KP * RotError + Rot_KD * diff; 
//    } 
//   }

   float PositonControll(){
    float increment = Velocity * LOOP_INTERVAL;
    FwdError += increment - encoders.robot_fwd_change();
    float diff = FwdError - PreviousFwdError;
    PreviousFwdError = FwdError;     
    if (fabs(Velocity) >= 1000.0f) {
        // Chạy Speed Run
        return HighSpeed_Fwd_KP * FwdError + HighSpeed_Fwd_KD * diff;
    } else {
        // Chạy dò đường bình thường
        return Fwd_KP * FwdError + Fwd_KD * diff;
    }
   }

   float AngleControll(float SteeringAdjustment){
    float increment = Omega * LOOP_INTERVAL;
    TargetHeading += increment;
    
    float enc_change = encoders.robot_rot_change(); 
    ActualHeading += enc_change; 
    ActualHeading = (ActualHeading * 0.98f) + (i2c.GetAngleZ() * 0.02f);
    
    if (fabs(Omega) < 0.1f && fabs(SteeringAdjustment) > 0.1f) {
        TargetHeading = ActualHeading;
    }

    RotError = TargetHeading - ActualHeading;
    RotError = constrain(RotError, -180.0f, 180.0f);
    
    float diff = RotError - PreviousRotError;
    PreviousRotError = RotError;
    
    if (fabs(Omega) > 0.1f || fabs(Velocity) < 1.0f) {
        return Turn_Rot_KP * RotError + Turn_Rot_KD * diff; 
    } 
    else if (fabs(Velocity) >= 1000.0f) {
        return HighSpeed_Rot_KP * RotError + HighSpeed_Rot_KD * diff; 
    } 
    else {
        return Rot_KP * RotError + Rot_KD * diff; 
    } 
   }

   float leftFeedFwd(float speed){
    static float oldSpeed = speed;
    float leftFF = speed * SpeedFF;
    if(speed > 0){
      leftFF += BiasFF;
    } else if(speed < 0){
      leftFF -= BiasFF;
    }
    float acc = (speed - oldSpeed) * LOOP_FREQUENCY;
    oldSpeed = speed;
    float accFF = AccFF * acc;
    leftFF += accFF;
    return leftFF;
   }

   float rightFeedFwd(float speed){
    static float oldSpeed = speed;
    float rightFF = speed * SpeedFF;
    if(speed > 0){
      rightFF += BiasFF;
    } else if(speed < 0){
      rightFF -= BiasFF;
    }
    float acc = (speed - oldSpeed) * LOOP_FREQUENCY;
    oldSpeed = speed;
    float accFF = AccFF * acc;
    rightFF += accFF;
    return rightFF;
   }

   void updateControll(float velocity, float omega, float SteeringAdjustment){
    Velocity = velocity;
    Omega = omega;

    if (fabs(Velocity) < 10.0f) {
      SteeringAdjustment = 0.0f;
    }
    
    float PosOutput = PositonControll();
    float RotOutput = AngleControll(SteeringAdjustment) + SteeringAdjustment;
    float LeftOutput = PosOutput + RotOutput;
    float RightOutput = PosOutput - RotOutput;

    float TangentSpeed = Omega * MouseRadius * RadianPerDegree;
    float LeftSpeed = Velocity - TangentSpeed;
    float RightSpeed = Velocity + TangentSpeed;
    
    float LeftFF = leftFeedFwd(LeftSpeed);
    float RightFF = rightFeedFwd(RightSpeed);

    if(FeedforwardEnable){
      LeftOutput += LeftFF;
      RightOutput += RightFF;
    }
    if(ControllerOuputEnable){
      SetRightMotorVolts(RightOutput);
      SetLeftMotorVolts(LeftOutput);
    }   
   }

   int PwmCompensated( float DesiresVolt, float BatteryVolt){
    int pwm = MotorMaxPwm * (DesiresVolt / BatteryVolt);
    return pwm;
   }

   void SetLeftMotorVolts(float Volts){
    Volts = constrain(Volts, -MaxMotorVolts, MaxMotorVolts);
    LeftMotorVolts = Volts;
    int PWM = PwmCompensated(Volts, ConstVolts);
    SetLeftMotor(PWM);
   }

   void SetRightMotorVolts(float Volts){
    Volts = constrain(Volts, -MaxMotorVolts, MaxMotorVolts);
    RightMotorVolts = Volts;
    int PWM = PwmCompensated(Volts, ConstVolts);
    SetRightMotor(PWM);
   }

   void SetLeftMotor(int pwm){
    pwm = MotorLeftPolarity * constrain(pwm, -MotorMaxPwm, MotorMaxPwm);
    if(pwm > 0){
      digitalWrite(AIN1, HIGH); 
      digitalWrite(AIN2, LOW);
      analogWrite(PWMA, pwm);
    }else if(pwm < 0){
      digitalWrite(AIN1, LOW);  
      digitalWrite(AIN2, HIGH);
      analogWrite(PWMA, -pwm);
    }else if(pwm == 0){
      digitalWrite(AIN1, HIGH);  
      digitalWrite(AIN2, HIGH);
      analogWrite(PWMA, 0);
    }
   }

   void SetRightMotor(int pwm){
    pwm = MotorRightPolarity * constrain(pwm, -MotorMaxPwm, MotorMaxPwm);
    if(pwm > 0){
      digitalWrite(BIN1, HIGH); 
      digitalWrite(BIN2, LOW);
      analogWrite(PWMB, pwm);
    }else if(pwm < 0){
      digitalWrite(BIN1, LOW);  
      digitalWrite(BIN2, HIGH);
      analogWrite(PWMB, -pwm);
    }else if(pwm == 0){
      digitalWrite(BIN1, HIGH);  
      digitalWrite(BIN2, HIGH);
      analogWrite(PWMB, 0);
    }
   }
};

#endif
