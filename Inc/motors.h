#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"
#include "batery.h"
#include "encoders.h"
#include "mpu_spi.h"

class Motors;
extern Motors motors;

class Motors {
  private: bool ControllerOuputEnable = true;
  bool FeedforwardEnable = true;
  float PreviousFwdError = 0,
  PreviousRotError = 0;
  float FwdError = 0,
  RotError = 0;
  float Velocity = 0,
  Omega = 0;
  float LeftMotorVolts = 0,
  RightMotorVolts = 0;
  float TargetHeading = 0.0f,
  ActualHeading = 0.0f;

  public: void EnableControll() {
    ControllerOuputEnable = true;
  }
  void DisableControll() {
    ControllerOuputEnable = false;
  }

  void ResetControll() {
    PreviousFwdError = 0;
    PreviousRotError = 0;
    FwdError = 0;
    RotError = 0;
    ActualHeading = mpu.GetAngleZ();
    TargetHeading = ActualHeading;
  }

  void begin() {
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;

    GPIOA -> MODER |= (1 << (1 * 2)) | (1 << (12 * 2));
    GPIOB -> MODER |= (1 << (9 * 2)) | (1 << (10 * 2));

    GPIOA -> MODER |= (2 << (8 * 2)) | (2 << (11 * 2));
    GPIOA -> AFR[1] |= (1 << ((8 - 8) * 4)) | (1 << ((11 - 8) * 4));

    TIM1 -> PSC = 10;
    TIM1 -> ARR = 255;
    TIM1 -> CCMR1 |= (6 << 4) | TIM_CCMR1_OC1PE;
    TIM1 -> CCMR2 |= (6 << 12) | TIM_CCMR2_OC4PE;
    TIM1 -> CCER |= TIM_CCER_CC1E | TIM_CCER_CC4E;
    TIM1 -> BDTR |= TIM_BDTR_MOE;
    TIM1 -> CR1 |= TIM_CR1_CEN;

    stop();
  }

  void stop() {
    SetLeftMotor(0);
    SetRightMotor(0);
  }

  float PositonControll() {
    float increment = Velocity * LOOP_INTERVAL;
    FwdError += increment - encoders.robot_fwd_change();
    float diff = FwdError - PreviousFwdError;
    PreviousFwdError = FwdError;
    if (fabs(Velocity) >= 1000.0f) return HighSpeed_Fwd_KP * FwdError + HighSpeed_Fwd_KD * diff;
    else return Fwd_KP * FwdError + Fwd_KD * diff;
  }

  float AngleControll(float SteeringAdjustment) {
    float increment = Omega * LOOP_INTERVAL;
    TargetHeading += increment;

    ActualHeading += encoders.robot_rot_change();
    ActualHeading = (ActualHeading * 0.98f) + (mpu.GetAngleZ() * 0.02f);

    if (fabs(Omega) < 0.1f && fabs(SteeringAdjustment) > 0.1f) TargetHeading = ActualHeading;

    RotError = TargetHeading - ActualHeading;
    RotError = constrain(RotError, -180.0f, 180.0f);

    float diff = RotError - PreviousRotError;
    PreviousRotError = RotError;

    if (fabs(Omega) > 0.1f || fabs(Velocity) < 1.0f) return Turn_Rot_KP * RotError + Turn_Rot_KD * diff;
    else if (fabs(Velocity) >= 1000.0f) return HighSpeed_Rot_KP * RotError + HighSpeed_Rot_KD * diff;
    else return Rot_KP * RotError + Rot_KD * diff;
  }

  float leftFeedFwd(float speed) {
    static float oldSpeed = speed;
    float leftFF = speed * SpeedFF;
    if (speed > 0) leftFF += BiasFF;
    else if (speed < 0) leftFF -= BiasFF;
    float acc = (speed - oldSpeed) * LOOP_FREQUENCY;
    oldSpeed = speed;
    return leftFF + AccFF * acc;
  }

  float rightFeedFwd(float speed) {
    static float oldSpeed = speed;
    float rightFF = speed * SpeedFF;
    if (speed > 0) rightFF += BiasFF;
    else if (speed < 0) rightFF -= BiasFF;
    float acc = (speed - oldSpeed) * LOOP_FREQUENCY;
    oldSpeed = speed;
    return rightFF + AccFF * acc;
  }

  void updateControll(float velocity, float omega, float SteeringAdjustment) {
    Velocity = velocity;
    Omega = omega;
    if (fabs(Velocity) < 10.0f) SteeringAdjustment = 0.0f;

    float PosOutput = PositonControll();
    float RotOutput = AngleControll(SteeringAdjustment) + SteeringAdjustment;
    float LeftOutput = PosOutput - RotOutput;
    float RightOutput = PosOutput + RotOutput;

    float TangentSpeed = Omega * MouseRadius * RadianPerDegree;
    float LeftSpeed = Velocity - TangentSpeed;
    float RightSpeed = Velocity + TangentSpeed;

    if (FeedforwardEnable) {
      LeftOutput += leftFeedFwd(LeftSpeed);
      RightOutput += rightFeedFwd(RightSpeed);
    }
    if (ControllerOuputEnable) {
      SetRightMotorVolts(RightOutput);
      SetLeftMotorVolts(LeftOutput);
    }
  }

  int PwmCompensated(float DesiresVolt, float BatteryVolt) {
    return MotorMaxPwm * (DesiresVolt / BatteryVolt);
  }

  void SetLeftMotorVolts(float Volts) {
    Volts = constrain(Volts, -MaxMotorVolts, MaxMotorVolts);
    LeftMotorVolts = Volts;
    SetLeftMotor(PwmCompensated(Volts, battery.voltage()));
  }

  void SetRightMotorVolts(float Volts) {
    Volts = constrain(Volts, -MaxMotorVolts, MaxMotorVolts);
    RightMotorVolts = Volts;
    SetRightMotor(PwmCompensated(Volts, battery.voltage()));
  }

  void SetLeftMotor(int pwm) {
    pwm = MotorLeftPolarity * constrain(pwm, -255, 255);
    if (pwm > 0) {
      GPIOA -> BSRR = GPIO_BSRR_BS12 | GPIO_BSRR_BR1;
      TIM1 -> CCR4 = pwm;
    } else if (pwm < 0) {
      GPIOA -> BSRR = GPIO_BSRR_BR12 | GPIO_BSRR_BS1;
      TIM1 -> CCR4 = -pwm;
    } else {
      GPIOA -> BSRR = GPIO_BSRR_BS12 | GPIO_BSRR_BS1;
      TIM1 -> CCR4 = 0;
    }
  }

  void SetRightMotor(int pwm) {
    pwm = MotorRightPolarity * constrain(pwm, -255, 255);
    if (pwm > 0) {
      GPIOB -> BSRR = GPIO_BSRR_BS9 | GPIO_BSRR_BR10;
      TIM1 -> CCR1 = pwm;
    } else if (pwm < 0) {
      GPIOB -> BSRR = GPIO_BSRR_BR9 | GPIO_BSRR_BS10;
      TIM1 -> CCR1 = -pwm;
    } else {
      GPIOB -> BSRR = GPIO_BSRR_BS9 | GPIO_BSRR_BS10;
      TIM1 -> CCR1 = 0;
    }
  }
};

#endif