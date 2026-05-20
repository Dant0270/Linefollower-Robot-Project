#ifndef ENCODERS_H
#define ENCODERS_H

#include "config.h"
#include "uart.h"

class Encoders {
  private: volatile float RobotDistance;
  volatile float RobotAngle;
  float FwdChange = 0;
  float RotChange = 0;
  volatile int32_t LeftCounter = 0;
  volatile int32_t RightCounter = 0;
  volatile int32_t LeftCounterRaw = 0;
  volatile int32_t RightCounterRaw = 0;

  public: Encoders() {
    reset();
  }

  void begin() {
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    GPIOA -> PUPDR |= (1 << (15 * 2));
    GPIOB -> PUPDR |= (1 << (3 * 2)) | (1 << (4 * 2)) | (1 << (5 * 2));

    SYSCFG -> EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PA;
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;
    SYSCFG -> EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;
    SYSCFG -> EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB;

    EXTI -> RTSR |= (1 << 15) | (1 << 3) | (1 << 4) | (1 << 5);
    EXTI -> FTSR |= (1 << 15) | (1 << 3) | (1 << 4) | (1 << 5);
    EXTI -> IMR |= (1 << 15) | (1 << 3) | (1 << 4) | (1 << 5);

    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
  }

  void handleLeft(bool isA) {
    bool b = (GPIOA -> IDR & (1 << 15));
    bool a = (GPIOB -> IDR & (1 << 3));
    if (isA) {
      (a == b) ? LeftCounter++ : LeftCounter--;
    } else {
      (a != b) ? LeftCounter++ : LeftCounter--;
    }
  }

  void handleRight(bool isA) {
    bool a = (GPIOB -> IDR & (1 << 4));
    bool b = (GPIOB -> IDR & (1 << 5));
    if (isA) {
      (a == b) ? RightCounter++ : RightCounter--;
    } else {
      (a != b) ? RightCounter++ : RightCounter--;
    }
  }

  void update() {
    __disable_irq();
    int LeftDelta = LeftCounter;
    int RightDelta = RightCounter;
    LeftCounterRaw += LeftDelta;
    RightCounterRaw += RightDelta;
    LeftCounter = 0;
    RightCounter = 0;
    __enable_irq();

    float LeftChange = LeftDelta * MmPerCountLeft;
    float RightChange = RightDelta * MmPerCountRight;
    FwdChange = (LeftChange + RightChange) * 0.5000f;
    RobotDistance += FwdChange;
    RotChange = (RightChange - LeftChange) * DegPerMmDiffrence;
    RobotAngle += RotChange;
  }

  long getRightCount() {
    long val;
    __disable_irq();
    val = RightCounterRaw;
    __enable_irq();
    return val;
  }
  float robot_distance() {
    float val;
    __disable_irq();
    val = RobotDistance;
    __enable_irq();
    return val;
  }
  float robot_speed() {
    float val;
    __disable_irq();
    val = LOOP_FREQUENCY * FwdChange;
    __enable_irq();
    return val;
  }
  float robot_omega() {
    float val;
    __disable_irq();
    val = LOOP_FREQUENCY * RotChange;
    __enable_irq();
    return val;
  }
  float robot_fwd_change() {
    float val;
    __disable_irq();
    val = FwdChange;
    __enable_irq();
    return val;
  }
  float robot_rot_change() {
    float val;
    __disable_irq();
    val = RotChange;
    __enable_irq();
    return val;
  }
  float robot_angle() {
    float val;
    __disable_irq();
    val = RobotAngle;
    __enable_irq();
    return val;
  }

  void reset() {
    __disable_irq();
    LeftCounter = RightCounter = RobotDistance = RobotAngle = 0;
    __enable_irq();
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

extern "C" {
  void EXTI3_IRQHandler(void) {
    if (EXTI -> PR & (1 << 3)) {
      EXTI -> PR = (1 << 3);
      encoders.handleLeft(false);
    }
  }
  void EXTI4_IRQHandler(void) {
    if (EXTI -> PR & (1 << 4)) {
      EXTI -> PR = (1 << 4);
      encoders.handleRight(true);
    }
  }
  void EXTI9_5_IRQHandler(void) {
    if (EXTI -> PR & (1 << 5)) {
      EXTI -> PR = (1 << 5);
      encoders.handleRight(false);
    }
  }
  void EXTI15_10_IRQHandler(void) {
    if (EXTI -> PR & (1 << 15)) {
      EXTI -> PR = (1 << 15);
      encoders.handleLeft(true);
    }
  }
}
#endif