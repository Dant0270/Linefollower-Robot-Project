#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f4xx.h"
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define constrain(amt, low, high)((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define abs(x)((x) > 0 ? (x) : -(x))

inline void DWT_Init(void) {
  CoreDebug -> DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT -> CYCCNT = 0;
  DWT -> CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
inline uint32_t micros(void) {
  return DWT -> CYCCNT / (SystemCoreClock / 1000000U);
}
inline uint32_t millis(void) {
  return micros() / 1000U;
}
inline void delay(uint32_t ms) {
  uint32_t start = micros();
  while ((micros() - start) < (ms * 1000U));
}

/**
 * @brief Unified Peripheral Clock Initialization
 * @note  Call once at startup instead of scattered clock enables
 */
inline void PeriphClock_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN | RCC_APB1ENR_PWREN | RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_SYSCFGEN;
}

const float WheelDiameter = 34.0f;
const float CountPerRev = 270.0f;
const float RotationBias = 0.0f;
const float MouseRadius = 76.935f;

const float MmPerCount = (WheelDiameter * M_PI) / CountPerRev;
const float MmPerCountLeft = MmPerCount;
const float MmPerCountRight = MmPerCount;
const float DegPerMmDiffrence = (180.0f / (2.0f * MouseRadius * M_PI));
const float RadianPerDegree = M_PI / 180.0f;
const float DegreePerRadian = 180.0f / M_PI;

const float LOOP_FREQUENCY = 1000.0f;
const float LOOP_INTERVAL = (1.0f / LOOP_FREQUENCY);

#define MotorLeftPolarity (1)
#define MotorRightPolarity (1)

//----------------Motors & PID--------------
const float MaxMotorVolts = 6.0f;
const int MotorMaxPwm = 255;

const float Fwd_Km = 590.3f;
const float Fwd_Tm = 0.272f;
const float Fwd_Zeta = 1.0f;
const float Fwd_Td = Fwd_Tm;
const float Rot_Km = 527.6f;
const float Rot_Tm = 0.128f;
const float Rot_Zeta = 1.2f;
const float Rot_Td = Rot_Tm;

const float SpeedFF = 1.0f / Fwd_Km;
const float AccFF = Fwd_Tm / Fwd_Km;
const float BiasFF = 0.991f;

const float HighSpeed_Fwd_KP = 2.5f;
const float HighSpeed_Fwd_KD = 5.0f;
const float HighSpeed_Rot_KP = 0.4f;
const float HighSpeed_Rot_KD = 10.0f;

const float Fwd_KP = 16.0f * Fwd_Tm / (Fwd_Km * Fwd_Zeta * Fwd_Zeta * Fwd_Td * Fwd_Td);
const float Fwd_KD = LOOP_FREQUENCY * (8.0f * Fwd_Tm - Fwd_Td) / (Fwd_Km * Fwd_Td);
//const float Fwd_KP = 7.5f;
//const float Fwd_KD = 10.0f;
const float Rot_KP = 16.0f * Rot_Tm / (Rot_Km * Rot_Zeta * Rot_Zeta * Rot_Td * Rot_Td);
const float Rot_KD = LOOP_FREQUENCY * (8.0f * Rot_Tm - Rot_Td) / (Rot_Km * Rot_Td);
//const float Rot_KP = 1.0f;
//const float Rot_KD = 5.0f;
const float Turn_Rot_KP = 4.0f;
const float Turn_Rot_KD = 10.0f;

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

const float R1 = 10000.0f;
const float R2 = 20000.0f;
const float Battery_Ratio = R1 / (R1 + R2);
const float Max_adc_val = 4095.0f;
const float ADC_volts = 3.2600f;
const float BATTERY_MULTIPLIER = 0.00241758242;//(ADC_volts / Max_adc_val / Battery_Ratio);

#define IR_PULSE_DELAY_US 40
#endif