#ifndef ADC_H
#define ADC_H

#include "stm32f4xx.h"
#include "config.h"
#include "uart.h"


#define ADC_CH_SENR  2  // PA2
#define ADC_CH_SENFR 3  // PA3
#define ADC_CH_SENFL 4  // PA4
#define ADC_CH_SENL  5  // PA5
#define ADC_CH_BAT   6  // PA6

class ADC_Sensor {
private:
    uint16_t final_val[4];  
    uint16_t battery_raw;   
    uint16_t analogRead(uint8_t channel) {
        ADC1->SQR3 = channel;                         
        ADC1->CR2 |= ADC_CR2_SWSTART;                
        while (!(ADC1->SR & ADC_SR_EOC));             
        return ADC1->DR;                             
    }

    void delay_20us() {
        for (volatile int wait = 0; wait < 600; wait++); 
    }

    int readEye(uint32_t led_set, uint32_t led_reset, uint8_t channel) {
        int ambient = analogRead(channel);
        GPIOB->BSRR = led_set; 
        delay_20us();
        
        int peak = analogRead(channel);
        
        GPIOB->BSRR = led_reset; 

        int value = peak - ambient;
        if (value < 0) {
            value = 0;
        }
        
        return value;
    }

public:
    void begin() {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
        GPIOA->MODER |= (3 << (2*2)) | (3 << (3*2)) | (3 << (4*2)) | (3 << (5*2)) | (3 << (6*2));
        GPIOB->MODER &= ~((3 << (6*2)) | (3 << (7*2)));
        GPIOB->MODER |=  ((1 << (6*2)) | (1 << (7*2)));
        GPIOB->OSPEEDR |= ((3 << (6*2)) | (3 << (7*2))); 
        GPIOB->BSRR = GPIO_BSRR_BR6 | GPIO_BSRR_BR7; 
        ADC1->CR1 = 0; 
        ADC1->CR2 = ADC_CR2_ADON; 
        ADC1->SQR1 = 0; 
     
        ADC1->SMPR2 = (7 << (3 * ADC_CH_SENR))  |
                      (7 << (3 * ADC_CH_SENFR)) |
                      (7 << (3 * ADC_CH_SENFL)) |
                      (7 << (3 * ADC_CH_SENL))  |
                      (7 << (3 * ADC_CH_BAT));
    }

    void update() {
        final_val[3] = readEye(GPIO_BSRR_BS6, GPIO_BSRR_BR6, ADC_CH_SENL);
        delay(2); 
        final_val[1] = readEye(GPIO_BSRR_BS6, GPIO_BSRR_BR6, ADC_CH_SENFR);
        delay(2); 
        final_val[2] = readEye(GPIO_BSRR_BS7, GPIO_BSRR_BR7, ADC_CH_SENFL);
        delay(2); 

        final_val[0] = readEye(GPIO_BSRR_BS7, GPIO_BSRR_BR7, ADC_CH_SENR);
        battery_raw = analogRead(ADC_CH_BAT);
    }

    uint16_t get_R()  { return final_val[0]; }
    uint16_t get_FR() { return final_val[1]; }
    uint16_t get_FL() { return final_val[2]; }
    uint16_t get_L()  { return final_val[3]; }
    uint16_t get_battery_raw() { return battery_raw; }

    void print_sensors() {
        Serial.print("L: ");  Serial.print(get_L());
        Serial.print("  |  FL: ");   Serial.print(get_FL());
        Serial.print("  |  FR: ");   Serial.print(get_FR());
        Serial.print("  |  R: ");    Serial.println(get_R());
    }
};

extern ADC_Sensor adc;

#endif