#ifndef MPU_SPI_H
#define MPU_SPI_H

#include "config.h"
#include "uart.h"
#include <math.h>

#define MPU6500_PWR_MGMT_1   0x6B
#define MPU6500_USER_CTRL    0x6A
#define MPU6500_CONFIG       0x1A
#define MPU6500_GYRO_CONFIG  0x1B
#define MPU6500_GYRO_ZOUT_H  0x47
#define MPU6500_WHO_AM_I     0x75

class MPU6500_SPI {
  private:
    volatile float angleZ = 0;
    volatile float angleZ_change = 0; 
    volatile float current_omega = 0; 
    float gyroZ_offset = 0;
    uint32_t lastTime = 0;

    uint8_t spi_transfer(uint8_t data) {
        while (!(SPI2->SR & SPI_SR_TXE));
        SPI2->DR = data;
        while (!(SPI2->SR & SPI_SR_RXNE));
        volatile uint8_t dummy = SPI2->DR;
        if (SPI2->SR & SPI_SR_OVR) {
            volatile uint8_t ovr_dummy = SPI2->DR;
            (void)ovr_dummy;
        }
        return dummy;
    }

    void writeRegister(uint8_t reg, uint8_t data) {
        GPIOB->BSRR = GPIO_BSRR_BR12; 
        asm volatile("nop; nop; nop; nop;"); 
        spi_transfer(reg);
        spi_transfer(data);
        GPIOB->BSRR = GPIO_BSRR_BS12; 
    }

    uint8_t readRegister8(uint8_t reg) {
        GPIOB->BSRR = GPIO_BSRR_BR12; 
        asm volatile("nop; nop; nop; nop;"); 
        spi_transfer(reg | 0x80);
        uint8_t data = spi_transfer(0x00);
        GPIOB->BSRR = GPIO_BSRR_BS12; 
        return data;
    }

    int16_t readRegisterInt16(uint8_t reg) {
        GPIOB->BSRR = GPIO_BSRR_BR12; 
        asm volatile("nop; nop; nop; nop;"); 
        spi_transfer(reg | 0x80);
        uint8_t high = spi_transfer(0x00);
        uint8_t low = spi_transfer(0x00);
        GPIOB->BSRR = GPIO_BSRR_BS12; 
        return (int16_t)((high << 8) | low);
    }

  public:
    void begin() {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

        GPIOB->MODER |= (2 << (13 * 2)) | (2 << (14 * 2)) | (2 << (15 * 2));
        GPIOB->AFR[1] |= (5 << ((13 - 8) * 4)) | (5 << ((14 - 8) * 4)) | (5 << ((15 - 8) * 4));
        GPIOB->OSPEEDR |= (3 << (13 * 2)) | (3 << (14 * 2)) | (3 << (15 * 2)); 

        GPIOB->MODER |= (1 << (12 * 2));
        GPIOB->OSPEEDR |= (3 << (12 * 2)); 
        GPIOB->BSRR = GPIO_BSRR_BS12; 
        SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_SSM | SPI_CR1_SSI;
        SPI2->CR1 |= SPI_CR1_SPE;
        delay(150); 
        while (SPI2->SR & SPI_SR_RXNE) { volatile uint8_t dummy = SPI2->DR; (void)dummy; }

        writeRegister(MPU6500_PWR_MGMT_1, 0x80); delay(100); // 1. Reset
        writeRegister(MPU6500_USER_CTRL, 0x10); delay(10);   // 2. Ép dùng SPI
        writeRegister(MPU6500_PWR_MGMT_1, 0x01); delay(100); // 3. Auto clock
        writeRegister(MPU6500_GYRO_CONFIG, 0x18);            // 4. 2000 dps
        writeRegister(MPU6500_CONFIG, 0x03);                 // 5. LPF
        
        Serial.print("Kiem tra ID 5 lan: ");
        for(int i=0; i<5; i++){
            uint8_t id = readRegister8(MPU6500_WHO_AM_I);
            Serial.print(id); Serial.print(" ");
            delay(10);
        }
       
        uint8_t whoAmI = readRegister8(MPU6500_WHO_AM_I);
        if (whoAmI == 112 || whoAmI == 113) { 
            Serial.println(">>> MPU6500 SAN SANG! <<<");
        } else {
            Serial.print(">>> LOI: GIAO TIEP SPI SAI PHA, ID CUOI CUNG = ");
            Serial.println(whoAmI);
        }

        lastTime = micros();
    }

    void calibrateWithLEDs() {
        long sum = 0;
        for (int i = 0; i < 500; i++) {
            sum += readRegisterInt16(MPU6500_GYRO_ZOUT_H);
            
            GPIOC->BSRR = GPIO_BSRR_BS13;
            GPIOA->BSRR = GPIO_BSRR_BR7;
            GPIOB->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BR1;
            
            int ledIndex = (i / 50) % 4; 
            if (ledIndex == 0) GPIOC->BSRR = GPIO_BSRR_BR13; 
            else if (ledIndex == 1) GPIOA->BSRR = GPIO_BSRR_BS7;  
            else if (ledIndex == 2) GPIOB->BSRR = GPIO_BSRR_BS0;  
            else if (ledIndex == 3) GPIOB->BSRR = GPIO_BSRR_BS1;  
            
            delay(2);
        }
        gyroZ_offset = (float)sum / 500.0f;
        
        GPIOC->BSRR = GPIO_BSRR_BS13;
        GPIOA->BSRR = GPIO_BSRR_BR7;
        GPIOB->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BR1;
        
        lastTime = micros(); 
    }

    void ReadMPU() {
        int16_t rawZ = readRegisterInt16(MPU6500_GYRO_ZOUT_H);
        
        uint32_t currentTime = micros();
        float dt = (currentTime - lastTime) / 1000000.0f;
        lastTime = currentTime;
        float gyroRateZ = ((float)rawZ - gyroZ_offset) / 16.4f; 
        if (fabs(gyroRateZ) < 0.5f) gyroRateZ = 0.0f;
        
        current_omega = gyroRateZ;
        
        const float GYRO_SCALE = 1.01262349865f; 
        float d_angle = (gyroRateZ * dt) * GYRO_SCALE;
        
        angleZ += d_angle;
        angleZ_change += d_angle; 
    }
    
    float GetAngleZ() { return angleZ; }
    float GetOmegaZ() { return current_omega; }
    
    float GetAndResetAngleZChange() {
        __disable_irq(); 
        float val = angleZ_change;
        angleZ_change = 0;
        __enable_irq();
        return val;
    }

    void ResetAngle() { 
        __disable_irq();
        angleZ = 0; 
        __enable_irq();
    }

    void PrintMPU() {
        ReadMPU();
        
        static unsigned long last_mpu_print_time = 0;
        if (millis() - last_mpu_print_time > 50) { 
            last_mpu_print_time = millis(); 
            
            Serial.print("Goc Z: "); 
            Serial.print(angleZ, 2);
            Serial.print(" do   |   Van toc xoay: "); 
            Serial.println(current_omega, 2);
        }
    }
};

extern MPU6500_SPI mpu;

#endif