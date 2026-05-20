#ifndef BATTERY_H
#define BATTERY_H

#include "adc.h"
#include "config.h"
#include "uart.h"

class Battery {
private:
    float m_voltage = 0;
    const float LPF_BETA = 0.05f;

public:
    void update() {
        uint16_t raw = adc.get_battery_raw();
        float current_v = (float)raw * BATTERY_MULTIPLIER;

        if (m_voltage == 0) m_voltage = current_v;
        else m_voltage = (m_voltage * (1.0f - LPF_BETA)) + (current_v * LPF_BETA);
    }

    float voltage() {
        return (m_voltage > 1.0f) ? m_voltage : 7.4f; 
    }

    void print_v() {
        Serial.print("Dien ap Pin: ");
        Serial.print(m_voltage, 2);
        Serial.println(" V");
    }
};

extern Battery battery;

#endif