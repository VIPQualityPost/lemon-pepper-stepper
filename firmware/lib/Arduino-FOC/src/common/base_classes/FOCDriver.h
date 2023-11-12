#ifndef FOCDRIVER_H
#define FOCDRIVER_H

#include "Arduino.h"
#include "../foc_utils.h"

class FOCDriver{
    public: 
        virtual int init() = 0;

        virtual void enable() = 0;

        virtual void disable() = 0;

        virtual void setPwm(float a, float b, float c) = 0;
        // virtual void setPwm(uint8_t a, uint8_t b);

        long pwm_frequency;
        float voltage_power_supply;
        float voltage_limit;
        bool initialized = false;
        void* params = 0;
};

#endif
