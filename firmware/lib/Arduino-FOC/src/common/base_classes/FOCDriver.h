#ifndef FOCMOTOR_H
#define FOTMOTOR_H

#include "Arduino.h"

class FOCDriver{
    public: 
        virtual int init() = 0;

        virtual void enable() = 0;

        virtual void disable() = 0;

        long pwm_frequency;
        float voltage_power_supply;
        float voltage_limit;
        bool initialized = false;
        void* params = 0;
};

#endif