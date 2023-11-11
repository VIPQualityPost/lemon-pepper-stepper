#ifndef STEPPERDRIVER_H
#define STEPPERDRIVER_H

#include "drivers/hardware_api.h"
#include "FOCDriver.h"

class StepperDriver: public FOCDriver{
    public:
        /** 
         * Set phase voltages to the harware 
         * 
         * @param Ua phase A voltage
         * @param Ub phase B voltage
        */
        virtual void setPwm(float Ua, float Ub, float Uc=NOT_SET) = 0;
};

#endif
