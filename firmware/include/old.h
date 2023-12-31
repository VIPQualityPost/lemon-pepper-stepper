#pragma once

// MOTOR DRIVER L6226Q
#define U_PWM       PA10
#define V_PWM       PA9
#define W_PWM       PA1

#define MOT_A1      PA0
#define MOT_A2      PA10
#define MOT_B1      PA9
#define MOT_B2      PA1

#define MOT_EN      PB12

// ENCODER MT6835
#define ENC_A       PB4
#define ENC_B       PB5
#define ENC_Z       PB3

#define ENC_COPI    PA7
#define ENC_CIPO    PA6
#define ENC_SCK     PA5
#define ENC_CS      PC4

// CURRENT SENSE
#define ISENSE_U    PA3
#define ISENSE_V    PB13
#define ISENSE_W    PB0

// COMMUNICATION
#define CAN_TX      PB8
#define CAN_RX      PB9

// PCB REV CHANGES
#ifdef PCB_REV1
#define STEP_PIN    PB14
#define DIR_PIN     PB15

#define CAL_EN     PB1
#endif

#ifdef PCB_REV2
#define STEP_PIN    PC11
#define DIR_PIN     PA8

#define I2C_SDA     PC11
#define I2C_SCL     PA8

#define CAL_EN     PA4
#endif

// AUX
#define LED_GOOD    PB10
#define LED_FAULT   PB11

