#pragma once

#ifdef LEMONPEPPER

// MOTOR DRIVER L6226
#define U_PWM       PA10    // TIM2_CH4
#define V_PWM       PA9     // TIM2_CH3
#define W_PWM       PA1     // TIM2_CH2

#define MOT_A1      PA0     // TIM2_CH1
#define MOT_A2      PA10    // TIM2_CH4
#define MOT_B1      PA9     // TIM2_CH3
#define MOT_B2      PA1     // TIM2_CH2

#define MOT_EN      PB12

// ENCODER MT6835
#define ENC_A       PB4     // TIM3_CH1
#define ENC_B       PB5     // TIM3_CH2
#define ENC_Z       PB3     // TIM3_ETR

#define ENC_COPI    PA7     // SPI1_MOSI
#define ENC_CIPO    PA6     // SPI1_MISO
#define ENC_SCK     PA5     // SPI1_SCK
#define ENC_CS      PC4     

#define CAL_EN      PA4

// CURRENT SENSE
#define ISENSE_U    PA3     // VOPAMP1_P, ADC1_IN4
#define ISENSE_V    PB13    // VOPAMP3_P

/**
 * 0b00 10V/V
 * 0b01 20V/V
 * 0b10 50V/V
 * 0b11 100V/V
*/
#define ISENSEGAIN_0 PC13   // GAIN0 
#define ISENSEGAIN_1 PC14   // GAIN1

// COMMUNICATION
/** 
 * Note that you can only use one input peripheral 
 * at a time. If  you want to use CANbus you also have 
 * to remove the resistors that join step-dir to CAN H/L
 * R104 R105 to protect MCU from high common mode voltages.
*/
#define CAN_TX      PB8
#define CAN_RX      PB9

#define UART3_TX    PC10
#define UART3_RX    PC11

#define I2C3_SDA    PC11
#define I2C3_SCL    PA8

#define STEP_PIN    PC10
#define DIR_PIN     PC11

// USER CONNECTOR
/**
 * Additional timer available for encoder interface. 
 * If not using, there are 5 GPIO available.
*/
#define USER_ENCA   PB6     // TIM4_CH1
#define USER_ENCB   PB7     // TIM4_CH2

#define USER1       PC6
#define USER2       PB15
#define USER3       PB14
#define USER4       PB7
#define USER5       PB6

// MISC
#define USER_BUTTON PC15
#define LED_GOOD    PB10
#define LED_FAULT   PB11
#define MOT_VOLTAGE PA2     //ADC1_IN3

#endif