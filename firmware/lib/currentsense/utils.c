#include "adc.h"
#include "opamp.h"
#include "dma.h"
#include "../stm32_mcu.h"
#include "../../../../drivers/hardware_specific/stm32/stm32_mcu.h"
#include "communication/SimpleFOCDebug.h"

volatile uint16_t adc1Result = {0};
volatile uint16_t adc2Result[2] = {0};

float adcSens = 3.3f * 1.440f / 4096.0f;

float _readVoltageInline(const uint8_t pin, const void *cs_params)
{
    switch (pin)
    {
    case PA3:
        return adc1Result * adcSens;
        break;

    case PB0:
        return adc2Result[0] * adcSens;
        break;

    case PA1:
        return adc2Result[1] * adcSens;
        break;

    default:
        return 0.0f;
        break;
    }
}

void *_configureADCInline(const void *driver_params, const int pinA, const int pinB, const int pinC)
{
    _UNUSED(driver_params);

    HAL_Init();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init(&hadc1);
    MX_ADC2_Init(&hadc2);
    configureOPAMPs();

    MX_DMA1_Init(&hadc1, &hdma_adc1, DMA1_Channel1, DMA_REQUEST_ADC1);
    MX_DMA1_Init(&hadc2, &hdma_adc2, DMA1_Channel2, DMA_REQUEST_ADC2);

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1Result, 1) != HAL_OK)
    {
        SIMPLEFOC_DEBUG("DMA read init failed");
    }
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2Result, 2) != HAL_OK)
    {
        SIMPLEFOC_DEBUG("DMA read init failed");
    }

    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    Stm32CurrentSenseParams *params = new Stm32CurrentSenseParams{
        .pins = {pinA, pinB, pinC},
        .adc_voltage_conv = (_ADC_VOLTAGE) / (_ADC_RESOLUTION),
        .timer_handle = (HardwareTimer *)(HardwareTimer_Handle[get_timer_index(TIM3)]->__this)};

    return params;
}

void *_configureADCInline(const void *driver_params, const int pinA, const int pinB, const int pinC)
{
    _UNUSED(driver_params);
    _UNUSED(pinA);
    _UNUSED(pinB);
    _UNUSED(pinC);

    SIMPLEFOC_DEBUG("Lemon-Pepper does not use lowside sensing. Use inline current sense instead.");
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
}