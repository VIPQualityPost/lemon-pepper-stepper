#if defined(SIMPLEFOC_STM32_CUSTOMCURRENTSENSE)

#include "utils.h"

#include "adc.h"
#include "dma.h"
#include "opamp.h"

float adcResolution = 4096.0f; // 12 bit ADC
float voltageScale = 3.3f;     // full scale voltage range of ADC
float adcSens = adcResolution * voltageScale;

volatile uint16_t adc1Result[2] = {0};
volatile uint16_t adc2Result[2] = {0};

void MX_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    __HAL_RCC_ADC12_CLK_ENABLE();
}

float _readVoltageInline(const uint8_t pin, const void *cs_params)
{
    uint32_t rawResult;
    switch (pin)
    {
    case PA3:
        rawResult = adc1Result[0]; // ADC1 CH13 -> Vopamp1 internal output
        break;

    case PB0:
        rawResult = adc2Result[0]; // ADC2 CH16 -> Vopamp2 internal output
        break;

    case PA1:
        rawResult = adc2Result[1]; // ADC2 CH18 -> Vopamp3 internal output
        break;

    case PA2:
        rawResult = adc1Result[1]; // ADC1 CH16 -> not sure what pin should represent this?
        break;

    default:
        return 0.0f;
        break;
    }

    return rawResult * ((Stm32CurrentSenseParams*)cs_params)->adc_voltage_conv;
}

float _readVoltageLowSide(const int pinA, const void *cs_params)
{
    return 0.0f;
}

void *_configureADCInline(const void *driver_params, const int pinA, const int pinB, const int pinC)
{
    _UNUSED(driver_params);

    HAL_Init();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    configureOPAMPs();

    ADC_DMA_Init(&hadc1);
    ADC_DMA_Init(&hadc2);

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1Result, 1) != HAL_OK)
    {
        SIMPLEFOC_DEBUG("DMA1 read init failed");
    }
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2Result, 2) != HAL_OK)
    {
        SIMPLEFOC_DEBUG("DMA2 read init failed");
    }

    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    Stm32CurrentSenseParams *params = new Stm32CurrentSenseParams{
        .pins = {pinA, pinB, pinC},
        .adc_voltage_conv = adcSens,
        .timer_handle = (HardwareTimer *)(HardwareTimer_Handle[get_timer_index(TIM2)]->__this)};

    return params;
}

void *_configureADCLowSide(const void *driver_params, const int pinA, const int pinB, const int pinC)
{
    _UNUSED(driver_params);
    _UNUSED(pinA);
    _UNUSED(pinB);
    _UNUSED(pinC);

    SIMPLEFOC_DEBUG("This board does not use lowside sensing. Use inline current sense instead.");
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
}

void _driverSyncLowSide(void *_driver_params, void *_cs_params)
{
    STM32DriverParams* driver_params = (STM32DriverParams*)_driver_params;
    Stm32CurrentSenseParams* cs_params = (Stm32CurrentSenseParams*)_cs_params;

    _stopTimers(driver_params->timers, 6);

    // See RM0440 pg. 1169
    // This grabs the timer handle used for ADC and sets the direction bit as upcounting (?)
    cs_params->timer_handle->getHandle()->Instance->CR1 |= TIM_CR1_DIR;

    // This sets the value of the timer to the reload value. I think this is so that an event is immediately fired
    cs_params->timer_handle->getHandle()->Instance->CNT = cs_params->timer_handle->getHandle()->Instance->ARR;

    _startTimers(driver_params->timers, 6);
}

#endif