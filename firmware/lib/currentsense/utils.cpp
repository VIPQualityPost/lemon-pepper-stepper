#if defined(SIMPLEFOC_STM32_CUSTOMCURRENTSENSE)

#include "utils.h"

#include "adc.h"
#include "dma.h"
#include "opamp.h"

float adcResolution = 4096.0f; // 12 bit ADC
float voltageScale = 3.3f;     // full scale voltage range of ADC
float adcSens = adcResolution * voltageScale;

volatile uint16_t adc1Result[3] = {0};
volatile uint16_t adc2Result[2] = {0};

void MX_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC12_CLK_ENABLE();

    OPAMP_GPIO_Init();
}

float _readADCVoltageInline(const int pin, const void *cs_params)
{
    // SIMPLEFOC_DEBUG("READ ADC VOLTAGE");
    uint32_t rawResult;
    switch (pin)
    {
    case PA2: 
        rawResult = adc1Result[1];
        break;

    case PA3:
        rawResult = adc1Result[0]; // ADC1 CH13 -> Vopamp1 internal output
        break;

    case PB0:
        rawResult = adc2Result[0]; // ADC2 CH16 -> Vopamp2 internal output
        break;

    case PA13:
        rawResult = adc2Result[2]; // ADC2 CH18 -> Vopamp3 internal output
        break;

    // case PA2:
    //     rawResult = adc1Result[1]; // ADC1 CH16 -> not sure what pin should represent this?
    //     break;

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
    SIMPLEFOC_DEBUG("Configure Utils ADCInline");
    _UNUSED(driver_params);

    HAL_Init();
    HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE2);
    HAL_SYSCFG_EnableVREFBUF();  
    HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    configureOPAMPs();

    ADC_DMA_Init();

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1Result, 3) != HAL_OK)
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
    SIMPLEFOC_DEBUG("CS: Sync CS to driver");
    STM32DriverParams* driver_params = (STM32DriverParams*)_driver_params;
    Stm32CurrentSenseParams* cs_params = (Stm32CurrentSenseParams*)_cs_params;

    _stopTimers(driver_params->timers, 6);

    // See RM0440 pg. 1169
    // Grab the timer handle used for ADC and sets the direction bit as upcounting
    cs_params->timer_handle->getHandle()->Instance->CR1 |= TIM_CR1_DIR;

    /** Set the value of the timer to the reload value, so that overflow event is generated immediately.
     * This is because we are using repetition counter to count every other event, skipping the underflow at 
     * valleys of PWM. We can downsample every other peak if repetition counter is increased to 3
     */
    cs_params->timer_handle->getHandle()->Instance->CNT = cs_params->timer_handle->getHandle()->Instance->ARR;

    // Set TIM_CR1_URS to 0b1 -> only update events are overflows/underflow
    cs_params->timer_handle->getHandle()->Instance->CR1 |= 0x0004;

    // Set TIM_CR2_MMS to 0b010 -> update event (overflow and underflow) mapped to tim2_trgo
    cs_params->timer_handle->getHandle()->Instance->CR2 |= 0x0020;

    _startTimers(driver_params->timers, 6);
}

#endif