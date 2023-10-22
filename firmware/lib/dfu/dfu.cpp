#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_rcc.h"

void perform_system_reset(void){
	__disable_irq();
	NVIC_SystemReset();
}

// https://stm32f4-discovery.net/2017/04/tutorial-jump-system-memory-software-stm32/
void jump_to_bootloader(void){
	__enable_irq();
	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = SysTick->LOAD = SysTick->VAL = 0;
	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

	const uint32_t p = (*((uint32_t *) 0x1FFF0000));
	__set_MSP( p );

	void (*SysMemBootJump)(void);
	SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1FFF0004));
	SysMemBootJump();

	while( 1 ) {}
}
