#include "board_SK.h"
#include "gr_soc.h"
#include "grx_hal.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+


//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+
void systick_init(void)
{
    SystemCoreUpdateClock();
    SysTick->LOAD = (uint32_t)(SystemCoreClock/1000 - 1UL); /* set reload register */
    SysTick->VAL = 0UL;                                   /* Load the SysTick Counter Value */
    SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk);

    soc_register_nvic(SysTick_IRQn, (uint32_t)SysTick_Handler);
}

void board_led_write(bool state)
{
    if(state)
    {
        bsp_led_open(BSP_LED_NUM_0);
    }
    else
    {
        bsp_led_close(BSP_LED_NUM_0);
    }
}


#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void SysTick_Handler (void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#endif

void tusb_bsp_init(void)
{
    systick_init();
}
