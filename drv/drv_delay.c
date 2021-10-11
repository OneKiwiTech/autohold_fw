#include "drv_delay.h"
#include "proj_config.h"


extern uint32_t SystemCoreClock;

#define US_TIME    (SystemCoreClock/100000)    /* 10 uS */

void delay_us (int us){
    volatile int    i;
    while (us--) {
        for (i = 0; i < US_TIME; i++) {
        	__asm("NOP");;    /* Burn cycles. */
        	__asm("NOP");;
        	__asm("NOP");;
        }
    }
}

#if USE_RTOS > 0
void delay_init(void)
{

}

void delay_ms(uint32_t ms)
{
	delay_us(300);
}

#else
volatile uint32_t systick_counter;

void delay_init(void)
{
    SystemCoreClockUpdate();
    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        while (1)
        {
        }
    }
}

void delay_ms(uint32_t ms)
{
    systick_counter = ms;
    while (systick_counter != 0U)
    {
    }
}

void SysTick_Handler(void)
{
   if (systick_counter != 0U)
   {
       systick_counter--;
   }
}

#endif



/************************ (C) COPYRIGHT Kien Minh Co.,Ltd *****END OF FILE****/
