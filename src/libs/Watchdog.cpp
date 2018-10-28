#include "Watchdog.h"
#include "Kernel.h"

#ifndef __STM32F4__
#include <lpc17xx_wdt.h>
#else
#include <stm32f4xx.h>
#endif

#include <mri.h>

#include "gpio.h"
extern GPIO leds[];

#ifdef __STM32F4__
static WWDG_HandleTypeDef m_wdt_handle;
#endif

// TODO : comment this
// Basically, when stuff stop answering, reset, or enter MRI mode, or something

Watchdog::Watchdog(uint32_t timeout, WDT_ACTION action)
{
#ifndef __STM32F4__
    WDT_Init(WDT_CLKSRC_IRC, (action == WDT_MRI)?WDT_MODE_INT_ONLY:WDT_MODE_RESET);
    WDT_Start(timeout);
    WDT_Feed();
    if(action == WDT_MRI) {
        // enable the interrupt
        NVIC_EnableIRQ(WDT_IRQn);
        NVIC_SetPriority(WDT_IRQn, 1);
    }
#else
    m_wdt_handle.Instance         = WWDG;
    m_wdt_handle.Init.Prescaler   = WWDG_CFR_WDGTB0 | WWDG_CFR_WDGTB1; // prescale /8
    m_wdt_handle.Init.Window      = WWDG_CFR_W; // load max values, still timeout ~ 100ms
    m_wdt_handle.Init.Counter     = WWDG_CR_T;  // TODO rewrite to use IWDG for longer timeouts
    //m_wdt_handle.Init.EWIMode     = (action == WDT_MRI) ? WWDG_SR_EWIF : 0;

    HAL_WWDG_Init(&m_wdt_handle);
    feed();
    if(action == WDT_MRI) {
        // enable the interrupt
        NVIC_EnableIRQ(WWDG_IRQn);
        NVIC_SetPriority(WWDG_IRQn, 1);
    }
#endif
}

void Watchdog::feed()
{
#ifndef __STM32F4__
    WDT_Feed();
#else
    HAL_WWDG_Refresh(&m_wdt_handle, WWDG_CR_T);
#endif
}

void Watchdog::on_module_loaded()
{
    register_for_event(ON_IDLE);
    feed();
}

void Watchdog::on_idle(void*)
{
    feed();
}


// when watchdog triggers, set a led pattern and enter MRI which turns everything off into a safe state
// TODO handle when MRI is disabled
#ifndef __STM32F4__
extern "C" void WDT_IRQHandler(void)
{
    if(THEKERNEL->is_using_leds()) {
        // set led pattern to show we are in watchdog timeout
        leds[0]= 0;
        leds[1]= 1;
        leds[2]= 0;
        leds[3]= 1;
    }

    WDT_ClrTimeOutFlag(); // bootloader uses this flag to enter DFU mode
    WDT_Feed();
    __debugbreak();
}
#else
extern "C" void WWDG_IRQHandler(void)
{
    if(THEKERNEL->is_using_leds()) {
        // set led pattern to show we are in watchdog timeout
        leds[0]= 0;
        //leds[1]= 1;
        //leds[2]= 0;
        //leds[3]= 1;
    }
    HAL_WWDG_IRQHandler(&m_wdt_handle); // clears int flag
    HAL_WWDG_Refresh(&m_wdt_handle, WWDG_CR_T);
    __debugbreak();
}
#endif // __STM32F4__
