#ifndef __CMSIS_DEVICE_H__
#define __CMSIS_DEVICE_H__

#if defined(TARGET_STM32F446xE)
#include <stm32f446xx.h>
#elif defined(TARGET_STM32F407xG)
#include <stm32f407xx.h>
#else
#error Unsupported target device.
#endif

#endif // __CMSIS_DEVICE_H__

