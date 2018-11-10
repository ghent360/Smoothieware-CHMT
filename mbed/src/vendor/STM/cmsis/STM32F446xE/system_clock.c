/* mbed Microcontroller Library
* Copyright (c) 2006-2017 ARM Limited
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

/*
 * Source: %mbed-os%/targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F446xE/TARGET_NUCLEO_F446RE
 */


/**
   * This file configures the system clock as follows:
  *-----------------------------------------------------------------------------
  * System clock source                | 1- USE_PLL_HSE_EXTC    | 3- USE_PLL_HSI
  *                                    | (external 8 MHz clock) | (internal 16 MHz)
  *                                    | 2- USE_PLL_HSE_XTAL    |
  *                                    | (external 8 MHz xtal)  |
  *-----------------------------------------------------------------------------
  * SYSCLK(MHz)                        | 180                    | 180
  *-----------------------------------------------------------------------------
  * AHBCLK (MHz)                       | 180                    | 180
  *-----------------------------------------------------------------------------
  * APB1CLK (MHz)                      |  45                    |  45
  *-----------------------------------------------------------------------------
  * APB2CLK (MHz)                      |  90                    |  90
  *-----------------------------------------------------------------------------
  * USB capable (48 MHz precise clock) | YES                    |  YES (HSI calibration needed)
  *-----------------------------------------------------------------------------
**/

#include "stm32f4xx.h"
#include "mbed_error.h"

#define CLOCK_SOURCE USE_PLL_HSE_EXTC
// clock source is selected with CLOCK_SOURCE in json config
#define USE_PLL_HSE_EXTC 0x8 // Use external clock (ST Link MCO)
#define USE_PLL_HSE_XTAL 0x4 // Use external xtal (X3 on board - not provided by default)
#define USE_PLL_HSI      0x2 // Use HSI internal clock

//#define DEBUG_MCO        (1) // Output the MCO1/MCO2 on PA8/PC9 for debugging (0=OFF, 1=ON)


#if ( ((CLOCK_SOURCE) & USE_PLL_HSE_XTAL) || ((CLOCK_SOURCE) & USE_PLL_HSE_EXTC) )
uint8_t SetSysClock_PLL_HSE(uint8_t bypass);
#endif /* ((CLOCK_SOURCE) & USE_PLL_HSE_XTAL) || ((CLOCK_SOURCE) & USE_PLL_HSE_EXTC) */

#if ((CLOCK_SOURCE) & USE_PLL_HSI)
uint8_t SetSysClock_PLL_HSI(void);
#endif /* ((CLOCK_SOURCE) & USE_PLL_HSI) */


/**
  * @brief  Setup the microcontroller system
  *         Initialize the FPU setting, vector table location and External memory
  *         configuration.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
    /* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
#endif
    /* Reset the RCC clock configuration to the default reset state ------------*/
    /* Set HSION bit */
    RCC->CR |= (uint32_t)0x00000001;

    /* Reset CFGR register */
    RCC->CFGR = 0x00000000;

    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= (uint32_t)0xFEF6FFFF;

    /* Reset PLLCFGR register */
    RCC->PLLCFGR = 0x24003010;

    /* Reset HSEBYP bit */
    RCC->CR &= (uint32_t)0xFFFBFFFF;

    /* Disable all interrupts */
    RCC->CIR = 0x00000000;

#if defined (DATA_IN_ExtSRAM) || defined (DATA_IN_ExtSDRAM)
    SystemInit_ExtMemCtl();
#endif /* DATA_IN_ExtSRAM || DATA_IN_ExtSDRAM */

}


/**
  * @brief  Configures the System clock source, PLL Multiplier and Divider factors,
  *               AHB/APBx prescalers and Flash settings
  * @note   This function should be called only once the RCC clock configuration
  *         is reset to the default reset state (done in SystemInit() function).
  * @param  None
  * @retval None
  */

void SetSysClock(void)
{
#if ((CLOCK_SOURCE) & USE_PLL_HSE_EXTC)
    /* 1- Try to start with HSE and external clock */
    if (SetSysClock_PLL_HSE(1) == 0)
#endif
    {
#if ((CLOCK_SOURCE) & USE_PLL_HSE_XTAL)
        /* 2- If fail try to start with HSE and external xtal */
        if (SetSysClock_PLL_HSE(0) == 0)
#endif
        {
#if ((CLOCK_SOURCE) & USE_PLL_HSI)
            /* 3- If fail start with HSI clock */
            if (SetSysClock_PLL_HSI() == 0)
#endif
            {
                {
                    error("SetSysClock failed\n");
                }
            }
        }
    }

    // Output clock on MCO2 pin(PC9) for debugging purpose
#if DEBUG_MCO == 1
    HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_4);
#endif
}

#if ( ((CLOCK_SOURCE) & USE_PLL_HSE_XTAL) || ((CLOCK_SOURCE) & USE_PLL_HSE_EXTC) )
/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/******************************************************************************/
uint8_t SetSysClock_PLL_HSE(uint8_t bypass)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet. */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Enable HSE oscillator and activate PLL with HSE as source
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    if (bypass == 0) {
        RCC_OscInitStruct.HSEState          = RCC_HSE_ON; // External 8 MHz xtal on OSC_IN/OSC_OUT
    } else {
        RCC_OscInitStruct.HSEState          = RCC_HSE_BYPASS; // External 8 MHz clock on OSC_IN
    }

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;             // VCO input clock = 1 MHz (8 MHz / 8)
    RCC_OscInitStruct.PLL.PLLN = 360;           // VCO output clock = 360 MHz (1 MHz * 360)
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLCLK = 180 MHz (360 MHz / 2)
    RCC_OscInitStruct.PLL.PLLQ = 7;             //
    RCC_OscInitStruct.PLL.PLLR = 2;             //
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        return 0; // FAIL
    }

    // Activate the OverDrive to reach the 180 MHz Frequency
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        return 0; // FAIL
    }

    // Select PLLSAI output as USB clock source
    PeriphClkInitStruct.PLLSAI.PLLSAIM = 8;
    PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
    PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // 180 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  //  45 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  //  90 MHz
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        return 0; // FAIL
    }

    // Output clock on MCO1 pin(PA8) for debugging purpose
#if DEBUG_MCO == 1
    if (bypass == 0) {
        HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_2);    // 4 MHz with xtal
    } else {
        HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);    // 8 MHz with external clock (MCO)
    }
#endif

    return 1; // OK
}
#endif /* ((CLOCK_SOURCE) & USE_PLL_HSE_XTAL) || ((CLOCK_SOURCE) & USE_PLL_HSE_EXTC) */

#if ((CLOCK_SOURCE) & USE_PLL_HSI)
/******************************************************************************/
/*            PLL (clocked by HSI) used as System clock source                */
/******************************************************************************/
uint8_t SetSysClock_PLL_HSI(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet. */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Enable HSI oscillator and activate PLL with HSI as source
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSEState            = RCC_HSE_OFF;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;            // VCO input clock = 1 MHz (16 MHz / 16)
    RCC_OscInitStruct.PLL.PLLN            = 360;           // VCO output clock = 360 MHz (1 MHz * 360)
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2; // PLLCLK = 180 MHz (360 MHz / 2)
    RCC_OscInitStruct.PLL.PLLQ            = 7;             //
    RCC_OscInitStruct.PLL.PLLQ            = 6;             //
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        return 0; // FAIL
    }

    // Activate the OverDrive to reach the 180 MHz Frequency
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        return 0; // FAIL
    }

    // Select PLLSAI output as USB clock source
    PeriphClkInitStruct.PLLSAI.PLLSAIM = 8;
    PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
    PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // 180 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  //  45 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  //  90 MHz
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        return 0; // FAIL
    }

    // Output clock on MCO1 pin(PA8) for debugging purpose
#if DEBUG_MCO == 1
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1); // 16 MHz
#endif

    return 1; // OK
}
#endif /* ((CLOCK_SOURCE) & USE_PLL_HSI) */

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
void HardFault_Handler( void ) __attribute__( ( naked ) );

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr; /* Link register. */
volatile uint32_t pc; /* Program counter. */
volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    /* When the following line is hit, the variables contain the register values. */
    for( ;; );
}

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}

void NMI_Handler() { while(1);}
//void HardFault_Handler() { while(1);}
void MemManage_Handler() { while(1);}
void BusFault_Handler() { while(1);}
void UsageFault_Handler() { while(1);}
void SVC_Handler() { while(1);}
void DebugMon_Handler() { while(1);}
void SysTick_Handler() { while(1);}

void PVD_IRQHandler        () { while(1); } /* PVD through EXTI Line detection */                        
void TAMP_STAMP_IRQHandler () { while(1); } /* Tamper and TimeStamps through the EXTI line */            
void RTC_WKUP_IRQHandler   () { while(1); } /* RTC Wakeup through the EXTI line */                      

void FLASH_IRQHandler() { while(1);}
void RCC_IRQHandler() { while(1);}
void EXTI0_IRQHandler() { while(1);}
void EXTI1_IRQHandler() { while(1);}
void EXTI2_IRQHandler() { while(1);}
void EXTI3_IRQHandler() { while(1);}
void EXTI4_IRQHandler() { while(1);}
void DMA1_Stream0_IRQHandler() { while(1);}
void DMA1_Stream1_IRQHandler() { while(1);}
void DMA1_Stream2_IRQHandler() { while(1);}
void DMA1_Stream3_IRQHandler() { while(1);}
void DMA1_Stream4_IRQHandler() { while(1);}
void DMA1_Stream5_IRQHandler() { while(1);}
void DMA1_Stream6_IRQHandler() { while(1);}
void ADC_IRQHandler() { while(1);}
void CAN1_TX_IRQHandler() { while(1);}
void CAN1_RX0_IRQHandler() { while(1);}
void CAN1_RX1_IRQHandler() { while(1);}
void CAN1_SCE_IRQHandler() { while(1);}

void EXTI9_5_IRQHandler           () { while(1); } /* External Line[9:5]s          */                          
void TIM1_BRK_TIM9_IRQHandler     () { while(1); } /* TIM1 Break and TIM9          */         
void TIM1_UP_TIM10_IRQHandler     () { while(1); } /* TIM1 Update and TIM10        */         
void TIM1_TRG_COM_TIM11_IRQHandler() { while(1); } /* TIM1 Trigger and Commutation and TIM11 */
void TIM1_CC_IRQHandler           () { while(1); } /* TIM1 Capture Compare         */                          
void TIM2_IRQHandler              () { while(1); } /* TIM2                         */                   
void TIM3_IRQHandler              () { while(1); } /* TIM3                         */                   
void TIM4_IRQHandler              () { while(1); } /* TIM4                         */                   
void I2C1_EV_IRQHandler           () { while(1); } /* I2C1 Event                   */                          
void I2C1_ER_IRQHandler           () { while(1); } /* I2C1 Error                   */                          
void I2C2_EV_IRQHandler           () { while(1); } /* I2C2 Event                   */                          
void I2C2_ER_IRQHandler           () { while(1); } /* I2C2 Error                   */                            
void SPI1_IRQHandler              () { while(1); } /* SPI1                         */                   
void SPI2_IRQHandler              () { while(1); } /* SPI2                         */                   
void USART1_IRQHandler            () { while(1); } /* USART1                       */                   
void USART2_IRQHandler            () { while(1); } /* USART2                       */                   
void USART3_IRQHandler            () { while(1); } /* USART3                       */                   
void EXTI15_10_IRQHandler         () { while(1); } /* External Line[15:10]s        */                          
void RTC_Alarm_IRQHandler         () { while(1); } /* RTC Alarm (A and B) through EXTI Line */                 
void OTG_FS_WKUP_IRQHandler       () { while(1); } /* USB OTG FS Wakeup through EXTI line */                       
void TIM8_BRK_TIM12_IRQHandler    () { while(1); } /* TIM8 Break and TIM12         */         
void TIM8_UP_TIM13_IRQHandler     () { while(1); } /* TIM8 Update and TIM13        */         
void TIM8_CC_IRQHandler           () { while(1); } /* TIM8 Capture Compare         */                          
void DMA1_Stream7_IRQHandler      () { while(1); } /* DMA1 Stream7                 */                          
void FMC_IRQHandler               () { while(1); } /* FMC                          */                   
void SDIO_IRQHandler              () { while(1); } /* SDIO                         */                   
void TIM5_IRQHandler              () { while(1); } /* TIM5                         */                   
void SPI3_IRQHandler              () { while(1); } /* SPI3                         */                   
void UART4_IRQHandler             () { while(1); } /* UART4                        */                   
void UART5_IRQHandler             () { while(1); } /* UART5                        */                   
void DMA2_Stream0_IRQHandler      () { while(1); } /* DMA2 Stream 0                */                   
void DMA2_Stream1_IRQHandler      () { while(1); } /* DMA2 Stream 1                */                   
void DMA2_Stream2_IRQHandler      () { while(1); } /* DMA2 Stream 2                */                   
void DMA2_Stream3_IRQHandler      () { while(1); } /* DMA2 Stream 3                */                   
void DMA2_Stream4_IRQHandler      () { while(1); } /* DMA2 Stream 4                */                   
void CAN2_TX_IRQHandler           () { while(1); } /* CAN2 TX                      */                          
void CAN2_RX0_IRQHandler          () { while(1); } /* CAN2 RX0                     */                          
void CAN2_RX1_IRQHandler          () { while(1); } /* CAN2 RX1                     */                          
void CAN2_SCE_IRQHandler          () { while(1); } /* CAN2 SCE                     */                          
void OTG_FS_IRQHandler            () { while(1); } /* USB OTG FS                   */                   
void DMA2_Stream5_IRQHandler      () { while(1); } /* DMA2 Stream 5                */                   
void DMA2_Stream6_IRQHandler      () { while(1); } /* DMA2 Stream 6                */                   
void DMA2_Stream7_IRQHandler      () { while(1); } /* DMA2 Stream 7                */                   
void USART6_IRQHandler            () { while(1); } /* USART6                       */                    
void I2C3_EV_IRQHandler           () { while(1); } /* I2C3 event                   */                          
void I2C3_ER_IRQHandler           () { while(1); } /* I2C3 error                   */                          
void OTG_HS_EP1_OUT_IRQHandler    () { while(1); } /* USB OTG HS End Point 1 Out   */                   
void OTG_HS_EP1_IN_IRQHandler     () { while(1); } /* USB OTG HS End Point 1 In    */                   
void OTG_HS_WKUP_IRQHandler       () { while(1); } /* USB OTG HS Wakeup through EXTI */                         
void OTG_HS_IRQHandler            () { while(1); } /* USB OTG HS                   */                   
void DCMI_IRQHandler              () { while(1); } /* DCMI                         */                   
void FPU_IRQHandler               () { while(1); } /* FPU                          */
void SPI4_IRQHandler              () { while(1); } /* SPI4                         */
void SAI1_IRQHandler              () { while(1); } /* SAI1						              */
void SAI2_IRQHandler              () { while(1); } /* SAI2                         */
void QuadSPI_IRQHandler           () { while(1); } /* QuadSPI                      */
void CEC_IRQHandler               () { while(1); } /* CEC                          */
void SPDIF_RX_IRQHandler          () { while(1); } /* SPDIF RX                     */
void FMPI2C1_Event_IRQHandler     () { while(1); } /* FMPI2C 1 Event               */
void FMPI2C1_Error_IRQHandler     () { while(1); } /* FMPI2C 1 Error               */
