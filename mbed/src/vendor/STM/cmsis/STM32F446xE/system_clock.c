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

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field.
                                   This value must be a multiple of 0x200. */

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

    /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
    SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
    SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif

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

#define DBG_HANDLER(n) \
void n##_Handler( void ) __attribute__( ( naked ) ); \
void n##_Handler(void) \
{ \
    while(1) __asm (" bkpt 255"); \
}

#define DBG_IRQ_HANDLER(n) \
void n##_IRQHandler( void ) __attribute__( ( naked ) ); \
void n##_IRQHandler(void) \
{ \
    while(1) __asm (" bkpt 255"); \
}


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
    while(1) __asm (" bkpt 255");
}

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

DBG_HANDLER(NMI)

//DBG_HANDLER(HardFault)
  
DBG_HANDLER(MemManage)
  
DBG_HANDLER(BusFault)

DBG_HANDLER(UsageFault)

DBG_HANDLER(SVC)

DBG_HANDLER(DebugMon)

//DBG_HANDLER(PendSV)

DBG_HANDLER(SysTick)
  
//DBG_IRQ_HANDLER(WWDG)
                  
DBG_IRQ_HANDLER(PVD)

               
DBG_IRQ_HANDLER(TAMP_STAMP)      

            
DBG_IRQ_HANDLER(RTC_WKUP)            

            
DBG_IRQ_HANDLER(FLASH)   

                  
DBG_IRQ_HANDLER(RCC)

                  
DBG_IRQ_HANDLER(EXTI0)   

                  
DBG_IRQ_HANDLER(EXTI1)   

                     
DBG_IRQ_HANDLER(EXTI2)   

                 
DBG_IRQ_HANDLER(EXTI3)   

                        
DBG_IRQ_HANDLER(EXTI4)   

                  
DBG_IRQ_HANDLER(DMA1_Stream0)         

         
DBG_IRQ_HANDLER(DMA1_Stream1)         

                  
DBG_IRQ_HANDLER(DMA1_Stream2)         

                  
DBG_IRQ_HANDLER(DMA1_Stream3)         

                 
DBG_IRQ_HANDLER(DMA1_Stream4)        

                  
DBG_IRQ_HANDLER(DMA1_Stream5)         

                  
DBG_IRQ_HANDLER(DMA1_Stream6)         

                  
DBG_IRQ_HANDLER(ADC)

               
DBG_IRQ_HANDLER(CAN1_TX)

            
DBG_IRQ_HANDLER(CAN1_RX0)            

                           
DBG_IRQ_HANDLER(CAN1_RX1)            

            
DBG_IRQ_HANDLER(CAN1_SCE)            

            
DBG_IRQ_HANDLER(EXTI9_5)

            
DBG_IRQ_HANDLER(TIM1_BRK_TIM9)      

            
DBG_IRQ_HANDLER(TIM1_UP_TIM10)      


DBG_IRQ_HANDLER(TIM1_TRG_COM_TIM11)

      
DBG_IRQ_HANDLER(TIM1_CC)

                  
DBG_IRQ_HANDLER(TIM2)      

                  
DBG_IRQ_HANDLER(TIM3)      

                  
DBG_IRQ_HANDLER(TIM4)      

                  
DBG_IRQ_HANDLER(I2C1_EV)

                     
DBG_IRQ_HANDLER(I2C1_ER)

                     
DBG_IRQ_HANDLER(I2C2_EV)

                  
DBG_IRQ_HANDLER(I2C2_ER)

                           
DBG_IRQ_HANDLER(SPI1)      

                        
DBG_IRQ_HANDLER(SPI2)      

                  
DBG_IRQ_HANDLER(USART1)

                     
DBG_IRQ_HANDLER(USART2)

                     
DBG_IRQ_HANDLER(USART3)

                  
DBG_IRQ_HANDLER(EXTI15_10)         

               
DBG_IRQ_HANDLER(RTC_Alarm)         

            
DBG_IRQ_HANDLER(OTG_FS_WKUP)   

            
DBG_IRQ_HANDLER(TIM8_BRK_TIM12)   

         
DBG_IRQ_HANDLER(TIM8_UP_TIM13)      

         
//DBG_IRQ_HANDLER(TIM8_TRG_COM_TIM14)

      
DBG_IRQ_HANDLER(TIM8_CC)

                  
DBG_IRQ_HANDLER(DMA1_Stream7)         

                     
DBG_IRQ_HANDLER(FMC)      

                     
DBG_IRQ_HANDLER(SDIO)      

                     
DBG_IRQ_HANDLER(TIM5)      

                     
DBG_IRQ_HANDLER(SPI3)      

                     
DBG_IRQ_HANDLER(UART4)   

                  
DBG_IRQ_HANDLER(UART5)   

                  
//DBG_IRQ_HANDLER(TIM6_DAC)

               
//DBG_IRQ_HANDLER(TIM7)

         
DBG_IRQ_HANDLER(DMA2_Stream0)         

               
DBG_IRQ_HANDLER(DMA2_Stream1)         

                  
DBG_IRQ_HANDLER(DMA2_Stream2)         

            
DBG_IRQ_HANDLER(DMA2_Stream3)         

            
DBG_IRQ_HANDLER(DMA2_Stream4)         


DBG_IRQ_HANDLER(CAN2_TX)

                           
DBG_IRQ_HANDLER(CAN2_RX0)            

                           
DBG_IRQ_HANDLER(CAN2_RX1)            

                           
DBG_IRQ_HANDLER(CAN2_SCE)            

                           
DBG_IRQ_HANDLER(OTG_FS)

                     
DBG_IRQ_HANDLER(DMA2_Stream5)         

                  
DBG_IRQ_HANDLER(DMA2_Stream6)         

                  
DBG_IRQ_HANDLER(DMA2_Stream7)         

                  
DBG_IRQ_HANDLER(USART6)

                        
DBG_IRQ_HANDLER(I2C3_EV)

                        
DBG_IRQ_HANDLER(I2C3_ER)

                        
DBG_IRQ_HANDLER(OTG_HS_EP1_OUT)   

               
DBG_IRQ_HANDLER(OTG_HS_EP1_IN)      

               
DBG_IRQ_HANDLER(OTG_HS_WKUP)   

            
DBG_IRQ_HANDLER(OTG_HS)

                  
DBG_IRQ_HANDLER(DCMI)      


DBG_IRQ_HANDLER(FPU)            


DBG_IRQ_HANDLER(SPI4)      


DBG_IRQ_HANDLER(SAI1)      


DBG_IRQ_HANDLER(SAI2)      

   
DBG_IRQ_HANDLER(QuadSPI)      

 
DBG_IRQ_HANDLER(CEC)      

   
DBG_IRQ_HANDLER(SPDIF_RX)      

 
DBG_IRQ_HANDLER(FMPI2C1_Event)      

   
DBG_IRQ_HANDLER(FMPI2C1_Error)      

