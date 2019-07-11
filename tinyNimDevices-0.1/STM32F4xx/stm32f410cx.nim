## *
## *****************************************************************************
##  @file    stm32f410cx.h
##  @author  MCD Application Team
##  @brief   CMSIS STM32F410Cx Device Peripheral Access Layer Header File.
##
##           This file contains:
##            - Data structures and the address mapping for all peripherals
##            - peripherals registers declarations and bits definition
##            - Macros to access peripheral’s registers hardware
##
## *****************************************************************************
##  @attention
##
##  <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
##
##  Redistribution and use in source and binary forms, with or without modification,
##  are permitted provided that the following conditions are met:
##    1. Redistributions of source code must retain the above copyright notice,
##       this list of conditions and the following disclaimer.
##    2. Redistributions in binary form must reproduce the above copyright notice,
##       this list of conditions and the following disclaimer in the documentation
##       and/or other materials provided with the distribution.
##    3. Neither the name of STMicroelectronics nor the names of its contributors
##       may be used to endorse or promote products derived from this software
##       without specific prior written permission.
##
##  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
##  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
##  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
##  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
##  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
##  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
##  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
##  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
##  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
##  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## *****************************************************************************
##
## * @addtogroup CMSIS_Device
##  @{
##
## * @addtogroup stm32f410cx
##  @{
##

## * @addtogroup Configuration_section_for_CMSIS
##  @{
##
## *
##  @brief Configuration of the Cortex-M4 Processor and Core Peripherals
##

const
  CM4_REV* = 0x00000001
  MPU_PRESENT* = 1
  NVIC_PRIO_BITS* = 4
  Vendor_SysTickConfig* = 0
  FPU_PRESENT* = 1

## *
##  @}
##
## * @addtogroup Peripheral_interrupt_number_definition
##  @{
##
## *
##  @brief STM32F4XX Interrupt Number Definition, according to the selected device
##         in @ref Library_configuration_section
##

type                          ## *****  Cortex-M4 Processor Exceptions Numbers ***************************************************************
  IRQn_Type* = enum
    NonMaskableInt_IRQn = -14,  ## !< 2 Non Maskable Interrupt
    MemoryManagement_IRQn = -12, ## !< 4 Cortex-M4 Memory Management Interrupt
    BusFault_IRQn = -11,        ## !< 5 Cortex-M4 Bus Fault Interrupt
    UsageFault_IRQn = -10,      ## !< 6 Cortex-M4 Usage Fault Interrupt
    SVCall_IRQn = -5,           ## !< 11 Cortex-M4 SV Call Interrupt
    DebugMonitor_IRQn = -4,     ## !< 12 Cortex-M4 Debug Monitor Interrupt
    PendSV_IRQn = -2,           ## !< 14 Cortex-M4 Pend SV Interrupt
    SysTick_IRQn = -1,          ## !< 15 Cortex-M4 System Tick Interrupt
                    ## *****  STM32 specific Interrupt Numbers *********************************************************************
    WWDG_IRQn = 0,              ## !< Window WatchDog Interrupt
    PVD_IRQn = 1,               ## !< PVD through EXTI Line detection Interrupt
    TAMP_STAMP_IRQn = 2,        ## !< Tamper and TimeStamp interrupts through the EXTI line
    RTC_WKUP_IRQn = 3,          ## !< RTC Wakeup interrupt through the EXTI line
    FLASH_IRQn = 4,             ## !< FLASH global Interrupt
    RCC_IRQn = 5,               ## !< RCC global Interrupt
    EXTI0_IRQn = 6,             ## !< EXTI Line0 Interrupt
    EXTI1_IRQn = 7,             ## !< EXTI Line1 Interrupt
    EXTI2_IRQn = 8,             ## !< EXTI Line2 Interrupt
    EXTI3_IRQn = 9,             ## !< EXTI Line3 Interrupt
    EXTI4_IRQn = 10,            ## !< EXTI Line4 Interrupt
    DMA1_Stream0_IRQn = 11,     ## !< DMA1 Stream 0 global Interrupt
    DMA1_Stream1_IRQn = 12,     ## !< DMA1 Stream 1 global Interrupt
    DMA1_Stream2_IRQn = 13,     ## !< DMA1 Stream 2 global Interrupt
    DMA1_Stream3_IRQn = 14,     ## !< DMA1 Stream 3 global Interrupt
    DMA1_Stream4_IRQn = 15,     ## !< DMA1 Stream 4 global Interrupt
    DMA1_Stream5_IRQn = 16,     ## !< DMA1 Stream 5 global Interrupt
    DMA1_Stream6_IRQn = 17,     ## !< DMA1 Stream 6 global Interrupt
    ADC_IRQn = 18,              ## !< ADC1 global Interrupts
    EXTI9_Bit5_IRQn = 23,       ## !< External Line[9:5] Interrupts
    TIM1_BRK_TIM9_IRQn = 24,    ## !< TIM1 Break interrupt and TIM9 global interrupt
    TIM1_UP_IRQn = 25,          ## !< TIM1 Update Interrupt
    TIM1_TRG_COM_TIM11_IRQn = 26, ## !< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
    TIM1_CC_IRQn = 27,          ## !< TIM1 Capture Compare Interrupt
    I2C1_EV_IRQn = 31,          ## !< I2C1 Event Interrupt
    I2C1_ER_IRQn = 32,          ## !< I2C1 Error Interrupt
    I2C2_EV_IRQn = 33,          ## !< I2C2 Event Interrupt
    I2C2_ER_IRQn = 34,          ## !< I2C2 Error Interrupt
    SPI1_IRQn = 35,             ## !< SPI1 global Interrupt
    SPI2_IRQn = 36,             ## !< SPI2 global Interrupt
    USART1_IRQn = 37,           ## !< USART1 global Interrupt
    USART2_IRQn = 38,           ## !< USART2 global Interrupt
    EXTI15_Bit10_IRQn = 40,     ## !< External Line[15:10] Interrupts
    RTC_Alarm_IRQn = 41,        ## !< RTC Alarm (A and B) through EXTI Line Interrupt
    DMA1_Stream7_IRQn = 47,     ## !< DMA1 Stream7 Interrupt
    TIM5_IRQn = 50,             ## !< TIM5 global Interrupt
    TIM6_DAC_IRQn = 54,         ## !< TIM6 global Interrupt and DAC Global Interrupt
    DMA2_Stream0_IRQn = 56,     ## !< DMA2 Stream 0 global Interrupt
    DMA2_Stream1_IRQn = 57,     ## !< DMA2 Stream 1 global Interrupt
    DMA2_Stream2_IRQn = 58,     ## !< DMA2 Stream 2 global Interrupt
    DMA2_Stream3_IRQn = 59,     ## !< DMA2 Stream 3 global Interrupt
    DMA2_Stream4_IRQn = 60,     ## !< DMA2 Stream 4 global Interrupt
    DMA2_Stream5_IRQn = 68,     ## !< DMA2 Stream 5 global interrupt
    DMA2_Stream6_IRQn = 69,     ## !< DMA2 Stream 6 global interrupt
    DMA2_Stream7_IRQn = 70,     ## !< DMA2 Stream 7 global interrupt
    USART6_IRQn = 71,           ## !< USART6 global interrupt
    RNG_IRQn = 80,              ## !< RNG global Interrupt
    FPU_IRQn = 81,              ## !< FPU global interrupt
    SPI5_IRQn = 85,             ## !< SPI5 global Interrupt
    FMPI2C1_EV_IRQn = 95,       ## !< FMPI2C1 Event Interrupt
    FMPI2C1_ER_IRQn = 96,       ## !< FMPI2C1 Error Interrupt
    LPTIM1_IRQn = 97


## *
##  @}
##

## * @addtogroup Peripheral_registers_structures
##  @{
##
## *
##  @brief Analog to Digital Converter
##

type
  ADC_TypeDef* {.bycopy.} = object
    SR*: uint32                ## !< ADC status register,                         Address offset: 0x00
    CR1*: uint32               ## !< ADC control register 1,                      Address offset: 0x04
    CR2*: uint32               ## !< ADC control register 2,                      Address offset: 0x08
    SMPR1*: uint32             ## !< ADC sample time register 1,                  Address offset: 0x0C
    SMPR2*: uint32             ## !< ADC sample time register 2,                  Address offset: 0x10
    JOFR1*: uint32             ## !< ADC injected channel data offset register 1, Address offset: 0x14
    JOFR2*: uint32             ## !< ADC injected channel data offset register 2, Address offset: 0x18
    JOFR3*: uint32             ## !< ADC injected channel data offset register 3, Address offset: 0x1C
    JOFR4*: uint32             ## !< ADC injected channel data offset register 4, Address offset: 0x20
    HTR*: uint32               ## !< ADC watchdog higher threshold register,      Address offset: 0x24
    LTR*: uint32               ## !< ADC watchdog lower threshold register,       Address offset: 0x28
    SQR1*: uint32              ## !< ADC regular sequence register 1,             Address offset: 0x2C
    SQR2*: uint32              ## !< ADC regular sequence register 2,             Address offset: 0x30
    SQR3*: uint32              ## !< ADC regular sequence register 3,             Address offset: 0x34
    JSQR*: uint32              ## !< ADC injected sequence register,              Address offset: 0x38
    JDR1*: uint32              ## !< ADC injected data register 1,                Address offset: 0x3C
    JDR2*: uint32              ## !< ADC injected data register 2,                Address offset: 0x40
    JDR3*: uint32              ## !< ADC injected data register 3,                Address offset: 0x44
    JDR4*: uint32              ## !< ADC injected data register 4,                Address offset: 0x48
    DR*: uint32                ## !< ADC regular data register,                   Address offset: 0x4C

  ADC_Common_TypeDef* {.bycopy.} = object
    CSR*: uint32               ## !< ADC Common status register,                  Address offset: ADC1 base address + 0x300
    CCR*: uint32               ## !< ADC common control register,                 Address offset: ADC1 base address + 0x304
    CDR*: uint32               ## !< ADC common regular data register for dual
               ##                              AND triple modes,                            Address offset: ADC1 base address + 0x308


## *
##  @brief CRC calculation unit
##

type
  CRC_TypeDef* {.bycopy.} = object
    DR*: uint32                ## !< CRC Data register,             Address offset: 0x00
    IDR*: uint8                ## !< CRC Independent data register, Address offset: 0x04
    RESERVED0*: uint8          ## !< Reserved, 0x05
    RESERVED1*: uint16         ## !< Reserved, 0x06
    CR*: uint32                ## !< CRC Control register,          Address offset: 0x08


## *
##  @brief Digital to Analog Converter
##

type
  DAC_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< DAC control register,                                    Address offset: 0x00
    SWTRIGR*: uint32           ## !< DAC software trigger register,                           Address offset: 0x04
    DHR12R1*: uint32           ## !< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08
    DHR12L1*: uint32           ## !< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C
    DHR8R1*: uint32            ## !< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10
    DHR12R2*: uint32           ## !< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14
    DHR12L2*: uint32           ## !< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18
    DHR8R2*: uint32            ## !< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C
    DHR12RD*: uint32           ## !< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20
    DHR12LD*: uint32           ## !< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24
    DHR8RD*: uint32            ## !< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28
    DOR1*: uint32              ## !< DAC channel1 data output register,                       Address offset: 0x2C
    DOR2*: uint32              ## !< DAC channel2 data output register,                       Address offset: 0x30
    SR*: uint32                ## !< DAC status register,                                     Address offset: 0x34


## *
##  @brief Debug MCU
##

type
  DBGMCU_TypeDef* {.bycopy.} = object
    IDCODE*: uint32            ## !< MCU device ID code,               Address offset: 0x00
    CR*: uint32                ## !< Debug MCU configuration register, Address offset: 0x04
    APB1FZ*: uint32            ## !< Debug MCU APB1 freeze register,   Address offset: 0x08
    APB2FZ*: uint32            ## !< Debug MCU APB2 freeze register,   Address offset: 0x0C


## *
##  @brief DMA Controller
##

type
  DMA_Stream_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< DMA stream x configuration register
    NDTR*: uint32              ## !< DMA stream x number of data register
    PAR*: uint32               ## !< DMA stream x peripheral address register
    M0AR*: uint32              ## !< DMA stream x memory 0 address register
    M1AR*: uint32              ## !< DMA stream x memory 1 address register
    FCR*: uint32               ## !< DMA stream x FIFO control register

  DMA_TypeDef* {.bycopy.} = object
    LISR*: uint32              ## !< DMA low interrupt status register,      Address offset: 0x00
    HISR*: uint32              ## !< DMA high interrupt status register,     Address offset: 0x04
    LIFCR*: uint32             ## !< DMA low interrupt flag clear register,  Address offset: 0x08
    HIFCR*: uint32             ## !< DMA high interrupt flag clear register, Address offset: 0x0C


## *
##  @brief External Interrupt/Event Controller
##

type
  EXTI_TypeDef* {.bycopy.} = object
    IMR*: uint32               ## !< EXTI Interrupt mask register,            Address offset: 0x00
    EMR*: uint32               ## !< EXTI Event mask register,                Address offset: 0x04
    RTSR*: uint32              ## !< EXTI Rising trigger selection register,  Address offset: 0x08
    FTSR*: uint32              ## !< EXTI Falling trigger selection register, Address offset: 0x0C
    SWIER*: uint32             ## !< EXTI Software interrupt event register,  Address offset: 0x10
    PR*: uint32                ## !< EXTI Pending register,                   Address offset: 0x14


## *
##  @brief FLASH Registers
##

type
  FLASH_TypeDef* {.bycopy.} = object
    ACR*: uint32               ## !< FLASH access control register,   Address offset: 0x00
    KEYR*: uint32              ## !< FLASH key register,              Address offset: 0x04
    OPTKEYR*: uint32           ## !< FLASH option key register,       Address offset: 0x08
    SR*: uint32                ## !< FLASH status register,           Address offset: 0x0C
    CR*: uint32                ## !< FLASH control register,          Address offset: 0x10
    OPTCR*: uint32             ## !< FLASH option control register ,  Address offset: 0x14
    OPTCR1*: uint32            ## !< FLASH option control register 1, Address offset: 0x18


## *
##  @brief General Purpose I/O
##

type
  GPIO_TypeDef* {.bycopy.} = object
    MODER*: uint32             ## !< GPIO port mode register,               Address offset: 0x00
    OTYPER*: uint32            ## !< GPIO port output type register,        Address offset: 0x04
    OSPEEDR*: uint32           ## !< GPIO port output speed register,       Address offset: 0x08
    PUPDR*: uint32             ## !< GPIO port pull-up/pull-down register,  Address offset: 0x0C
    IDR*: uint32               ## !< GPIO port input data register,         Address offset: 0x10
    ODR*: uint32               ## !< GPIO port output data register,        Address offset: 0x14
    BSRR*: uint32              ## !< GPIO port bit set/reset register,      Address offset: 0x18
    LCKR*: uint32              ## !< GPIO port configuration lock register, Address offset: 0x1C
    AFR*: array[2, uint32]      ## !< GPIO alternate function registers,     Address offset: 0x20-0x24


## *
##  @brief System configuration controller
##

type
  SYSCFG_TypeDef* {.bycopy.} = object
    MEMRMP*: uint32            ## !< SYSCFG memory remap register,                      Address offset: 0x00
    PMC*: uint32               ## !< SYSCFG peripheral mode configuration register,     Address offset: 0x04
    EXTICR*: array[4, uint32]   ## !< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14
    RESERVED*: uint32          ## !< Reserved, 0x18
    CFGR2*: uint32             ## !< SYSCFG Configuration register2,                    Address offset: 0x1C
    CMPCR*: uint32             ## !< SYSCFG Compensation cell control register,         Address offset: 0x20
    CFGR*: uint32              ## !< SYSCFG Configuration register,                     Address offset: 0x24


## *
##  @brief Inter-integrated Circuit Interface
##

type
  I2C_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< I2C Control register 1,     Address offset: 0x00
    CR2*: uint32               ## !< I2C Control register 2,     Address offset: 0x04
    OAR1*: uint32              ## !< I2C Own address register 1, Address offset: 0x08
    OAR2*: uint32              ## !< I2C Own address register 2, Address offset: 0x0C
    DR*: uint32                ## !< I2C Data register,          Address offset: 0x10
    SR1*: uint32               ## !< I2C Status register 1,      Address offset: 0x14
    SR2*: uint32               ## !< I2C Status register 2,      Address offset: 0x18
    CCR*: uint32               ## !< I2C Clock control register, Address offset: 0x1C
    TRISE*: uint32             ## !< I2C TRISE register,         Address offset: 0x20
    FLTR*: uint32              ## !< I2C FLTR register,          Address offset: 0x24


## *
##  @brief Inter-integrated Circuit Interface
##

type
  FMPI2C_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< FMPI2C Control register 1,            Address offset: 0x00
    CR2*: uint32               ## !< FMPI2C Control register 2,            Address offset: 0x04
    OAR1*: uint32              ## !< FMPI2C Own address 1 register,        Address offset: 0x08
    OAR2*: uint32              ## !< FMPI2C Own address 2 register,        Address offset: 0x0C
    TIMINGR*: uint32           ## !< FMPI2C Timing register,               Address offset: 0x10
    TIMEOUTR*: uint32          ## !< FMPI2C Timeout register,              Address offset: 0x14
    ISR*: uint32               ## !< FMPI2C Interrupt and status register, Address offset: 0x18
    ICR*: uint32               ## !< FMPI2C Interrupt clear register,      Address offset: 0x1C
    PECR*: uint32              ## !< FMPI2C PEC register,                  Address offset: 0x20
    RXDR*: uint32              ## !< FMPI2C Receive data register,         Address offset: 0x24
    TXDR*: uint32              ## !< FMPI2C Transmit data register,        Address offset: 0x28


## *
##  @brief Independent WATCHDOG
##

type
  IWDG_TypeDef* {.bycopy.} = object
    KR*: uint32                ## !< IWDG Key register,       Address offset: 0x00
    PR*: uint32                ## !< IWDG Prescaler register, Address offset: 0x04
    RLR*: uint32               ## !< IWDG Reload register,    Address offset: 0x08
    SR*: uint32                ## !< IWDG Status register,    Address offset: 0x0C


## *
##  @brief Power Control
##

type
  PWR_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< PWR power control register,        Address offset: 0x00
    CSR*: uint32               ## !< PWR power control/status register, Address offset: 0x04


## *
##  @brief Reset and Clock Control
##

type
  RCC_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< RCC clock control register,                                  Address offset: 0x00
    PLLCFGR*: uint32           ## !< RCC PLL configuration register,                              Address offset: 0x04
    CFGR*: uint32              ## !< RCC clock configuration register,                            Address offset: 0x08
    CIR*: uint32               ## !< RCC clock interrupt register,                                Address offset: 0x0C
    AHB1RSTR*: uint32          ## !< RCC AHB1 peripheral reset register,                          Address offset: 0x10
    RESERVED0*: array[3, uint32] ## !< Reserved, 0x14-0x1C
    APB1RSTR*: uint32          ## !< RCC APB1 peripheral reset register,                          Address offset: 0x20
    APB2RSTR*: uint32          ## !< RCC APB2 peripheral reset register,                          Address offset: 0x24
    RESERVED1*: array[2, uint32] ## !< Reserved, 0x28-0x2C
    AHB1ENR*: uint32           ## !< RCC AHB1 peripheral clock register,                          Address offset: 0x30
    RESERVED2*: array[3, uint32] ## !< Reserved, 0x34-0x3C
    APB1ENR*: uint32           ## !< RCC APB1 peripheral clock enable register,                   Address offset: 0x40
    APB2ENR*: uint32           ## !< RCC APB2 peripheral clock enable register,                   Address offset: 0x44
    RESERVED3*: array[2, uint32] ## !< Reserved, 0x48-0x4C
    AHB1LPENR*: uint32         ## !< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50
    RESERVED4*: array[3, uint32] ## !< Reserved, 0x54-0x5C
    APB1LPENR*: uint32         ## !< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60
    APB2LPENR*: uint32         ## !< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64
    RESERVED5*: array[2, uint32] ## !< Reserved, 0x68-0x6C
    BDCR*: uint32              ## !< RCC Backup domain control register,                          Address offset: 0x70
    CSR*: uint32               ## !< RCC clock control & status register,                         Address offset: 0x74
    RESERVED6*: array[2, uint32] ## !< Reserved, 0x78-0x7C
    SSCGR*: uint32             ## !< RCC spread spectrum clock generation register,               Address offset: 0x80
    RESERVED7*: array[2, uint32] ## !< Reserved, 0x84-0x88
    DCKCFGR*: uint32           ## !< RCC DCKCFGR configuration register,                         Address offset: 0x8C
    CKGATENR*: uint32          ## !< RCC Clocks Gated ENable Register,                           Address offset: 0x90
    DCKCFGR2*: uint32          ## !< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94


## *
##  @brief Real-Time Clock
##

type
  RTC_TypeDef* {.bycopy.} = object
    TR*: uint32                ## !< RTC time register,                                        Address offset: 0x00
    DR*: uint32                ## !< RTC date register,                                        Address offset: 0x04
    CR*: uint32                ## !< RTC control register,                                     Address offset: 0x08
    ISR*: uint32               ## !< RTC initialization and status register,                   Address offset: 0x0C
    PRER*: uint32              ## !< RTC prescaler register,                                   Address offset: 0x10
    WUTR*: uint32              ## !< RTC wakeup timer register,                                Address offset: 0x14
    CALIBR*: uint32            ## !< RTC calibration register,                                 Address offset: 0x18
    ALRMAR*: uint32            ## !< RTC alarm A register,                                     Address offset: 0x1C
    ALRMBR*: uint32            ## !< RTC alarm B register,                                     Address offset: 0x20
    WPR*: uint32               ## !< RTC write protection register,                            Address offset: 0x24
    SSR*: uint32               ## !< RTC sub second register,                                  Address offset: 0x28
    SHIFTR*: uint32            ## !< RTC shift control register,                               Address offset: 0x2C
    TSTR*: uint32              ## !< RTC time stamp time register,                             Address offset: 0x30
    TSDR*: uint32              ## !< RTC time stamp date register,                             Address offset: 0x34
    TSSSR*: uint32             ## !< RTC time-stamp sub second register,                       Address offset: 0x38
    CALR*: uint32              ## !< RTC calibration register,                                 Address offset: 0x3C
    TAFCR*: uint32             ## !< RTC tamper and alternate function configuration register, Address offset: 0x40
    ALRMASSR*: uint32          ## !< RTC alarm A sub second register,                          Address offset: 0x44
    ALRMBSSR*: uint32          ## !< RTC alarm B sub second register,                          Address offset: 0x48
    RESERVED7*: uint32         ## !< Reserved, 0x4C
    BKP0R*: uint32             ## !< RTC backup register 1,                                    Address offset: 0x50
    BKP1R*: uint32             ## !< RTC backup register 1,                                    Address offset: 0x54
    BKP2R*: uint32             ## !< RTC backup register 2,                                    Address offset: 0x58
    BKP3R*: uint32             ## !< RTC backup register 3,                                    Address offset: 0x5C
    BKP4R*: uint32             ## !< RTC backup register 4,                                    Address offset: 0x60
    BKP5R*: uint32             ## !< RTC backup register 5,                                    Address offset: 0x64
    BKP6R*: uint32             ## !< RTC backup register 6,                                    Address offset: 0x68
    BKP7R*: uint32             ## !< RTC backup register 7,                                    Address offset: 0x6C
    BKP8R*: uint32             ## !< RTC backup register 8,                                    Address offset: 0x70
    BKP9R*: uint32             ## !< RTC backup register 9,                                    Address offset: 0x74
    BKP10R*: uint32            ## !< RTC backup register 10,                                   Address offset: 0x78
    BKP11R*: uint32            ## !< RTC backup register 11,                                   Address offset: 0x7C
    BKP12R*: uint32            ## !< RTC backup register 12,                                   Address offset: 0x80
    BKP13R*: uint32            ## !< RTC backup register 13,                                   Address offset: 0x84
    BKP14R*: uint32            ## !< RTC backup register 14,                                   Address offset: 0x88
    BKP15R*: uint32            ## !< RTC backup register 15,                                   Address offset: 0x8C
    BKP16R*: uint32            ## !< RTC backup register 16,                                   Address offset: 0x90
    BKP17R*: uint32            ## !< RTC backup register 17,                                   Address offset: 0x94
    BKP18R*: uint32            ## !< RTC backup register 18,                                   Address offset: 0x98
    BKP19R*: uint32            ## !< RTC backup register 19,                                   Address offset: 0x9C


## *
##  @brief Serial Peripheral Interface
##

type
  SPI_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< SPI control register 1 (not used in I2S mode),      Address offset: 0x00
    CR2*: uint32               ## !< SPI control register 2,                             Address offset: 0x04
    SR*: uint32                ## !< SPI status register,                                Address offset: 0x08
    DR*: uint32                ## !< SPI data register,                                  Address offset: 0x0C
    CRCPR*: uint32             ## !< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10
    RXCRCR*: uint32            ## !< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14
    TXCRCR*: uint32            ## !< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18
    I2SCFGR*: uint32           ## !< SPI_I2S configuration register,                     Address offset: 0x1C
    I2SPR*: uint32             ## !< SPI_I2S prescaler register,                         Address offset: 0x20


## *
##  @brief TIM
##

type
  TIM_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< TIM control register 1,              Address offset: 0x00
    CR2*: uint32               ## !< TIM control register 2,              Address offset: 0x04
    SMCR*: uint32              ## !< TIM slave mode control register,     Address offset: 0x08
    DIER*: uint32              ## !< TIM DMA/interrupt enable register,   Address offset: 0x0C
    SR*: uint32                ## !< TIM status register,                 Address offset: 0x10
    EGR*: uint32               ## !< TIM event generation register,       Address offset: 0x14
    CCMR1*: uint32             ## !< TIM capture/compare mode register 1, Address offset: 0x18
    CCMR2*: uint32             ## !< TIM capture/compare mode register 2, Address offset: 0x1C
    CCER*: uint32              ## !< TIM capture/compare enable register, Address offset: 0x20
    CNT*: uint32               ## !< TIM counter register,                Address offset: 0x24
    PSC*: uint32               ## !< TIM prescaler,                       Address offset: 0x28
    ARR*: uint32               ## !< TIM auto-reload register,            Address offset: 0x2C
    RCR*: uint32               ## !< TIM repetition counter register,     Address offset: 0x30
    CCR1*: uint32              ## !< TIM capture/compare register 1,      Address offset: 0x34
    CCR2*: uint32              ## !< TIM capture/compare register 2,      Address offset: 0x38
    CCR3*: uint32              ## !< TIM capture/compare register 3,      Address offset: 0x3C
    CCR4*: uint32              ## !< TIM capture/compare register 4,      Address offset: 0x40
    BDTR*: uint32              ## !< TIM break and dead-time register,    Address offset: 0x44
    DCR*: uint32               ## !< TIM DMA control register,            Address offset: 0x48
    DMAR*: uint32              ## !< TIM DMA address for full transfer,   Address offset: 0x4C
    OR*: uint32                ## !< TIM option register,                 Address offset: 0x50


## *
##  @brief Universal Synchronous Asynchronous Receiver Transmitter
##

type
  USART_TypeDef* {.bycopy.} = object
    SR*: uint32                ## !< USART Status register,                   Address offset: 0x00
    DR*: uint32                ## !< USART Data register,                     Address offset: 0x04
    BRR*: uint32               ## !< USART Baud rate register,                Address offset: 0x08
    CR1*: uint32               ## !< USART Control register 1,                Address offset: 0x0C
    CR2*: uint32               ## !< USART Control register 2,                Address offset: 0x10
    CR3*: uint32               ## !< USART Control register 3,                Address offset: 0x14
    GTPR*: uint32              ## !< USART Guard time and prescaler register, Address offset: 0x18


## *
##  @brief Window WATCHDOG
##

type
  WWDG_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< WWDG Control register,       Address offset: 0x00
    CFR*: uint32               ## !< WWDG Configuration register, Address offset: 0x04
    SR*: uint32                ## !< WWDG Status register,        Address offset: 0x08


## *
##  @brief RNG
##

type
  RNG_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< RNG control register, Address offset: 0x00
    SR*: uint32                ## !< RNG status register,  Address offset: 0x04
    DR*: uint32                ## !< RNG data register,    Address offset: 0x08


## *
##  @brief LPTIMER
##

type
  LPTIM_TypeDef* {.bycopy.} = object
    ISR*: uint32               ## !< LPTIM Interrupt and Status register,                Address offset: 0x00
    ICR*: uint32               ## !< LPTIM Interrupt Clear register,                     Address offset: 0x04
    IER*: uint32               ## !< LPTIM Interrupt Enable register,                    Address offset: 0x08
    CFGR*: uint32              ## !< LPTIM Configuration register,                       Address offset: 0x0C
    CR*: uint32                ## !< LPTIM Control register,                             Address offset: 0x10
    CMP*: uint32               ## !< LPTIM Compare register,                             Address offset: 0x14
    ARR*: uint32               ## !< LPTIM Autoreload register,                          Address offset: 0x18
    CNT*: uint32               ## !< LPTIM Counter register,                             Address offset: 0x1C
    OR*: uint32                ## !< LPTIM Option register,                              Address offset: 0x20


## *
##  @}
##
## * @addtogroup Peripheral_memory_map
##  @{
##

const
  FLASH_BASE* = 0x08000000
  SRAM1_BASE* = 0x20000000
  PERIPH_BASE* = 0x40000000
  SRAM1_BB_BASE* = 0x22000000
  PERIPH_BB_BASE* = 0x42000000
  FLASH_END* = 0x0801FFFF
  FLASH_OTP_BASE* = 0x1FFF7800
  FLASH_OTP_END* = 0x1FFF7A0F

##  Legacy defines

const
  SRAM_BASE* = SRAM1_BASE
  SRAM_BB_BASE* = SRAM1_BB_BASE

## !< Peripheral memory map

const
  APB1PERIPH_BASE* = PERIPH_BASE
  APB2PERIPH_BASE* = (PERIPH_BASE + 0x00010000)
  AHB1PERIPH_BASE* = (PERIPH_BASE + 0x00020000)

## !< APB1 peripherals

const
  TIM5_BASE* = (APB1PERIPH_BASE + 0x00000C00)
  TIM6_BASE* = (APB1PERIPH_BASE + 0x00001000)
  LPTIM1_BASE* = (APB1PERIPH_BASE + 0x00002400)
  RTC_BASE* = (APB1PERIPH_BASE + 0x00002800)
  WWDG_BASE* = (APB1PERIPH_BASE + 0x00002C00)
  IWDG_BASE* = (APB1PERIPH_BASE + 0x00003000)
  I2S2ext_BASE* = (APB1PERIPH_BASE + 0x00003400)
  SPI2_BASE* = (APB1PERIPH_BASE + 0x00003800)
  USART2_BASE* = (APB1PERIPH_BASE + 0x00004400)
  I2C1_BASE* = (APB1PERIPH_BASE + 0x00005400)
  I2C2_BASE* = (APB1PERIPH_BASE + 0x00005800)
  FMPI2C1_BASE* = (APB1PERIPH_BASE + 0x00006000)
  PWR_BASE* = (APB1PERIPH_BASE + 0x00007000)
  DAC_BASE* = (APB1PERIPH_BASE + 0x00007400)

## !< APB2 peripherals

const
  TIM1_BASE* = (APB2PERIPH_BASE + 0x00000000)
  USART1_BASE* = (APB2PERIPH_BASE + 0x00001000)
  USART6_BASE* = (APB2PERIPH_BASE + 0x00001400)
  ADC1_BASE* = (APB2PERIPH_BASE + 0x00002000)
  ADC1_COMMON_BASE* = (APB2PERIPH_BASE + 0x00002300)

##  Legacy define

const
  ADC_BASE* = ADC1_COMMON_BASE
  SPI1_BASE* = (APB2PERIPH_BASE + 0x00003000)
  SYSCFG_BASE* = (APB2PERIPH_BASE + 0x00003800)
  EXTI_BASE* = (APB2PERIPH_BASE + 0x00003C00)
  TIM9_BASE* = (APB2PERIPH_BASE + 0x00004000)
  TIM11_BASE* = (APB2PERIPH_BASE + 0x00004800)
  SPI5_BASE* = (APB2PERIPH_BASE + 0x00005000)

## !< AHB1 peripherals

const
  GPIOA_BASE* = (AHB1PERIPH_BASE + 0x00000000)
  GPIOB_BASE* = (AHB1PERIPH_BASE + 0x00000400)
  GPIOC_BASE* = (AHB1PERIPH_BASE + 0x00000800)
  GPIOH_BASE* = (AHB1PERIPH_BASE + 0x00001C00)
  CRC_BASE* = (AHB1PERIPH_BASE + 0x00003000)
  RCC_BASE* = (AHB1PERIPH_BASE + 0x00003800)
  FLASH_R_BASE* = (AHB1PERIPH_BASE + 0x00003C00)
  DMA1_BASE* = (AHB1PERIPH_BASE + 0x00006000)
  DMA1_Stream0_BASE* = (DMA1_BASE + 0x00000010)
  DMA1_Stream1_BASE* = (DMA1_BASE + 0x00000028)
  DMA1_Stream2_BASE* = (DMA1_BASE + 0x00000040)
  DMA1_Stream3_BASE* = (DMA1_BASE + 0x00000058)
  DMA1_Stream4_BASE* = (DMA1_BASE + 0x00000070)
  DMA1_Stream5_BASE* = (DMA1_BASE + 0x00000088)
  DMA1_Stream6_BASE* = (DMA1_BASE + 0x000000A0)
  DMA1_Stream7_BASE* = (DMA1_BASE + 0x000000B8)
  DMA2_BASE* = (AHB1PERIPH_BASE + 0x00006400)
  DMA2_Stream0_BASE* = (DMA2_BASE + 0x00000010)
  DMA2_Stream1_BASE* = (DMA2_BASE + 0x00000028)
  DMA2_Stream2_BASE* = (DMA2_BASE + 0x00000040)
  DMA2_Stream3_BASE* = (DMA2_BASE + 0x00000058)
  DMA2_Stream4_BASE* = (DMA2_BASE + 0x00000070)
  DMA2_Stream5_BASE* = (DMA2_BASE + 0x00000088)
  DMA2_Stream6_BASE* = (DMA2_BASE + 0x000000A0)
  DMA2_Stream7_BASE* = (DMA2_BASE + 0x000000B8)
  RNG_BASE* = (PERIPH_BASE + 0x00080000)

## !< Debug MCU registers base address

const
  DBGMCU_BASE* = 0xE0042000
  UID_BASE* = 0x1FFF7A10
  FLASHSIZE_BASE* = 0x1FFF7A22
  PACKAGE_BASE* = 0x1FFF7BF0

## *
##  @}
##
## * @addtogroup Peripheral_declaration
##  @{
##

const
  TIM5* = (cast[ptr TIM_TypeDef](TIM5_BASE))
  TIM6* = (cast[ptr TIM_TypeDef](TIM6_BASE))
  RTC* = (cast[ptr RTC_TypeDef](RTC_BASE))
  WWDG* = (cast[ptr WWDG_TypeDef](WWDG_BASE))
  IWDG* = (cast[ptr IWDG_TypeDef](IWDG_BASE))
  SPI2* = (cast[ptr SPI_TypeDef](SPI2_BASE))
  USART2* = (cast[ptr USART_TypeDef](USART2_BASE))
  I2C1* = (cast[ptr I2C_TypeDef](I2C1_BASE))
  I2C2* = (cast[ptr I2C_TypeDef](I2C2_BASE))
  FMPI2C1* = (cast[ptr FMPI2C_TypeDef](FMPI2C1_BASE))
  LPTIM1* = (cast[ptr LPTIM_TypeDef](LPTIM1_BASE))
  PWR* = (cast[ptr PWR_TypeDef](PWR_BASE))
  DAC1* = (cast[ptr DAC_TypeDef](DAC_BASE))
  DAC* = (cast[ptr DAC_TypeDef](DAC_BASE)) ##  Kept for legacy purpose
  TIM1* = (cast[ptr TIM_TypeDef](TIM1_BASE))
  USART1* = (cast[ptr USART_TypeDef](USART1_BASE))
  USART6* = (cast[ptr USART_TypeDef](USART6_BASE))
  ADC1* = (cast[ptr ADC_TypeDef](ADC1_BASE))
  ADC1_COMMON* = (cast[ptr ADC_Common_TypeDef](ADC1_COMMON_BASE))

##  Legacy define

const
  ADC* = ADC1_COMMON
  SPI1* = (cast[ptr SPI_TypeDef](SPI1_BASE))
  SYSCFG* = (cast[ptr SYSCFG_TypeDef](SYSCFG_BASE))
  EXTI* = (cast[ptr EXTI_TypeDef](EXTI_BASE))
  TIM9* = (cast[ptr TIM_TypeDef](TIM9_BASE))
  TIM11* = (cast[ptr TIM_TypeDef](TIM11_BASE))
  SPI5* = (cast[ptr SPI_TypeDef](SPI5_BASE))
  GPIOA* = (cast[ptr GPIO_TypeDef](GPIOA_BASE))
  GPIOB* = (cast[ptr GPIO_TypeDef](GPIOB_BASE))
  GPIOC* = (cast[ptr GPIO_TypeDef](GPIOC_BASE))
  GPIOH* = (cast[ptr GPIO_TypeDef](GPIOH_BASE))
  CRC* = (cast[ptr CRC_TypeDef](CRC_BASE))
  RCC* = (cast[ptr RCC_TypeDef](RCC_BASE))
  FLASH* = (cast[ptr FLASH_TypeDef](FLASH_R_BASE))
  DMA1* = (cast[ptr DMA_TypeDef](DMA1_BASE))
  DMA1_Stream0* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream0_BASE))
  DMA1_Stream1* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream1_BASE))
  DMA1_Stream2* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream2_BASE))
  DMA1_Stream3* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream3_BASE))
  DMA1_Stream4* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream4_BASE))
  DMA1_Stream5* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream5_BASE))
  DMA1_Stream6* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream6_BASE))
  DMA1_Stream7* = (cast[ptr DMA_Stream_TypeDef](DMA1_Stream7_BASE))
  DMA2* = (cast[ptr DMA_TypeDef](DMA2_BASE))
  DMA2_Stream0* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream0_BASE))
  DMA2_Stream1* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream1_BASE))
  DMA2_Stream2* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream2_BASE))
  DMA2_Stream3* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream3_BASE))
  DMA2_Stream4* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream4_BASE))
  DMA2_Stream5* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream5_BASE))
  DMA2_Stream6* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream6_BASE))
  DMA2_Stream7* = (cast[ptr DMA_Stream_TypeDef](DMA2_Stream7_BASE))
  RNG* = (cast[ptr RNG_TypeDef](RNG_BASE))
  DBGMCU* = (cast[ptr DBGMCU_TypeDef](DBGMCU_BASE))

## *
##  @}
##
## * @addtogroup Exported_constants
##  @{
##
## * @addtogroup Peripheral_Registers_Bits_Definition
##  @{
##
## ****************************************************************************
##                          Peripheral Registers_Bits_Definition
## ****************************************************************************
## ****************************************************************************
##
##                         Analog to Digital Converter
##
## ****************************************************************************
## *******************  Bit definition for ADC_SR register  *******************

const
  ADC_SR_AWD_Pos* = (0)
  ADC_SR_AWD_Msk* = (0x00000001 shl ADC_SR_AWD_Pos) ## !< 0x00000001
  ADC_SR_AWD* = ADC_SR_AWD_Msk
  ADC_SR_EOC_Pos* = (1)
  ADC_SR_EOC_Msk* = (0x00000001 shl ADC_SR_EOC_Pos) ## !< 0x00000002
  ADC_SR_EOC* = ADC_SR_EOC_Msk
  ADC_SR_JEOC_Pos* = (2)
  ADC_SR_JEOC_Msk* = (0x00000001 shl ADC_SR_JEOC_Pos) ## !< 0x00000004
  ADC_SR_JEOC* = ADC_SR_JEOC_Msk
  ADC_SR_JSTRT_Pos* = (3)
  ADC_SR_JSTRT_Msk* = (0x00000001 shl ADC_SR_JSTRT_Pos) ## !< 0x00000008
  ADC_SR_JSTRT* = ADC_SR_JSTRT_Msk
  ADC_SR_STRT_Pos* = (4)
  ADC_SR_STRT_Msk* = (0x00000001 shl ADC_SR_STRT_Pos) ## !< 0x00000010
  ADC_SR_STRT* = ADC_SR_STRT_Msk
  ADC_SR_OVR_Pos* = (5)
  ADC_SR_OVR_Msk* = (0x00000001 shl ADC_SR_OVR_Pos) ## !< 0x00000020
  ADC_SR_OVR* = ADC_SR_OVR_Msk

## ******************  Bit definition for ADC_CR1 register  *******************

const
  ADC_CR1_AWDCH_Pos* = (0)
  ADC_CR1_AWDCH_Msk* = (0x0000001F shl ADC_CR1_AWDCH_Pos) ## !< 0x0000001F
  ADC_CR1_AWDCH* = ADC_CR1_AWDCH_Msk
  ADC_CR1_AWDCH_Bit0* = (0x00000001 shl ADC_CR1_AWDCH_Pos) ## !< 0x00000001
  ADC_CR1_AWDCH_Bit1* = (0x00000002 shl ADC_CR1_AWDCH_Pos) ## !< 0x00000002
  ADC_CR1_AWDCH_Bit2* = (0x00000004 shl ADC_CR1_AWDCH_Pos) ## !< 0x00000004
  ADC_CR1_AWDCH_Bit3* = (0x00000008 shl ADC_CR1_AWDCH_Pos) ## !< 0x00000008
  ADC_CR1_AWDCH_Bit4* = (0x00000010 shl ADC_CR1_AWDCH_Pos) ## !< 0x00000010
  ADC_CR1_EOCIE_Pos* = (5)
  ADC_CR1_EOCIE_Msk* = (0x00000001 shl ADC_CR1_EOCIE_Pos) ## !< 0x00000020
  ADC_CR1_EOCIE* = ADC_CR1_EOCIE_Msk
  ADC_CR1_AWDIE_Pos* = (6)
  ADC_CR1_AWDIE_Msk* = (0x00000001 shl ADC_CR1_AWDIE_Pos) ## !< 0x00000040
  ADC_CR1_AWDIE* = ADC_CR1_AWDIE_Msk
  ADC_CR1_JEOCIE_Pos* = (7)
  ADC_CR1_JEOCIE_Msk* = (0x00000001 shl ADC_CR1_JEOCIE_Pos) ## !< 0x00000080
  ADC_CR1_JEOCIE* = ADC_CR1_JEOCIE_Msk
  ADC_CR1_SCAN_Pos* = (8)
  ADC_CR1_SCAN_Msk* = (0x00000001 shl ADC_CR1_SCAN_Pos) ## !< 0x00000100
  ADC_CR1_SCAN* = ADC_CR1_SCAN_Msk
  ADC_CR1_AWDSGL_Pos* = (9)
  ADC_CR1_AWDSGL_Msk* = (0x00000001 shl ADC_CR1_AWDSGL_Pos) ## !< 0x00000200
  ADC_CR1_AWDSGL* = ADC_CR1_AWDSGL_Msk
  ADC_CR1_JAUTO_Pos* = (10)
  ADC_CR1_JAUTO_Msk* = (0x00000001 shl ADC_CR1_JAUTO_Pos) ## !< 0x00000400
  ADC_CR1_JAUTO* = ADC_CR1_JAUTO_Msk
  ADC_CR1_DISCEN_Pos* = (11)
  ADC_CR1_DISCEN_Msk* = (0x00000001 shl ADC_CR1_DISCEN_Pos) ## !< 0x00000800
  ADC_CR1_DISCEN* = ADC_CR1_DISCEN_Msk
  ADC_CR1_JDISCEN_Pos* = (12)
  ADC_CR1_JDISCEN_Msk* = (0x00000001 shl ADC_CR1_JDISCEN_Pos) ## !< 0x00001000
  ADC_CR1_JDISCEN* = ADC_CR1_JDISCEN_Msk
  ADC_CR1_DISCNUM_Pos* = (13)
  ADC_CR1_DISCNUM_Msk* = (0x00000007 shl ADC_CR1_DISCNUM_Pos) ## !< 0x0000E000
  ADC_CR1_DISCNUM* = ADC_CR1_DISCNUM_Msk
  ADC_CR1_DISCNUM_Bit0* = (0x00000001 shl ADC_CR1_DISCNUM_Pos) ## !< 0x00002000
  ADC_CR1_DISCNUM_Bit1* = (0x00000002 shl ADC_CR1_DISCNUM_Pos) ## !< 0x00004000
  ADC_CR1_DISCNUM_Bit2* = (0x00000004 shl ADC_CR1_DISCNUM_Pos) ## !< 0x00008000
  ADC_CR1_JAWDEN_Pos* = (22)
  ADC_CR1_JAWDEN_Msk* = (0x00000001 shl ADC_CR1_JAWDEN_Pos) ## !< 0x00400000
  ADC_CR1_JAWDEN* = ADC_CR1_JAWDEN_Msk
  ADC_CR1_AWDEN_Pos* = (23)
  ADC_CR1_AWDEN_Msk* = (0x00000001 shl ADC_CR1_AWDEN_Pos) ## !< 0x00800000
  ADC_CR1_AWDEN* = ADC_CR1_AWDEN_Msk
  ADC_CR1_RES_Pos* = (24)
  ADC_CR1_RES_Msk* = (0x00000003 shl ADC_CR1_RES_Pos) ## !< 0x03000000
  ADC_CR1_RES* = ADC_CR1_RES_Msk
  ADC_CR1_RES_Bit0* = (0x00000001 shl ADC_CR1_RES_Pos) ## !< 0x01000000
  ADC_CR1_RES_Bit1* = (0x00000002 shl ADC_CR1_RES_Pos) ## !< 0x02000000
  ADC_CR1_OVRIE_Pos* = (26)
  ADC_CR1_OVRIE_Msk* = (0x00000001 shl ADC_CR1_OVRIE_Pos) ## !< 0x04000000
  ADC_CR1_OVRIE* = ADC_CR1_OVRIE_Msk

## ******************  Bit definition for ADC_CR2 register  *******************

const
  ADC_CR2_ADON_Pos* = (0)
  ADC_CR2_ADON_Msk* = (0x00000001 shl ADC_CR2_ADON_Pos) ## !< 0x00000001
  ADC_CR2_ADON* = ADC_CR2_ADON_Msk
  ADC_CR2_CONT_Pos* = (1)
  ADC_CR2_CONT_Msk* = (0x00000001 shl ADC_CR2_CONT_Pos) ## !< 0x00000002
  ADC_CR2_CONT* = ADC_CR2_CONT_Msk
  ADC_CR2_DMA_Pos* = (8)
  ADC_CR2_DMA_Msk* = (0x00000001 shl ADC_CR2_DMA_Pos) ## !< 0x00000100
  ADC_CR2_DMA* = ADC_CR2_DMA_Msk
  ADC_CR2_DDS_Pos* = (9)
  ADC_CR2_DDS_Msk* = (0x00000001 shl ADC_CR2_DDS_Pos) ## !< 0x00000200
  ADC_CR2_DDS* = ADC_CR2_DDS_Msk
  ADC_CR2_EOCS_Pos* = (10)
  ADC_CR2_EOCS_Msk* = (0x00000001 shl ADC_CR2_EOCS_Pos) ## !< 0x00000400
  ADC_CR2_EOCS* = ADC_CR2_EOCS_Msk
  ADC_CR2_ALIGN_Pos* = (11)
  ADC_CR2_ALIGN_Msk* = (0x00000001 shl ADC_CR2_ALIGN_Pos) ## !< 0x00000800
  ADC_CR2_ALIGN* = ADC_CR2_ALIGN_Msk
  ADC_CR2_JEXTSEL_Pos* = (16)
  ADC_CR2_JEXTSEL_Msk* = (0x0000000F shl ADC_CR2_JEXTSEL_Pos) ## !< 0x000F0000
  ADC_CR2_JEXTSEL* = ADC_CR2_JEXTSEL_Msk
  ADC_CR2_JEXTSEL_Bit0* = (0x00000001 shl ADC_CR2_JEXTSEL_Pos) ## !< 0x00010000
  ADC_CR2_JEXTSEL_Bit1* = (0x00000002 shl ADC_CR2_JEXTSEL_Pos) ## !< 0x00020000
  ADC_CR2_JEXTSEL_Bit2* = (0x00000004 shl ADC_CR2_JEXTSEL_Pos) ## !< 0x00040000
  ADC_CR2_JEXTSEL_Bit3* = (0x00000008 shl ADC_CR2_JEXTSEL_Pos) ## !< 0x00080000
  ADC_CR2_JEXTEN_Pos* = (20)
  ADC_CR2_JEXTEN_Msk* = (0x00000003 shl ADC_CR2_JEXTEN_Pos) ## !< 0x00300000
  ADC_CR2_JEXTEN* = ADC_CR2_JEXTEN_Msk
  ADC_CR2_JEXTEN_Bit0* = (0x00000001 shl ADC_CR2_JEXTEN_Pos) ## !< 0x00100000
  ADC_CR2_JEXTEN_Bit1* = (0x00000002 shl ADC_CR2_JEXTEN_Pos) ## !< 0x00200000
  ADC_CR2_JSWSTART_Pos* = (22)
  ADC_CR2_JSWSTART_Msk* = (0x00000001 shl ADC_CR2_JSWSTART_Pos) ## !< 0x00400000
  ADC_CR2_JSWSTART* = ADC_CR2_JSWSTART_Msk
  ADC_CR2_EXTSEL_Pos* = (24)
  ADC_CR2_EXTSEL_Msk* = (0x0000000F shl ADC_CR2_EXTSEL_Pos) ## !< 0x0F000000
  ADC_CR2_EXTSEL* = ADC_CR2_EXTSEL_Msk
  ADC_CR2_EXTSEL_Bit0* = (0x00000001 shl ADC_CR2_EXTSEL_Pos) ## !< 0x01000000
  ADC_CR2_EXTSEL_Bit1* = (0x00000002 shl ADC_CR2_EXTSEL_Pos) ## !< 0x02000000
  ADC_CR2_EXTSEL_Bit2* = (0x00000004 shl ADC_CR2_EXTSEL_Pos) ## !< 0x04000000
  ADC_CR2_EXTSEL_Bit3* = (0x00000008 shl ADC_CR2_EXTSEL_Pos) ## !< 0x08000000
  ADC_CR2_EXTEN_Pos* = (28)
  ADC_CR2_EXTEN_Msk* = (0x00000003 shl ADC_CR2_EXTEN_Pos) ## !< 0x30000000
  ADC_CR2_EXTEN* = ADC_CR2_EXTEN_Msk
  ADC_CR2_EXTEN_Bit0* = (0x00000001 shl ADC_CR2_EXTEN_Pos) ## !< 0x10000000
  ADC_CR2_EXTEN_Bit1* = (0x00000002 shl ADC_CR2_EXTEN_Pos) ## !< 0x20000000
  ADC_CR2_SWSTART_Pos* = (30)
  ADC_CR2_SWSTART_Msk* = (0x00000001 shl ADC_CR2_SWSTART_Pos) ## !< 0x40000000
  ADC_CR2_SWSTART* = ADC_CR2_SWSTART_Msk

## *****************  Bit definition for ADC_SMPR1 register  ******************

const
  ADC_SMPR1_SMP10_Pos* = (0)
  ADC_SMPR1_SMP10_Msk* = (0x00000007 shl ADC_SMPR1_SMP10_Pos) ## !< 0x00000007
  ADC_SMPR1_SMP10* = ADC_SMPR1_SMP10_Msk
  ADC_SMPR1_SMP10_Bit0* = (0x00000001 shl ADC_SMPR1_SMP10_Pos) ## !< 0x00000001
  ADC_SMPR1_SMP10_Bit1* = (0x00000002 shl ADC_SMPR1_SMP10_Pos) ## !< 0x00000002
  ADC_SMPR1_SMP10_Bit2* = (0x00000004 shl ADC_SMPR1_SMP10_Pos) ## !< 0x00000004
  ADC_SMPR1_SMP11_Pos* = (3)
  ADC_SMPR1_SMP11_Msk* = (0x00000007 shl ADC_SMPR1_SMP11_Pos) ## !< 0x00000038
  ADC_SMPR1_SMP11* = ADC_SMPR1_SMP11_Msk
  ADC_SMPR1_SMP11_Bit0* = (0x00000001 shl ADC_SMPR1_SMP11_Pos) ## !< 0x00000008
  ADC_SMPR1_SMP11_Bit1* = (0x00000002 shl ADC_SMPR1_SMP11_Pos) ## !< 0x00000010
  ADC_SMPR1_SMP11_Bit2* = (0x00000004 shl ADC_SMPR1_SMP11_Pos) ## !< 0x00000020
  ADC_SMPR1_SMP12_Pos* = (6)
  ADC_SMPR1_SMP12_Msk* = (0x00000007 shl ADC_SMPR1_SMP12_Pos) ## !< 0x000001C0
  ADC_SMPR1_SMP12* = ADC_SMPR1_SMP12_Msk
  ADC_SMPR1_SMP12_Bit0* = (0x00000001 shl ADC_SMPR1_SMP12_Pos) ## !< 0x00000040
  ADC_SMPR1_SMP12_Bit1* = (0x00000002 shl ADC_SMPR1_SMP12_Pos) ## !< 0x00000080
  ADC_SMPR1_SMP12_Bit2* = (0x00000004 shl ADC_SMPR1_SMP12_Pos) ## !< 0x00000100
  ADC_SMPR1_SMP13_Pos* = (9)
  ADC_SMPR1_SMP13_Msk* = (0x00000007 shl ADC_SMPR1_SMP13_Pos) ## !< 0x00000E00
  ADC_SMPR1_SMP13* = ADC_SMPR1_SMP13_Msk
  ADC_SMPR1_SMP13_Bit0* = (0x00000001 shl ADC_SMPR1_SMP13_Pos) ## !< 0x00000200
  ADC_SMPR1_SMP13_Bit1* = (0x00000002 shl ADC_SMPR1_SMP13_Pos) ## !< 0x00000400
  ADC_SMPR1_SMP13_Bit2* = (0x00000004 shl ADC_SMPR1_SMP13_Pos) ## !< 0x00000800
  ADC_SMPR1_SMP14_Pos* = (12)
  ADC_SMPR1_SMP14_Msk* = (0x00000007 shl ADC_SMPR1_SMP14_Pos) ## !< 0x00007000
  ADC_SMPR1_SMP14* = ADC_SMPR1_SMP14_Msk
  ADC_SMPR1_SMP14_Bit0* = (0x00000001 shl ADC_SMPR1_SMP14_Pos) ## !< 0x00001000
  ADC_SMPR1_SMP14_Bit1* = (0x00000002 shl ADC_SMPR1_SMP14_Pos) ## !< 0x00002000
  ADC_SMPR1_SMP14_Bit2* = (0x00000004 shl ADC_SMPR1_SMP14_Pos) ## !< 0x00004000
  ADC_SMPR1_SMP15_Pos* = (15)
  ADC_SMPR1_SMP15_Msk* = (0x00000007 shl ADC_SMPR1_SMP15_Pos) ## !< 0x00038000
  ADC_SMPR1_SMP15* = ADC_SMPR1_SMP15_Msk
  ADC_SMPR1_SMP15_Bit0* = (0x00000001 shl ADC_SMPR1_SMP15_Pos) ## !< 0x00008000
  ADC_SMPR1_SMP15_Bit1* = (0x00000002 shl ADC_SMPR1_SMP15_Pos) ## !< 0x00010000
  ADC_SMPR1_SMP15_Bit2* = (0x00000004 shl ADC_SMPR1_SMP15_Pos) ## !< 0x00020000
  ADC_SMPR1_SMP16_Pos* = (18)
  ADC_SMPR1_SMP16_Msk* = (0x00000007 shl ADC_SMPR1_SMP16_Pos) ## !< 0x001C0000
  ADC_SMPR1_SMP16* = ADC_SMPR1_SMP16_Msk
  ADC_SMPR1_SMP16_Bit0* = (0x00000001 shl ADC_SMPR1_SMP16_Pos) ## !< 0x00040000
  ADC_SMPR1_SMP16_Bit1* = (0x00000002 shl ADC_SMPR1_SMP16_Pos) ## !< 0x00080000
  ADC_SMPR1_SMP16_Bit2* = (0x00000004 shl ADC_SMPR1_SMP16_Pos) ## !< 0x00100000
  ADC_SMPR1_SMP17_Pos* = (21)
  ADC_SMPR1_SMP17_Msk* = (0x00000007 shl ADC_SMPR1_SMP17_Pos) ## !< 0x00E00000
  ADC_SMPR1_SMP17* = ADC_SMPR1_SMP17_Msk
  ADC_SMPR1_SMP17_Bit0* = (0x00000001 shl ADC_SMPR1_SMP17_Pos) ## !< 0x00200000
  ADC_SMPR1_SMP17_Bit1* = (0x00000002 shl ADC_SMPR1_SMP17_Pos) ## !< 0x00400000
  ADC_SMPR1_SMP17_Bit2* = (0x00000004 shl ADC_SMPR1_SMP17_Pos) ## !< 0x00800000
  ADC_SMPR1_SMP18_Pos* = (24)
  ADC_SMPR1_SMP18_Msk* = (0x00000007 shl ADC_SMPR1_SMP18_Pos) ## !< 0x07000000
  ADC_SMPR1_SMP18* = ADC_SMPR1_SMP18_Msk
  ADC_SMPR1_SMP18_Bit0* = (0x00000001 shl ADC_SMPR1_SMP18_Pos) ## !< 0x01000000
  ADC_SMPR1_SMP18_Bit1* = (0x00000002 shl ADC_SMPR1_SMP18_Pos) ## !< 0x02000000
  ADC_SMPR1_SMP18_Bit2* = (0x00000004 shl ADC_SMPR1_SMP18_Pos) ## !< 0x04000000

## *****************  Bit definition for ADC_SMPR2 register  ******************

const
  ADC_SMPR2_SMP0_Pos* = (0)
  ADC_SMPR2_SMP0_Msk* = (0x00000007 shl ADC_SMPR2_SMP0_Pos) ## !< 0x00000007
  ADC_SMPR2_SMP0* = ADC_SMPR2_SMP0_Msk
  ADC_SMPR2_SMP0_Bit0* = (0x00000001 shl ADC_SMPR2_SMP0_Pos) ## !< 0x00000001
  ADC_SMPR2_SMP0_Bit1* = (0x00000002 shl ADC_SMPR2_SMP0_Pos) ## !< 0x00000002
  ADC_SMPR2_SMP0_Bit2* = (0x00000004 shl ADC_SMPR2_SMP0_Pos) ## !< 0x00000004
  ADC_SMPR2_SMP1_Pos* = (3)
  ADC_SMPR2_SMP1_Msk* = (0x00000007 shl ADC_SMPR2_SMP1_Pos) ## !< 0x00000038
  ADC_SMPR2_SMP1* = ADC_SMPR2_SMP1_Msk
  ADC_SMPR2_SMP1_Bit0* = (0x00000001 shl ADC_SMPR2_SMP1_Pos) ## !< 0x00000008
  ADC_SMPR2_SMP1_Bit1* = (0x00000002 shl ADC_SMPR2_SMP1_Pos) ## !< 0x00000010
  ADC_SMPR2_SMP1_Bit2* = (0x00000004 shl ADC_SMPR2_SMP1_Pos) ## !< 0x00000020
  ADC_SMPR2_SMP2_Pos* = (6)
  ADC_SMPR2_SMP2_Msk* = (0x00000007 shl ADC_SMPR2_SMP2_Pos) ## !< 0x000001C0
  ADC_SMPR2_SMP2* = ADC_SMPR2_SMP2_Msk
  ADC_SMPR2_SMP2_Bit0* = (0x00000001 shl ADC_SMPR2_SMP2_Pos) ## !< 0x00000040
  ADC_SMPR2_SMP2_Bit1* = (0x00000002 shl ADC_SMPR2_SMP2_Pos) ## !< 0x00000080
  ADC_SMPR2_SMP2_Bit2* = (0x00000004 shl ADC_SMPR2_SMP2_Pos) ## !< 0x00000100
  ADC_SMPR2_SMP3_Pos* = (9)
  ADC_SMPR2_SMP3_Msk* = (0x00000007 shl ADC_SMPR2_SMP3_Pos) ## !< 0x00000E00
  ADC_SMPR2_SMP3* = ADC_SMPR2_SMP3_Msk
  ADC_SMPR2_SMP3_Bit0* = (0x00000001 shl ADC_SMPR2_SMP3_Pos) ## !< 0x00000200
  ADC_SMPR2_SMP3_Bit1* = (0x00000002 shl ADC_SMPR2_SMP3_Pos) ## !< 0x00000400
  ADC_SMPR2_SMP3_Bit2* = (0x00000004 shl ADC_SMPR2_SMP3_Pos) ## !< 0x00000800
  ADC_SMPR2_SMP4_Pos* = (12)
  ADC_SMPR2_SMP4_Msk* = (0x00000007 shl ADC_SMPR2_SMP4_Pos) ## !< 0x00007000
  ADC_SMPR2_SMP4* = ADC_SMPR2_SMP4_Msk
  ADC_SMPR2_SMP4_Bit0* = (0x00000001 shl ADC_SMPR2_SMP4_Pos) ## !< 0x00001000
  ADC_SMPR2_SMP4_Bit1* = (0x00000002 shl ADC_SMPR2_SMP4_Pos) ## !< 0x00002000
  ADC_SMPR2_SMP4_Bit2* = (0x00000004 shl ADC_SMPR2_SMP4_Pos) ## !< 0x00004000
  ADC_SMPR2_SMP5_Pos* = (15)
  ADC_SMPR2_SMP5_Msk* = (0x00000007 shl ADC_SMPR2_SMP5_Pos) ## !< 0x00038000
  ADC_SMPR2_SMP5* = ADC_SMPR2_SMP5_Msk
  ADC_SMPR2_SMP5_Bit0* = (0x00000001 shl ADC_SMPR2_SMP5_Pos) ## !< 0x00008000
  ADC_SMPR2_SMP5_Bit1* = (0x00000002 shl ADC_SMPR2_SMP5_Pos) ## !< 0x00010000
  ADC_SMPR2_SMP5_Bit2* = (0x00000004 shl ADC_SMPR2_SMP5_Pos) ## !< 0x00020000
  ADC_SMPR2_SMP6_Pos* = (18)
  ADC_SMPR2_SMP6_Msk* = (0x00000007 shl ADC_SMPR2_SMP6_Pos) ## !< 0x001C0000
  ADC_SMPR2_SMP6* = ADC_SMPR2_SMP6_Msk
  ADC_SMPR2_SMP6_Bit0* = (0x00000001 shl ADC_SMPR2_SMP6_Pos) ## !< 0x00040000
  ADC_SMPR2_SMP6_Bit1* = (0x00000002 shl ADC_SMPR2_SMP6_Pos) ## !< 0x00080000
  ADC_SMPR2_SMP6_Bit2* = (0x00000004 shl ADC_SMPR2_SMP6_Pos) ## !< 0x00100000
  ADC_SMPR2_SMP7_Pos* = (21)
  ADC_SMPR2_SMP7_Msk* = (0x00000007 shl ADC_SMPR2_SMP7_Pos) ## !< 0x00E00000
  ADC_SMPR2_SMP7* = ADC_SMPR2_SMP7_Msk
  ADC_SMPR2_SMP7_Bit0* = (0x00000001 shl ADC_SMPR2_SMP7_Pos) ## !< 0x00200000
  ADC_SMPR2_SMP7_Bit1* = (0x00000002 shl ADC_SMPR2_SMP7_Pos) ## !< 0x00400000
  ADC_SMPR2_SMP7_Bit2* = (0x00000004 shl ADC_SMPR2_SMP7_Pos) ## !< 0x00800000
  ADC_SMPR2_SMP8_Pos* = (24)
  ADC_SMPR2_SMP8_Msk* = (0x00000007 shl ADC_SMPR2_SMP8_Pos) ## !< 0x07000000
  ADC_SMPR2_SMP8* = ADC_SMPR2_SMP8_Msk
  ADC_SMPR2_SMP8_Bit0* = (0x00000001 shl ADC_SMPR2_SMP8_Pos) ## !< 0x01000000
  ADC_SMPR2_SMP8_Bit1* = (0x00000002 shl ADC_SMPR2_SMP8_Pos) ## !< 0x02000000
  ADC_SMPR2_SMP8_Bit2* = (0x00000004 shl ADC_SMPR2_SMP8_Pos) ## !< 0x04000000
  ADC_SMPR2_SMP9_Pos* = (27)
  ADC_SMPR2_SMP9_Msk* = (0x00000007 shl ADC_SMPR2_SMP9_Pos) ## !< 0x38000000
  ADC_SMPR2_SMP9* = ADC_SMPR2_SMP9_Msk
  ADC_SMPR2_SMP9_Bit0* = (0x00000001 shl ADC_SMPR2_SMP9_Pos) ## !< 0x08000000
  ADC_SMPR2_SMP9_Bit1* = (0x00000002 shl ADC_SMPR2_SMP9_Pos) ## !< 0x10000000
  ADC_SMPR2_SMP9_Bit2* = (0x00000004 shl ADC_SMPR2_SMP9_Pos) ## !< 0x20000000

## *****************  Bit definition for ADC_JOFR1 register  ******************

const
  ADC_JOFR1_JOFFSET1_Pos* = (0)
  ADC_JOFR1_JOFFSET1_Msk* = (0x00000FFF shl ADC_JOFR1_JOFFSET1_Pos) ## !< 0x00000FFF
  ADC_JOFR1_JOFFSET1* = ADC_JOFR1_JOFFSET1_Msk

## *****************  Bit definition for ADC_JOFR2 register  ******************

const
  ADC_JOFR2_JOFFSET2_Pos* = (0)
  ADC_JOFR2_JOFFSET2_Msk* = (0x00000FFF shl ADC_JOFR2_JOFFSET2_Pos) ## !< 0x00000FFF
  ADC_JOFR2_JOFFSET2* = ADC_JOFR2_JOFFSET2_Msk

## *****************  Bit definition for ADC_JOFR3 register  ******************

const
  ADC_JOFR3_JOFFSET3_Pos* = (0)
  ADC_JOFR3_JOFFSET3_Msk* = (0x00000FFF shl ADC_JOFR3_JOFFSET3_Pos) ## !< 0x00000FFF
  ADC_JOFR3_JOFFSET3* = ADC_JOFR3_JOFFSET3_Msk

## *****************  Bit definition for ADC_JOFR4 register  ******************

const
  ADC_JOFR4_JOFFSET4_Pos* = (0)
  ADC_JOFR4_JOFFSET4_Msk* = (0x00000FFF shl ADC_JOFR4_JOFFSET4_Pos) ## !< 0x00000FFF
  ADC_JOFR4_JOFFSET4* = ADC_JOFR4_JOFFSET4_Msk

## ******************  Bit definition for ADC_HTR register  *******************

const
  ADC_HTR_HT_Pos* = (0)
  ADC_HTR_HT_Msk* = (0x00000FFF shl ADC_HTR_HT_Pos) ## !< 0x00000FFF
  ADC_HTR_HT* = ADC_HTR_HT_Msk

## ******************  Bit definition for ADC_LTR register  *******************

const
  ADC_LTR_LT_Pos* = (0)
  ADC_LTR_LT_Msk* = (0x00000FFF shl ADC_LTR_LT_Pos) ## !< 0x00000FFF
  ADC_LTR_LT* = ADC_LTR_LT_Msk

## ******************  Bit definition for ADC_SQR1 register  ******************

const
  ADC_SQR1_SQ13_Pos* = (0)
  ADC_SQR1_SQ13_Msk* = (0x0000001F shl ADC_SQR1_SQ13_Pos) ## !< 0x0000001F
  ADC_SQR1_SQ13* = ADC_SQR1_SQ13_Msk
  ADC_SQR1_SQ13_Bit0* = (0x00000001 shl ADC_SQR1_SQ13_Pos) ## !< 0x00000001
  ADC_SQR1_SQ13_Bit1* = (0x00000002 shl ADC_SQR1_SQ13_Pos) ## !< 0x00000002
  ADC_SQR1_SQ13_Bit2* = (0x00000004 shl ADC_SQR1_SQ13_Pos) ## !< 0x00000004
  ADC_SQR1_SQ13_Bit3* = (0x00000008 shl ADC_SQR1_SQ13_Pos) ## !< 0x00000008
  ADC_SQR1_SQ13_Bit4* = (0x00000010 shl ADC_SQR1_SQ13_Pos) ## !< 0x00000010
  ADC_SQR1_SQ14_Pos* = (5)
  ADC_SQR1_SQ14_Msk* = (0x0000001F shl ADC_SQR1_SQ14_Pos) ## !< 0x000003E0
  ADC_SQR1_SQ14* = ADC_SQR1_SQ14_Msk
  ADC_SQR1_SQ14_Bit0* = (0x00000001 shl ADC_SQR1_SQ14_Pos) ## !< 0x00000020
  ADC_SQR1_SQ14_Bit1* = (0x00000002 shl ADC_SQR1_SQ14_Pos) ## !< 0x00000040
  ADC_SQR1_SQ14_Bit2* = (0x00000004 shl ADC_SQR1_SQ14_Pos) ## !< 0x00000080
  ADC_SQR1_SQ14_Bit3* = (0x00000008 shl ADC_SQR1_SQ14_Pos) ## !< 0x00000100
  ADC_SQR1_SQ14_Bit4* = (0x00000010 shl ADC_SQR1_SQ14_Pos) ## !< 0x00000200
  ADC_SQR1_SQ15_Pos* = (10)
  ADC_SQR1_SQ15_Msk* = (0x0000001F shl ADC_SQR1_SQ15_Pos) ## !< 0x00007C00
  ADC_SQR1_SQ15* = ADC_SQR1_SQ15_Msk
  ADC_SQR1_SQ15_Bit0* = (0x00000001 shl ADC_SQR1_SQ15_Pos) ## !< 0x00000400
  ADC_SQR1_SQ15_Bit1* = (0x00000002 shl ADC_SQR1_SQ15_Pos) ## !< 0x00000800
  ADC_SQR1_SQ15_Bit2* = (0x00000004 shl ADC_SQR1_SQ15_Pos) ## !< 0x00001000
  ADC_SQR1_SQ15_Bit3* = (0x00000008 shl ADC_SQR1_SQ15_Pos) ## !< 0x00002000
  ADC_SQR1_SQ15_Bit4* = (0x00000010 shl ADC_SQR1_SQ15_Pos) ## !< 0x00004000
  ADC_SQR1_SQ16_Pos* = (15)
  ADC_SQR1_SQ16_Msk* = (0x0000001F shl ADC_SQR1_SQ16_Pos) ## !< 0x000F8000
  ADC_SQR1_SQ16* = ADC_SQR1_SQ16_Msk
  ADC_SQR1_SQ16_Bit0* = (0x00000001 shl ADC_SQR1_SQ16_Pos) ## !< 0x00008000
  ADC_SQR1_SQ16_Bit1* = (0x00000002 shl ADC_SQR1_SQ16_Pos) ## !< 0x00010000
  ADC_SQR1_SQ16_Bit2* = (0x00000004 shl ADC_SQR1_SQ16_Pos) ## !< 0x00020000
  ADC_SQR1_SQ16_Bit3* = (0x00000008 shl ADC_SQR1_SQ16_Pos) ## !< 0x00040000
  ADC_SQR1_SQ16_Bit4* = (0x00000010 shl ADC_SQR1_SQ16_Pos) ## !< 0x00080000
  ADC_SQR1_L_Pos* = (20)
  ADC_SQR1_L_Msk* = (0x0000000F shl ADC_SQR1_L_Pos) ## !< 0x00F00000
  ADC_SQR1_L* = ADC_SQR1_L_Msk
  ADC_SQR1_L_Bit0* = (0x00000001 shl ADC_SQR1_L_Pos) ## !< 0x00100000
  ADC_SQR1_L_Bit1* = (0x00000002 shl ADC_SQR1_L_Pos) ## !< 0x00200000
  ADC_SQR1_L_Bit2* = (0x00000004 shl ADC_SQR1_L_Pos) ## !< 0x00400000
  ADC_SQR1_L_Bit3* = (0x00000008 shl ADC_SQR1_L_Pos) ## !< 0x00800000

## ******************  Bit definition for ADC_SQR2 register  ******************

const
  ADC_SQR2_SQ7_Pos* = (0)
  ADC_SQR2_SQ7_Msk* = (0x0000001F shl ADC_SQR2_SQ7_Pos) ## !< 0x0000001F
  ADC_SQR2_SQ7* = ADC_SQR2_SQ7_Msk
  ADC_SQR2_SQ7_Bit0* = (0x00000001 shl ADC_SQR2_SQ7_Pos) ## !< 0x00000001
  ADC_SQR2_SQ7_Bit1* = (0x00000002 shl ADC_SQR2_SQ7_Pos) ## !< 0x00000002
  ADC_SQR2_SQ7_Bit2* = (0x00000004 shl ADC_SQR2_SQ7_Pos) ## !< 0x00000004
  ADC_SQR2_SQ7_Bit3* = (0x00000008 shl ADC_SQR2_SQ7_Pos) ## !< 0x00000008
  ADC_SQR2_SQ7_Bit4* = (0x00000010 shl ADC_SQR2_SQ7_Pos) ## !< 0x00000010
  ADC_SQR2_SQ8_Pos* = (5)
  ADC_SQR2_SQ8_Msk* = (0x0000001F shl ADC_SQR2_SQ8_Pos) ## !< 0x000003E0
  ADC_SQR2_SQ8* = ADC_SQR2_SQ8_Msk
  ADC_SQR2_SQ8_Bit0* = (0x00000001 shl ADC_SQR2_SQ8_Pos) ## !< 0x00000020
  ADC_SQR2_SQ8_Bit1* = (0x00000002 shl ADC_SQR2_SQ8_Pos) ## !< 0x00000040
  ADC_SQR2_SQ8_Bit2* = (0x00000004 shl ADC_SQR2_SQ8_Pos) ## !< 0x00000080
  ADC_SQR2_SQ8_Bit3* = (0x00000008 shl ADC_SQR2_SQ8_Pos) ## !< 0x00000100
  ADC_SQR2_SQ8_Bit4* = (0x00000010 shl ADC_SQR2_SQ8_Pos) ## !< 0x00000200
  ADC_SQR2_SQ9_Pos* = (10)
  ADC_SQR2_SQ9_Msk* = (0x0000001F shl ADC_SQR2_SQ9_Pos) ## !< 0x00007C00
  ADC_SQR2_SQ9* = ADC_SQR2_SQ9_Msk
  ADC_SQR2_SQ9_Bit0* = (0x00000001 shl ADC_SQR2_SQ9_Pos) ## !< 0x00000400
  ADC_SQR2_SQ9_Bit1* = (0x00000002 shl ADC_SQR2_SQ9_Pos) ## !< 0x00000800
  ADC_SQR2_SQ9_Bit2* = (0x00000004 shl ADC_SQR2_SQ9_Pos) ## !< 0x00001000
  ADC_SQR2_SQ9_Bit3* = (0x00000008 shl ADC_SQR2_SQ9_Pos) ## !< 0x00002000
  ADC_SQR2_SQ9_Bit4* = (0x00000010 shl ADC_SQR2_SQ9_Pos) ## !< 0x00004000
  ADC_SQR2_SQ10_Pos* = (15)
  ADC_SQR2_SQ10_Msk* = (0x0000001F shl ADC_SQR2_SQ10_Pos) ## !< 0x000F8000
  ADC_SQR2_SQ10* = ADC_SQR2_SQ10_Msk
  ADC_SQR2_SQ10_Bit0* = (0x00000001 shl ADC_SQR2_SQ10_Pos) ## !< 0x00008000
  ADC_SQR2_SQ10_Bit1* = (0x00000002 shl ADC_SQR2_SQ10_Pos) ## !< 0x00010000
  ADC_SQR2_SQ10_Bit2* = (0x00000004 shl ADC_SQR2_SQ10_Pos) ## !< 0x00020000
  ADC_SQR2_SQ10_Bit3* = (0x00000008 shl ADC_SQR2_SQ10_Pos) ## !< 0x00040000
  ADC_SQR2_SQ10_Bit4* = (0x00000010 shl ADC_SQR2_SQ10_Pos) ## !< 0x00080000
  ADC_SQR2_SQ11_Pos* = (20)
  ADC_SQR2_SQ11_Msk* = (0x0000001F shl ADC_SQR2_SQ11_Pos) ## !< 0x01F00000
  ADC_SQR2_SQ11* = ADC_SQR2_SQ11_Msk
  ADC_SQR2_SQ11_Bit0* = (0x00000001 shl ADC_SQR2_SQ11_Pos) ## !< 0x00100000
  ADC_SQR2_SQ11_Bit1* = (0x00000002 shl ADC_SQR2_SQ11_Pos) ## !< 0x00200000
  ADC_SQR2_SQ11_Bit2* = (0x00000004 shl ADC_SQR2_SQ11_Pos) ## !< 0x00400000
  ADC_SQR2_SQ11_Bit3* = (0x00000008 shl ADC_SQR2_SQ11_Pos) ## !< 0x00800000
  ADC_SQR2_SQ11_Bit4* = (0x00000010 shl ADC_SQR2_SQ11_Pos) ## !< 0x01000000
  ADC_SQR2_SQ12_Pos* = (25)
  ADC_SQR2_SQ12_Msk* = (0x0000001F shl ADC_SQR2_SQ12_Pos) ## !< 0x3E000000
  ADC_SQR2_SQ12* = ADC_SQR2_SQ12_Msk
  ADC_SQR2_SQ12_Bit0* = (0x00000001 shl ADC_SQR2_SQ12_Pos) ## !< 0x02000000
  ADC_SQR2_SQ12_Bit1* = (0x00000002 shl ADC_SQR2_SQ12_Pos) ## !< 0x04000000
  ADC_SQR2_SQ12_Bit2* = (0x00000004 shl ADC_SQR2_SQ12_Pos) ## !< 0x08000000
  ADC_SQR2_SQ12_Bit3* = (0x00000008 shl ADC_SQR2_SQ12_Pos) ## !< 0x10000000
  ADC_SQR2_SQ12_Bit4* = (0x00000010 shl ADC_SQR2_SQ12_Pos) ## !< 0x20000000

## ******************  Bit definition for ADC_SQR3 register  ******************

const
  ADC_SQR3_SQ1_Pos* = (0)
  ADC_SQR3_SQ1_Msk* = (0x0000001F shl ADC_SQR3_SQ1_Pos) ## !< 0x0000001F
  ADC_SQR3_SQ1* = ADC_SQR3_SQ1_Msk
  ADC_SQR3_SQ1_Bit0* = (0x00000001 shl ADC_SQR3_SQ1_Pos) ## !< 0x00000001
  ADC_SQR3_SQ1_Bit1* = (0x00000002 shl ADC_SQR3_SQ1_Pos) ## !< 0x00000002
  ADC_SQR3_SQ1_Bit2* = (0x00000004 shl ADC_SQR3_SQ1_Pos) ## !< 0x00000004
  ADC_SQR3_SQ1_Bit3* = (0x00000008 shl ADC_SQR3_SQ1_Pos) ## !< 0x00000008
  ADC_SQR3_SQ1_Bit4* = (0x00000010 shl ADC_SQR3_SQ1_Pos) ## !< 0x00000010
  ADC_SQR3_SQ2_Pos* = (5)
  ADC_SQR3_SQ2_Msk* = (0x0000001F shl ADC_SQR3_SQ2_Pos) ## !< 0x000003E0
  ADC_SQR3_SQ2* = ADC_SQR3_SQ2_Msk
  ADC_SQR3_SQ2_Bit0* = (0x00000001 shl ADC_SQR3_SQ2_Pos) ## !< 0x00000020
  ADC_SQR3_SQ2_Bit1* = (0x00000002 shl ADC_SQR3_SQ2_Pos) ## !< 0x00000040
  ADC_SQR3_SQ2_Bit2* = (0x00000004 shl ADC_SQR3_SQ2_Pos) ## !< 0x00000080
  ADC_SQR3_SQ2_Bit3* = (0x00000008 shl ADC_SQR3_SQ2_Pos) ## !< 0x00000100
  ADC_SQR3_SQ2_Bit4* = (0x00000010 shl ADC_SQR3_SQ2_Pos) ## !< 0x00000200
  ADC_SQR3_SQ3_Pos* = (10)
  ADC_SQR3_SQ3_Msk* = (0x0000001F shl ADC_SQR3_SQ3_Pos) ## !< 0x00007C00
  ADC_SQR3_SQ3* = ADC_SQR3_SQ3_Msk
  ADC_SQR3_SQ3_Bit0* = (0x00000001 shl ADC_SQR3_SQ3_Pos) ## !< 0x00000400
  ADC_SQR3_SQ3_Bit1* = (0x00000002 shl ADC_SQR3_SQ3_Pos) ## !< 0x00000800
  ADC_SQR3_SQ3_Bit2* = (0x00000004 shl ADC_SQR3_SQ3_Pos) ## !< 0x00001000
  ADC_SQR3_SQ3_Bit3* = (0x00000008 shl ADC_SQR3_SQ3_Pos) ## !< 0x00002000
  ADC_SQR3_SQ3_Bit4* = (0x00000010 shl ADC_SQR3_SQ3_Pos) ## !< 0x00004000
  ADC_SQR3_SQ4_Pos* = (15)
  ADC_SQR3_SQ4_Msk* = (0x0000001F shl ADC_SQR3_SQ4_Pos) ## !< 0x000F8000
  ADC_SQR3_SQ4* = ADC_SQR3_SQ4_Msk
  ADC_SQR3_SQ4_Bit0* = (0x00000001 shl ADC_SQR3_SQ4_Pos) ## !< 0x00008000
  ADC_SQR3_SQ4_Bit1* = (0x00000002 shl ADC_SQR3_SQ4_Pos) ## !< 0x00010000
  ADC_SQR3_SQ4_Bit2* = (0x00000004 shl ADC_SQR3_SQ4_Pos) ## !< 0x00020000
  ADC_SQR3_SQ4_Bit3* = (0x00000008 shl ADC_SQR3_SQ4_Pos) ## !< 0x00040000
  ADC_SQR3_SQ4_Bit4* = (0x00000010 shl ADC_SQR3_SQ4_Pos) ## !< 0x00080000
  ADC_SQR3_SQ5_Pos* = (20)
  ADC_SQR3_SQ5_Msk* = (0x0000001F shl ADC_SQR3_SQ5_Pos) ## !< 0x01F00000
  ADC_SQR3_SQ5* = ADC_SQR3_SQ5_Msk
  ADC_SQR3_SQ5_Bit0* = (0x00000001 shl ADC_SQR3_SQ5_Pos) ## !< 0x00100000
  ADC_SQR3_SQ5_Bit1* = (0x00000002 shl ADC_SQR3_SQ5_Pos) ## !< 0x00200000
  ADC_SQR3_SQ5_Bit2* = (0x00000004 shl ADC_SQR3_SQ5_Pos) ## !< 0x00400000
  ADC_SQR3_SQ5_Bit3* = (0x00000008 shl ADC_SQR3_SQ5_Pos) ## !< 0x00800000
  ADC_SQR3_SQ5_Bit4* = (0x00000010 shl ADC_SQR3_SQ5_Pos) ## !< 0x01000000
  ADC_SQR3_SQ6_Pos* = (25)
  ADC_SQR3_SQ6_Msk* = (0x0000001F shl ADC_SQR3_SQ6_Pos) ## !< 0x3E000000
  ADC_SQR3_SQ6* = ADC_SQR3_SQ6_Msk
  ADC_SQR3_SQ6_Bit0* = (0x00000001 shl ADC_SQR3_SQ6_Pos) ## !< 0x02000000
  ADC_SQR3_SQ6_Bit1* = (0x00000002 shl ADC_SQR3_SQ6_Pos) ## !< 0x04000000
  ADC_SQR3_SQ6_Bit2* = (0x00000004 shl ADC_SQR3_SQ6_Pos) ## !< 0x08000000
  ADC_SQR3_SQ6_Bit3* = (0x00000008 shl ADC_SQR3_SQ6_Pos) ## !< 0x10000000
  ADC_SQR3_SQ6_Bit4* = (0x00000010 shl ADC_SQR3_SQ6_Pos) ## !< 0x20000000

## ******************  Bit definition for ADC_JSQR register  ******************

const
  ADC_JSQR_JSQ1_Pos* = (0)
  ADC_JSQR_JSQ1_Msk* = (0x0000001F shl ADC_JSQR_JSQ1_Pos) ## !< 0x0000001F
  ADC_JSQR_JSQ1* = ADC_JSQR_JSQ1_Msk
  ADC_JSQR_JSQ1_Bit0* = (0x00000001 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000001
  ADC_JSQR_JSQ1_Bit1* = (0x00000002 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000002
  ADC_JSQR_JSQ1_Bit2* = (0x00000004 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000004
  ADC_JSQR_JSQ1_Bit3* = (0x00000008 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000008
  ADC_JSQR_JSQ1_Bit4* = (0x00000010 shl ADC_JSQR_JSQ1_Pos) ## !< 0x00000010
  ADC_JSQR_JSQ2_Pos* = (5)
  ADC_JSQR_JSQ2_Msk* = (0x0000001F shl ADC_JSQR_JSQ2_Pos) ## !< 0x000003E0
  ADC_JSQR_JSQ2* = ADC_JSQR_JSQ2_Msk
  ADC_JSQR_JSQ2_Bit0* = (0x00000001 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00000020
  ADC_JSQR_JSQ2_Bit1* = (0x00000002 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00000040
  ADC_JSQR_JSQ2_Bit2* = (0x00000004 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00000080
  ADC_JSQR_JSQ2_Bit3* = (0x00000008 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00000100
  ADC_JSQR_JSQ2_Bit4* = (0x00000010 shl ADC_JSQR_JSQ2_Pos) ## !< 0x00000200
  ADC_JSQR_JSQ3_Pos* = (10)
  ADC_JSQR_JSQ3_Msk* = (0x0000001F shl ADC_JSQR_JSQ3_Pos) ## !< 0x00007C00
  ADC_JSQR_JSQ3* = ADC_JSQR_JSQ3_Msk
  ADC_JSQR_JSQ3_Bit0* = (0x00000001 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00000400
  ADC_JSQR_JSQ3_Bit1* = (0x00000002 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00000800
  ADC_JSQR_JSQ3_Bit2* = (0x00000004 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00001000
  ADC_JSQR_JSQ3_Bit3* = (0x00000008 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00002000
  ADC_JSQR_JSQ3_Bit4* = (0x00000010 shl ADC_JSQR_JSQ3_Pos) ## !< 0x00004000
  ADC_JSQR_JSQ4_Pos* = (15)
  ADC_JSQR_JSQ4_Msk* = (0x0000001F shl ADC_JSQR_JSQ4_Pos) ## !< 0x000F8000
  ADC_JSQR_JSQ4* = ADC_JSQR_JSQ4_Msk
  ADC_JSQR_JSQ4_Bit0* = (0x00000001 shl ADC_JSQR_JSQ4_Pos) ## !< 0x00008000
  ADC_JSQR_JSQ4_Bit1* = (0x00000002 shl ADC_JSQR_JSQ4_Pos) ## !< 0x00010000
  ADC_JSQR_JSQ4_Bit2* = (0x00000004 shl ADC_JSQR_JSQ4_Pos) ## !< 0x00020000
  ADC_JSQR_JSQ4_Bit3* = (0x00000008 shl ADC_JSQR_JSQ4_Pos) ## !< 0x00040000
  ADC_JSQR_JSQ4_Bit4* = (0x00000010 shl ADC_JSQR_JSQ4_Pos) ## !< 0x00080000
  ADC_JSQR_JL_Pos* = (20)
  ADC_JSQR_JL_Msk* = (0x00000003 shl ADC_JSQR_JL_Pos) ## !< 0x00300000
  ADC_JSQR_JL* = ADC_JSQR_JL_Msk
  ADC_JSQR_JL_Bit0* = (0x00000001 shl ADC_JSQR_JL_Pos) ## !< 0x00100000
  ADC_JSQR_JL_Bit1* = (0x00000002 shl ADC_JSQR_JL_Pos) ## !< 0x00200000

## ******************  Bit definition for ADC_JDR1 register  ******************

const
  ADC_JDR1_JDATA_Pos* = (0)
  ADC_JDR1_JDATA_Msk* = (0x0000FFFF shl ADC_JDR1_JDATA_Pos) ## !< 0x0000FFFF
  ADC_JDR1_JDATA* = ADC_JDR1_JDATA_Msk

## ******************  Bit definition for ADC_JDR2 register  ******************

const
  ADC_JDR2_JDATA_Pos* = (0)
  ADC_JDR2_JDATA_Msk* = (0x0000FFFF shl ADC_JDR2_JDATA_Pos) ## !< 0x0000FFFF
  ADC_JDR2_JDATA* = ADC_JDR2_JDATA_Msk

## ******************  Bit definition for ADC_JDR3 register  ******************

const
  ADC_JDR3_JDATA_Pos* = (0)
  ADC_JDR3_JDATA_Msk* = (0x0000FFFF shl ADC_JDR3_JDATA_Pos) ## !< 0x0000FFFF
  ADC_JDR3_JDATA* = ADC_JDR3_JDATA_Msk

## ******************  Bit definition for ADC_JDR4 register  ******************

const
  ADC_JDR4_JDATA_Pos* = (0)
  ADC_JDR4_JDATA_Msk* = (0x0000FFFF shl ADC_JDR4_JDATA_Pos) ## !< 0x0000FFFF
  ADC_JDR4_JDATA* = ADC_JDR4_JDATA_Msk

## *******************  Bit definition for ADC_DR register  *******************

const
  ADC_DR_DATA_Pos* = (0)
  ADC_DR_DATA_Msk* = (0x0000FFFF shl ADC_DR_DATA_Pos) ## !< 0x0000FFFF
  ADC_DR_DATA* = ADC_DR_DATA_Msk
  ADC_DR_ADC2DATA_Pos* = (16)
  ADC_DR_ADC2DATA_Msk* = (0x0000FFFF shl ADC_DR_ADC2DATA_Pos) ## !< 0xFFFF0000
  ADC_DR_ADC2DATA* = ADC_DR_ADC2DATA_Msk

## ******************  Bit definition for ADC_CSR register  *******************

const
  ADC_CSR_AWD1_Pos* = (0)
  ADC_CSR_AWD1_Msk* = (0x00000001 shl ADC_CSR_AWD1_Pos) ## !< 0x00000001
  ADC_CSR_AWD1* = ADC_CSR_AWD1_Msk
  ADC_CSR_EOC1_Pos* = (1)
  ADC_CSR_EOC1_Msk* = (0x00000001 shl ADC_CSR_EOC1_Pos) ## !< 0x00000002
  ADC_CSR_EOC1* = ADC_CSR_EOC1_Msk
  ADC_CSR_JEOC1_Pos* = (2)
  ADC_CSR_JEOC1_Msk* = (0x00000001 shl ADC_CSR_JEOC1_Pos) ## !< 0x00000004
  ADC_CSR_JEOC1* = ADC_CSR_JEOC1_Msk
  ADC_CSR_JSTRT1_Pos* = (3)
  ADC_CSR_JSTRT1_Msk* = (0x00000001 shl ADC_CSR_JSTRT1_Pos) ## !< 0x00000008
  ADC_CSR_JSTRT1* = ADC_CSR_JSTRT1_Msk
  ADC_CSR_STRT1_Pos* = (4)
  ADC_CSR_STRT1_Msk* = (0x00000001 shl ADC_CSR_STRT1_Pos) ## !< 0x00000010
  ADC_CSR_STRT1* = ADC_CSR_STRT1_Msk
  ADC_CSR_OVR1_Pos* = (5)
  ADC_CSR_OVR1_Msk* = (0x00000001 shl ADC_CSR_OVR1_Pos) ## !< 0x00000020
  ADC_CSR_OVR1* = ADC_CSR_OVR1_Msk

##  Legacy defines

const
  ADC_CSR_DOVR1* = ADC_CSR_OVR1

## ******************  Bit definition for ADC_CCR register  *******************

const
  ADC_CCR_MULTI_Pos* = (0)
  ADC_CCR_MULTI_Msk* = (0x0000001F shl ADC_CCR_MULTI_Pos) ## !< 0x0000001F
  ADC_CCR_MULTI* = ADC_CCR_MULTI_Msk
  ADC_CCR_MULTI_Bit0* = (0x00000001 shl ADC_CCR_MULTI_Pos) ## !< 0x00000001
  ADC_CCR_MULTI_Bit1* = (0x00000002 shl ADC_CCR_MULTI_Pos) ## !< 0x00000002
  ADC_CCR_MULTI_Bit2* = (0x00000004 shl ADC_CCR_MULTI_Pos) ## !< 0x00000004
  ADC_CCR_MULTI_Bit3* = (0x00000008 shl ADC_CCR_MULTI_Pos) ## !< 0x00000008
  ADC_CCR_MULTI_Bit4* = (0x00000010 shl ADC_CCR_MULTI_Pos) ## !< 0x00000010
  ADC_CCR_DELAY_Pos* = (8)
  ADC_CCR_DELAY_Msk* = (0x0000000F shl ADC_CCR_DELAY_Pos) ## !< 0x00000F00
  ADC_CCR_DELAY* = ADC_CCR_DELAY_Msk
  ADC_CCR_DELAY_Bit0* = (0x00000001 shl ADC_CCR_DELAY_Pos) ## !< 0x00000100
  ADC_CCR_DELAY_Bit1* = (0x00000002 shl ADC_CCR_DELAY_Pos) ## !< 0x00000200
  ADC_CCR_DELAY_Bit2* = (0x00000004 shl ADC_CCR_DELAY_Pos) ## !< 0x00000400
  ADC_CCR_DELAY_Bit3* = (0x00000008 shl ADC_CCR_DELAY_Pos) ## !< 0x00000800
  ADC_CCR_DDS_Pos* = (13)
  ADC_CCR_DDS_Msk* = (0x00000001 shl ADC_CCR_DDS_Pos) ## !< 0x00002000
  ADC_CCR_DDS* = ADC_CCR_DDS_Msk
  ADC_CCR_DMA_Pos* = (14)
  ADC_CCR_DMA_Msk* = (0x00000003 shl ADC_CCR_DMA_Pos) ## !< 0x0000C000
  ADC_CCR_DMA* = ADC_CCR_DMA_Msk
  ADC_CCR_DMA_Bit0* = (0x00000001 shl ADC_CCR_DMA_Pos) ## !< 0x00004000
  ADC_CCR_DMA_Bit1* = (0x00000002 shl ADC_CCR_DMA_Pos) ## !< 0x00008000
  ADC_CCR_ADCPRE_Pos* = (16)
  ADC_CCR_ADCPRE_Msk* = (0x00000003 shl ADC_CCR_ADCPRE_Pos) ## !< 0x00030000
  ADC_CCR_ADCPRE* = ADC_CCR_ADCPRE_Msk
  ADC_CCR_ADCPRE_Bit0* = (0x00000001 shl ADC_CCR_ADCPRE_Pos) ## !< 0x00010000
  ADC_CCR_ADCPRE_Bit1* = (0x00000002 shl ADC_CCR_ADCPRE_Pos) ## !< 0x00020000
  ADC_CCR_VBATE_Pos* = (22)
  ADC_CCR_VBATE_Msk* = (0x00000001 shl ADC_CCR_VBATE_Pos) ## !< 0x00400000
  ADC_CCR_VBATE* = ADC_CCR_VBATE_Msk
  ADC_CCR_TSVREFE_Pos* = (23)
  ADC_CCR_TSVREFE_Msk* = (0x00000001 shl ADC_CCR_TSVREFE_Pos) ## !< 0x00800000
  ADC_CCR_TSVREFE* = ADC_CCR_TSVREFE_Msk

## ******************  Bit definition for ADC_CDR register  *******************

const
  ADC_CDR_DATA1_Pos* = (0)
  ADC_CDR_DATA1_Msk* = (0x0000FFFF shl ADC_CDR_DATA1_Pos) ## !< 0x0000FFFF
  ADC_CDR_DATA1* = ADC_CDR_DATA1_Msk
  ADC_CDR_DATA2_Pos* = (16)
  ADC_CDR_DATA2_Msk* = (0x0000FFFF shl ADC_CDR_DATA2_Pos) ## !< 0xFFFF0000
  ADC_CDR_DATA2* = ADC_CDR_DATA2_Msk

##  Legacy defines

const
  ADC_CDR_RDATA_MST* = ADC_CDR_DATA1
  ADC_CDR_RDATA_SLV* = ADC_CDR_DATA2

## ****************************************************************************
##
##                           CRC calculation unit
##
## ****************************************************************************
## ******************  Bit definition for CRC_DR register  ********************

const
  CRC_DR_DR_Pos* = (0)
  CRC_DR_DR_Msk* = (0xFFFFFFFF shl CRC_DR_DR_Pos) ## !< 0xFFFFFFFF
  CRC_DR_DR* = CRC_DR_DR_Msk

## ******************  Bit definition for CRC_IDR register  *******************

const
  CRC_IDR_IDR_Pos* = (0)
  CRC_IDR_IDR_Msk* = (0x000000FF shl CRC_IDR_IDR_Pos) ## !< 0x000000FF
  CRC_IDR_IDR* = CRC_IDR_IDR_Msk

## *******************  Bit definition for CRC_CR register  *******************

const
  CRC_CR_RESET_Pos* = (0)
  CRC_CR_RESET_Msk* = (0x00000001 shl CRC_CR_RESET_Pos) ## !< 0x00000001
  CRC_CR_RESET* = CRC_CR_RESET_Msk

## ****************************************************************************
##
##                       Digital to Analog Converter
##
## ****************************************************************************
## *******************  Bit definition for DAC_CR register  *******************

const
  DAC_CR_EN1_Pos* = (0)
  DAC_CR_EN1_Msk* = (0x00000001 shl DAC_CR_EN1_Pos) ## !< 0x00000001
  DAC_CR_EN1* = DAC_CR_EN1_Msk
  DAC_CR_BOFF1_Pos* = (1)
  DAC_CR_BOFF1_Msk* = (0x00000001 shl DAC_CR_BOFF1_Pos) ## !< 0x00000002
  DAC_CR_BOFF1* = DAC_CR_BOFF1_Msk
  DAC_CR_TEN1_Pos* = (2)
  DAC_CR_TEN1_Msk* = (0x00000001 shl DAC_CR_TEN1_Pos) ## !< 0x00000004
  DAC_CR_TEN1* = DAC_CR_TEN1_Msk
  DAC_CR_TSEL1_Pos* = (3)
  DAC_CR_TSEL1_Msk* = (0x00000007 shl DAC_CR_TSEL1_Pos) ## !< 0x00000038
  DAC_CR_TSEL1* = DAC_CR_TSEL1_Msk
  DAC_CR_TSEL1_Bit0* = (0x00000001 shl DAC_CR_TSEL1_Pos) ## !< 0x00000008
  DAC_CR_TSEL1_Bit1* = (0x00000002 shl DAC_CR_TSEL1_Pos) ## !< 0x00000010
  DAC_CR_TSEL1_Bit2* = (0x00000004 shl DAC_CR_TSEL1_Pos) ## !< 0x00000020
  DAC_CR_WAVE1_Pos* = (6)
  DAC_CR_WAVE1_Msk* = (0x00000003 shl DAC_CR_WAVE1_Pos) ## !< 0x000000C0
  DAC_CR_WAVE1* = DAC_CR_WAVE1_Msk
  DAC_CR_WAVE1_Bit0* = (0x00000001 shl DAC_CR_WAVE1_Pos) ## !< 0x00000040
  DAC_CR_WAVE1_Bit1* = (0x00000002 shl DAC_CR_WAVE1_Pos) ## !< 0x00000080
  DAC_CR_MAMP1_Pos* = (8)
  DAC_CR_MAMP1_Msk* = (0x0000000F shl DAC_CR_MAMP1_Pos) ## !< 0x00000F00
  DAC_CR_MAMP1* = DAC_CR_MAMP1_Msk
  DAC_CR_MAMP1_Bit0* = (0x00000001 shl DAC_CR_MAMP1_Pos) ## !< 0x00000100
  DAC_CR_MAMP1_Bit1* = (0x00000002 shl DAC_CR_MAMP1_Pos) ## !< 0x00000200
  DAC_CR_MAMP1_Bit2* = (0x00000004 shl DAC_CR_MAMP1_Pos) ## !< 0x00000400
  DAC_CR_MAMP1_Bit3* = (0x00000008 shl DAC_CR_MAMP1_Pos) ## !< 0x00000800
  DAC_CR_DMAEN1_Pos* = (12)
  DAC_CR_DMAEN1_Msk* = (0x00000001 shl DAC_CR_DMAEN1_Pos) ## !< 0x00001000
  DAC_CR_DMAEN1* = DAC_CR_DMAEN1_Msk
  DAC_CR_DMAUDRIE1_Pos* = (13)
  DAC_CR_DMAUDRIE1_Msk* = (0x00000001 shl DAC_CR_DMAUDRIE1_Pos) ## !< 0x00002000
  DAC_CR_DMAUDRIE1* = DAC_CR_DMAUDRIE1_Msk
  DAC_CR_EN2_Pos* = (16)
  DAC_CR_EN2_Msk* = (0x00000001 shl DAC_CR_EN2_Pos) ## !< 0x00010000
  DAC_CR_EN2* = DAC_CR_EN2_Msk
  DAC_CR_BOFF2_Pos* = (17)
  DAC_CR_BOFF2_Msk* = (0x00000001 shl DAC_CR_BOFF2_Pos) ## !< 0x00020000
  DAC_CR_BOFF2* = DAC_CR_BOFF2_Msk
  DAC_CR_TEN2_Pos* = (18)
  DAC_CR_TEN2_Msk* = (0x00000001 shl DAC_CR_TEN2_Pos) ## !< 0x00040000
  DAC_CR_TEN2* = DAC_CR_TEN2_Msk
  DAC_CR_TSEL2_Pos* = (19)
  DAC_CR_TSEL2_Msk* = (0x00000007 shl DAC_CR_TSEL2_Pos) ## !< 0x00380000
  DAC_CR_TSEL2* = DAC_CR_TSEL2_Msk
  DAC_CR_TSEL2_Bit0* = (0x00000001 shl DAC_CR_TSEL2_Pos) ## !< 0x00080000
  DAC_CR_TSEL2_Bit1* = (0x00000002 shl DAC_CR_TSEL2_Pos) ## !< 0x00100000
  DAC_CR_TSEL2_Bit2* = (0x00000004 shl DAC_CR_TSEL2_Pos) ## !< 0x00200000
  DAC_CR_WAVE2_Pos* = (22)
  DAC_CR_WAVE2_Msk* = (0x00000003 shl DAC_CR_WAVE2_Pos) ## !< 0x00C00000
  DAC_CR_WAVE2* = DAC_CR_WAVE2_Msk
  DAC_CR_WAVE2_Bit0* = (0x00000001 shl DAC_CR_WAVE2_Pos) ## !< 0x00400000
  DAC_CR_WAVE2_Bit1* = (0x00000002 shl DAC_CR_WAVE2_Pos) ## !< 0x00800000
  DAC_CR_MAMP2_Pos* = (24)
  DAC_CR_MAMP2_Msk* = (0x0000000F shl DAC_CR_MAMP2_Pos) ## !< 0x0F000000
  DAC_CR_MAMP2* = DAC_CR_MAMP2_Msk
  DAC_CR_MAMP2_Bit0* = (0x00000001 shl DAC_CR_MAMP2_Pos) ## !< 0x01000000
  DAC_CR_MAMP2_Bit1* = (0x00000002 shl DAC_CR_MAMP2_Pos) ## !< 0x02000000
  DAC_CR_MAMP2_Bit2* = (0x00000004 shl DAC_CR_MAMP2_Pos) ## !< 0x04000000
  DAC_CR_MAMP2_Bit3* = (0x00000008 shl DAC_CR_MAMP2_Pos) ## !< 0x08000000
  DAC_CR_DMAEN2_Pos* = (28)
  DAC_CR_DMAEN2_Msk* = (0x00000001 shl DAC_CR_DMAEN2_Pos) ## !< 0x10000000
  DAC_CR_DMAEN2* = DAC_CR_DMAEN2_Msk
  DAC_CR_DMAUDRIE2_Pos* = (29)
  DAC_CR_DMAUDRIE2_Msk* = (0x00000001 shl DAC_CR_DMAUDRIE2_Pos) ## !< 0x20000000
  DAC_CR_DMAUDRIE2* = DAC_CR_DMAUDRIE2_Msk

## ****************  Bit definition for DAC_SWTRIGR register  *****************

const
  DAC_SWTRIGR_SWTRIG1_Pos* = (0)
  DAC_SWTRIGR_SWTRIG1_Msk* = (0x00000001 shl DAC_SWTRIGR_SWTRIG1_Pos) ## !< 0x00000001
  DAC_SWTRIGR_SWTRIG1* = DAC_SWTRIGR_SWTRIG1_Msk
  DAC_SWTRIGR_SWTRIG2_Pos* = (1)
  DAC_SWTRIGR_SWTRIG2_Msk* = (0x00000001 shl DAC_SWTRIGR_SWTRIG2_Pos) ## !< 0x00000002
  DAC_SWTRIGR_SWTRIG2* = DAC_SWTRIGR_SWTRIG2_Msk

## ****************  Bit definition for DAC_DHR12R1 register  *****************

const
  DAC_DHR12R1_DACC1DHR_Pos* = (0)
  DAC_DHR12R1_DACC1DHR_Msk* = (0x00000FFF shl DAC_DHR12R1_DACC1DHR_Pos) ## !< 0x00000FFF
  DAC_DHR12R1_DACC1DHR* = DAC_DHR12R1_DACC1DHR_Msk

## ****************  Bit definition for DAC_DHR12L1 register  *****************

const
  DAC_DHR12L1_DACC1DHR_Pos* = (4)
  DAC_DHR12L1_DACC1DHR_Msk* = (0x00000FFF shl DAC_DHR12L1_DACC1DHR_Pos) ## !< 0x0000FFF0
  DAC_DHR12L1_DACC1DHR* = DAC_DHR12L1_DACC1DHR_Msk

## *****************  Bit definition for DAC_DHR8R1 register  *****************

const
  DAC_DHR8R1_DACC1DHR_Pos* = (0)
  DAC_DHR8R1_DACC1DHR_Msk* = (0x000000FF shl DAC_DHR8R1_DACC1DHR_Pos) ## !< 0x000000FF
  DAC_DHR8R1_DACC1DHR* = DAC_DHR8R1_DACC1DHR_Msk

## ****************  Bit definition for DAC_DHR12R2 register  *****************

const
  DAC_DHR12R2_DACC2DHR_Pos* = (0)
  DAC_DHR12R2_DACC2DHR_Msk* = (0x00000FFF shl DAC_DHR12R2_DACC2DHR_Pos) ## !< 0x00000FFF
  DAC_DHR12R2_DACC2DHR* = DAC_DHR12R2_DACC2DHR_Msk

## ****************  Bit definition for DAC_DHR12L2 register  *****************

const
  DAC_DHR12L2_DACC2DHR_Pos* = (4)
  DAC_DHR12L2_DACC2DHR_Msk* = (0x00000FFF shl DAC_DHR12L2_DACC2DHR_Pos) ## !< 0x0000FFF0
  DAC_DHR12L2_DACC2DHR* = DAC_DHR12L2_DACC2DHR_Msk

## *****************  Bit definition for DAC_DHR8R2 register  *****************

const
  DAC_DHR8R2_DACC2DHR_Pos* = (0)
  DAC_DHR8R2_DACC2DHR_Msk* = (0x000000FF shl DAC_DHR8R2_DACC2DHR_Pos) ## !< 0x000000FF
  DAC_DHR8R2_DACC2DHR* = DAC_DHR8R2_DACC2DHR_Msk

## ****************  Bit definition for DAC_DHR12RD register  *****************

const
  DAC_DHR12RD_DACC1DHR_Pos* = (0)
  DAC_DHR12RD_DACC1DHR_Msk* = (0x00000FFF shl DAC_DHR12RD_DACC1DHR_Pos) ## !< 0x00000FFF
  DAC_DHR12RD_DACC1DHR* = DAC_DHR12RD_DACC1DHR_Msk
  DAC_DHR12RD_DACC2DHR_Pos* = (16)
  DAC_DHR12RD_DACC2DHR_Msk* = (0x00000FFF shl DAC_DHR12RD_DACC2DHR_Pos) ## !< 0x0FFF0000
  DAC_DHR12RD_DACC2DHR* = DAC_DHR12RD_DACC2DHR_Msk

## ****************  Bit definition for DAC_DHR12LD register  *****************

const
  DAC_DHR12LD_DACC1DHR_Pos* = (4)
  DAC_DHR12LD_DACC1DHR_Msk* = (0x00000FFF shl DAC_DHR12LD_DACC1DHR_Pos) ## !< 0x0000FFF0
  DAC_DHR12LD_DACC1DHR* = DAC_DHR12LD_DACC1DHR_Msk
  DAC_DHR12LD_DACC2DHR_Pos* = (20)
  DAC_DHR12LD_DACC2DHR_Msk* = (0x00000FFF shl DAC_DHR12LD_DACC2DHR_Pos) ## !< 0xFFF00000
  DAC_DHR12LD_DACC2DHR* = DAC_DHR12LD_DACC2DHR_Msk

## *****************  Bit definition for DAC_DHR8RD register  *****************

const
  DAC_DHR8RD_DACC1DHR_Pos* = (0)
  DAC_DHR8RD_DACC1DHR_Msk* = (0x000000FF shl DAC_DHR8RD_DACC1DHR_Pos) ## !< 0x000000FF
  DAC_DHR8RD_DACC1DHR* = DAC_DHR8RD_DACC1DHR_Msk
  DAC_DHR8RD_DACC2DHR_Pos* = (8)
  DAC_DHR8RD_DACC2DHR_Msk* = (0x000000FF shl DAC_DHR8RD_DACC2DHR_Pos) ## !< 0x0000FF00
  DAC_DHR8RD_DACC2DHR* = DAC_DHR8RD_DACC2DHR_Msk

## ******************  Bit definition for DAC_DOR1 register  ******************

const
  DAC_DOR1_DACC1DOR_Pos* = (0)
  DAC_DOR1_DACC1DOR_Msk* = (0x00000FFF shl DAC_DOR1_DACC1DOR_Pos) ## !< 0x00000FFF
  DAC_DOR1_DACC1DOR* = DAC_DOR1_DACC1DOR_Msk

## ******************  Bit definition for DAC_DOR2 register  ******************

const
  DAC_DOR2_DACC2DOR_Pos* = (0)
  DAC_DOR2_DACC2DOR_Msk* = (0x00000FFF shl DAC_DOR2_DACC2DOR_Pos) ## !< 0x00000FFF
  DAC_DOR2_DACC2DOR* = DAC_DOR2_DACC2DOR_Msk

## *******************  Bit definition for DAC_SR register  *******************

const
  DAC_SR_DMAUDR1_Pos* = (13)
  DAC_SR_DMAUDR1_Msk* = (0x00000001 shl DAC_SR_DMAUDR1_Pos) ## !< 0x00002000
  DAC_SR_DMAUDR1* = DAC_SR_DMAUDR1_Msk
  DAC_SR_DMAUDR2_Pos* = (29)
  DAC_SR_DMAUDR2_Msk* = (0x00000001 shl DAC_SR_DMAUDR2_Pos) ## !< 0x20000000
  DAC_SR_DMAUDR2* = DAC_SR_DMAUDR2_Msk

## ****************************************************************************
##
##                              DMA Controller
##
## ****************************************************************************
## *******************  Bits definition for DMA_SxCR register  ****************

const
  DMA_SxCR_CHSEL_Pos* = (25)
  DMA_SxCR_CHSEL_Msk* = (0x00000007 shl DMA_SxCR_CHSEL_Pos) ## !< 0x0E000000
  DMA_SxCR_CHSEL* = DMA_SxCR_CHSEL_Msk
  DMA_SxCR_CHSEL_Bit0* = 0x02000000
  DMA_SxCR_CHSEL_Bit1* = 0x04000000
  DMA_SxCR_CHSEL_Bit2* = 0x08000000
  DMA_SxCR_MBURST_Pos* = (23)
  DMA_SxCR_MBURST_Msk* = (0x00000003 shl DMA_SxCR_MBURST_Pos) ## !< 0x01800000
  DMA_SxCR_MBURST* = DMA_SxCR_MBURST_Msk
  DMA_SxCR_MBURST_Bit0* = (0x00000001 shl DMA_SxCR_MBURST_Pos) ## !< 0x00800000
  DMA_SxCR_MBURST_Bit1* = (0x00000002 shl DMA_SxCR_MBURST_Pos) ## !< 0x01000000
  DMA_SxCR_PBURST_Pos* = (21)
  DMA_SxCR_PBURST_Msk* = (0x00000003 shl DMA_SxCR_PBURST_Pos) ## !< 0x00600000
  DMA_SxCR_PBURST* = DMA_SxCR_PBURST_Msk
  DMA_SxCR_PBURST_Bit0* = (0x00000001 shl DMA_SxCR_PBURST_Pos) ## !< 0x00200000
  DMA_SxCR_PBURST_Bit1* = (0x00000002 shl DMA_SxCR_PBURST_Pos) ## !< 0x00400000
  DMA_SxCR_CT_Pos* = (19)
  DMA_SxCR_CT_Msk* = (0x00000001 shl DMA_SxCR_CT_Pos) ## !< 0x00080000
  DMA_SxCR_CT* = DMA_SxCR_CT_Msk
  DMA_SxCR_DBM_Pos* = (18)
  DMA_SxCR_DBM_Msk* = (0x00000001 shl DMA_SxCR_DBM_Pos) ## !< 0x00040000
  DMA_SxCR_DBM* = DMA_SxCR_DBM_Msk
  DMA_SxCR_PL_Pos* = (16)
  DMA_SxCR_PL_Msk* = (0x00000003 shl DMA_SxCR_PL_Pos) ## !< 0x00030000
  DMA_SxCR_PL* = DMA_SxCR_PL_Msk
  DMA_SxCR_PL_Bit0* = (0x00000001 shl DMA_SxCR_PL_Pos) ## !< 0x00010000
  DMA_SxCR_PL_Bit1* = (0x00000002 shl DMA_SxCR_PL_Pos) ## !< 0x00020000
  DMA_SxCR_PINCOS_Pos* = (15)
  DMA_SxCR_PINCOS_Msk* = (0x00000001 shl DMA_SxCR_PINCOS_Pos) ## !< 0x00008000
  DMA_SxCR_PINCOS* = DMA_SxCR_PINCOS_Msk
  DMA_SxCR_MSIZE_Pos* = (13)
  DMA_SxCR_MSIZE_Msk* = (0x00000003 shl DMA_SxCR_MSIZE_Pos) ## !< 0x00006000
  DMA_SxCR_MSIZE* = DMA_SxCR_MSIZE_Msk
  DMA_SxCR_MSIZE_Bit0* = (0x00000001 shl DMA_SxCR_MSIZE_Pos) ## !< 0x00002000
  DMA_SxCR_MSIZE_Bit1* = (0x00000002 shl DMA_SxCR_MSIZE_Pos) ## !< 0x00004000
  DMA_SxCR_PSIZE_Pos* = (11)
  DMA_SxCR_PSIZE_Msk* = (0x00000003 shl DMA_SxCR_PSIZE_Pos) ## !< 0x00001800
  DMA_SxCR_PSIZE* = DMA_SxCR_PSIZE_Msk
  DMA_SxCR_PSIZE_Bit0* = (0x00000001 shl DMA_SxCR_PSIZE_Pos) ## !< 0x00000800
  DMA_SxCR_PSIZE_Bit1* = (0x00000002 shl DMA_SxCR_PSIZE_Pos) ## !< 0x00001000
  DMA_SxCR_MINC_Pos* = (10)
  DMA_SxCR_MINC_Msk* = (0x00000001 shl DMA_SxCR_MINC_Pos) ## !< 0x00000400
  DMA_SxCR_MINC* = DMA_SxCR_MINC_Msk
  DMA_SxCR_PINC_Pos* = (9)
  DMA_SxCR_PINC_Msk* = (0x00000001 shl DMA_SxCR_PINC_Pos) ## !< 0x00000200
  DMA_SxCR_PINC* = DMA_SxCR_PINC_Msk
  DMA_SxCR_CIRC_Pos* = (8)
  DMA_SxCR_CIRC_Msk* = (0x00000001 shl DMA_SxCR_CIRC_Pos) ## !< 0x00000100
  DMA_SxCR_CIRC* = DMA_SxCR_CIRC_Msk
  DMA_SxCR_DIR_Pos* = (6)
  DMA_SxCR_DIR_Msk* = (0x00000003 shl DMA_SxCR_DIR_Pos) ## !< 0x000000C0
  DMA_SxCR_DIR* = DMA_SxCR_DIR_Msk
  DMA_SxCR_DIR_Bit0* = (0x00000001 shl DMA_SxCR_DIR_Pos) ## !< 0x00000040
  DMA_SxCR_DIR_Bit1* = (0x00000002 shl DMA_SxCR_DIR_Pos) ## !< 0x00000080
  DMA_SxCR_PFCTRL_Pos* = (5)
  DMA_SxCR_PFCTRL_Msk* = (0x00000001 shl DMA_SxCR_PFCTRL_Pos) ## !< 0x00000020
  DMA_SxCR_PFCTRL* = DMA_SxCR_PFCTRL_Msk
  DMA_SxCR_TCIE_Pos* = (4)
  DMA_SxCR_TCIE_Msk* = (0x00000001 shl DMA_SxCR_TCIE_Pos) ## !< 0x00000010
  DMA_SxCR_TCIE* = DMA_SxCR_TCIE_Msk
  DMA_SxCR_HTIE_Pos* = (3)
  DMA_SxCR_HTIE_Msk* = (0x00000001 shl DMA_SxCR_HTIE_Pos) ## !< 0x00000008
  DMA_SxCR_HTIE* = DMA_SxCR_HTIE_Msk
  DMA_SxCR_TEIE_Pos* = (2)
  DMA_SxCR_TEIE_Msk* = (0x00000001 shl DMA_SxCR_TEIE_Pos) ## !< 0x00000004
  DMA_SxCR_TEIE* = DMA_SxCR_TEIE_Msk
  DMA_SxCR_DMEIE_Pos* = (1)
  DMA_SxCR_DMEIE_Msk* = (0x00000001 shl DMA_SxCR_DMEIE_Pos) ## !< 0x00000002
  DMA_SxCR_DMEIE* = DMA_SxCR_DMEIE_Msk
  DMA_SxCR_EN_Pos* = (0)
  DMA_SxCR_EN_Msk* = (0x00000001 shl DMA_SxCR_EN_Pos) ## !< 0x00000001
  DMA_SxCR_EN* = DMA_SxCR_EN_Msk

##  Legacy defines

const
  DMA_SxCR_ACK_Pos* = (20)
  DMA_SxCR_ACK_Msk* = (0x00000001 shl DMA_SxCR_ACK_Pos) ## !< 0x00100000
  DMA_SxCR_ACK* = DMA_SxCR_ACK_Msk

## *******************  Bits definition for DMA_SxCNDTR register  *************

const
  DMA_SxNDT_Pos* = (0)
  DMA_SxNDT_Msk* = (0x0000FFFF shl DMA_SxNDT_Pos) ## !< 0x0000FFFF
  DMA_SxNDT* = DMA_SxNDT_Msk
  DMA_SxNDT_Bit0* = (0x00000001 shl DMA_SxNDT_Pos) ## !< 0x00000001
  DMA_SxNDT_Bit1* = (0x00000002 shl DMA_SxNDT_Pos) ## !< 0x00000002
  DMA_SxNDT_Bit2* = (0x00000004 shl DMA_SxNDT_Pos) ## !< 0x00000004
  DMA_SxNDT_Bit3* = (0x00000008 shl DMA_SxNDT_Pos) ## !< 0x00000008
  DMA_SxNDT_Bit4* = (0x00000010 shl DMA_SxNDT_Pos) ## !< 0x00000010
  DMA_SxNDT_Bit5* = (0x00000020 shl DMA_SxNDT_Pos) ## !< 0x00000020
  DMA_SxNDT_Bit6* = (0x00000040 shl DMA_SxNDT_Pos) ## !< 0x00000040
  DMA_SxNDT_Bit7* = (0x00000080 shl DMA_SxNDT_Pos) ## !< 0x00000080
  DMA_SxNDT_Bit8* = (0x00000100 shl DMA_SxNDT_Pos) ## !< 0x00000100
  DMA_SxNDT_Bit9* = (0x00000200 shl DMA_SxNDT_Pos) ## !< 0x00000200
  DMA_SxNDT_Bit10* = (0x00000400 shl DMA_SxNDT_Pos) ## !< 0x00000400
  DMA_SxNDT_Bit11* = (0x00000800 shl DMA_SxNDT_Pos) ## !< 0x00000800
  DMA_SxNDT_Bit12* = (0x00001000 shl DMA_SxNDT_Pos) ## !< 0x00001000
  DMA_SxNDT_Bit13* = (0x00002000 shl DMA_SxNDT_Pos) ## !< 0x00002000
  DMA_SxNDT_Bit14* = (0x00004000 shl DMA_SxNDT_Pos) ## !< 0x00004000
  DMA_SxNDT_Bit15* = (0x00008000 shl DMA_SxNDT_Pos) ## !< 0x00008000

## *******************  Bits definition for DMA_SxFCR register  ***************

const
  DMA_SxFCR_FEIE_Pos* = (7)
  DMA_SxFCR_FEIE_Msk* = (0x00000001 shl DMA_SxFCR_FEIE_Pos) ## !< 0x00000080
  DMA_SxFCR_FEIE* = DMA_SxFCR_FEIE_Msk
  DMA_SxFCR_FS_Pos* = (3)
  DMA_SxFCR_FS_Msk* = (0x00000007 shl DMA_SxFCR_FS_Pos) ## !< 0x00000038
  DMA_SxFCR_FS* = DMA_SxFCR_FS_Msk
  DMA_SxFCR_FS_Bit0* = (0x00000001 shl DMA_SxFCR_FS_Pos) ## !< 0x00000008
  DMA_SxFCR_FS_Bit1* = (0x00000002 shl DMA_SxFCR_FS_Pos) ## !< 0x00000010
  DMA_SxFCR_FS_Bit2* = (0x00000004 shl DMA_SxFCR_FS_Pos) ## !< 0x00000020
  DMA_SxFCR_DMDIS_Pos* = (2)
  DMA_SxFCR_DMDIS_Msk* = (0x00000001 shl DMA_SxFCR_DMDIS_Pos) ## !< 0x00000004
  DMA_SxFCR_DMDIS* = DMA_SxFCR_DMDIS_Msk
  DMA_SxFCR_FTH_Pos* = (0)
  DMA_SxFCR_FTH_Msk* = (0x00000003 shl DMA_SxFCR_FTH_Pos) ## !< 0x00000003
  DMA_SxFCR_FTH* = DMA_SxFCR_FTH_Msk
  DMA_SxFCR_FTH_Bit0* = (0x00000001 shl DMA_SxFCR_FTH_Pos) ## !< 0x00000001
  DMA_SxFCR_FTH_Bit1* = (0x00000002 shl DMA_SxFCR_FTH_Pos) ## !< 0x00000002

## *******************  Bits definition for DMA_LISR register  ****************

const
  DMA_LISR_TCIF3_Pos* = (27)
  DMA_LISR_TCIF3_Msk* = (0x00000001 shl DMA_LISR_TCIF3_Pos) ## !< 0x08000000
  DMA_LISR_TCIF3* = DMA_LISR_TCIF3_Msk
  DMA_LISR_HTIF3_Pos* = (26)
  DMA_LISR_HTIF3_Msk* = (0x00000001 shl DMA_LISR_HTIF3_Pos) ## !< 0x04000000
  DMA_LISR_HTIF3* = DMA_LISR_HTIF3_Msk
  DMA_LISR_TEIF3_Pos* = (25)
  DMA_LISR_TEIF3_Msk* = (0x00000001 shl DMA_LISR_TEIF3_Pos) ## !< 0x02000000
  DMA_LISR_TEIF3* = DMA_LISR_TEIF3_Msk
  DMA_LISR_DMEIF3_Pos* = (24)
  DMA_LISR_DMEIF3_Msk* = (0x00000001 shl DMA_LISR_DMEIF3_Pos) ## !< 0x01000000
  DMA_LISR_DMEIF3* = DMA_LISR_DMEIF3_Msk
  DMA_LISR_FEIF3_Pos* = (22)
  DMA_LISR_FEIF3_Msk* = (0x00000001 shl DMA_LISR_FEIF3_Pos) ## !< 0x00400000
  DMA_LISR_FEIF3* = DMA_LISR_FEIF3_Msk
  DMA_LISR_TCIF2_Pos* = (21)
  DMA_LISR_TCIF2_Msk* = (0x00000001 shl DMA_LISR_TCIF2_Pos) ## !< 0x00200000
  DMA_LISR_TCIF2* = DMA_LISR_TCIF2_Msk
  DMA_LISR_HTIF2_Pos* = (20)
  DMA_LISR_HTIF2_Msk* = (0x00000001 shl DMA_LISR_HTIF2_Pos) ## !< 0x00100000
  DMA_LISR_HTIF2* = DMA_LISR_HTIF2_Msk
  DMA_LISR_TEIF2_Pos* = (19)
  DMA_LISR_TEIF2_Msk* = (0x00000001 shl DMA_LISR_TEIF2_Pos) ## !< 0x00080000
  DMA_LISR_TEIF2* = DMA_LISR_TEIF2_Msk
  DMA_LISR_DMEIF2_Pos* = (18)
  DMA_LISR_DMEIF2_Msk* = (0x00000001 shl DMA_LISR_DMEIF2_Pos) ## !< 0x00040000
  DMA_LISR_DMEIF2* = DMA_LISR_DMEIF2_Msk
  DMA_LISR_FEIF2_Pos* = (16)
  DMA_LISR_FEIF2_Msk* = (0x00000001 shl DMA_LISR_FEIF2_Pos) ## !< 0x00010000
  DMA_LISR_FEIF2* = DMA_LISR_FEIF2_Msk
  DMA_LISR_TCIF1_Pos* = (11)
  DMA_LISR_TCIF1_Msk* = (0x00000001 shl DMA_LISR_TCIF1_Pos) ## !< 0x00000800
  DMA_LISR_TCIF1* = DMA_LISR_TCIF1_Msk
  DMA_LISR_HTIF1_Pos* = (10)
  DMA_LISR_HTIF1_Msk* = (0x00000001 shl DMA_LISR_HTIF1_Pos) ## !< 0x00000400
  DMA_LISR_HTIF1* = DMA_LISR_HTIF1_Msk
  DMA_LISR_TEIF1_Pos* = (9)
  DMA_LISR_TEIF1_Msk* = (0x00000001 shl DMA_LISR_TEIF1_Pos) ## !< 0x00000200
  DMA_LISR_TEIF1* = DMA_LISR_TEIF1_Msk
  DMA_LISR_DMEIF1_Pos* = (8)
  DMA_LISR_DMEIF1_Msk* = (0x00000001 shl DMA_LISR_DMEIF1_Pos) ## !< 0x00000100
  DMA_LISR_DMEIF1* = DMA_LISR_DMEIF1_Msk
  DMA_LISR_FEIF1_Pos* = (6)
  DMA_LISR_FEIF1_Msk* = (0x00000001 shl DMA_LISR_FEIF1_Pos) ## !< 0x00000040
  DMA_LISR_FEIF1* = DMA_LISR_FEIF1_Msk
  DMA_LISR_TCIF0_Pos* = (5)
  DMA_LISR_TCIF0_Msk* = (0x00000001 shl DMA_LISR_TCIF0_Pos) ## !< 0x00000020
  DMA_LISR_TCIF0* = DMA_LISR_TCIF0_Msk
  DMA_LISR_HTIF0_Pos* = (4)
  DMA_LISR_HTIF0_Msk* = (0x00000001 shl DMA_LISR_HTIF0_Pos) ## !< 0x00000010
  DMA_LISR_HTIF0* = DMA_LISR_HTIF0_Msk
  DMA_LISR_TEIF0_Pos* = (3)
  DMA_LISR_TEIF0_Msk* = (0x00000001 shl DMA_LISR_TEIF0_Pos) ## !< 0x00000008
  DMA_LISR_TEIF0* = DMA_LISR_TEIF0_Msk
  DMA_LISR_DMEIF0_Pos* = (2)
  DMA_LISR_DMEIF0_Msk* = (0x00000001 shl DMA_LISR_DMEIF0_Pos) ## !< 0x00000004
  DMA_LISR_DMEIF0* = DMA_LISR_DMEIF0_Msk
  DMA_LISR_FEIF0_Pos* = (0)
  DMA_LISR_FEIF0_Msk* = (0x00000001 shl DMA_LISR_FEIF0_Pos) ## !< 0x00000001
  DMA_LISR_FEIF0* = DMA_LISR_FEIF0_Msk

## *******************  Bits definition for DMA_HISR register  ****************

const
  DMA_HISR_TCIF7_Pos* = (27)
  DMA_HISR_TCIF7_Msk* = (0x00000001 shl DMA_HISR_TCIF7_Pos) ## !< 0x08000000
  DMA_HISR_TCIF7* = DMA_HISR_TCIF7_Msk
  DMA_HISR_HTIF7_Pos* = (26)
  DMA_HISR_HTIF7_Msk* = (0x00000001 shl DMA_HISR_HTIF7_Pos) ## !< 0x04000000
  DMA_HISR_HTIF7* = DMA_HISR_HTIF7_Msk
  DMA_HISR_TEIF7_Pos* = (25)
  DMA_HISR_TEIF7_Msk* = (0x00000001 shl DMA_HISR_TEIF7_Pos) ## !< 0x02000000
  DMA_HISR_TEIF7* = DMA_HISR_TEIF7_Msk
  DMA_HISR_DMEIF7_Pos* = (24)
  DMA_HISR_DMEIF7_Msk* = (0x00000001 shl DMA_HISR_DMEIF7_Pos) ## !< 0x01000000
  DMA_HISR_DMEIF7* = DMA_HISR_DMEIF7_Msk
  DMA_HISR_FEIF7_Pos* = (22)
  DMA_HISR_FEIF7_Msk* = (0x00000001 shl DMA_HISR_FEIF7_Pos) ## !< 0x00400000
  DMA_HISR_FEIF7* = DMA_HISR_FEIF7_Msk
  DMA_HISR_TCIF6_Pos* = (21)
  DMA_HISR_TCIF6_Msk* = (0x00000001 shl DMA_HISR_TCIF6_Pos) ## !< 0x00200000
  DMA_HISR_TCIF6* = DMA_HISR_TCIF6_Msk
  DMA_HISR_HTIF6_Pos* = (20)
  DMA_HISR_HTIF6_Msk* = (0x00000001 shl DMA_HISR_HTIF6_Pos) ## !< 0x00100000
  DMA_HISR_HTIF6* = DMA_HISR_HTIF6_Msk
  DMA_HISR_TEIF6_Pos* = (19)
  DMA_HISR_TEIF6_Msk* = (0x00000001 shl DMA_HISR_TEIF6_Pos) ## !< 0x00080000
  DMA_HISR_TEIF6* = DMA_HISR_TEIF6_Msk
  DMA_HISR_DMEIF6_Pos* = (18)
  DMA_HISR_DMEIF6_Msk* = (0x00000001 shl DMA_HISR_DMEIF6_Pos) ## !< 0x00040000
  DMA_HISR_DMEIF6* = DMA_HISR_DMEIF6_Msk
  DMA_HISR_FEIF6_Pos* = (16)
  DMA_HISR_FEIF6_Msk* = (0x00000001 shl DMA_HISR_FEIF6_Pos) ## !< 0x00010000
  DMA_HISR_FEIF6* = DMA_HISR_FEIF6_Msk
  DMA_HISR_TCIF5_Pos* = (11)
  DMA_HISR_TCIF5_Msk* = (0x00000001 shl DMA_HISR_TCIF5_Pos) ## !< 0x00000800
  DMA_HISR_TCIF5* = DMA_HISR_TCIF5_Msk
  DMA_HISR_HTIF5_Pos* = (10)
  DMA_HISR_HTIF5_Msk* = (0x00000001 shl DMA_HISR_HTIF5_Pos) ## !< 0x00000400
  DMA_HISR_HTIF5* = DMA_HISR_HTIF5_Msk
  DMA_HISR_TEIF5_Pos* = (9)
  DMA_HISR_TEIF5_Msk* = (0x00000001 shl DMA_HISR_TEIF5_Pos) ## !< 0x00000200
  DMA_HISR_TEIF5* = DMA_HISR_TEIF5_Msk
  DMA_HISR_DMEIF5_Pos* = (8)
  DMA_HISR_DMEIF5_Msk* = (0x00000001 shl DMA_HISR_DMEIF5_Pos) ## !< 0x00000100
  DMA_HISR_DMEIF5* = DMA_HISR_DMEIF5_Msk
  DMA_HISR_FEIF5_Pos* = (6)
  DMA_HISR_FEIF5_Msk* = (0x00000001 shl DMA_HISR_FEIF5_Pos) ## !< 0x00000040
  DMA_HISR_FEIF5* = DMA_HISR_FEIF5_Msk
  DMA_HISR_TCIF4_Pos* = (5)
  DMA_HISR_TCIF4_Msk* = (0x00000001 shl DMA_HISR_TCIF4_Pos) ## !< 0x00000020
  DMA_HISR_TCIF4* = DMA_HISR_TCIF4_Msk
  DMA_HISR_HTIF4_Pos* = (4)
  DMA_HISR_HTIF4_Msk* = (0x00000001 shl DMA_HISR_HTIF4_Pos) ## !< 0x00000010
  DMA_HISR_HTIF4* = DMA_HISR_HTIF4_Msk
  DMA_HISR_TEIF4_Pos* = (3)
  DMA_HISR_TEIF4_Msk* = (0x00000001 shl DMA_HISR_TEIF4_Pos) ## !< 0x00000008
  DMA_HISR_TEIF4* = DMA_HISR_TEIF4_Msk
  DMA_HISR_DMEIF4_Pos* = (2)
  DMA_HISR_DMEIF4_Msk* = (0x00000001 shl DMA_HISR_DMEIF4_Pos) ## !< 0x00000004
  DMA_HISR_DMEIF4* = DMA_HISR_DMEIF4_Msk
  DMA_HISR_FEIF4_Pos* = (0)
  DMA_HISR_FEIF4_Msk* = (0x00000001 shl DMA_HISR_FEIF4_Pos) ## !< 0x00000001
  DMA_HISR_FEIF4* = DMA_HISR_FEIF4_Msk

## *******************  Bits definition for DMA_LIFCR register  ***************

const
  DMA_LIFCR_CTCIF3_Pos* = (27)
  DMA_LIFCR_CTCIF3_Msk* = (0x00000001 shl DMA_LIFCR_CTCIF3_Pos) ## !< 0x08000000
  DMA_LIFCR_CTCIF3* = DMA_LIFCR_CTCIF3_Msk
  DMA_LIFCR_CHTIF3_Pos* = (26)
  DMA_LIFCR_CHTIF3_Msk* = (0x00000001 shl DMA_LIFCR_CHTIF3_Pos) ## !< 0x04000000
  DMA_LIFCR_CHTIF3* = DMA_LIFCR_CHTIF3_Msk
  DMA_LIFCR_CTEIF3_Pos* = (25)
  DMA_LIFCR_CTEIF3_Msk* = (0x00000001 shl DMA_LIFCR_CTEIF3_Pos) ## !< 0x02000000
  DMA_LIFCR_CTEIF3* = DMA_LIFCR_CTEIF3_Msk
  DMA_LIFCR_CDMEIF3_Pos* = (24)
  DMA_LIFCR_CDMEIF3_Msk* = (0x00000001 shl DMA_LIFCR_CDMEIF3_Pos) ## !< 0x01000000
  DMA_LIFCR_CDMEIF3* = DMA_LIFCR_CDMEIF3_Msk
  DMA_LIFCR_CFEIF3_Pos* = (22)
  DMA_LIFCR_CFEIF3_Msk* = (0x00000001 shl DMA_LIFCR_CFEIF3_Pos) ## !< 0x00400000
  DMA_LIFCR_CFEIF3* = DMA_LIFCR_CFEIF3_Msk
  DMA_LIFCR_CTCIF2_Pos* = (21)
  DMA_LIFCR_CTCIF2_Msk* = (0x00000001 shl DMA_LIFCR_CTCIF2_Pos) ## !< 0x00200000
  DMA_LIFCR_CTCIF2* = DMA_LIFCR_CTCIF2_Msk
  DMA_LIFCR_CHTIF2_Pos* = (20)
  DMA_LIFCR_CHTIF2_Msk* = (0x00000001 shl DMA_LIFCR_CHTIF2_Pos) ## !< 0x00100000
  DMA_LIFCR_CHTIF2* = DMA_LIFCR_CHTIF2_Msk
  DMA_LIFCR_CTEIF2_Pos* = (19)
  DMA_LIFCR_CTEIF2_Msk* = (0x00000001 shl DMA_LIFCR_CTEIF2_Pos) ## !< 0x00080000
  DMA_LIFCR_CTEIF2* = DMA_LIFCR_CTEIF2_Msk
  DMA_LIFCR_CDMEIF2_Pos* = (18)
  DMA_LIFCR_CDMEIF2_Msk* = (0x00000001 shl DMA_LIFCR_CDMEIF2_Pos) ## !< 0x00040000
  DMA_LIFCR_CDMEIF2* = DMA_LIFCR_CDMEIF2_Msk
  DMA_LIFCR_CFEIF2_Pos* = (16)
  DMA_LIFCR_CFEIF2_Msk* = (0x00000001 shl DMA_LIFCR_CFEIF2_Pos) ## !< 0x00010000
  DMA_LIFCR_CFEIF2* = DMA_LIFCR_CFEIF2_Msk
  DMA_LIFCR_CTCIF1_Pos* = (11)
  DMA_LIFCR_CTCIF1_Msk* = (0x00000001 shl DMA_LIFCR_CTCIF1_Pos) ## !< 0x00000800
  DMA_LIFCR_CTCIF1* = DMA_LIFCR_CTCIF1_Msk
  DMA_LIFCR_CHTIF1_Pos* = (10)
  DMA_LIFCR_CHTIF1_Msk* = (0x00000001 shl DMA_LIFCR_CHTIF1_Pos) ## !< 0x00000400
  DMA_LIFCR_CHTIF1* = DMA_LIFCR_CHTIF1_Msk
  DMA_LIFCR_CTEIF1_Pos* = (9)
  DMA_LIFCR_CTEIF1_Msk* = (0x00000001 shl DMA_LIFCR_CTEIF1_Pos) ## !< 0x00000200
  DMA_LIFCR_CTEIF1* = DMA_LIFCR_CTEIF1_Msk
  DMA_LIFCR_CDMEIF1_Pos* = (8)
  DMA_LIFCR_CDMEIF1_Msk* = (0x00000001 shl DMA_LIFCR_CDMEIF1_Pos) ## !< 0x00000100
  DMA_LIFCR_CDMEIF1* = DMA_LIFCR_CDMEIF1_Msk
  DMA_LIFCR_CFEIF1_Pos* = (6)
  DMA_LIFCR_CFEIF1_Msk* = (0x00000001 shl DMA_LIFCR_CFEIF1_Pos) ## !< 0x00000040
  DMA_LIFCR_CFEIF1* = DMA_LIFCR_CFEIF1_Msk
  DMA_LIFCR_CTCIF0_Pos* = (5)
  DMA_LIFCR_CTCIF0_Msk* = (0x00000001 shl DMA_LIFCR_CTCIF0_Pos) ## !< 0x00000020
  DMA_LIFCR_CTCIF0* = DMA_LIFCR_CTCIF0_Msk
  DMA_LIFCR_CHTIF0_Pos* = (4)
  DMA_LIFCR_CHTIF0_Msk* = (0x00000001 shl DMA_LIFCR_CHTIF0_Pos) ## !< 0x00000010
  DMA_LIFCR_CHTIF0* = DMA_LIFCR_CHTIF0_Msk
  DMA_LIFCR_CTEIF0_Pos* = (3)
  DMA_LIFCR_CTEIF0_Msk* = (0x00000001 shl DMA_LIFCR_CTEIF0_Pos) ## !< 0x00000008
  DMA_LIFCR_CTEIF0* = DMA_LIFCR_CTEIF0_Msk
  DMA_LIFCR_CDMEIF0_Pos* = (2)
  DMA_LIFCR_CDMEIF0_Msk* = (0x00000001 shl DMA_LIFCR_CDMEIF0_Pos) ## !< 0x00000004
  DMA_LIFCR_CDMEIF0* = DMA_LIFCR_CDMEIF0_Msk
  DMA_LIFCR_CFEIF0_Pos* = (0)
  DMA_LIFCR_CFEIF0_Msk* = (0x00000001 shl DMA_LIFCR_CFEIF0_Pos) ## !< 0x00000001
  DMA_LIFCR_CFEIF0* = DMA_LIFCR_CFEIF0_Msk

## *******************  Bits definition for DMA_HIFCR  register  ***************

const
  DMA_HIFCR_CTCIF7_Pos* = (27)
  DMA_HIFCR_CTCIF7_Msk* = (0x00000001 shl DMA_HIFCR_CTCIF7_Pos) ## !< 0x08000000
  DMA_HIFCR_CTCIF7* = DMA_HIFCR_CTCIF7_Msk
  DMA_HIFCR_CHTIF7_Pos* = (26)
  DMA_HIFCR_CHTIF7_Msk* = (0x00000001 shl DMA_HIFCR_CHTIF7_Pos) ## !< 0x04000000
  DMA_HIFCR_CHTIF7* = DMA_HIFCR_CHTIF7_Msk
  DMA_HIFCR_CTEIF7_Pos* = (25)
  DMA_HIFCR_CTEIF7_Msk* = (0x00000001 shl DMA_HIFCR_CTEIF7_Pos) ## !< 0x02000000
  DMA_HIFCR_CTEIF7* = DMA_HIFCR_CTEIF7_Msk
  DMA_HIFCR_CDMEIF7_Pos* = (24)
  DMA_HIFCR_CDMEIF7_Msk* = (0x00000001 shl DMA_HIFCR_CDMEIF7_Pos) ## !< 0x01000000
  DMA_HIFCR_CDMEIF7* = DMA_HIFCR_CDMEIF7_Msk
  DMA_HIFCR_CFEIF7_Pos* = (22)
  DMA_HIFCR_CFEIF7_Msk* = (0x00000001 shl DMA_HIFCR_CFEIF7_Pos) ## !< 0x00400000
  DMA_HIFCR_CFEIF7* = DMA_HIFCR_CFEIF7_Msk
  DMA_HIFCR_CTCIF6_Pos* = (21)
  DMA_HIFCR_CTCIF6_Msk* = (0x00000001 shl DMA_HIFCR_CTCIF6_Pos) ## !< 0x00200000
  DMA_HIFCR_CTCIF6* = DMA_HIFCR_CTCIF6_Msk
  DMA_HIFCR_CHTIF6_Pos* = (20)
  DMA_HIFCR_CHTIF6_Msk* = (0x00000001 shl DMA_HIFCR_CHTIF6_Pos) ## !< 0x00100000
  DMA_HIFCR_CHTIF6* = DMA_HIFCR_CHTIF6_Msk
  DMA_HIFCR_CTEIF6_Pos* = (19)
  DMA_HIFCR_CTEIF6_Msk* = (0x00000001 shl DMA_HIFCR_CTEIF6_Pos) ## !< 0x00080000
  DMA_HIFCR_CTEIF6* = DMA_HIFCR_CTEIF6_Msk
  DMA_HIFCR_CDMEIF6_Pos* = (18)
  DMA_HIFCR_CDMEIF6_Msk* = (0x00000001 shl DMA_HIFCR_CDMEIF6_Pos) ## !< 0x00040000
  DMA_HIFCR_CDMEIF6* = DMA_HIFCR_CDMEIF6_Msk
  DMA_HIFCR_CFEIF6_Pos* = (16)
  DMA_HIFCR_CFEIF6_Msk* = (0x00000001 shl DMA_HIFCR_CFEIF6_Pos) ## !< 0x00010000
  DMA_HIFCR_CFEIF6* = DMA_HIFCR_CFEIF6_Msk
  DMA_HIFCR_CTCIF5_Pos* = (11)
  DMA_HIFCR_CTCIF5_Msk* = (0x00000001 shl DMA_HIFCR_CTCIF5_Pos) ## !< 0x00000800
  DMA_HIFCR_CTCIF5* = DMA_HIFCR_CTCIF5_Msk
  DMA_HIFCR_CHTIF5_Pos* = (10)
  DMA_HIFCR_CHTIF5_Msk* = (0x00000001 shl DMA_HIFCR_CHTIF5_Pos) ## !< 0x00000400
  DMA_HIFCR_CHTIF5* = DMA_HIFCR_CHTIF5_Msk
  DMA_HIFCR_CTEIF5_Pos* = (9)
  DMA_HIFCR_CTEIF5_Msk* = (0x00000001 shl DMA_HIFCR_CTEIF5_Pos) ## !< 0x00000200
  DMA_HIFCR_CTEIF5* = DMA_HIFCR_CTEIF5_Msk
  DMA_HIFCR_CDMEIF5_Pos* = (8)
  DMA_HIFCR_CDMEIF5_Msk* = (0x00000001 shl DMA_HIFCR_CDMEIF5_Pos) ## !< 0x00000100
  DMA_HIFCR_CDMEIF5* = DMA_HIFCR_CDMEIF5_Msk
  DMA_HIFCR_CFEIF5_Pos* = (6)
  DMA_HIFCR_CFEIF5_Msk* = (0x00000001 shl DMA_HIFCR_CFEIF5_Pos) ## !< 0x00000040
  DMA_HIFCR_CFEIF5* = DMA_HIFCR_CFEIF5_Msk
  DMA_HIFCR_CTCIF4_Pos* = (5)
  DMA_HIFCR_CTCIF4_Msk* = (0x00000001 shl DMA_HIFCR_CTCIF4_Pos) ## !< 0x00000020
  DMA_HIFCR_CTCIF4* = DMA_HIFCR_CTCIF4_Msk
  DMA_HIFCR_CHTIF4_Pos* = (4)
  DMA_HIFCR_CHTIF4_Msk* = (0x00000001 shl DMA_HIFCR_CHTIF4_Pos) ## !< 0x00000010
  DMA_HIFCR_CHTIF4* = DMA_HIFCR_CHTIF4_Msk
  DMA_HIFCR_CTEIF4_Pos* = (3)
  DMA_HIFCR_CTEIF4_Msk* = (0x00000001 shl DMA_HIFCR_CTEIF4_Pos) ## !< 0x00000008
  DMA_HIFCR_CTEIF4* = DMA_HIFCR_CTEIF4_Msk
  DMA_HIFCR_CDMEIF4_Pos* = (2)
  DMA_HIFCR_CDMEIF4_Msk* = (0x00000001 shl DMA_HIFCR_CDMEIF4_Pos) ## !< 0x00000004
  DMA_HIFCR_CDMEIF4* = DMA_HIFCR_CDMEIF4_Msk
  DMA_HIFCR_CFEIF4_Pos* = (0)
  DMA_HIFCR_CFEIF4_Msk* = (0x00000001 shl DMA_HIFCR_CFEIF4_Pos) ## !< 0x00000001
  DMA_HIFCR_CFEIF4* = DMA_HIFCR_CFEIF4_Msk

## *****************  Bit definition for DMA_SxPAR register  *******************

const
  DMA_SxPAR_PA_Pos* = (0)
  DMA_SxPAR_PA_Msk* = (0xFFFFFFFF shl DMA_SxPAR_PA_Pos) ## !< 0xFFFFFFFF
  DMA_SxPAR_PA* = DMA_SxPAR_PA_Msk

## *****************  Bit definition for DMA_SxM0AR register  *******************

const
  DMA_SxM0AR_M0A_Pos* = (0)
  DMA_SxM0AR_M0A_Msk* = (0xFFFFFFFF shl DMA_SxM0AR_M0A_Pos) ## !< 0xFFFFFFFF
  DMA_SxM0AR_M0A* = DMA_SxM0AR_M0A_Msk

## *****************  Bit definition for DMA_SxM1AR register  *******************

const
  DMA_SxM1AR_M1A_Pos* = (0)
  DMA_SxM1AR_M1A_Msk* = (0xFFFFFFFF shl DMA_SxM1AR_M1A_Pos) ## !< 0xFFFFFFFF
  DMA_SxM1AR_M1A* = DMA_SxM1AR_M1A_Msk

## ****************************************************************************
##
##                     External Interrupt/Event Controller
##
## ****************************************************************************
## ******************  Bit definition for EXTI_IMR register  ******************

const
  EXTI_IMR_MR0_Pos* = (0)
  EXTI_IMR_MR0_Msk* = (0x00000001 shl EXTI_IMR_MR0_Pos) ## !< 0x00000001
  EXTI_IMR_MR0* = EXTI_IMR_MR0_Msk
  EXTI_IMR_MR1_Pos* = (1)
  EXTI_IMR_MR1_Msk* = (0x00000001 shl EXTI_IMR_MR1_Pos) ## !< 0x00000002
  EXTI_IMR_MR1* = EXTI_IMR_MR1_Msk
  EXTI_IMR_MR2_Pos* = (2)
  EXTI_IMR_MR2_Msk* = (0x00000001 shl EXTI_IMR_MR2_Pos) ## !< 0x00000004
  EXTI_IMR_MR2* = EXTI_IMR_MR2_Msk
  EXTI_IMR_MR3_Pos* = (3)
  EXTI_IMR_MR3_Msk* = (0x00000001 shl EXTI_IMR_MR3_Pos) ## !< 0x00000008
  EXTI_IMR_MR3* = EXTI_IMR_MR3_Msk
  EXTI_IMR_MR4_Pos* = (4)
  EXTI_IMR_MR4_Msk* = (0x00000001 shl EXTI_IMR_MR4_Pos) ## !< 0x00000010
  EXTI_IMR_MR4* = EXTI_IMR_MR4_Msk
  EXTI_IMR_MR5_Pos* = (5)
  EXTI_IMR_MR5_Msk* = (0x00000001 shl EXTI_IMR_MR5_Pos) ## !< 0x00000020
  EXTI_IMR_MR5* = EXTI_IMR_MR5_Msk
  EXTI_IMR_MR6_Pos* = (6)
  EXTI_IMR_MR6_Msk* = (0x00000001 shl EXTI_IMR_MR6_Pos) ## !< 0x00000040
  EXTI_IMR_MR6* = EXTI_IMR_MR6_Msk
  EXTI_IMR_MR7_Pos* = (7)
  EXTI_IMR_MR7_Msk* = (0x00000001 shl EXTI_IMR_MR7_Pos) ## !< 0x00000080
  EXTI_IMR_MR7* = EXTI_IMR_MR7_Msk
  EXTI_IMR_MR8_Pos* = (8)
  EXTI_IMR_MR8_Msk* = (0x00000001 shl EXTI_IMR_MR8_Pos) ## !< 0x00000100
  EXTI_IMR_MR8* = EXTI_IMR_MR8_Msk
  EXTI_IMR_MR9_Pos* = (9)
  EXTI_IMR_MR9_Msk* = (0x00000001 shl EXTI_IMR_MR9_Pos) ## !< 0x00000200
  EXTI_IMR_MR9* = EXTI_IMR_MR9_Msk
  EXTI_IMR_MR10_Pos* = (10)
  EXTI_IMR_MR10_Msk* = (0x00000001 shl EXTI_IMR_MR10_Pos) ## !< 0x00000400
  EXTI_IMR_MR10* = EXTI_IMR_MR10_Msk
  EXTI_IMR_MR11_Pos* = (11)
  EXTI_IMR_MR11_Msk* = (0x00000001 shl EXTI_IMR_MR11_Pos) ## !< 0x00000800
  EXTI_IMR_MR11* = EXTI_IMR_MR11_Msk
  EXTI_IMR_MR12_Pos* = (12)
  EXTI_IMR_MR12_Msk* = (0x00000001 shl EXTI_IMR_MR12_Pos) ## !< 0x00001000
  EXTI_IMR_MR12* = EXTI_IMR_MR12_Msk
  EXTI_IMR_MR13_Pos* = (13)
  EXTI_IMR_MR13_Msk* = (0x00000001 shl EXTI_IMR_MR13_Pos) ## !< 0x00002000
  EXTI_IMR_MR13* = EXTI_IMR_MR13_Msk
  EXTI_IMR_MR14_Pos* = (14)
  EXTI_IMR_MR14_Msk* = (0x00000001 shl EXTI_IMR_MR14_Pos) ## !< 0x00004000
  EXTI_IMR_MR14* = EXTI_IMR_MR14_Msk
  EXTI_IMR_MR15_Pos* = (15)
  EXTI_IMR_MR15_Msk* = (0x00000001 shl EXTI_IMR_MR15_Pos) ## !< 0x00008000
  EXTI_IMR_MR15* = EXTI_IMR_MR15_Msk
  EXTI_IMR_MR16_Pos* = (16)
  EXTI_IMR_MR16_Msk* = (0x00000001 shl EXTI_IMR_MR16_Pos) ## !< 0x00010000
  EXTI_IMR_MR16* = EXTI_IMR_MR16_Msk
  EXTI_IMR_MR17_Pos* = (17)
  EXTI_IMR_MR17_Msk* = (0x00000001 shl EXTI_IMR_MR17_Pos) ## !< 0x00020000
  EXTI_IMR_MR17* = EXTI_IMR_MR17_Msk
  EXTI_IMR_MR18_Pos* = (18)
  EXTI_IMR_MR18_Msk* = (0x00000001 shl EXTI_IMR_MR18_Pos) ## !< 0x00040000
  EXTI_IMR_MR18* = EXTI_IMR_MR18_Msk
  EXTI_IMR_MR19_Pos* = (19)
  EXTI_IMR_MR19_Msk* = (0x00000001 shl EXTI_IMR_MR19_Pos) ## !< 0x00080000
  EXTI_IMR_MR19* = EXTI_IMR_MR19_Msk
  EXTI_IMR_MR20_Pos* = (20)
  EXTI_IMR_MR20_Msk* = (0x00000001 shl EXTI_IMR_MR20_Pos) ## !< 0x00100000
  EXTI_IMR_MR20* = EXTI_IMR_MR20_Msk
  EXTI_IMR_MR21_Pos* = (21)
  EXTI_IMR_MR21_Msk* = (0x00000001 shl EXTI_IMR_MR21_Pos) ## !< 0x00200000
  EXTI_IMR_MR21* = EXTI_IMR_MR21_Msk
  EXTI_IMR_MR22_Pos* = (22)
  EXTI_IMR_MR22_Msk* = (0x00000001 shl EXTI_IMR_MR22_Pos) ## !< 0x00400000
  EXTI_IMR_MR22* = EXTI_IMR_MR22_Msk
  EXTI_IMR_MR23_Pos* = (23)
  EXTI_IMR_MR23_Msk* = (0x00000001 shl EXTI_IMR_MR23_Pos) ## !< 0x00800000
  EXTI_IMR_MR23* = EXTI_IMR_MR23_Msk

##  Reference Defines

const
  EXTI_IMR_IM0* = EXTI_IMR_MR0
  EXTI_IMR_IM1* = EXTI_IMR_MR1
  EXTI_IMR_IM2* = EXTI_IMR_MR2
  EXTI_IMR_IM3* = EXTI_IMR_MR3
  EXTI_IMR_IM4* = EXTI_IMR_MR4
  EXTI_IMR_IM5* = EXTI_IMR_MR5
  EXTI_IMR_IM6* = EXTI_IMR_MR6
  EXTI_IMR_IM7* = EXTI_IMR_MR7
  EXTI_IMR_IM8* = EXTI_IMR_MR8
  EXTI_IMR_IM9* = EXTI_IMR_MR9
  EXTI_IMR_IM10* = EXTI_IMR_MR10
  EXTI_IMR_IM11* = EXTI_IMR_MR11
  EXTI_IMR_IM12* = EXTI_IMR_MR12
  EXTI_IMR_IM13* = EXTI_IMR_MR13
  EXTI_IMR_IM14* = EXTI_IMR_MR14
  EXTI_IMR_IM15* = EXTI_IMR_MR15
  EXTI_IMR_IM16* = EXTI_IMR_MR16
  EXTI_IMR_IM17* = EXTI_IMR_MR17
  EXTI_IMR_IM18* = EXTI_IMR_MR18
  EXTI_IMR_IM19* = EXTI_IMR_MR19
  EXTI_IMR_IM20* = EXTI_IMR_MR20
  EXTI_IMR_IM21* = EXTI_IMR_MR21
  EXTI_IMR_IM22* = EXTI_IMR_MR22
  EXTI_IMR_IM23* = EXTI_IMR_MR23
  EXTI_IMR_IM_Pos* = (0)
  EXTI_IMR_IM_Msk* = (0x00FFFFFF shl EXTI_IMR_IM_Pos) ## !< 0x00FFFFFF
  EXTI_IMR_IM* = EXTI_IMR_IM_Msk

## ******************  Bit definition for EXTI_EMR register  ******************

const
  EXTI_EMR_MR0_Pos* = (0)
  EXTI_EMR_MR0_Msk* = (0x00000001 shl EXTI_EMR_MR0_Pos) ## !< 0x00000001
  EXTI_EMR_MR0* = EXTI_EMR_MR0_Msk
  EXTI_EMR_MR1_Pos* = (1)
  EXTI_EMR_MR1_Msk* = (0x00000001 shl EXTI_EMR_MR1_Pos) ## !< 0x00000002
  EXTI_EMR_MR1* = EXTI_EMR_MR1_Msk
  EXTI_EMR_MR2_Pos* = (2)
  EXTI_EMR_MR2_Msk* = (0x00000001 shl EXTI_EMR_MR2_Pos) ## !< 0x00000004
  EXTI_EMR_MR2* = EXTI_EMR_MR2_Msk
  EXTI_EMR_MR3_Pos* = (3)
  EXTI_EMR_MR3_Msk* = (0x00000001 shl EXTI_EMR_MR3_Pos) ## !< 0x00000008
  EXTI_EMR_MR3* = EXTI_EMR_MR3_Msk
  EXTI_EMR_MR4_Pos* = (4)
  EXTI_EMR_MR4_Msk* = (0x00000001 shl EXTI_EMR_MR4_Pos) ## !< 0x00000010
  EXTI_EMR_MR4* = EXTI_EMR_MR4_Msk
  EXTI_EMR_MR5_Pos* = (5)
  EXTI_EMR_MR5_Msk* = (0x00000001 shl EXTI_EMR_MR5_Pos) ## !< 0x00000020
  EXTI_EMR_MR5* = EXTI_EMR_MR5_Msk
  EXTI_EMR_MR6_Pos* = (6)
  EXTI_EMR_MR6_Msk* = (0x00000001 shl EXTI_EMR_MR6_Pos) ## !< 0x00000040
  EXTI_EMR_MR6* = EXTI_EMR_MR6_Msk
  EXTI_EMR_MR7_Pos* = (7)
  EXTI_EMR_MR7_Msk* = (0x00000001 shl EXTI_EMR_MR7_Pos) ## !< 0x00000080
  EXTI_EMR_MR7* = EXTI_EMR_MR7_Msk
  EXTI_EMR_MR8_Pos* = (8)
  EXTI_EMR_MR8_Msk* = (0x00000001 shl EXTI_EMR_MR8_Pos) ## !< 0x00000100
  EXTI_EMR_MR8* = EXTI_EMR_MR8_Msk
  EXTI_EMR_MR9_Pos* = (9)
  EXTI_EMR_MR9_Msk* = (0x00000001 shl EXTI_EMR_MR9_Pos) ## !< 0x00000200
  EXTI_EMR_MR9* = EXTI_EMR_MR9_Msk
  EXTI_EMR_MR10_Pos* = (10)
  EXTI_EMR_MR10_Msk* = (0x00000001 shl EXTI_EMR_MR10_Pos) ## !< 0x00000400
  EXTI_EMR_MR10* = EXTI_EMR_MR10_Msk
  EXTI_EMR_MR11_Pos* = (11)
  EXTI_EMR_MR11_Msk* = (0x00000001 shl EXTI_EMR_MR11_Pos) ## !< 0x00000800
  EXTI_EMR_MR11* = EXTI_EMR_MR11_Msk
  EXTI_EMR_MR12_Pos* = (12)
  EXTI_EMR_MR12_Msk* = (0x00000001 shl EXTI_EMR_MR12_Pos) ## !< 0x00001000
  EXTI_EMR_MR12* = EXTI_EMR_MR12_Msk
  EXTI_EMR_MR13_Pos* = (13)
  EXTI_EMR_MR13_Msk* = (0x00000001 shl EXTI_EMR_MR13_Pos) ## !< 0x00002000
  EXTI_EMR_MR13* = EXTI_EMR_MR13_Msk
  EXTI_EMR_MR14_Pos* = (14)
  EXTI_EMR_MR14_Msk* = (0x00000001 shl EXTI_EMR_MR14_Pos) ## !< 0x00004000
  EXTI_EMR_MR14* = EXTI_EMR_MR14_Msk
  EXTI_EMR_MR15_Pos* = (15)
  EXTI_EMR_MR15_Msk* = (0x00000001 shl EXTI_EMR_MR15_Pos) ## !< 0x00008000
  EXTI_EMR_MR15* = EXTI_EMR_MR15_Msk
  EXTI_EMR_MR16_Pos* = (16)
  EXTI_EMR_MR16_Msk* = (0x00000001 shl EXTI_EMR_MR16_Pos) ## !< 0x00010000
  EXTI_EMR_MR16* = EXTI_EMR_MR16_Msk
  EXTI_EMR_MR17_Pos* = (17)
  EXTI_EMR_MR17_Msk* = (0x00000001 shl EXTI_EMR_MR17_Pos) ## !< 0x00020000
  EXTI_EMR_MR17* = EXTI_EMR_MR17_Msk
  EXTI_EMR_MR18_Pos* = (18)
  EXTI_EMR_MR18_Msk* = (0x00000001 shl EXTI_EMR_MR18_Pos) ## !< 0x00040000
  EXTI_EMR_MR18* = EXTI_EMR_MR18_Msk
  EXTI_EMR_MR19_Pos* = (19)
  EXTI_EMR_MR19_Msk* = (0x00000001 shl EXTI_EMR_MR19_Pos) ## !< 0x00080000
  EXTI_EMR_MR19* = EXTI_EMR_MR19_Msk
  EXTI_EMR_MR20_Pos* = (20)
  EXTI_EMR_MR20_Msk* = (0x00000001 shl EXTI_EMR_MR20_Pos) ## !< 0x00100000
  EXTI_EMR_MR20* = EXTI_EMR_MR20_Msk
  EXTI_EMR_MR21_Pos* = (21)
  EXTI_EMR_MR21_Msk* = (0x00000001 shl EXTI_EMR_MR21_Pos) ## !< 0x00200000
  EXTI_EMR_MR21* = EXTI_EMR_MR21_Msk
  EXTI_EMR_MR22_Pos* = (22)
  EXTI_EMR_MR22_Msk* = (0x00000001 shl EXTI_EMR_MR22_Pos) ## !< 0x00400000
  EXTI_EMR_MR22* = EXTI_EMR_MR22_Msk
  EXTI_EMR_MR23_Pos* = (23)
  EXTI_EMR_MR23_Msk* = (0x00000001 shl EXTI_EMR_MR23_Pos) ## !< 0x00800000
  EXTI_EMR_MR23* = EXTI_EMR_MR23_Msk

##  Reference Defines

const
  EXTI_EMR_EM0* = EXTI_EMR_MR0
  EXTI_EMR_EM1* = EXTI_EMR_MR1
  EXTI_EMR_EM2* = EXTI_EMR_MR2
  EXTI_EMR_EM3* = EXTI_EMR_MR3
  EXTI_EMR_EM4* = EXTI_EMR_MR4
  EXTI_EMR_EM5* = EXTI_EMR_MR5
  EXTI_EMR_EM6* = EXTI_EMR_MR6
  EXTI_EMR_EM7* = EXTI_EMR_MR7
  EXTI_EMR_EM8* = EXTI_EMR_MR8
  EXTI_EMR_EM9* = EXTI_EMR_MR9
  EXTI_EMR_EM10* = EXTI_EMR_MR10
  EXTI_EMR_EM11* = EXTI_EMR_MR11
  EXTI_EMR_EM12* = EXTI_EMR_MR12
  EXTI_EMR_EM13* = EXTI_EMR_MR13
  EXTI_EMR_EM14* = EXTI_EMR_MR14
  EXTI_EMR_EM15* = EXTI_EMR_MR15
  EXTI_EMR_EM16* = EXTI_EMR_MR16
  EXTI_EMR_EM17* = EXTI_EMR_MR17
  EXTI_EMR_EM18* = EXTI_EMR_MR18
  EXTI_EMR_EM19* = EXTI_EMR_MR19
  EXTI_EMR_EM20* = EXTI_EMR_MR20
  EXTI_EMR_EM21* = EXTI_EMR_MR21
  EXTI_EMR_EM22* = EXTI_EMR_MR22
  EXTI_EMR_EM23* = EXTI_EMR_MR23

## *****************  Bit definition for EXTI_RTSR register  ******************

const
  EXTI_RTSR_TR0_Pos* = (0)
  EXTI_RTSR_TR0_Msk* = (0x00000001 shl EXTI_RTSR_TR0_Pos) ## !< 0x00000001
  EXTI_RTSR_TR0* = EXTI_RTSR_TR0_Msk
  EXTI_RTSR_TR1_Pos* = (1)
  EXTI_RTSR_TR1_Msk* = (0x00000001 shl EXTI_RTSR_TR1_Pos) ## !< 0x00000002
  EXTI_RTSR_TR1* = EXTI_RTSR_TR1_Msk
  EXTI_RTSR_TR2_Pos* = (2)
  EXTI_RTSR_TR2_Msk* = (0x00000001 shl EXTI_RTSR_TR2_Pos) ## !< 0x00000004
  EXTI_RTSR_TR2* = EXTI_RTSR_TR2_Msk
  EXTI_RTSR_TR3_Pos* = (3)
  EXTI_RTSR_TR3_Msk* = (0x00000001 shl EXTI_RTSR_TR3_Pos) ## !< 0x00000008
  EXTI_RTSR_TR3* = EXTI_RTSR_TR3_Msk
  EXTI_RTSR_TR4_Pos* = (4)
  EXTI_RTSR_TR4_Msk* = (0x00000001 shl EXTI_RTSR_TR4_Pos) ## !< 0x00000010
  EXTI_RTSR_TR4* = EXTI_RTSR_TR4_Msk
  EXTI_RTSR_TR5_Pos* = (5)
  EXTI_RTSR_TR5_Msk* = (0x00000001 shl EXTI_RTSR_TR5_Pos) ## !< 0x00000020
  EXTI_RTSR_TR5* = EXTI_RTSR_TR5_Msk
  EXTI_RTSR_TR6_Pos* = (6)
  EXTI_RTSR_TR6_Msk* = (0x00000001 shl EXTI_RTSR_TR6_Pos) ## !< 0x00000040
  EXTI_RTSR_TR6* = EXTI_RTSR_TR6_Msk
  EXTI_RTSR_TR7_Pos* = (7)
  EXTI_RTSR_TR7_Msk* = (0x00000001 shl EXTI_RTSR_TR7_Pos) ## !< 0x00000080
  EXTI_RTSR_TR7* = EXTI_RTSR_TR7_Msk
  EXTI_RTSR_TR8_Pos* = (8)
  EXTI_RTSR_TR8_Msk* = (0x00000001 shl EXTI_RTSR_TR8_Pos) ## !< 0x00000100
  EXTI_RTSR_TR8* = EXTI_RTSR_TR8_Msk
  EXTI_RTSR_TR9_Pos* = (9)
  EXTI_RTSR_TR9_Msk* = (0x00000001 shl EXTI_RTSR_TR9_Pos) ## !< 0x00000200
  EXTI_RTSR_TR9* = EXTI_RTSR_TR9_Msk
  EXTI_RTSR_TR10_Pos* = (10)
  EXTI_RTSR_TR10_Msk* = (0x00000001 shl EXTI_RTSR_TR10_Pos) ## !< 0x00000400
  EXTI_RTSR_TR10* = EXTI_RTSR_TR10_Msk
  EXTI_RTSR_TR11_Pos* = (11)
  EXTI_RTSR_TR11_Msk* = (0x00000001 shl EXTI_RTSR_TR11_Pos) ## !< 0x00000800
  EXTI_RTSR_TR11* = EXTI_RTSR_TR11_Msk
  EXTI_RTSR_TR12_Pos* = (12)
  EXTI_RTSR_TR12_Msk* = (0x00000001 shl EXTI_RTSR_TR12_Pos) ## !< 0x00001000
  EXTI_RTSR_TR12* = EXTI_RTSR_TR12_Msk
  EXTI_RTSR_TR13_Pos* = (13)
  EXTI_RTSR_TR13_Msk* = (0x00000001 shl EXTI_RTSR_TR13_Pos) ## !< 0x00002000
  EXTI_RTSR_TR13* = EXTI_RTSR_TR13_Msk
  EXTI_RTSR_TR14_Pos* = (14)
  EXTI_RTSR_TR14_Msk* = (0x00000001 shl EXTI_RTSR_TR14_Pos) ## !< 0x00004000
  EXTI_RTSR_TR14* = EXTI_RTSR_TR14_Msk
  EXTI_RTSR_TR15_Pos* = (15)
  EXTI_RTSR_TR15_Msk* = (0x00000001 shl EXTI_RTSR_TR15_Pos) ## !< 0x00008000
  EXTI_RTSR_TR15* = EXTI_RTSR_TR15_Msk
  EXTI_RTSR_TR16_Pos* = (16)
  EXTI_RTSR_TR16_Msk* = (0x00000001 shl EXTI_RTSR_TR16_Pos) ## !< 0x00010000
  EXTI_RTSR_TR16* = EXTI_RTSR_TR16_Msk
  EXTI_RTSR_TR17_Pos* = (17)
  EXTI_RTSR_TR17_Msk* = (0x00000001 shl EXTI_RTSR_TR17_Pos) ## !< 0x00020000
  EXTI_RTSR_TR17* = EXTI_RTSR_TR17_Msk
  EXTI_RTSR_TR18_Pos* = (18)
  EXTI_RTSR_TR18_Msk* = (0x00000001 shl EXTI_RTSR_TR18_Pos) ## !< 0x00040000
  EXTI_RTSR_TR18* = EXTI_RTSR_TR18_Msk
  EXTI_RTSR_TR19_Pos* = (19)
  EXTI_RTSR_TR19_Msk* = (0x00000001 shl EXTI_RTSR_TR19_Pos) ## !< 0x00080000
  EXTI_RTSR_TR19* = EXTI_RTSR_TR19_Msk
  EXTI_RTSR_TR20_Pos* = (20)
  EXTI_RTSR_TR20_Msk* = (0x00000001 shl EXTI_RTSR_TR20_Pos) ## !< 0x00100000
  EXTI_RTSR_TR20* = EXTI_RTSR_TR20_Msk
  EXTI_RTSR_TR21_Pos* = (21)
  EXTI_RTSR_TR21_Msk* = (0x00000001 shl EXTI_RTSR_TR21_Pos) ## !< 0x00200000
  EXTI_RTSR_TR21* = EXTI_RTSR_TR21_Msk
  EXTI_RTSR_TR22_Pos* = (22)
  EXTI_RTSR_TR22_Msk* = (0x00000001 shl EXTI_RTSR_TR22_Pos) ## !< 0x00400000
  EXTI_RTSR_TR22* = EXTI_RTSR_TR22_Msk
  EXTI_RTSR_TR23_Pos* = (23)
  EXTI_RTSR_TR23_Msk* = (0x00000001 shl EXTI_RTSR_TR23_Pos) ## !< 0x00800000
  EXTI_RTSR_TR23* = EXTI_RTSR_TR23_Msk

## *****************  Bit definition for EXTI_FTSR register  ******************

const
  EXTI_FTSR_TR0_Pos* = (0)
  EXTI_FTSR_TR0_Msk* = (0x00000001 shl EXTI_FTSR_TR0_Pos) ## !< 0x00000001
  EXTI_FTSR_TR0* = EXTI_FTSR_TR0_Msk
  EXTI_FTSR_TR1_Pos* = (1)
  EXTI_FTSR_TR1_Msk* = (0x00000001 shl EXTI_FTSR_TR1_Pos) ## !< 0x00000002
  EXTI_FTSR_TR1* = EXTI_FTSR_TR1_Msk
  EXTI_FTSR_TR2_Pos* = (2)
  EXTI_FTSR_TR2_Msk* = (0x00000001 shl EXTI_FTSR_TR2_Pos) ## !< 0x00000004
  EXTI_FTSR_TR2* = EXTI_FTSR_TR2_Msk
  EXTI_FTSR_TR3_Pos* = (3)
  EXTI_FTSR_TR3_Msk* = (0x00000001 shl EXTI_FTSR_TR3_Pos) ## !< 0x00000008
  EXTI_FTSR_TR3* = EXTI_FTSR_TR3_Msk
  EXTI_FTSR_TR4_Pos* = (4)
  EXTI_FTSR_TR4_Msk* = (0x00000001 shl EXTI_FTSR_TR4_Pos) ## !< 0x00000010
  EXTI_FTSR_TR4* = EXTI_FTSR_TR4_Msk
  EXTI_FTSR_TR5_Pos* = (5)
  EXTI_FTSR_TR5_Msk* = (0x00000001 shl EXTI_FTSR_TR5_Pos) ## !< 0x00000020
  EXTI_FTSR_TR5* = EXTI_FTSR_TR5_Msk
  EXTI_FTSR_TR6_Pos* = (6)
  EXTI_FTSR_TR6_Msk* = (0x00000001 shl EXTI_FTSR_TR6_Pos) ## !< 0x00000040
  EXTI_FTSR_TR6* = EXTI_FTSR_TR6_Msk
  EXTI_FTSR_TR7_Pos* = (7)
  EXTI_FTSR_TR7_Msk* = (0x00000001 shl EXTI_FTSR_TR7_Pos) ## !< 0x00000080
  EXTI_FTSR_TR7* = EXTI_FTSR_TR7_Msk
  EXTI_FTSR_TR8_Pos* = (8)
  EXTI_FTSR_TR8_Msk* = (0x00000001 shl EXTI_FTSR_TR8_Pos) ## !< 0x00000100
  EXTI_FTSR_TR8* = EXTI_FTSR_TR8_Msk
  EXTI_FTSR_TR9_Pos* = (9)
  EXTI_FTSR_TR9_Msk* = (0x00000001 shl EXTI_FTSR_TR9_Pos) ## !< 0x00000200
  EXTI_FTSR_TR9* = EXTI_FTSR_TR9_Msk
  EXTI_FTSR_TR10_Pos* = (10)
  EXTI_FTSR_TR10_Msk* = (0x00000001 shl EXTI_FTSR_TR10_Pos) ## !< 0x00000400
  EXTI_FTSR_TR10* = EXTI_FTSR_TR10_Msk
  EXTI_FTSR_TR11_Pos* = (11)
  EXTI_FTSR_TR11_Msk* = (0x00000001 shl EXTI_FTSR_TR11_Pos) ## !< 0x00000800
  EXTI_FTSR_TR11* = EXTI_FTSR_TR11_Msk
  EXTI_FTSR_TR12_Pos* = (12)
  EXTI_FTSR_TR12_Msk* = (0x00000001 shl EXTI_FTSR_TR12_Pos) ## !< 0x00001000
  EXTI_FTSR_TR12* = EXTI_FTSR_TR12_Msk
  EXTI_FTSR_TR13_Pos* = (13)
  EXTI_FTSR_TR13_Msk* = (0x00000001 shl EXTI_FTSR_TR13_Pos) ## !< 0x00002000
  EXTI_FTSR_TR13* = EXTI_FTSR_TR13_Msk
  EXTI_FTSR_TR14_Pos* = (14)
  EXTI_FTSR_TR14_Msk* = (0x00000001 shl EXTI_FTSR_TR14_Pos) ## !< 0x00004000
  EXTI_FTSR_TR14* = EXTI_FTSR_TR14_Msk
  EXTI_FTSR_TR15_Pos* = (15)
  EXTI_FTSR_TR15_Msk* = (0x00000001 shl EXTI_FTSR_TR15_Pos) ## !< 0x00008000
  EXTI_FTSR_TR15* = EXTI_FTSR_TR15_Msk
  EXTI_FTSR_TR16_Pos* = (16)
  EXTI_FTSR_TR16_Msk* = (0x00000001 shl EXTI_FTSR_TR16_Pos) ## !< 0x00010000
  EXTI_FTSR_TR16* = EXTI_FTSR_TR16_Msk
  EXTI_FTSR_TR17_Pos* = (17)
  EXTI_FTSR_TR17_Msk* = (0x00000001 shl EXTI_FTSR_TR17_Pos) ## !< 0x00020000
  EXTI_FTSR_TR17* = EXTI_FTSR_TR17_Msk
  EXTI_FTSR_TR18_Pos* = (18)
  EXTI_FTSR_TR18_Msk* = (0x00000001 shl EXTI_FTSR_TR18_Pos) ## !< 0x00040000
  EXTI_FTSR_TR18* = EXTI_FTSR_TR18_Msk
  EXTI_FTSR_TR19_Pos* = (19)
  EXTI_FTSR_TR19_Msk* = (0x00000001 shl EXTI_FTSR_TR19_Pos) ## !< 0x00080000
  EXTI_FTSR_TR19* = EXTI_FTSR_TR19_Msk
  EXTI_FTSR_TR20_Pos* = (20)
  EXTI_FTSR_TR20_Msk* = (0x00000001 shl EXTI_FTSR_TR20_Pos) ## !< 0x00100000
  EXTI_FTSR_TR20* = EXTI_FTSR_TR20_Msk
  EXTI_FTSR_TR21_Pos* = (21)
  EXTI_FTSR_TR21_Msk* = (0x00000001 shl EXTI_FTSR_TR21_Pos) ## !< 0x00200000
  EXTI_FTSR_TR21* = EXTI_FTSR_TR21_Msk
  EXTI_FTSR_TR22_Pos* = (22)
  EXTI_FTSR_TR22_Msk* = (0x00000001 shl EXTI_FTSR_TR22_Pos) ## !< 0x00400000
  EXTI_FTSR_TR22* = EXTI_FTSR_TR22_Msk
  EXTI_FTSR_TR23_Pos* = (23)
  EXTI_FTSR_TR23_Msk* = (0x00000001 shl EXTI_FTSR_TR23_Pos) ## !< 0x00800000
  EXTI_FTSR_TR23* = EXTI_FTSR_TR23_Msk

## *****************  Bit definition for EXTI_SWIER register  *****************

const
  EXTI_SWIER_SWIER0_Pos* = (0)
  EXTI_SWIER_SWIER0_Msk* = (0x00000001 shl EXTI_SWIER_SWIER0_Pos) ## !< 0x00000001
  EXTI_SWIER_SWIER0* = EXTI_SWIER_SWIER0_Msk
  EXTI_SWIER_SWIER1_Pos* = (1)
  EXTI_SWIER_SWIER1_Msk* = (0x00000001 shl EXTI_SWIER_SWIER1_Pos) ## !< 0x00000002
  EXTI_SWIER_SWIER1* = EXTI_SWIER_SWIER1_Msk
  EXTI_SWIER_SWIER2_Pos* = (2)
  EXTI_SWIER_SWIER2_Msk* = (0x00000001 shl EXTI_SWIER_SWIER2_Pos) ## !< 0x00000004
  EXTI_SWIER_SWIER2* = EXTI_SWIER_SWIER2_Msk
  EXTI_SWIER_SWIER3_Pos* = (3)
  EXTI_SWIER_SWIER3_Msk* = (0x00000001 shl EXTI_SWIER_SWIER3_Pos) ## !< 0x00000008
  EXTI_SWIER_SWIER3* = EXTI_SWIER_SWIER3_Msk
  EXTI_SWIER_SWIER4_Pos* = (4)
  EXTI_SWIER_SWIER4_Msk* = (0x00000001 shl EXTI_SWIER_SWIER4_Pos) ## !< 0x00000010
  EXTI_SWIER_SWIER4* = EXTI_SWIER_SWIER4_Msk
  EXTI_SWIER_SWIER5_Pos* = (5)
  EXTI_SWIER_SWIER5_Msk* = (0x00000001 shl EXTI_SWIER_SWIER5_Pos) ## !< 0x00000020
  EXTI_SWIER_SWIER5* = EXTI_SWIER_SWIER5_Msk
  EXTI_SWIER_SWIER6_Pos* = (6)
  EXTI_SWIER_SWIER6_Msk* = (0x00000001 shl EXTI_SWIER_SWIER6_Pos) ## !< 0x00000040
  EXTI_SWIER_SWIER6* = EXTI_SWIER_SWIER6_Msk
  EXTI_SWIER_SWIER7_Pos* = (7)
  EXTI_SWIER_SWIER7_Msk* = (0x00000001 shl EXTI_SWIER_SWIER7_Pos) ## !< 0x00000080
  EXTI_SWIER_SWIER7* = EXTI_SWIER_SWIER7_Msk
  EXTI_SWIER_SWIER8_Pos* = (8)
  EXTI_SWIER_SWIER8_Msk* = (0x00000001 shl EXTI_SWIER_SWIER8_Pos) ## !< 0x00000100
  EXTI_SWIER_SWIER8* = EXTI_SWIER_SWIER8_Msk
  EXTI_SWIER_SWIER9_Pos* = (9)
  EXTI_SWIER_SWIER9_Msk* = (0x00000001 shl EXTI_SWIER_SWIER9_Pos) ## !< 0x00000200
  EXTI_SWIER_SWIER9* = EXTI_SWIER_SWIER9_Msk
  EXTI_SWIER_SWIER10_Pos* = (10)
  EXTI_SWIER_SWIER10_Msk* = (0x00000001 shl EXTI_SWIER_SWIER10_Pos) ## !< 0x00000400
  EXTI_SWIER_SWIER10* = EXTI_SWIER_SWIER10_Msk
  EXTI_SWIER_SWIER11_Pos* = (11)
  EXTI_SWIER_SWIER11_Msk* = (0x00000001 shl EXTI_SWIER_SWIER11_Pos) ## !< 0x00000800
  EXTI_SWIER_SWIER11* = EXTI_SWIER_SWIER11_Msk
  EXTI_SWIER_SWIER12_Pos* = (12)
  EXTI_SWIER_SWIER12_Msk* = (0x00000001 shl EXTI_SWIER_SWIER12_Pos) ## !< 0x00001000
  EXTI_SWIER_SWIER12* = EXTI_SWIER_SWIER12_Msk
  EXTI_SWIER_SWIER13_Pos* = (13)
  EXTI_SWIER_SWIER13_Msk* = (0x00000001 shl EXTI_SWIER_SWIER13_Pos) ## !< 0x00002000
  EXTI_SWIER_SWIER13* = EXTI_SWIER_SWIER13_Msk
  EXTI_SWIER_SWIER14_Pos* = (14)
  EXTI_SWIER_SWIER14_Msk* = (0x00000001 shl EXTI_SWIER_SWIER14_Pos) ## !< 0x00004000
  EXTI_SWIER_SWIER14* = EXTI_SWIER_SWIER14_Msk
  EXTI_SWIER_SWIER15_Pos* = (15)
  EXTI_SWIER_SWIER15_Msk* = (0x00000001 shl EXTI_SWIER_SWIER15_Pos) ## !< 0x00008000
  EXTI_SWIER_SWIER15* = EXTI_SWIER_SWIER15_Msk
  EXTI_SWIER_SWIER16_Pos* = (16)
  EXTI_SWIER_SWIER16_Msk* = (0x00000001 shl EXTI_SWIER_SWIER16_Pos) ## !< 0x00010000
  EXTI_SWIER_SWIER16* = EXTI_SWIER_SWIER16_Msk
  EXTI_SWIER_SWIER17_Pos* = (17)
  EXTI_SWIER_SWIER17_Msk* = (0x00000001 shl EXTI_SWIER_SWIER17_Pos) ## !< 0x00020000
  EXTI_SWIER_SWIER17* = EXTI_SWIER_SWIER17_Msk
  EXTI_SWIER_SWIER18_Pos* = (18)
  EXTI_SWIER_SWIER18_Msk* = (0x00000001 shl EXTI_SWIER_SWIER18_Pos) ## !< 0x00040000
  EXTI_SWIER_SWIER18* = EXTI_SWIER_SWIER18_Msk
  EXTI_SWIER_SWIER19_Pos* = (19)
  EXTI_SWIER_SWIER19_Msk* = (0x00000001 shl EXTI_SWIER_SWIER19_Pos) ## !< 0x00080000
  EXTI_SWIER_SWIER19* = EXTI_SWIER_SWIER19_Msk
  EXTI_SWIER_SWIER20_Pos* = (20)
  EXTI_SWIER_SWIER20_Msk* = (0x00000001 shl EXTI_SWIER_SWIER20_Pos) ## !< 0x00100000
  EXTI_SWIER_SWIER20* = EXTI_SWIER_SWIER20_Msk
  EXTI_SWIER_SWIER21_Pos* = (21)
  EXTI_SWIER_SWIER21_Msk* = (0x00000001 shl EXTI_SWIER_SWIER21_Pos) ## !< 0x00200000
  EXTI_SWIER_SWIER21* = EXTI_SWIER_SWIER21_Msk
  EXTI_SWIER_SWIER22_Pos* = (22)
  EXTI_SWIER_SWIER22_Msk* = (0x00000001 shl EXTI_SWIER_SWIER22_Pos) ## !< 0x00400000
  EXTI_SWIER_SWIER22* = EXTI_SWIER_SWIER22_Msk
  EXTI_SWIER_SWIER23_Pos* = (23)
  EXTI_SWIER_SWIER23_Msk* = (0x00000001 shl EXTI_SWIER_SWIER23_Pos) ## !< 0x00800000
  EXTI_SWIER_SWIER23* = EXTI_SWIER_SWIER23_Msk

## ******************  Bit definition for EXTI_PR register  *******************

const
  EXTI_PR_PR0_Pos* = (0)
  EXTI_PR_PR0_Msk* = (0x00000001 shl EXTI_PR_PR0_Pos) ## !< 0x00000001
  EXTI_PR_PR0* = EXTI_PR_PR0_Msk
  EXTI_PR_PR1_Pos* = (1)
  EXTI_PR_PR1_Msk* = (0x00000001 shl EXTI_PR_PR1_Pos) ## !< 0x00000002
  EXTI_PR_PR1* = EXTI_PR_PR1_Msk
  EXTI_PR_PR2_Pos* = (2)
  EXTI_PR_PR2_Msk* = (0x00000001 shl EXTI_PR_PR2_Pos) ## !< 0x00000004
  EXTI_PR_PR2* = EXTI_PR_PR2_Msk
  EXTI_PR_PR3_Pos* = (3)
  EXTI_PR_PR3_Msk* = (0x00000001 shl EXTI_PR_PR3_Pos) ## !< 0x00000008
  EXTI_PR_PR3* = EXTI_PR_PR3_Msk
  EXTI_PR_PR4_Pos* = (4)
  EXTI_PR_PR4_Msk* = (0x00000001 shl EXTI_PR_PR4_Pos) ## !< 0x00000010
  EXTI_PR_PR4* = EXTI_PR_PR4_Msk
  EXTI_PR_PR5_Pos* = (5)
  EXTI_PR_PR5_Msk* = (0x00000001 shl EXTI_PR_PR5_Pos) ## !< 0x00000020
  EXTI_PR_PR5* = EXTI_PR_PR5_Msk
  EXTI_PR_PR6_Pos* = (6)
  EXTI_PR_PR6_Msk* = (0x00000001 shl EXTI_PR_PR6_Pos) ## !< 0x00000040
  EXTI_PR_PR6* = EXTI_PR_PR6_Msk
  EXTI_PR_PR7_Pos* = (7)
  EXTI_PR_PR7_Msk* = (0x00000001 shl EXTI_PR_PR7_Pos) ## !< 0x00000080
  EXTI_PR_PR7* = EXTI_PR_PR7_Msk
  EXTI_PR_PR8_Pos* = (8)
  EXTI_PR_PR8_Msk* = (0x00000001 shl EXTI_PR_PR8_Pos) ## !< 0x00000100
  EXTI_PR_PR8* = EXTI_PR_PR8_Msk
  EXTI_PR_PR9_Pos* = (9)
  EXTI_PR_PR9_Msk* = (0x00000001 shl EXTI_PR_PR9_Pos) ## !< 0x00000200
  EXTI_PR_PR9* = EXTI_PR_PR9_Msk
  EXTI_PR_PR10_Pos* = (10)
  EXTI_PR_PR10_Msk* = (0x00000001 shl EXTI_PR_PR10_Pos) ## !< 0x00000400
  EXTI_PR_PR10* = EXTI_PR_PR10_Msk
  EXTI_PR_PR11_Pos* = (11)
  EXTI_PR_PR11_Msk* = (0x00000001 shl EXTI_PR_PR11_Pos) ## !< 0x00000800
  EXTI_PR_PR11* = EXTI_PR_PR11_Msk
  EXTI_PR_PR12_Pos* = (12)
  EXTI_PR_PR12_Msk* = (0x00000001 shl EXTI_PR_PR12_Pos) ## !< 0x00001000
  EXTI_PR_PR12* = EXTI_PR_PR12_Msk
  EXTI_PR_PR13_Pos* = (13)
  EXTI_PR_PR13_Msk* = (0x00000001 shl EXTI_PR_PR13_Pos) ## !< 0x00002000
  EXTI_PR_PR13* = EXTI_PR_PR13_Msk
  EXTI_PR_PR14_Pos* = (14)
  EXTI_PR_PR14_Msk* = (0x00000001 shl EXTI_PR_PR14_Pos) ## !< 0x00004000
  EXTI_PR_PR14* = EXTI_PR_PR14_Msk
  EXTI_PR_PR15_Pos* = (15)
  EXTI_PR_PR15_Msk* = (0x00000001 shl EXTI_PR_PR15_Pos) ## !< 0x00008000
  EXTI_PR_PR15* = EXTI_PR_PR15_Msk
  EXTI_PR_PR16_Pos* = (16)
  EXTI_PR_PR16_Msk* = (0x00000001 shl EXTI_PR_PR16_Pos) ## !< 0x00010000
  EXTI_PR_PR16* = EXTI_PR_PR16_Msk
  EXTI_PR_PR17_Pos* = (17)
  EXTI_PR_PR17_Msk* = (0x00000001 shl EXTI_PR_PR17_Pos) ## !< 0x00020000
  EXTI_PR_PR17* = EXTI_PR_PR17_Msk
  EXTI_PR_PR18_Pos* = (18)
  EXTI_PR_PR18_Msk* = (0x00000001 shl EXTI_PR_PR18_Pos) ## !< 0x00040000
  EXTI_PR_PR18* = EXTI_PR_PR18_Msk
  EXTI_PR_PR19_Pos* = (19)
  EXTI_PR_PR19_Msk* = (0x00000001 shl EXTI_PR_PR19_Pos) ## !< 0x00080000
  EXTI_PR_PR19* = EXTI_PR_PR19_Msk
  EXTI_PR_PR20_Pos* = (20)
  EXTI_PR_PR20_Msk* = (0x00000001 shl EXTI_PR_PR20_Pos) ## !< 0x00100000
  EXTI_PR_PR20* = EXTI_PR_PR20_Msk
  EXTI_PR_PR21_Pos* = (21)
  EXTI_PR_PR21_Msk* = (0x00000001 shl EXTI_PR_PR21_Pos) ## !< 0x00200000
  EXTI_PR_PR21* = EXTI_PR_PR21_Msk
  EXTI_PR_PR22_Pos* = (22)
  EXTI_PR_PR22_Msk* = (0x00000001 shl EXTI_PR_PR22_Pos) ## !< 0x00400000
  EXTI_PR_PR22* = EXTI_PR_PR22_Msk
  EXTI_PR_PR23_Pos* = (23)
  EXTI_PR_PR23_Msk* = (0x00000001 shl EXTI_PR_PR23_Pos) ## !< 0x00800000
  EXTI_PR_PR23* = EXTI_PR_PR23_Msk

## ****************************************************************************
##
##                                     FLASH
##
## ****************************************************************************
## ******************  Bits definition for FLASH_ACR register  ****************

const
  FLASH_ACR_LATENCY_Pos* = (0)
  FLASH_ACR_LATENCY_Msk* = (0x0000000F shl FLASH_ACR_LATENCY_Pos) ## !< 0x0000000F
  FLASH_ACR_LATENCY* = FLASH_ACR_LATENCY_Msk
  FLASH_ACR_LATENCY_Bit0WS* = 0x00000000
  FLASH_ACR_LATENCY_Bit1WS* = 0x00000001
  FLASH_ACR_LATENCY_Bit2WS* = 0x00000002
  FLASH_ACR_LATENCY_Bit3WS* = 0x00000003
  FLASH_ACR_LATENCY_Bit4WS* = 0x00000004
  FLASH_ACR_LATENCY_Bit5WS* = 0x00000005
  FLASH_ACR_LATENCY_Bit6WS* = 0x00000006
  FLASH_ACR_LATENCY_Bit7WS* = 0x00000007
  FLASH_ACR_PRFTEN_Pos* = (8)
  FLASH_ACR_PRFTEN_Msk* = (0x00000001 shl FLASH_ACR_PRFTEN_Pos) ## !< 0x00000100
  FLASH_ACR_PRFTEN* = FLASH_ACR_PRFTEN_Msk
  FLASH_ACR_ICEN_Pos* = (9)
  FLASH_ACR_ICEN_Msk* = (0x00000001 shl FLASH_ACR_ICEN_Pos) ## !< 0x00000200
  FLASH_ACR_ICEN* = FLASH_ACR_ICEN_Msk
  FLASH_ACR_DCEN_Pos* = (10)
  FLASH_ACR_DCEN_Msk* = (0x00000001 shl FLASH_ACR_DCEN_Pos) ## !< 0x00000400
  FLASH_ACR_DCEN* = FLASH_ACR_DCEN_Msk
  FLASH_ACR_ICRST_Pos* = (11)
  FLASH_ACR_ICRST_Msk* = (0x00000001 shl FLASH_ACR_ICRST_Pos) ## !< 0x00000800
  FLASH_ACR_ICRST* = FLASH_ACR_ICRST_Msk
  FLASH_ACR_DCRST_Pos* = (12)
  FLASH_ACR_DCRST_Msk* = (0x00000001 shl FLASH_ACR_DCRST_Pos) ## !< 0x00001000
  FLASH_ACR_DCRST* = FLASH_ACR_DCRST_Msk
  FLASH_ACR_BYTE0_ADDRESS_Pos* = (10)
  FLASH_ACR_BYTE0_ADDRESS_Msk* = (0x0010008F shl FLASH_ACR_BYTE0_ADDRESS_Pos) ## !< 0x40023C00
  FLASH_ACR_BYTE0_ADDRESS* = FLASH_ACR_BYTE0_ADDRESS_Msk
  FLASH_ACR_BYTE2_ADDRESS_Pos* = (0)
  FLASH_ACR_BYTE2_ADDRESS_Msk* = (0x40023C03 shl FLASH_ACR_BYTE2_ADDRESS_Pos) ## !< 0x40023C03
  FLASH_ACR_BYTE2_ADDRESS* = FLASH_ACR_BYTE2_ADDRESS_Msk

## ******************  Bits definition for FLASH_SR register  *****************

const
  FLASH_SR_EOP_Pos* = (0)
  FLASH_SR_EOP_Msk* = (0x00000001 shl FLASH_SR_EOP_Pos) ## !< 0x00000001
  FLASH_SR_EOP* = FLASH_SR_EOP_Msk
  FLASH_SR_SOP_Pos* = (1)
  FLASH_SR_SOP_Msk* = (0x00000001 shl FLASH_SR_SOP_Pos) ## !< 0x00000002
  FLASH_SR_SOP* = FLASH_SR_SOP_Msk
  FLASH_SR_WRPERR_Pos* = (4)
  FLASH_SR_WRPERR_Msk* = (0x00000001 shl FLASH_SR_WRPERR_Pos) ## !< 0x00000010
  FLASH_SR_WRPERR* = FLASH_SR_WRPERR_Msk
  FLASH_SR_PGAERR_Pos* = (5)
  FLASH_SR_PGAERR_Msk* = (0x00000001 shl FLASH_SR_PGAERR_Pos) ## !< 0x00000020
  FLASH_SR_PGAERR* = FLASH_SR_PGAERR_Msk
  FLASH_SR_PGPERR_Pos* = (6)
  FLASH_SR_PGPERR_Msk* = (0x00000001 shl FLASH_SR_PGPERR_Pos) ## !< 0x00000040
  FLASH_SR_PGPERR* = FLASH_SR_PGPERR_Msk
  FLASH_SR_PGSERR_Pos* = (7)
  FLASH_SR_PGSERR_Msk* = (0x00000001 shl FLASH_SR_PGSERR_Pos) ## !< 0x00000080
  FLASH_SR_PGSERR* = FLASH_SR_PGSERR_Msk
  FLASH_SR_RDERR_Pos* = (8)
  FLASH_SR_RDERR_Msk* = (0x00000001 shl FLASH_SR_RDERR_Pos) ## !< 0x00000100
  FLASH_SR_RDERR* = FLASH_SR_RDERR_Msk
  FLASH_SR_BSY_Pos* = (16)
  FLASH_SR_BSY_Msk* = (0x00000001 shl FLASH_SR_BSY_Pos) ## !< 0x00010000
  FLASH_SR_BSY* = FLASH_SR_BSY_Msk

## ******************  Bits definition for FLASH_CR register  *****************

const
  FLASH_CR_PG_Pos* = (0)
  FLASH_CR_PG_Msk* = (0x00000001 shl FLASH_CR_PG_Pos) ## !< 0x00000001
  FLASH_CR_PG* = FLASH_CR_PG_Msk
  FLASH_CR_SER_Pos* = (1)
  FLASH_CR_SER_Msk* = (0x00000001 shl FLASH_CR_SER_Pos) ## !< 0x00000002
  FLASH_CR_SER* = FLASH_CR_SER_Msk
  FLASH_CR_MER_Pos* = (2)
  FLASH_CR_MER_Msk* = (0x00000001 shl FLASH_CR_MER_Pos) ## !< 0x00000004
  FLASH_CR_MER* = FLASH_CR_MER_Msk
  FLASH_CR_SNB_Pos* = (3)
  FLASH_CR_SNB_Msk* = (0x0000001F shl FLASH_CR_SNB_Pos) ## !< 0x000000F8
  FLASH_CR_SNB* = FLASH_CR_SNB_Msk
  FLASH_CR_SNB_Bit0* = (0x00000001 shl FLASH_CR_SNB_Pos) ## !< 0x00000008
  FLASH_CR_SNB_Bit1* = (0x00000002 shl FLASH_CR_SNB_Pos) ## !< 0x00000010
  FLASH_CR_SNB_Bit2* = (0x00000004 shl FLASH_CR_SNB_Pos) ## !< 0x00000020
  FLASH_CR_SNB_Bit3* = (0x00000008 shl FLASH_CR_SNB_Pos) ## !< 0x00000040
  FLASH_CR_SNB_Bit4* = (0x00000010 shl FLASH_CR_SNB_Pos) ## !< 0x00000080
  FLASH_CR_PSIZE_Pos* = (8)
  FLASH_CR_PSIZE_Msk* = (0x00000003 shl FLASH_CR_PSIZE_Pos) ## !< 0x00000300
  FLASH_CR_PSIZE* = FLASH_CR_PSIZE_Msk
  FLASH_CR_PSIZE_Bit0* = (0x00000001 shl FLASH_CR_PSIZE_Pos) ## !< 0x00000100
  FLASH_CR_PSIZE_Bit1* = (0x00000002 shl FLASH_CR_PSIZE_Pos) ## !< 0x00000200
  FLASH_CR_STRT_Pos* = (16)
  FLASH_CR_STRT_Msk* = (0x00000001 shl FLASH_CR_STRT_Pos) ## !< 0x00010000
  FLASH_CR_STRT* = FLASH_CR_STRT_Msk
  FLASH_CR_EOPIE_Pos* = (24)
  FLASH_CR_EOPIE_Msk* = (0x00000001 shl FLASH_CR_EOPIE_Pos) ## !< 0x01000000
  FLASH_CR_EOPIE* = FLASH_CR_EOPIE_Msk
  FLASH_CR_LOCK_Pos* = (31)
  FLASH_CR_LOCK_Msk* = (0x00000001 shl FLASH_CR_LOCK_Pos) ## !< 0x80000000
  FLASH_CR_LOCK* = FLASH_CR_LOCK_Msk

## ******************  Bits definition for FLASH_OPTCR register  **************

const
  FLASH_OPTCR_OPTLOCK_Pos* = (0)
  FLASH_OPTCR_OPTLOCK_Msk* = (0x00000001 shl FLASH_OPTCR_OPTLOCK_Pos) ## !< 0x00000001
  FLASH_OPTCR_OPTLOCK* = FLASH_OPTCR_OPTLOCK_Msk
  FLASH_OPTCR_OPTSTRT_Pos* = (1)
  FLASH_OPTCR_OPTSTRT_Msk* = (0x00000001 shl FLASH_OPTCR_OPTSTRT_Pos) ## !< 0x00000002
  FLASH_OPTCR_OPTSTRT* = FLASH_OPTCR_OPTSTRT_Msk
  FLASH_OPTCR_BOR_LEV_Bit0* = 0x00000004
  FLASH_OPTCR_BOR_LEV_Bit1* = 0x00000008
  FLASH_OPTCR_BOR_LEV_Pos* = (2)
  FLASH_OPTCR_BOR_LEV_Msk* = (0x00000003 shl FLASH_OPTCR_BOR_LEV_Pos) ## !< 0x0000000C
  FLASH_OPTCR_BOR_LEV* = FLASH_OPTCR_BOR_LEV_Msk
  FLASH_OPTCR_WDG_SW_Pos* = (5)
  FLASH_OPTCR_WDG_SW_Msk* = (0x00000001 shl FLASH_OPTCR_WDG_SW_Pos) ## !< 0x00000020
  FLASH_OPTCR_WDG_SW* = FLASH_OPTCR_WDG_SW_Msk
  FLASH_OPTCR_nRST_STOP_Pos* = (6)
  FLASH_OPTCR_nRST_STOP_Msk* = (0x00000001 shl FLASH_OPTCR_nRST_STOP_Pos) ## !< 0x00000040
  FLASH_OPTCR_nRST_STOP* = FLASH_OPTCR_nRST_STOP_Msk
  FLASH_OPTCR_nRST_STDBY_Pos* = (7)
  FLASH_OPTCR_nRST_STDBY_Msk* = (0x00000001 shl FLASH_OPTCR_nRST_STDBY_Pos) ## !< 0x00000080
  FLASH_OPTCR_nRST_STDBY* = FLASH_OPTCR_nRST_STDBY_Msk
  FLASH_OPTCR_RDP_Pos* = (8)
  FLASH_OPTCR_RDP_Msk* = (0x000000FF shl FLASH_OPTCR_RDP_Pos) ## !< 0x0000FF00
  FLASH_OPTCR_RDP* = FLASH_OPTCR_RDP_Msk
  FLASH_OPTCR_RDP_Bit0* = (0x00000001 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00000100
  FLASH_OPTCR_RDP_Bit1* = (0x00000002 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00000200
  FLASH_OPTCR_RDP_Bit2* = (0x00000004 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00000400
  FLASH_OPTCR_RDP_Bit3* = (0x00000008 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00000800
  FLASH_OPTCR_RDP_Bit4* = (0x00000010 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00001000
  FLASH_OPTCR_RDP_Bit5* = (0x00000020 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00002000
  FLASH_OPTCR_RDP_Bit6* = (0x00000040 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00004000
  FLASH_OPTCR_RDP_Bit7* = (0x00000080 shl FLASH_OPTCR_RDP_Pos) ## !< 0x00008000
  FLASH_OPTCR_nWRP_Pos* = (16)
  FLASH_OPTCR_nWRP_Msk* = (0x00000FFF shl FLASH_OPTCR_nWRP_Pos) ## !< 0x0FFF0000
  FLASH_OPTCR_nWRP* = FLASH_OPTCR_nWRP_Msk
  FLASH_OPTCR_nWRP_Bit0* = 0x00010000
  FLASH_OPTCR_nWRP_Bit1* = 0x00020000
  FLASH_OPTCR_nWRP_Bit2* = 0x00040000
  FLASH_OPTCR_nWRP_Bit3* = 0x00080000
  FLASH_OPTCR_nWRP_Bit4* = 0x00100000
  FLASH_OPTCR_nWRP_Bit5* = 0x00200000
  FLASH_OPTCR_nWRP_Bit6* = 0x00400000
  FLASH_OPTCR_nWRP_Bit7* = 0x00800000
  FLASH_OPTCR_nWRP_Bit8* = 0x01000000
  FLASH_OPTCR_nWRP_Bit9* = 0x02000000
  FLASH_OPTCR_nWRP_Bit10* = 0x04000000
  FLASH_OPTCR_nWRP_Bit11* = 0x08000000

## *****************  Bits definition for FLASH_OPTCR1 register  **************

const
  FLASH_OPTCR1_nWRP_Pos* = (16)
  FLASH_OPTCR1_nWRP_Msk* = (0x00000FFF shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x0FFF0000
  FLASH_OPTCR1_nWRP* = FLASH_OPTCR1_nWRP_Msk
  FLASH_OPTCR1_nWRP_Bit0* = (0x00000001 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00010000
  FLASH_OPTCR1_nWRP_Bit1* = (0x00000002 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00020000
  FLASH_OPTCR1_nWRP_Bit2* = (0x00000004 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00040000
  FLASH_OPTCR1_nWRP_Bit3* = (0x00000008 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00080000
  FLASH_OPTCR1_nWRP_Bit4* = (0x00000010 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00100000
  FLASH_OPTCR1_nWRP_Bit5* = (0x00000020 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00200000
  FLASH_OPTCR1_nWRP_Bit6* = (0x00000040 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00400000
  FLASH_OPTCR1_nWRP_Bit7* = (0x00000080 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x00800000
  FLASH_OPTCR1_nWRP_Bit8* = (0x00000100 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x01000000
  FLASH_OPTCR1_nWRP_Bit9* = (0x00000200 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x02000000
  FLASH_OPTCR1_nWRP_Bit10* = (0x00000400 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x04000000
  FLASH_OPTCR1_nWRP_Bit11* = (0x00000800 shl FLASH_OPTCR1_nWRP_Pos) ## !< 0x08000000

## ****************************************************************************
##
##                             General Purpose I/O
##
## ****************************************************************************
## *****************  Bits definition for GPIO_MODER register  ****************

const
  GPIO_MODER_MODE0_Pos* = (0)
  GPIO_MODER_MODE0_Msk* = (0x00000003 shl GPIO_MODER_MODE0_Pos) ## !< 0x00000003
  GPIO_MODER_MODE0* = GPIO_MODER_MODE0_Msk
  GPIO_MODER_MODE0_Bit0* = (0x00000001 shl GPIO_MODER_MODE0_Pos) ## !< 0x00000001
  GPIO_MODER_MODE0_Bit1* = (0x00000002 shl GPIO_MODER_MODE0_Pos) ## !< 0x00000002
  GPIO_MODER_MODE1_Pos* = (2)
  GPIO_MODER_MODE1_Msk* = (0x00000003 shl GPIO_MODER_MODE1_Pos) ## !< 0x0000000C
  GPIO_MODER_MODE1* = GPIO_MODER_MODE1_Msk
  GPIO_MODER_MODE1_Bit0* = (0x00000001 shl GPIO_MODER_MODE1_Pos) ## !< 0x00000004
  GPIO_MODER_MODE1_Bit1* = (0x00000002 shl GPIO_MODER_MODE1_Pos) ## !< 0x00000008
  GPIO_MODER_MODE2_Pos* = (4)
  GPIO_MODER_MODE2_Msk* = (0x00000003 shl GPIO_MODER_MODE2_Pos) ## !< 0x00000030
  GPIO_MODER_MODE2* = GPIO_MODER_MODE2_Msk
  GPIO_MODER_MODE2_Bit0* = (0x00000001 shl GPIO_MODER_MODE2_Pos) ## !< 0x00000010
  GPIO_MODER_MODE2_Bit1* = (0x00000002 shl GPIO_MODER_MODE2_Pos) ## !< 0x00000020
  GPIO_MODER_MODE3_Pos* = (6)
  GPIO_MODER_MODE3_Msk* = (0x00000003 shl GPIO_MODER_MODE3_Pos) ## !< 0x000000C0
  GPIO_MODER_MODE3* = GPIO_MODER_MODE3_Msk
  GPIO_MODER_MODE3_Bit0* = (0x00000001 shl GPIO_MODER_MODE3_Pos) ## !< 0x00000040
  GPIO_MODER_MODE3_Bit1* = (0x00000002 shl GPIO_MODER_MODE3_Pos) ## !< 0x00000080
  GPIO_MODER_MODE4_Pos* = (8)
  GPIO_MODER_MODE4_Msk* = (0x00000003 shl GPIO_MODER_MODE4_Pos) ## !< 0x00000300
  GPIO_MODER_MODE4* = GPIO_MODER_MODE4_Msk
  GPIO_MODER_MODE4_Bit0* = (0x00000001 shl GPIO_MODER_MODE4_Pos) ## !< 0x00000100
  GPIO_MODER_MODE4_Bit1* = (0x00000002 shl GPIO_MODER_MODE4_Pos) ## !< 0x00000200
  GPIO_MODER_MODE5_Pos* = (10)
  GPIO_MODER_MODE5_Msk* = (0x00000003 shl GPIO_MODER_MODE5_Pos) ## !< 0x00000C00
  GPIO_MODER_MODE5* = GPIO_MODER_MODE5_Msk
  GPIO_MODER_MODE5_Bit0* = (0x00000001 shl GPIO_MODER_MODE5_Pos) ## !< 0x00000400
  GPIO_MODER_MODE5_Bit1* = (0x00000002 shl GPIO_MODER_MODE5_Pos) ## !< 0x00000800
  GPIO_MODER_MODE6_Pos* = (12)
  GPIO_MODER_MODE6_Msk* = (0x00000003 shl GPIO_MODER_MODE6_Pos) ## !< 0x00003000
  GPIO_MODER_MODE6* = GPIO_MODER_MODE6_Msk
  GPIO_MODER_MODE6_Bit0* = (0x00000001 shl GPIO_MODER_MODE6_Pos) ## !< 0x00001000
  GPIO_MODER_MODE6_Bit1* = (0x00000002 shl GPIO_MODER_MODE6_Pos) ## !< 0x00002000
  GPIO_MODER_MODE7_Pos* = (14)
  GPIO_MODER_MODE7_Msk* = (0x00000003 shl GPIO_MODER_MODE7_Pos) ## !< 0x0000C000
  GPIO_MODER_MODE7* = GPIO_MODER_MODE7_Msk
  GPIO_MODER_MODE7_Bit0* = (0x00000001 shl GPIO_MODER_MODE7_Pos) ## !< 0x00004000
  GPIO_MODER_MODE7_Bit1* = (0x00000002 shl GPIO_MODER_MODE7_Pos) ## !< 0x00008000
  GPIO_MODER_MODE8_Pos* = (16)
  GPIO_MODER_MODE8_Msk* = (0x00000003 shl GPIO_MODER_MODE8_Pos) ## !< 0x00030000
  GPIO_MODER_MODE8* = GPIO_MODER_MODE8_Msk
  GPIO_MODER_MODE8_Bit0* = (0x00000001 shl GPIO_MODER_MODE8_Pos) ## !< 0x00010000
  GPIO_MODER_MODE8_Bit1* = (0x00000002 shl GPIO_MODER_MODE8_Pos) ## !< 0x00020000
  GPIO_MODER_MODE9_Pos* = (18)
  GPIO_MODER_MODE9_Msk* = (0x00000003 shl GPIO_MODER_MODE9_Pos) ## !< 0x000C0000
  GPIO_MODER_MODE9* = GPIO_MODER_MODE9_Msk
  GPIO_MODER_MODE9_Bit0* = (0x00000001 shl GPIO_MODER_MODE9_Pos) ## !< 0x00040000
  GPIO_MODER_MODE9_Bit1* = (0x00000002 shl GPIO_MODER_MODE9_Pos) ## !< 0x00080000
  GPIO_MODER_MODE10_Pos* = (20)
  GPIO_MODER_MODE10_Msk* = (0x00000003 shl GPIO_MODER_MODE10_Pos) ## !< 0x00300000
  GPIO_MODER_MODE10* = GPIO_MODER_MODE10_Msk
  GPIO_MODER_MODE10_Bit0* = (0x00000001 shl GPIO_MODER_MODE10_Pos) ## !< 0x00100000
  GPIO_MODER_MODE10_Bit1* = (0x00000002 shl GPIO_MODER_MODE10_Pos) ## !< 0x00200000
  GPIO_MODER_MODE11_Pos* = (22)
  GPIO_MODER_MODE11_Msk* = (0x00000003 shl GPIO_MODER_MODE11_Pos) ## !< 0x00C00000
  GPIO_MODER_MODE11* = GPIO_MODER_MODE11_Msk
  GPIO_MODER_MODE11_Bit0* = (0x00000001 shl GPIO_MODER_MODE11_Pos) ## !< 0x00400000
  GPIO_MODER_MODE11_Bit1* = (0x00000002 shl GPIO_MODER_MODE11_Pos) ## !< 0x00800000
  GPIO_MODER_MODE12_Pos* = (24)
  GPIO_MODER_MODE12_Msk* = (0x00000003 shl GPIO_MODER_MODE12_Pos) ## !< 0x03000000
  GPIO_MODER_MODE12* = GPIO_MODER_MODE12_Msk
  GPIO_MODER_MODE12_Bit0* = (0x00000001 shl GPIO_MODER_MODE12_Pos) ## !< 0x01000000
  GPIO_MODER_MODE12_Bit1* = (0x00000002 shl GPIO_MODER_MODE12_Pos) ## !< 0x02000000
  GPIO_MODER_MODE13_Pos* = (26)
  GPIO_MODER_MODE13_Msk* = (0x00000003 shl GPIO_MODER_MODE13_Pos) ## !< 0x0C000000
  GPIO_MODER_MODE13* = GPIO_MODER_MODE13_Msk
  GPIO_MODER_MODE13_Bit0* = (0x00000001 shl GPIO_MODER_MODE13_Pos) ## !< 0x04000000
  GPIO_MODER_MODE13_Bit1* = (0x00000002 shl GPIO_MODER_MODE13_Pos) ## !< 0x08000000
  GPIO_MODER_MODE14_Pos* = (28)
  GPIO_MODER_MODE14_Msk* = (0x00000003 shl GPIO_MODER_MODE14_Pos) ## !< 0x30000000
  GPIO_MODER_MODE14* = GPIO_MODER_MODE14_Msk
  GPIO_MODER_MODE14_Bit0* = (0x00000001 shl GPIO_MODER_MODE14_Pos) ## !< 0x10000000
  GPIO_MODER_MODE14_Bit1* = (0x00000002 shl GPIO_MODER_MODE14_Pos) ## !< 0x20000000
  GPIO_MODER_MODE15_Pos* = (30)
  GPIO_MODER_MODE15_Msk* = (0x00000003 shl GPIO_MODER_MODE15_Pos) ## !< 0xC0000000
  GPIO_MODER_MODE15* = GPIO_MODER_MODE15_Msk
  GPIO_MODER_MODE15_Bit0* = (0x00000001 shl GPIO_MODER_MODE15_Pos) ## !< 0x40000000
  GPIO_MODER_MODE15_Bit1* = (0x00000002 shl GPIO_MODER_MODE15_Pos) ## !< 0x80000000

##  Legacy defines

const
  GPIO_MODER_MODER0_Pos* = (0)
  GPIO_MODER_MODER0_Msk* = (0x00000003 shl GPIO_MODER_MODER0_Pos) ## !< 0x00000003
  GPIO_MODER_MODER0* = GPIO_MODER_MODER0_Msk
  GPIO_MODER_MODER0_Bit0* = (0x00000001 shl GPIO_MODER_MODER0_Pos) ## !< 0x00000001
  GPIO_MODER_MODER0_Bit1* = (0x00000002 shl GPIO_MODER_MODER0_Pos) ## !< 0x00000002
  GPIO_MODER_MODER1_Pos* = (2)
  GPIO_MODER_MODER1_Msk* = (0x00000003 shl GPIO_MODER_MODER1_Pos) ## !< 0x0000000C
  GPIO_MODER_MODER1* = GPIO_MODER_MODER1_Msk
  GPIO_MODER_MODER1_Bit0* = (0x00000001 shl GPIO_MODER_MODER1_Pos) ## !< 0x00000004
  GPIO_MODER_MODER1_Bit1* = (0x00000002 shl GPIO_MODER_MODER1_Pos) ## !< 0x00000008
  GPIO_MODER_MODER2_Pos* = (4)
  GPIO_MODER_MODER2_Msk* = (0x00000003 shl GPIO_MODER_MODER2_Pos) ## !< 0x00000030
  GPIO_MODER_MODER2* = GPIO_MODER_MODER2_Msk
  GPIO_MODER_MODER2_Bit0* = (0x00000001 shl GPIO_MODER_MODER2_Pos) ## !< 0x00000010
  GPIO_MODER_MODER2_Bit1* = (0x00000002 shl GPIO_MODER_MODER2_Pos) ## !< 0x00000020
  GPIO_MODER_MODER3_Pos* = (6)
  GPIO_MODER_MODER3_Msk* = (0x00000003 shl GPIO_MODER_MODER3_Pos) ## !< 0x000000C0
  GPIO_MODER_MODER3* = GPIO_MODER_MODER3_Msk
  GPIO_MODER_MODER3_Bit0* = (0x00000001 shl GPIO_MODER_MODER3_Pos) ## !< 0x00000040
  GPIO_MODER_MODER3_Bit1* = (0x00000002 shl GPIO_MODER_MODER3_Pos) ## !< 0x00000080
  GPIO_MODER_MODER4_Pos* = (8)
  GPIO_MODER_MODER4_Msk* = (0x00000003 shl GPIO_MODER_MODER4_Pos) ## !< 0x00000300
  GPIO_MODER_MODER4* = GPIO_MODER_MODER4_Msk
  GPIO_MODER_MODER4_Bit0* = (0x00000001 shl GPIO_MODER_MODER4_Pos) ## !< 0x00000100
  GPIO_MODER_MODER4_Bit1* = (0x00000002 shl GPIO_MODER_MODER4_Pos) ## !< 0x00000200
  GPIO_MODER_MODER5_Pos* = (10)
  GPIO_MODER_MODER5_Msk* = (0x00000003 shl GPIO_MODER_MODER5_Pos) ## !< 0x00000C00
  GPIO_MODER_MODER5* = GPIO_MODER_MODER5_Msk
  GPIO_MODER_MODER5_Bit0* = (0x00000001 shl GPIO_MODER_MODER5_Pos) ## !< 0x00000400
  GPIO_MODER_MODER5_Bit1* = (0x00000002 shl GPIO_MODER_MODER5_Pos) ## !< 0x00000800
  GPIO_MODER_MODER6_Pos* = (12)
  GPIO_MODER_MODER6_Msk* = (0x00000003 shl GPIO_MODER_MODER6_Pos) ## !< 0x00003000
  GPIO_MODER_MODER6* = GPIO_MODER_MODER6_Msk
  GPIO_MODER_MODER6_Bit0* = (0x00000001 shl GPIO_MODER_MODER6_Pos) ## !< 0x00001000
  GPIO_MODER_MODER6_Bit1* = (0x00000002 shl GPIO_MODER_MODER6_Pos) ## !< 0x00002000
  GPIO_MODER_MODER7_Pos* = (14)
  GPIO_MODER_MODER7_Msk* = (0x00000003 shl GPIO_MODER_MODER7_Pos) ## !< 0x0000C000
  GPIO_MODER_MODER7* = GPIO_MODER_MODER7_Msk
  GPIO_MODER_MODER7_Bit0* = (0x00000001 shl GPIO_MODER_MODER7_Pos) ## !< 0x00004000
  GPIO_MODER_MODER7_Bit1* = (0x00000002 shl GPIO_MODER_MODER7_Pos) ## !< 0x00008000
  GPIO_MODER_MODER8_Pos* = (16)
  GPIO_MODER_MODER8_Msk* = (0x00000003 shl GPIO_MODER_MODER8_Pos) ## !< 0x00030000
  GPIO_MODER_MODER8* = GPIO_MODER_MODER8_Msk
  GPIO_MODER_MODER8_Bit0* = (0x00000001 shl GPIO_MODER_MODER8_Pos) ## !< 0x00010000
  GPIO_MODER_MODER8_Bit1* = (0x00000002 shl GPIO_MODER_MODER8_Pos) ## !< 0x00020000
  GPIO_MODER_MODER9_Pos* = (18)
  GPIO_MODER_MODER9_Msk* = (0x00000003 shl GPIO_MODER_MODER9_Pos) ## !< 0x000C0000
  GPIO_MODER_MODER9* = GPIO_MODER_MODER9_Msk
  GPIO_MODER_MODER9_Bit0* = (0x00000001 shl GPIO_MODER_MODER9_Pos) ## !< 0x00040000
  GPIO_MODER_MODER9_Bit1* = (0x00000002 shl GPIO_MODER_MODER9_Pos) ## !< 0x00080000
  GPIO_MODER_MODER10_Pos* = (20)
  GPIO_MODER_MODER10_Msk* = (0x00000003 shl GPIO_MODER_MODER10_Pos) ## !< 0x00300000
  GPIO_MODER_MODER10* = GPIO_MODER_MODER10_Msk
  GPIO_MODER_MODER10_Bit0* = (0x00000001 shl GPIO_MODER_MODER10_Pos) ## !< 0x00100000
  GPIO_MODER_MODER10_Bit1* = (0x00000002 shl GPIO_MODER_MODER10_Pos) ## !< 0x00200000
  GPIO_MODER_MODER11_Pos* = (22)
  GPIO_MODER_MODER11_Msk* = (0x00000003 shl GPIO_MODER_MODER11_Pos) ## !< 0x00C00000
  GPIO_MODER_MODER11* = GPIO_MODER_MODER11_Msk
  GPIO_MODER_MODER11_Bit0* = (0x00000001 shl GPIO_MODER_MODER11_Pos) ## !< 0x00400000
  GPIO_MODER_MODER11_Bit1* = (0x00000002 shl GPIO_MODER_MODER11_Pos) ## !< 0x00800000
  GPIO_MODER_MODER12_Pos* = (24)
  GPIO_MODER_MODER12_Msk* = (0x00000003 shl GPIO_MODER_MODER12_Pos) ## !< 0x03000000
  GPIO_MODER_MODER12* = GPIO_MODER_MODER12_Msk
  GPIO_MODER_MODER12_Bit0* = (0x00000001 shl GPIO_MODER_MODER12_Pos) ## !< 0x01000000
  GPIO_MODER_MODER12_Bit1* = (0x00000002 shl GPIO_MODER_MODER12_Pos) ## !< 0x02000000
  GPIO_MODER_MODER13_Pos* = (26)
  GPIO_MODER_MODER13_Msk* = (0x00000003 shl GPIO_MODER_MODER13_Pos) ## !< 0x0C000000
  GPIO_MODER_MODER13* = GPIO_MODER_MODER13_Msk
  GPIO_MODER_MODER13_Bit0* = (0x00000001 shl GPIO_MODER_MODER13_Pos) ## !< 0x04000000
  GPIO_MODER_MODER13_Bit1* = (0x00000002 shl GPIO_MODER_MODER13_Pos) ## !< 0x08000000
  GPIO_MODER_MODER14_Pos* = (28)
  GPIO_MODER_MODER14_Msk* = (0x00000003 shl GPIO_MODER_MODER14_Pos) ## !< 0x30000000
  GPIO_MODER_MODER14* = GPIO_MODER_MODER14_Msk
  GPIO_MODER_MODER14_Bit0* = (0x00000001 shl GPIO_MODER_MODER14_Pos) ## !< 0x10000000
  GPIO_MODER_MODER14_Bit1* = (0x00000002 shl GPIO_MODER_MODER14_Pos) ## !< 0x20000000
  GPIO_MODER_MODER15_Pos* = (30)
  GPIO_MODER_MODER15_Msk* = (0x00000003 shl GPIO_MODER_MODER15_Pos) ## !< 0xC0000000
  GPIO_MODER_MODER15* = GPIO_MODER_MODER15_Msk
  GPIO_MODER_MODER15_Bit0* = (0x00000001 shl GPIO_MODER_MODER15_Pos) ## !< 0x40000000
  GPIO_MODER_MODER15_Bit1* = (0x00000002 shl GPIO_MODER_MODER15_Pos) ## !< 0x80000000

## *****************  Bits definition for GPIO_OTYPER register  ***************

const
  GPIO_OTYPER_OT0_Pos* = (0)
  GPIO_OTYPER_OT0_Msk* = (0x00000001 shl GPIO_OTYPER_OT0_Pos) ## !< 0x00000001
  GPIO_OTYPER_OT0* = GPIO_OTYPER_OT0_Msk
  GPIO_OTYPER_OT1_Pos* = (1)
  GPIO_OTYPER_OT1_Msk* = (0x00000001 shl GPIO_OTYPER_OT1_Pos) ## !< 0x00000002
  GPIO_OTYPER_OT1* = GPIO_OTYPER_OT1_Msk
  GPIO_OTYPER_OT2_Pos* = (2)
  GPIO_OTYPER_OT2_Msk* = (0x00000001 shl GPIO_OTYPER_OT2_Pos) ## !< 0x00000004
  GPIO_OTYPER_OT2* = GPIO_OTYPER_OT2_Msk
  GPIO_OTYPER_OT3_Pos* = (3)
  GPIO_OTYPER_OT3_Msk* = (0x00000001 shl GPIO_OTYPER_OT3_Pos) ## !< 0x00000008
  GPIO_OTYPER_OT3* = GPIO_OTYPER_OT3_Msk
  GPIO_OTYPER_OT4_Pos* = (4)
  GPIO_OTYPER_OT4_Msk* = (0x00000001 shl GPIO_OTYPER_OT4_Pos) ## !< 0x00000010
  GPIO_OTYPER_OT4* = GPIO_OTYPER_OT4_Msk
  GPIO_OTYPER_OT5_Pos* = (5)
  GPIO_OTYPER_OT5_Msk* = (0x00000001 shl GPIO_OTYPER_OT5_Pos) ## !< 0x00000020
  GPIO_OTYPER_OT5* = GPIO_OTYPER_OT5_Msk
  GPIO_OTYPER_OT6_Pos* = (6)
  GPIO_OTYPER_OT6_Msk* = (0x00000001 shl GPIO_OTYPER_OT6_Pos) ## !< 0x00000040
  GPIO_OTYPER_OT6* = GPIO_OTYPER_OT6_Msk
  GPIO_OTYPER_OT7_Pos* = (7)
  GPIO_OTYPER_OT7_Msk* = (0x00000001 shl GPIO_OTYPER_OT7_Pos) ## !< 0x00000080
  GPIO_OTYPER_OT7* = GPIO_OTYPER_OT7_Msk
  GPIO_OTYPER_OT8_Pos* = (8)
  GPIO_OTYPER_OT8_Msk* = (0x00000001 shl GPIO_OTYPER_OT8_Pos) ## !< 0x00000100
  GPIO_OTYPER_OT8* = GPIO_OTYPER_OT8_Msk
  GPIO_OTYPER_OT9_Pos* = (9)
  GPIO_OTYPER_OT9_Msk* = (0x00000001 shl GPIO_OTYPER_OT9_Pos) ## !< 0x00000200
  GPIO_OTYPER_OT9* = GPIO_OTYPER_OT9_Msk
  GPIO_OTYPER_OT10_Pos* = (10)
  GPIO_OTYPER_OT10_Msk* = (0x00000001 shl GPIO_OTYPER_OT10_Pos) ## !< 0x00000400
  GPIO_OTYPER_OT10* = GPIO_OTYPER_OT10_Msk
  GPIO_OTYPER_OT11_Pos* = (11)
  GPIO_OTYPER_OT11_Msk* = (0x00000001 shl GPIO_OTYPER_OT11_Pos) ## !< 0x00000800
  GPIO_OTYPER_OT11* = GPIO_OTYPER_OT11_Msk
  GPIO_OTYPER_OT12_Pos* = (12)
  GPIO_OTYPER_OT12_Msk* = (0x00000001 shl GPIO_OTYPER_OT12_Pos) ## !< 0x00001000
  GPIO_OTYPER_OT12* = GPIO_OTYPER_OT12_Msk
  GPIO_OTYPER_OT13_Pos* = (13)
  GPIO_OTYPER_OT13_Msk* = (0x00000001 shl GPIO_OTYPER_OT13_Pos) ## !< 0x00002000
  GPIO_OTYPER_OT13* = GPIO_OTYPER_OT13_Msk
  GPIO_OTYPER_OT14_Pos* = (14)
  GPIO_OTYPER_OT14_Msk* = (0x00000001 shl GPIO_OTYPER_OT14_Pos) ## !< 0x00004000
  GPIO_OTYPER_OT14* = GPIO_OTYPER_OT14_Msk
  GPIO_OTYPER_OT15_Pos* = (15)
  GPIO_OTYPER_OT15_Msk* = (0x00000001 shl GPIO_OTYPER_OT15_Pos) ## !< 0x00008000
  GPIO_OTYPER_OT15* = GPIO_OTYPER_OT15_Msk

##  Legacy defines

const
  GPIO_OTYPER_OT_Bit0* = GPIO_OTYPER_OT0
  GPIO_OTYPER_OT_Bit1* = GPIO_OTYPER_OT1
  GPIO_OTYPER_OT_Bit2* = GPIO_OTYPER_OT2
  GPIO_OTYPER_OT_Bit3* = GPIO_OTYPER_OT3
  GPIO_OTYPER_OT_Bit4* = GPIO_OTYPER_OT4
  GPIO_OTYPER_OT_Bit5* = GPIO_OTYPER_OT5
  GPIO_OTYPER_OT_Bit6* = GPIO_OTYPER_OT6
  GPIO_OTYPER_OT_Bit7* = GPIO_OTYPER_OT7
  GPIO_OTYPER_OT_Bit8* = GPIO_OTYPER_OT8
  GPIO_OTYPER_OT_Bit9* = GPIO_OTYPER_OT9
  GPIO_OTYPER_OT_Bit10* = GPIO_OTYPER_OT10
  GPIO_OTYPER_OT_Bit11* = GPIO_OTYPER_OT11
  GPIO_OTYPER_OT_Bit12* = GPIO_OTYPER_OT12
  GPIO_OTYPER_OT_Bit13* = GPIO_OTYPER_OT13
  GPIO_OTYPER_OT_Bit14* = GPIO_OTYPER_OT14
  GPIO_OTYPER_OT_Bit15* = GPIO_OTYPER_OT15

## *****************  Bits definition for GPIO_OSPEEDR register  **************

const
  GPIO_OSPEEDR_OSPEED0_Pos* = (0)
  GPIO_OSPEEDR_OSPEED0_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED0_Pos) ## !< 0x00000003
  GPIO_OSPEEDR_OSPEED0* = GPIO_OSPEEDR_OSPEED0_Msk
  GPIO_OSPEEDR_OSPEED0_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED0_Pos) ## !< 0x00000001
  GPIO_OSPEEDR_OSPEED0_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED0_Pos) ## !< 0x00000002
  GPIO_OSPEEDR_OSPEED1_Pos* = (2)
  GPIO_OSPEEDR_OSPEED1_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED1_Pos) ## !< 0x0000000C
  GPIO_OSPEEDR_OSPEED1* = GPIO_OSPEEDR_OSPEED1_Msk
  GPIO_OSPEEDR_OSPEED1_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED1_Pos) ## !< 0x00000004
  GPIO_OSPEEDR_OSPEED1_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED1_Pos) ## !< 0x00000008
  GPIO_OSPEEDR_OSPEED2_Pos* = (4)
  GPIO_OSPEEDR_OSPEED2_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED2_Pos) ## !< 0x00000030
  GPIO_OSPEEDR_OSPEED2* = GPIO_OSPEEDR_OSPEED2_Msk
  GPIO_OSPEEDR_OSPEED2_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED2_Pos) ## !< 0x00000010
  GPIO_OSPEEDR_OSPEED2_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED2_Pos) ## !< 0x00000020
  GPIO_OSPEEDR_OSPEED3_Pos* = (6)
  GPIO_OSPEEDR_OSPEED3_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED3_Pos) ## !< 0x000000C0
  GPIO_OSPEEDR_OSPEED3* = GPIO_OSPEEDR_OSPEED3_Msk
  GPIO_OSPEEDR_OSPEED3_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED3_Pos) ## !< 0x00000040
  GPIO_OSPEEDR_OSPEED3_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED3_Pos) ## !< 0x00000080
  GPIO_OSPEEDR_OSPEED4_Pos* = (8)
  GPIO_OSPEEDR_OSPEED4_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED4_Pos) ## !< 0x00000300
  GPIO_OSPEEDR_OSPEED4* = GPIO_OSPEEDR_OSPEED4_Msk
  GPIO_OSPEEDR_OSPEED4_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED4_Pos) ## !< 0x00000100
  GPIO_OSPEEDR_OSPEED4_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED4_Pos) ## !< 0x00000200
  GPIO_OSPEEDR_OSPEED5_Pos* = (10)
  GPIO_OSPEEDR_OSPEED5_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED5_Pos) ## !< 0x00000C00
  GPIO_OSPEEDR_OSPEED5* = GPIO_OSPEEDR_OSPEED5_Msk
  GPIO_OSPEEDR_OSPEED5_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED5_Pos) ## !< 0x00000400
  GPIO_OSPEEDR_OSPEED5_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED5_Pos) ## !< 0x00000800
  GPIO_OSPEEDR_OSPEED6_Pos* = (12)
  GPIO_OSPEEDR_OSPEED6_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED6_Pos) ## !< 0x00003000
  GPIO_OSPEEDR_OSPEED6* = GPIO_OSPEEDR_OSPEED6_Msk
  GPIO_OSPEEDR_OSPEED6_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED6_Pos) ## !< 0x00001000
  GPIO_OSPEEDR_OSPEED6_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED6_Pos) ## !< 0x00002000
  GPIO_OSPEEDR_OSPEED7_Pos* = (14)
  GPIO_OSPEEDR_OSPEED7_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED7_Pos) ## !< 0x0000C000
  GPIO_OSPEEDR_OSPEED7* = GPIO_OSPEEDR_OSPEED7_Msk
  GPIO_OSPEEDR_OSPEED7_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED7_Pos) ## !< 0x00004000
  GPIO_OSPEEDR_OSPEED7_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED7_Pos) ## !< 0x00008000
  GPIO_OSPEEDR_OSPEED8_Pos* = (16)
  GPIO_OSPEEDR_OSPEED8_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED8_Pos) ## !< 0x00030000
  GPIO_OSPEEDR_OSPEED8* = GPIO_OSPEEDR_OSPEED8_Msk
  GPIO_OSPEEDR_OSPEED8_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED8_Pos) ## !< 0x00010000
  GPIO_OSPEEDR_OSPEED8_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED8_Pos) ## !< 0x00020000
  GPIO_OSPEEDR_OSPEED9_Pos* = (18)
  GPIO_OSPEEDR_OSPEED9_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED9_Pos) ## !< 0x000C0000
  GPIO_OSPEEDR_OSPEED9* = GPIO_OSPEEDR_OSPEED9_Msk
  GPIO_OSPEEDR_OSPEED9_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED9_Pos) ## !< 0x00040000
  GPIO_OSPEEDR_OSPEED9_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED9_Pos) ## !< 0x00080000
  GPIO_OSPEEDR_OSPEED10_Pos* = (20)
  GPIO_OSPEEDR_OSPEED10_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED10_Pos) ## !< 0x00300000
  GPIO_OSPEEDR_OSPEED10* = GPIO_OSPEEDR_OSPEED10_Msk
  GPIO_OSPEEDR_OSPEED10_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED10_Pos) ## !< 0x00100000
  GPIO_OSPEEDR_OSPEED10_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED10_Pos) ## !< 0x00200000
  GPIO_OSPEEDR_OSPEED11_Pos* = (22)
  GPIO_OSPEEDR_OSPEED11_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED11_Pos) ## !< 0x00C00000
  GPIO_OSPEEDR_OSPEED11* = GPIO_OSPEEDR_OSPEED11_Msk
  GPIO_OSPEEDR_OSPEED11_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED11_Pos) ## !< 0x00400000
  GPIO_OSPEEDR_OSPEED11_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED11_Pos) ## !< 0x00800000
  GPIO_OSPEEDR_OSPEED12_Pos* = (24)
  GPIO_OSPEEDR_OSPEED12_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED12_Pos) ## !< 0x03000000
  GPIO_OSPEEDR_OSPEED12* = GPIO_OSPEEDR_OSPEED12_Msk
  GPIO_OSPEEDR_OSPEED12_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED12_Pos) ## !< 0x01000000
  GPIO_OSPEEDR_OSPEED12_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED12_Pos) ## !< 0x02000000
  GPIO_OSPEEDR_OSPEED13_Pos* = (26)
  GPIO_OSPEEDR_OSPEED13_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED13_Pos) ## !< 0x0C000000
  GPIO_OSPEEDR_OSPEED13* = GPIO_OSPEEDR_OSPEED13_Msk
  GPIO_OSPEEDR_OSPEED13_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED13_Pos) ## !< 0x04000000
  GPIO_OSPEEDR_OSPEED13_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED13_Pos) ## !< 0x08000000
  GPIO_OSPEEDR_OSPEED14_Pos* = (28)
  GPIO_OSPEEDR_OSPEED14_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED14_Pos) ## !< 0x30000000
  GPIO_OSPEEDR_OSPEED14* = GPIO_OSPEEDR_OSPEED14_Msk
  GPIO_OSPEEDR_OSPEED14_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED14_Pos) ## !< 0x10000000
  GPIO_OSPEEDR_OSPEED14_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED14_Pos) ## !< 0x20000000
  GPIO_OSPEEDR_OSPEED15_Pos* = (30)
  GPIO_OSPEEDR_OSPEED15_Msk* = (0x00000003 shl GPIO_OSPEEDR_OSPEED15_Pos) ## !< 0xC0000000
  GPIO_OSPEEDR_OSPEED15* = GPIO_OSPEEDR_OSPEED15_Msk
  GPIO_OSPEEDR_OSPEED15_Bit0* = (0x00000001 shl GPIO_OSPEEDR_OSPEED15_Pos) ## !< 0x40000000
  GPIO_OSPEEDR_OSPEED15_Bit1* = (0x00000002 shl GPIO_OSPEEDR_OSPEED15_Pos) ## !< 0x80000000

##  Legacy defines

const
  GPIO_OSPEEDER_OSPEEDR0* = GPIO_OSPEEDR_OSPEED0
  GPIO_OSPEEDER_OSPEEDR0_Bit0* = GPIO_OSPEEDR_OSPEED0_Bit0
  GPIO_OSPEEDER_OSPEEDR0_Bit1* = GPIO_OSPEEDR_OSPEED0_Bit1
  GPIO_OSPEEDER_OSPEEDR1* = GPIO_OSPEEDR_OSPEED1
  GPIO_OSPEEDER_OSPEEDR1_Bit0* = GPIO_OSPEEDR_OSPEED1_Bit0
  GPIO_OSPEEDER_OSPEEDR1_Bit1* = GPIO_OSPEEDR_OSPEED1_Bit1
  GPIO_OSPEEDER_OSPEEDR2* = GPIO_OSPEEDR_OSPEED2
  GPIO_OSPEEDER_OSPEEDR2_Bit0* = GPIO_OSPEEDR_OSPEED2_Bit0
  GPIO_OSPEEDER_OSPEEDR2_Bit1* = GPIO_OSPEEDR_OSPEED2_Bit1
  GPIO_OSPEEDER_OSPEEDR3* = GPIO_OSPEEDR_OSPEED3
  GPIO_OSPEEDER_OSPEEDR3_Bit0* = GPIO_OSPEEDR_OSPEED3_Bit0
  GPIO_OSPEEDER_OSPEEDR3_Bit1* = GPIO_OSPEEDR_OSPEED3_Bit1
  GPIO_OSPEEDER_OSPEEDR4* = GPIO_OSPEEDR_OSPEED4
  GPIO_OSPEEDER_OSPEEDR4_Bit0* = GPIO_OSPEEDR_OSPEED4_Bit0
  GPIO_OSPEEDER_OSPEEDR4_Bit1* = GPIO_OSPEEDR_OSPEED4_Bit1
  GPIO_OSPEEDER_OSPEEDR5* = GPIO_OSPEEDR_OSPEED5
  GPIO_OSPEEDER_OSPEEDR5_Bit0* = GPIO_OSPEEDR_OSPEED5_Bit0
  GPIO_OSPEEDER_OSPEEDR5_Bit1* = GPIO_OSPEEDR_OSPEED5_Bit1
  GPIO_OSPEEDER_OSPEEDR6* = GPIO_OSPEEDR_OSPEED6
  GPIO_OSPEEDER_OSPEEDR6_Bit0* = GPIO_OSPEEDR_OSPEED6_Bit0
  GPIO_OSPEEDER_OSPEEDR6_Bit1* = GPIO_OSPEEDR_OSPEED6_Bit1
  GPIO_OSPEEDER_OSPEEDR7* = GPIO_OSPEEDR_OSPEED7
  GPIO_OSPEEDER_OSPEEDR7_Bit0* = GPIO_OSPEEDR_OSPEED7_Bit0
  GPIO_OSPEEDER_OSPEEDR7_Bit1* = GPIO_OSPEEDR_OSPEED7_Bit1
  GPIO_OSPEEDER_OSPEEDR8* = GPIO_OSPEEDR_OSPEED8
  GPIO_OSPEEDER_OSPEEDR8_Bit0* = GPIO_OSPEEDR_OSPEED8_Bit0
  GPIO_OSPEEDER_OSPEEDR8_Bit1* = GPIO_OSPEEDR_OSPEED8_Bit1
  GPIO_OSPEEDER_OSPEEDR9* = GPIO_OSPEEDR_OSPEED9
  GPIO_OSPEEDER_OSPEEDR9_Bit0* = GPIO_OSPEEDR_OSPEED9_Bit0
  GPIO_OSPEEDER_OSPEEDR9_Bit1* = GPIO_OSPEEDR_OSPEED9_Bit1
  GPIO_OSPEEDER_OSPEEDR10* = GPIO_OSPEEDR_OSPEED10
  GPIO_OSPEEDER_OSPEEDR10_Bit0* = GPIO_OSPEEDR_OSPEED10_Bit0
  GPIO_OSPEEDER_OSPEEDR10_Bit1* = GPIO_OSPEEDR_OSPEED10_Bit1
  GPIO_OSPEEDER_OSPEEDR11* = GPIO_OSPEEDR_OSPEED11
  GPIO_OSPEEDER_OSPEEDR11_Bit0* = GPIO_OSPEEDR_OSPEED11_Bit0
  GPIO_OSPEEDER_OSPEEDR11_Bit1* = GPIO_OSPEEDR_OSPEED11_Bit1
  GPIO_OSPEEDER_OSPEEDR12* = GPIO_OSPEEDR_OSPEED12
  GPIO_OSPEEDER_OSPEEDR12_Bit0* = GPIO_OSPEEDR_OSPEED12_Bit0
  GPIO_OSPEEDER_OSPEEDR12_Bit1* = GPIO_OSPEEDR_OSPEED12_Bit1
  GPIO_OSPEEDER_OSPEEDR13* = GPIO_OSPEEDR_OSPEED13
  GPIO_OSPEEDER_OSPEEDR13_Bit0* = GPIO_OSPEEDR_OSPEED13_Bit0
  GPIO_OSPEEDER_OSPEEDR13_Bit1* = GPIO_OSPEEDR_OSPEED13_Bit1
  GPIO_OSPEEDER_OSPEEDR14* = GPIO_OSPEEDR_OSPEED14
  GPIO_OSPEEDER_OSPEEDR14_Bit0* = GPIO_OSPEEDR_OSPEED14_Bit0
  GPIO_OSPEEDER_OSPEEDR14_Bit1* = GPIO_OSPEEDR_OSPEED14_Bit1
  GPIO_OSPEEDER_OSPEEDR15* = GPIO_OSPEEDR_OSPEED15
  GPIO_OSPEEDER_OSPEEDR15_Bit0* = GPIO_OSPEEDR_OSPEED15_Bit0
  GPIO_OSPEEDER_OSPEEDR15_Bit1* = GPIO_OSPEEDR_OSPEED15_Bit1

## *****************  Bits definition for GPIO_PUPDR register  ****************

const
  GPIO_PUPDR_PUPD0_Pos* = (0)
  GPIO_PUPDR_PUPD0_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD0_Pos) ## !< 0x00000003
  GPIO_PUPDR_PUPD0* = GPIO_PUPDR_PUPD0_Msk
  GPIO_PUPDR_PUPD0_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD0_Pos) ## !< 0x00000001
  GPIO_PUPDR_PUPD0_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD0_Pos) ## !< 0x00000002
  GPIO_PUPDR_PUPD1_Pos* = (2)
  GPIO_PUPDR_PUPD1_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD1_Pos) ## !< 0x0000000C
  GPIO_PUPDR_PUPD1* = GPIO_PUPDR_PUPD1_Msk
  GPIO_PUPDR_PUPD1_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD1_Pos) ## !< 0x00000004
  GPIO_PUPDR_PUPD1_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD1_Pos) ## !< 0x00000008
  GPIO_PUPDR_PUPD2_Pos* = (4)
  GPIO_PUPDR_PUPD2_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD2_Pos) ## !< 0x00000030
  GPIO_PUPDR_PUPD2* = GPIO_PUPDR_PUPD2_Msk
  GPIO_PUPDR_PUPD2_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD2_Pos) ## !< 0x00000010
  GPIO_PUPDR_PUPD2_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD2_Pos) ## !< 0x00000020
  GPIO_PUPDR_PUPD3_Pos* = (6)
  GPIO_PUPDR_PUPD3_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD3_Pos) ## !< 0x000000C0
  GPIO_PUPDR_PUPD3* = GPIO_PUPDR_PUPD3_Msk
  GPIO_PUPDR_PUPD3_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD3_Pos) ## !< 0x00000040
  GPIO_PUPDR_PUPD3_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD3_Pos) ## !< 0x00000080
  GPIO_PUPDR_PUPD4_Pos* = (8)
  GPIO_PUPDR_PUPD4_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD4_Pos) ## !< 0x00000300
  GPIO_PUPDR_PUPD4* = GPIO_PUPDR_PUPD4_Msk
  GPIO_PUPDR_PUPD4_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD4_Pos) ## !< 0x00000100
  GPIO_PUPDR_PUPD4_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD4_Pos) ## !< 0x00000200
  GPIO_PUPDR_PUPD5_Pos* = (10)
  GPIO_PUPDR_PUPD5_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD5_Pos) ## !< 0x00000C00
  GPIO_PUPDR_PUPD5* = GPIO_PUPDR_PUPD5_Msk
  GPIO_PUPDR_PUPD5_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD5_Pos) ## !< 0x00000400
  GPIO_PUPDR_PUPD5_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD5_Pos) ## !< 0x00000800
  GPIO_PUPDR_PUPD6_Pos* = (12)
  GPIO_PUPDR_PUPD6_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD6_Pos) ## !< 0x00003000
  GPIO_PUPDR_PUPD6* = GPIO_PUPDR_PUPD6_Msk
  GPIO_PUPDR_PUPD6_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD6_Pos) ## !< 0x00001000
  GPIO_PUPDR_PUPD6_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD6_Pos) ## !< 0x00002000
  GPIO_PUPDR_PUPD7_Pos* = (14)
  GPIO_PUPDR_PUPD7_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD7_Pos) ## !< 0x0000C000
  GPIO_PUPDR_PUPD7* = GPIO_PUPDR_PUPD7_Msk
  GPIO_PUPDR_PUPD7_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD7_Pos) ## !< 0x00004000
  GPIO_PUPDR_PUPD7_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD7_Pos) ## !< 0x00008000
  GPIO_PUPDR_PUPD8_Pos* = (16)
  GPIO_PUPDR_PUPD8_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD8_Pos) ## !< 0x00030000
  GPIO_PUPDR_PUPD8* = GPIO_PUPDR_PUPD8_Msk
  GPIO_PUPDR_PUPD8_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD8_Pos) ## !< 0x00010000
  GPIO_PUPDR_PUPD8_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD8_Pos) ## !< 0x00020000
  GPIO_PUPDR_PUPD9_Pos* = (18)
  GPIO_PUPDR_PUPD9_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD9_Pos) ## !< 0x000C0000
  GPIO_PUPDR_PUPD9* = GPIO_PUPDR_PUPD9_Msk
  GPIO_PUPDR_PUPD9_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD9_Pos) ## !< 0x00040000
  GPIO_PUPDR_PUPD9_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD9_Pos) ## !< 0x00080000
  GPIO_PUPDR_PUPD10_Pos* = (20)
  GPIO_PUPDR_PUPD10_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD10_Pos) ## !< 0x00300000
  GPIO_PUPDR_PUPD10* = GPIO_PUPDR_PUPD10_Msk
  GPIO_PUPDR_PUPD10_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD10_Pos) ## !< 0x00100000
  GPIO_PUPDR_PUPD10_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD10_Pos) ## !< 0x00200000
  GPIO_PUPDR_PUPD11_Pos* = (22)
  GPIO_PUPDR_PUPD11_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD11_Pos) ## !< 0x00C00000
  GPIO_PUPDR_PUPD11* = GPIO_PUPDR_PUPD11_Msk
  GPIO_PUPDR_PUPD11_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD11_Pos) ## !< 0x00400000
  GPIO_PUPDR_PUPD11_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD11_Pos) ## !< 0x00800000
  GPIO_PUPDR_PUPD12_Pos* = (24)
  GPIO_PUPDR_PUPD12_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD12_Pos) ## !< 0x03000000
  GPIO_PUPDR_PUPD12* = GPIO_PUPDR_PUPD12_Msk
  GPIO_PUPDR_PUPD12_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD12_Pos) ## !< 0x01000000
  GPIO_PUPDR_PUPD12_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD12_Pos) ## !< 0x02000000
  GPIO_PUPDR_PUPD13_Pos* = (26)
  GPIO_PUPDR_PUPD13_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD13_Pos) ## !< 0x0C000000
  GPIO_PUPDR_PUPD13* = GPIO_PUPDR_PUPD13_Msk
  GPIO_PUPDR_PUPD13_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD13_Pos) ## !< 0x04000000
  GPIO_PUPDR_PUPD13_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD13_Pos) ## !< 0x08000000
  GPIO_PUPDR_PUPD14_Pos* = (28)
  GPIO_PUPDR_PUPD14_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD14_Pos) ## !< 0x30000000
  GPIO_PUPDR_PUPD14* = GPIO_PUPDR_PUPD14_Msk
  GPIO_PUPDR_PUPD14_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD14_Pos) ## !< 0x10000000
  GPIO_PUPDR_PUPD14_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD14_Pos) ## !< 0x20000000
  GPIO_PUPDR_PUPD15_Pos* = (30)
  GPIO_PUPDR_PUPD15_Msk* = (0x00000003 shl GPIO_PUPDR_PUPD15_Pos) ## !< 0xC0000000
  GPIO_PUPDR_PUPD15* = GPIO_PUPDR_PUPD15_Msk
  GPIO_PUPDR_PUPD15_Bit0* = (0x00000001 shl GPIO_PUPDR_PUPD15_Pos) ## !< 0x40000000
  GPIO_PUPDR_PUPD15_Bit1* = (0x00000002 shl GPIO_PUPDR_PUPD15_Pos) ## !< 0x80000000

##  Legacy defines

const
  GPIO_PUPDR_PUPDR0* = GPIO_PUPDR_PUPD0
  GPIO_PUPDR_PUPDR0_Bit0* = GPIO_PUPDR_PUPD0_Bit0
  GPIO_PUPDR_PUPDR0_Bit1* = GPIO_PUPDR_PUPD0_Bit1
  GPIO_PUPDR_PUPDR1* = GPIO_PUPDR_PUPD1
  GPIO_PUPDR_PUPDR1_Bit0* = GPIO_PUPDR_PUPD1_Bit0
  GPIO_PUPDR_PUPDR1_Bit1* = GPIO_PUPDR_PUPD1_Bit1
  GPIO_PUPDR_PUPDR2* = GPIO_PUPDR_PUPD2
  GPIO_PUPDR_PUPDR2_Bit0* = GPIO_PUPDR_PUPD2_Bit0
  GPIO_PUPDR_PUPDR2_Bit1* = GPIO_PUPDR_PUPD2_Bit1
  GPIO_PUPDR_PUPDR3* = GPIO_PUPDR_PUPD3
  GPIO_PUPDR_PUPDR3_Bit0* = GPIO_PUPDR_PUPD3_Bit0
  GPIO_PUPDR_PUPDR3_Bit1* = GPIO_PUPDR_PUPD3_Bit1
  GPIO_PUPDR_PUPDR4* = GPIO_PUPDR_PUPD4
  GPIO_PUPDR_PUPDR4_Bit0* = GPIO_PUPDR_PUPD4_Bit0
  GPIO_PUPDR_PUPDR4_Bit1* = GPIO_PUPDR_PUPD4_Bit1
  GPIO_PUPDR_PUPDR5* = GPIO_PUPDR_PUPD5
  GPIO_PUPDR_PUPDR5_Bit0* = GPIO_PUPDR_PUPD5_Bit0
  GPIO_PUPDR_PUPDR5_Bit1* = GPIO_PUPDR_PUPD5_Bit1
  GPIO_PUPDR_PUPDR6* = GPIO_PUPDR_PUPD6
  GPIO_PUPDR_PUPDR6_Bit0* = GPIO_PUPDR_PUPD6_Bit0
  GPIO_PUPDR_PUPDR6_Bit1* = GPIO_PUPDR_PUPD6_Bit1
  GPIO_PUPDR_PUPDR7* = GPIO_PUPDR_PUPD7
  GPIO_PUPDR_PUPDR7_Bit0* = GPIO_PUPDR_PUPD7_Bit0
  GPIO_PUPDR_PUPDR7_Bit1* = GPIO_PUPDR_PUPD7_Bit1
  GPIO_PUPDR_PUPDR8* = GPIO_PUPDR_PUPD8
  GPIO_PUPDR_PUPDR8_Bit0* = GPIO_PUPDR_PUPD8_Bit0
  GPIO_PUPDR_PUPDR8_Bit1* = GPIO_PUPDR_PUPD8_Bit1
  GPIO_PUPDR_PUPDR9* = GPIO_PUPDR_PUPD9
  GPIO_PUPDR_PUPDR9_Bit0* = GPIO_PUPDR_PUPD9_Bit0
  GPIO_PUPDR_PUPDR9_Bit1* = GPIO_PUPDR_PUPD9_Bit1
  GPIO_PUPDR_PUPDR10* = GPIO_PUPDR_PUPD10
  GPIO_PUPDR_PUPDR10_Bit0* = GPIO_PUPDR_PUPD10_Bit0
  GPIO_PUPDR_PUPDR10_Bit1* = GPIO_PUPDR_PUPD10_Bit1
  GPIO_PUPDR_PUPDR11* = GPIO_PUPDR_PUPD11
  GPIO_PUPDR_PUPDR11_Bit0* = GPIO_PUPDR_PUPD11_Bit0
  GPIO_PUPDR_PUPDR11_Bit1* = GPIO_PUPDR_PUPD11_Bit1
  GPIO_PUPDR_PUPDR12* = GPIO_PUPDR_PUPD12
  GPIO_PUPDR_PUPDR12_Bit0* = GPIO_PUPDR_PUPD12_Bit0
  GPIO_PUPDR_PUPDR12_Bit1* = GPIO_PUPDR_PUPD12_Bit1
  GPIO_PUPDR_PUPDR13* = GPIO_PUPDR_PUPD13
  GPIO_PUPDR_PUPDR13_Bit0* = GPIO_PUPDR_PUPD13_Bit0
  GPIO_PUPDR_PUPDR13_Bit1* = GPIO_PUPDR_PUPD13_Bit1
  GPIO_PUPDR_PUPDR14* = GPIO_PUPDR_PUPD14
  GPIO_PUPDR_PUPDR14_Bit0* = GPIO_PUPDR_PUPD14_Bit0
  GPIO_PUPDR_PUPDR14_Bit1* = GPIO_PUPDR_PUPD14_Bit1
  GPIO_PUPDR_PUPDR15* = GPIO_PUPDR_PUPD15
  GPIO_PUPDR_PUPDR15_Bit0* = GPIO_PUPDR_PUPD15_Bit0
  GPIO_PUPDR_PUPDR15_Bit1* = GPIO_PUPDR_PUPD15_Bit1

## *****************  Bits definition for GPIO_IDR register  ******************

const
  GPIO_IDR_ID0_Pos* = (0)
  GPIO_IDR_ID0_Msk* = (0x00000001 shl GPIO_IDR_ID0_Pos) ## !< 0x00000001
  GPIO_IDR_ID0* = GPIO_IDR_ID0_Msk
  GPIO_IDR_ID1_Pos* = (1)
  GPIO_IDR_ID1_Msk* = (0x00000001 shl GPIO_IDR_ID1_Pos) ## !< 0x00000002
  GPIO_IDR_ID1* = GPIO_IDR_ID1_Msk
  GPIO_IDR_ID2_Pos* = (2)
  GPIO_IDR_ID2_Msk* = (0x00000001 shl GPIO_IDR_ID2_Pos) ## !< 0x00000004
  GPIO_IDR_ID2* = GPIO_IDR_ID2_Msk
  GPIO_IDR_ID3_Pos* = (3)
  GPIO_IDR_ID3_Msk* = (0x00000001 shl GPIO_IDR_ID3_Pos) ## !< 0x00000008
  GPIO_IDR_ID3* = GPIO_IDR_ID3_Msk
  GPIO_IDR_ID4_Pos* = (4)
  GPIO_IDR_ID4_Msk* = (0x00000001 shl GPIO_IDR_ID4_Pos) ## !< 0x00000010
  GPIO_IDR_ID4* = GPIO_IDR_ID4_Msk
  GPIO_IDR_ID5_Pos* = (5)
  GPIO_IDR_ID5_Msk* = (0x00000001 shl GPIO_IDR_ID5_Pos) ## !< 0x00000020
  GPIO_IDR_ID5* = GPIO_IDR_ID5_Msk
  GPIO_IDR_ID6_Pos* = (6)
  GPIO_IDR_ID6_Msk* = (0x00000001 shl GPIO_IDR_ID6_Pos) ## !< 0x00000040
  GPIO_IDR_ID6* = GPIO_IDR_ID6_Msk
  GPIO_IDR_ID7_Pos* = (7)
  GPIO_IDR_ID7_Msk* = (0x00000001 shl GPIO_IDR_ID7_Pos) ## !< 0x00000080
  GPIO_IDR_ID7* = GPIO_IDR_ID7_Msk
  GPIO_IDR_ID8_Pos* = (8)
  GPIO_IDR_ID8_Msk* = (0x00000001 shl GPIO_IDR_ID8_Pos) ## !< 0x00000100
  GPIO_IDR_ID8* = GPIO_IDR_ID8_Msk
  GPIO_IDR_ID9_Pos* = (9)
  GPIO_IDR_ID9_Msk* = (0x00000001 shl GPIO_IDR_ID9_Pos) ## !< 0x00000200
  GPIO_IDR_ID9* = GPIO_IDR_ID9_Msk
  GPIO_IDR_ID10_Pos* = (10)
  GPIO_IDR_ID10_Msk* = (0x00000001 shl GPIO_IDR_ID10_Pos) ## !< 0x00000400
  GPIO_IDR_ID10* = GPIO_IDR_ID10_Msk
  GPIO_IDR_ID11_Pos* = (11)
  GPIO_IDR_ID11_Msk* = (0x00000001 shl GPIO_IDR_ID11_Pos) ## !< 0x00000800
  GPIO_IDR_ID11* = GPIO_IDR_ID11_Msk
  GPIO_IDR_ID12_Pos* = (12)
  GPIO_IDR_ID12_Msk* = (0x00000001 shl GPIO_IDR_ID12_Pos) ## !< 0x00001000
  GPIO_IDR_ID12* = GPIO_IDR_ID12_Msk
  GPIO_IDR_ID13_Pos* = (13)
  GPIO_IDR_ID13_Msk* = (0x00000001 shl GPIO_IDR_ID13_Pos) ## !< 0x00002000
  GPIO_IDR_ID13* = GPIO_IDR_ID13_Msk
  GPIO_IDR_ID14_Pos* = (14)
  GPIO_IDR_ID14_Msk* = (0x00000001 shl GPIO_IDR_ID14_Pos) ## !< 0x00004000
  GPIO_IDR_ID14* = GPIO_IDR_ID14_Msk
  GPIO_IDR_ID15_Pos* = (15)
  GPIO_IDR_ID15_Msk* = (0x00000001 shl GPIO_IDR_ID15_Pos) ## !< 0x00008000
  GPIO_IDR_ID15* = GPIO_IDR_ID15_Msk

##  Legacy defines

const
  GPIO_IDR_IDR_Bit0* = GPIO_IDR_ID0
  GPIO_IDR_IDR_Bit1* = GPIO_IDR_ID1
  GPIO_IDR_IDR_Bit2* = GPIO_IDR_ID2
  GPIO_IDR_IDR_Bit3* = GPIO_IDR_ID3
  GPIO_IDR_IDR_Bit4* = GPIO_IDR_ID4
  GPIO_IDR_IDR_Bit5* = GPIO_IDR_ID5
  GPIO_IDR_IDR_Bit6* = GPIO_IDR_ID6
  GPIO_IDR_IDR_Bit7* = GPIO_IDR_ID7
  GPIO_IDR_IDR_Bit8* = GPIO_IDR_ID8
  GPIO_IDR_IDR_Bit9* = GPIO_IDR_ID9
  GPIO_IDR_IDR_Bit10* = GPIO_IDR_ID10
  GPIO_IDR_IDR_Bit11* = GPIO_IDR_ID11
  GPIO_IDR_IDR_Bit12* = GPIO_IDR_ID12
  GPIO_IDR_IDR_Bit13* = GPIO_IDR_ID13
  GPIO_IDR_IDR_Bit14* = GPIO_IDR_ID14
  GPIO_IDR_IDR_Bit15* = GPIO_IDR_ID15

## *****************  Bits definition for GPIO_ODR register  ******************

const
  GPIO_ODR_OD0_Pos* = (0)
  GPIO_ODR_OD0_Msk* = (0x00000001 shl GPIO_ODR_OD0_Pos) ## !< 0x00000001
  GPIO_ODR_OD0* = GPIO_ODR_OD0_Msk
  GPIO_ODR_OD1_Pos* = (1)
  GPIO_ODR_OD1_Msk* = (0x00000001 shl GPIO_ODR_OD1_Pos) ## !< 0x00000002
  GPIO_ODR_OD1* = GPIO_ODR_OD1_Msk
  GPIO_ODR_OD2_Pos* = (2)
  GPIO_ODR_OD2_Msk* = (0x00000001 shl GPIO_ODR_OD2_Pos) ## !< 0x00000004
  GPIO_ODR_OD2* = GPIO_ODR_OD2_Msk
  GPIO_ODR_OD3_Pos* = (3)
  GPIO_ODR_OD3_Msk* = (0x00000001 shl GPIO_ODR_OD3_Pos) ## !< 0x00000008
  GPIO_ODR_OD3* = GPIO_ODR_OD3_Msk
  GPIO_ODR_OD4_Pos* = (4)
  GPIO_ODR_OD4_Msk* = (0x00000001 shl GPIO_ODR_OD4_Pos) ## !< 0x00000010
  GPIO_ODR_OD4* = GPIO_ODR_OD4_Msk
  GPIO_ODR_OD5_Pos* = (5)
  GPIO_ODR_OD5_Msk* = (0x00000001 shl GPIO_ODR_OD5_Pos) ## !< 0x00000020
  GPIO_ODR_OD5* = GPIO_ODR_OD5_Msk
  GPIO_ODR_OD6_Pos* = (6)
  GPIO_ODR_OD6_Msk* = (0x00000001 shl GPIO_ODR_OD6_Pos) ## !< 0x00000040
  GPIO_ODR_OD6* = GPIO_ODR_OD6_Msk
  GPIO_ODR_OD7_Pos* = (7)
  GPIO_ODR_OD7_Msk* = (0x00000001 shl GPIO_ODR_OD7_Pos) ## !< 0x00000080
  GPIO_ODR_OD7* = GPIO_ODR_OD7_Msk
  GPIO_ODR_OD8_Pos* = (8)
  GPIO_ODR_OD8_Msk* = (0x00000001 shl GPIO_ODR_OD8_Pos) ## !< 0x00000100
  GPIO_ODR_OD8* = GPIO_ODR_OD8_Msk
  GPIO_ODR_OD9_Pos* = (9)
  GPIO_ODR_OD9_Msk* = (0x00000001 shl GPIO_ODR_OD9_Pos) ## !< 0x00000200
  GPIO_ODR_OD9* = GPIO_ODR_OD9_Msk
  GPIO_ODR_OD10_Pos* = (10)
  GPIO_ODR_OD10_Msk* = (0x00000001 shl GPIO_ODR_OD10_Pos) ## !< 0x00000400
  GPIO_ODR_OD10* = GPIO_ODR_OD10_Msk
  GPIO_ODR_OD11_Pos* = (11)
  GPIO_ODR_OD11_Msk* = (0x00000001 shl GPIO_ODR_OD11_Pos) ## !< 0x00000800
  GPIO_ODR_OD11* = GPIO_ODR_OD11_Msk
  GPIO_ODR_OD12_Pos* = (12)
  GPIO_ODR_OD12_Msk* = (0x00000001 shl GPIO_ODR_OD12_Pos) ## !< 0x00001000
  GPIO_ODR_OD12* = GPIO_ODR_OD12_Msk
  GPIO_ODR_OD13_Pos* = (13)
  GPIO_ODR_OD13_Msk* = (0x00000001 shl GPIO_ODR_OD13_Pos) ## !< 0x00002000
  GPIO_ODR_OD13* = GPIO_ODR_OD13_Msk
  GPIO_ODR_OD14_Pos* = (14)
  GPIO_ODR_OD14_Msk* = (0x00000001 shl GPIO_ODR_OD14_Pos) ## !< 0x00004000
  GPIO_ODR_OD14* = GPIO_ODR_OD14_Msk
  GPIO_ODR_OD15_Pos* = (15)
  GPIO_ODR_OD15_Msk* = (0x00000001 shl GPIO_ODR_OD15_Pos) ## !< 0x00008000
  GPIO_ODR_OD15* = GPIO_ODR_OD15_Msk

##  Legacy defines

const
  GPIO_ODR_ODR_Bit0* = GPIO_ODR_OD0
  GPIO_ODR_ODR_Bit1* = GPIO_ODR_OD1
  GPIO_ODR_ODR_Bit2* = GPIO_ODR_OD2
  GPIO_ODR_ODR_Bit3* = GPIO_ODR_OD3
  GPIO_ODR_ODR_Bit4* = GPIO_ODR_OD4
  GPIO_ODR_ODR_Bit5* = GPIO_ODR_OD5
  GPIO_ODR_ODR_Bit6* = GPIO_ODR_OD6
  GPIO_ODR_ODR_Bit7* = GPIO_ODR_OD7
  GPIO_ODR_ODR_Bit8* = GPIO_ODR_OD8
  GPIO_ODR_ODR_Bit9* = GPIO_ODR_OD9
  GPIO_ODR_ODR_Bit10* = GPIO_ODR_OD10
  GPIO_ODR_ODR_Bit11* = GPIO_ODR_OD11
  GPIO_ODR_ODR_Bit12* = GPIO_ODR_OD12
  GPIO_ODR_ODR_Bit13* = GPIO_ODR_OD13
  GPIO_ODR_ODR_Bit14* = GPIO_ODR_OD14
  GPIO_ODR_ODR_Bit15* = GPIO_ODR_OD15

## *****************  Bits definition for GPIO_BSRR register  *****************

const
  GPIO_BSRR_BS0_Pos* = (0)
  GPIO_BSRR_BS0_Msk* = (0x00000001 shl GPIO_BSRR_BS0_Pos) ## !< 0x00000001
  GPIO_BSRR_BS0* = GPIO_BSRR_BS0_Msk
  GPIO_BSRR_BS1_Pos* = (1)
  GPIO_BSRR_BS1_Msk* = (0x00000001 shl GPIO_BSRR_BS1_Pos) ## !< 0x00000002
  GPIO_BSRR_BS1* = GPIO_BSRR_BS1_Msk
  GPIO_BSRR_BS2_Pos* = (2)
  GPIO_BSRR_BS2_Msk* = (0x00000001 shl GPIO_BSRR_BS2_Pos) ## !< 0x00000004
  GPIO_BSRR_BS2* = GPIO_BSRR_BS2_Msk
  GPIO_BSRR_BS3_Pos* = (3)
  GPIO_BSRR_BS3_Msk* = (0x00000001 shl GPIO_BSRR_BS3_Pos) ## !< 0x00000008
  GPIO_BSRR_BS3* = GPIO_BSRR_BS3_Msk
  GPIO_BSRR_BS4_Pos* = (4)
  GPIO_BSRR_BS4_Msk* = (0x00000001 shl GPIO_BSRR_BS4_Pos) ## !< 0x00000010
  GPIO_BSRR_BS4* = GPIO_BSRR_BS4_Msk
  GPIO_BSRR_BS5_Pos* = (5)
  GPIO_BSRR_BS5_Msk* = (0x00000001 shl GPIO_BSRR_BS5_Pos) ## !< 0x00000020
  GPIO_BSRR_BS5* = GPIO_BSRR_BS5_Msk
  GPIO_BSRR_BS6_Pos* = (6)
  GPIO_BSRR_BS6_Msk* = (0x00000001 shl GPIO_BSRR_BS6_Pos) ## !< 0x00000040
  GPIO_BSRR_BS6* = GPIO_BSRR_BS6_Msk
  GPIO_BSRR_BS7_Pos* = (7)
  GPIO_BSRR_BS7_Msk* = (0x00000001 shl GPIO_BSRR_BS7_Pos) ## !< 0x00000080
  GPIO_BSRR_BS7* = GPIO_BSRR_BS7_Msk
  GPIO_BSRR_BS8_Pos* = (8)
  GPIO_BSRR_BS8_Msk* = (0x00000001 shl GPIO_BSRR_BS8_Pos) ## !< 0x00000100
  GPIO_BSRR_BS8* = GPIO_BSRR_BS8_Msk
  GPIO_BSRR_BS9_Pos* = (9)
  GPIO_BSRR_BS9_Msk* = (0x00000001 shl GPIO_BSRR_BS9_Pos) ## !< 0x00000200
  GPIO_BSRR_BS9* = GPIO_BSRR_BS9_Msk
  GPIO_BSRR_BS10_Pos* = (10)
  GPIO_BSRR_BS10_Msk* = (0x00000001 shl GPIO_BSRR_BS10_Pos) ## !< 0x00000400
  GPIO_BSRR_BS10* = GPIO_BSRR_BS10_Msk
  GPIO_BSRR_BS11_Pos* = (11)
  GPIO_BSRR_BS11_Msk* = (0x00000001 shl GPIO_BSRR_BS11_Pos) ## !< 0x00000800
  GPIO_BSRR_BS11* = GPIO_BSRR_BS11_Msk
  GPIO_BSRR_BS12_Pos* = (12)
  GPIO_BSRR_BS12_Msk* = (0x00000001 shl GPIO_BSRR_BS12_Pos) ## !< 0x00001000
  GPIO_BSRR_BS12* = GPIO_BSRR_BS12_Msk
  GPIO_BSRR_BS13_Pos* = (13)
  GPIO_BSRR_BS13_Msk* = (0x00000001 shl GPIO_BSRR_BS13_Pos) ## !< 0x00002000
  GPIO_BSRR_BS13* = GPIO_BSRR_BS13_Msk
  GPIO_BSRR_BS14_Pos* = (14)
  GPIO_BSRR_BS14_Msk* = (0x00000001 shl GPIO_BSRR_BS14_Pos) ## !< 0x00004000
  GPIO_BSRR_BS14* = GPIO_BSRR_BS14_Msk
  GPIO_BSRR_BS15_Pos* = (15)
  GPIO_BSRR_BS15_Msk* = (0x00000001 shl GPIO_BSRR_BS15_Pos) ## !< 0x00008000
  GPIO_BSRR_BS15* = GPIO_BSRR_BS15_Msk
  GPIO_BSRR_BR0_Pos* = (16)
  GPIO_BSRR_BR0_Msk* = (0x00000001 shl GPIO_BSRR_BR0_Pos) ## !< 0x00010000
  GPIO_BSRR_BR0* = GPIO_BSRR_BR0_Msk
  GPIO_BSRR_BR1_Pos* = (17)
  GPIO_BSRR_BR1_Msk* = (0x00000001 shl GPIO_BSRR_BR1_Pos) ## !< 0x00020000
  GPIO_BSRR_BR1* = GPIO_BSRR_BR1_Msk
  GPIO_BSRR_BR2_Pos* = (18)
  GPIO_BSRR_BR2_Msk* = (0x00000001 shl GPIO_BSRR_BR2_Pos) ## !< 0x00040000
  GPIO_BSRR_BR2* = GPIO_BSRR_BR2_Msk
  GPIO_BSRR_BR3_Pos* = (19)
  GPIO_BSRR_BR3_Msk* = (0x00000001 shl GPIO_BSRR_BR3_Pos) ## !< 0x00080000
  GPIO_BSRR_BR3* = GPIO_BSRR_BR3_Msk
  GPIO_BSRR_BR4_Pos* = (20)
  GPIO_BSRR_BR4_Msk* = (0x00000001 shl GPIO_BSRR_BR4_Pos) ## !< 0x00100000
  GPIO_BSRR_BR4* = GPIO_BSRR_BR4_Msk
  GPIO_BSRR_BR5_Pos* = (21)
  GPIO_BSRR_BR5_Msk* = (0x00000001 shl GPIO_BSRR_BR5_Pos) ## !< 0x00200000
  GPIO_BSRR_BR5* = GPIO_BSRR_BR5_Msk
  GPIO_BSRR_BR6_Pos* = (22)
  GPIO_BSRR_BR6_Msk* = (0x00000001 shl GPIO_BSRR_BR6_Pos) ## !< 0x00400000
  GPIO_BSRR_BR6* = GPIO_BSRR_BR6_Msk
  GPIO_BSRR_BR7_Pos* = (23)
  GPIO_BSRR_BR7_Msk* = (0x00000001 shl GPIO_BSRR_BR7_Pos) ## !< 0x00800000
  GPIO_BSRR_BR7* = GPIO_BSRR_BR7_Msk
  GPIO_BSRR_BR8_Pos* = (24)
  GPIO_BSRR_BR8_Msk* = (0x00000001 shl GPIO_BSRR_BR8_Pos) ## !< 0x01000000
  GPIO_BSRR_BR8* = GPIO_BSRR_BR8_Msk
  GPIO_BSRR_BR9_Pos* = (25)
  GPIO_BSRR_BR9_Msk* = (0x00000001 shl GPIO_BSRR_BR9_Pos) ## !< 0x02000000
  GPIO_BSRR_BR9* = GPIO_BSRR_BR9_Msk
  GPIO_BSRR_BR10_Pos* = (26)
  GPIO_BSRR_BR10_Msk* = (0x00000001 shl GPIO_BSRR_BR10_Pos) ## !< 0x04000000
  GPIO_BSRR_BR10* = GPIO_BSRR_BR10_Msk
  GPIO_BSRR_BR11_Pos* = (27)
  GPIO_BSRR_BR11_Msk* = (0x00000001 shl GPIO_BSRR_BR11_Pos) ## !< 0x08000000
  GPIO_BSRR_BR11* = GPIO_BSRR_BR11_Msk
  GPIO_BSRR_BR12_Pos* = (28)
  GPIO_BSRR_BR12_Msk* = (0x00000001 shl GPIO_BSRR_BR12_Pos) ## !< 0x10000000
  GPIO_BSRR_BR12* = GPIO_BSRR_BR12_Msk
  GPIO_BSRR_BR13_Pos* = (29)
  GPIO_BSRR_BR13_Msk* = (0x00000001 shl GPIO_BSRR_BR13_Pos) ## !< 0x20000000
  GPIO_BSRR_BR13* = GPIO_BSRR_BR13_Msk
  GPIO_BSRR_BR14_Pos* = (30)
  GPIO_BSRR_BR14_Msk* = (0x00000001 shl GPIO_BSRR_BR14_Pos) ## !< 0x40000000
  GPIO_BSRR_BR14* = GPIO_BSRR_BR14_Msk
  GPIO_BSRR_BR15_Pos* = (31)
  GPIO_BSRR_BR15_Msk* = (0x00000001 shl GPIO_BSRR_BR15_Pos) ## !< 0x80000000
  GPIO_BSRR_BR15* = GPIO_BSRR_BR15_Msk

##  Legacy defines

const
  GPIO_BSRR_BS_Bit0* = GPIO_BSRR_BS0
  GPIO_BSRR_BS_Bit1* = GPIO_BSRR_BS1
  GPIO_BSRR_BS_Bit2* = GPIO_BSRR_BS2
  GPIO_BSRR_BS_Bit3* = GPIO_BSRR_BS3
  GPIO_BSRR_BS_Bit4* = GPIO_BSRR_BS4
  GPIO_BSRR_BS_Bit5* = GPIO_BSRR_BS5
  GPIO_BSRR_BS_Bit6* = GPIO_BSRR_BS6
  GPIO_BSRR_BS_Bit7* = GPIO_BSRR_BS7
  GPIO_BSRR_BS_Bit8* = GPIO_BSRR_BS8
  GPIO_BSRR_BS_Bit9* = GPIO_BSRR_BS9
  GPIO_BSRR_BS_Bit10* = GPIO_BSRR_BS10
  GPIO_BSRR_BS_Bit11* = GPIO_BSRR_BS11
  GPIO_BSRR_BS_Bit12* = GPIO_BSRR_BS12
  GPIO_BSRR_BS_Bit13* = GPIO_BSRR_BS13
  GPIO_BSRR_BS_Bit14* = GPIO_BSRR_BS14
  GPIO_BSRR_BS_Bit15* = GPIO_BSRR_BS15
  GPIO_BSRR_BR_Bit0* = GPIO_BSRR_BR0
  GPIO_BSRR_BR_Bit1* = GPIO_BSRR_BR1
  GPIO_BSRR_BR_Bit2* = GPIO_BSRR_BR2
  GPIO_BSRR_BR_Bit3* = GPIO_BSRR_BR3
  GPIO_BSRR_BR_Bit4* = GPIO_BSRR_BR4
  GPIO_BSRR_BR_Bit5* = GPIO_BSRR_BR5
  GPIO_BSRR_BR_Bit6* = GPIO_BSRR_BR6
  GPIO_BSRR_BR_Bit7* = GPIO_BSRR_BR7
  GPIO_BSRR_BR_Bit8* = GPIO_BSRR_BR8
  GPIO_BSRR_BR_Bit9* = GPIO_BSRR_BR9
  GPIO_BSRR_BR_Bit10* = GPIO_BSRR_BR10
  GPIO_BSRR_BR_Bit11* = GPIO_BSRR_BR11
  GPIO_BSRR_BR_Bit12* = GPIO_BSRR_BR12
  GPIO_BSRR_BR_Bit13* = GPIO_BSRR_BR13
  GPIO_BSRR_BR_Bit14* = GPIO_BSRR_BR14
  GPIO_BSRR_BR_Bit15* = GPIO_BSRR_BR15

## ***************** Bit definition for GPIO_LCKR register ********************

const
  GPIO_LCKR_LCK0_Pos* = (0)
  GPIO_LCKR_LCK0_Msk* = (0x00000001 shl GPIO_LCKR_LCK0_Pos) ## !< 0x00000001
  GPIO_LCKR_LCK0* = GPIO_LCKR_LCK0_Msk
  GPIO_LCKR_LCK1_Pos* = (1)
  GPIO_LCKR_LCK1_Msk* = (0x00000001 shl GPIO_LCKR_LCK1_Pos) ## !< 0x00000002
  GPIO_LCKR_LCK1* = GPIO_LCKR_LCK1_Msk
  GPIO_LCKR_LCK2_Pos* = (2)
  GPIO_LCKR_LCK2_Msk* = (0x00000001 shl GPIO_LCKR_LCK2_Pos) ## !< 0x00000004
  GPIO_LCKR_LCK2* = GPIO_LCKR_LCK2_Msk
  GPIO_LCKR_LCK3_Pos* = (3)
  GPIO_LCKR_LCK3_Msk* = (0x00000001 shl GPIO_LCKR_LCK3_Pos) ## !< 0x00000008
  GPIO_LCKR_LCK3* = GPIO_LCKR_LCK3_Msk
  GPIO_LCKR_LCK4_Pos* = (4)
  GPIO_LCKR_LCK4_Msk* = (0x00000001 shl GPIO_LCKR_LCK4_Pos) ## !< 0x00000010
  GPIO_LCKR_LCK4* = GPIO_LCKR_LCK4_Msk
  GPIO_LCKR_LCK5_Pos* = (5)
  GPIO_LCKR_LCK5_Msk* = (0x00000001 shl GPIO_LCKR_LCK5_Pos) ## !< 0x00000020
  GPIO_LCKR_LCK5* = GPIO_LCKR_LCK5_Msk
  GPIO_LCKR_LCK6_Pos* = (6)
  GPIO_LCKR_LCK6_Msk* = (0x00000001 shl GPIO_LCKR_LCK6_Pos) ## !< 0x00000040
  GPIO_LCKR_LCK6* = GPIO_LCKR_LCK6_Msk
  GPIO_LCKR_LCK7_Pos* = (7)
  GPIO_LCKR_LCK7_Msk* = (0x00000001 shl GPIO_LCKR_LCK7_Pos) ## !< 0x00000080
  GPIO_LCKR_LCK7* = GPIO_LCKR_LCK7_Msk
  GPIO_LCKR_LCK8_Pos* = (8)
  GPIO_LCKR_LCK8_Msk* = (0x00000001 shl GPIO_LCKR_LCK8_Pos) ## !< 0x00000100
  GPIO_LCKR_LCK8* = GPIO_LCKR_LCK8_Msk
  GPIO_LCKR_LCK9_Pos* = (9)
  GPIO_LCKR_LCK9_Msk* = (0x00000001 shl GPIO_LCKR_LCK9_Pos) ## !< 0x00000200
  GPIO_LCKR_LCK9* = GPIO_LCKR_LCK9_Msk
  GPIO_LCKR_LCK10_Pos* = (10)
  GPIO_LCKR_LCK10_Msk* = (0x00000001 shl GPIO_LCKR_LCK10_Pos) ## !< 0x00000400
  GPIO_LCKR_LCK10* = GPIO_LCKR_LCK10_Msk
  GPIO_LCKR_LCK11_Pos* = (11)
  GPIO_LCKR_LCK11_Msk* = (0x00000001 shl GPIO_LCKR_LCK11_Pos) ## !< 0x00000800
  GPIO_LCKR_LCK11* = GPIO_LCKR_LCK11_Msk
  GPIO_LCKR_LCK12_Pos* = (12)
  GPIO_LCKR_LCK12_Msk* = (0x00000001 shl GPIO_LCKR_LCK12_Pos) ## !< 0x00001000
  GPIO_LCKR_LCK12* = GPIO_LCKR_LCK12_Msk
  GPIO_LCKR_LCK13_Pos* = (13)
  GPIO_LCKR_LCK13_Msk* = (0x00000001 shl GPIO_LCKR_LCK13_Pos) ## !< 0x00002000
  GPIO_LCKR_LCK13* = GPIO_LCKR_LCK13_Msk
  GPIO_LCKR_LCK14_Pos* = (14)
  GPIO_LCKR_LCK14_Msk* = (0x00000001 shl GPIO_LCKR_LCK14_Pos) ## !< 0x00004000
  GPIO_LCKR_LCK14* = GPIO_LCKR_LCK14_Msk
  GPIO_LCKR_LCK15_Pos* = (15)
  GPIO_LCKR_LCK15_Msk* = (0x00000001 shl GPIO_LCKR_LCK15_Pos) ## !< 0x00008000
  GPIO_LCKR_LCK15* = GPIO_LCKR_LCK15_Msk
  GPIO_LCKR_LCKK_Pos* = (16)
  GPIO_LCKR_LCKK_Msk* = (0x00000001 shl GPIO_LCKR_LCKK_Pos) ## !< 0x00010000
  GPIO_LCKR_LCKK* = GPIO_LCKR_LCKK_Msk

## ***************** Bit definition for GPIO_AFRL register ********************

const
  GPIO_AFRL_AFSEL0_Pos* = (0)
  GPIO_AFRL_AFSEL0_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL0_Pos) ## !< 0x0000000F
  GPIO_AFRL_AFSEL0* = GPIO_AFRL_AFSEL0_Msk
  GPIO_AFRL_AFSEL0_Bit0* = (0x00000001 shl GPIO_AFRL_AFSEL0_Pos) ## !< 0x00000001
  GPIO_AFRL_AFSEL0_Bit1* = (0x00000002 shl GPIO_AFRL_AFSEL0_Pos) ## !< 0x00000002
  GPIO_AFRL_AFSEL0_Bit2* = (0x00000004 shl GPIO_AFRL_AFSEL0_Pos) ## !< 0x00000004
  GPIO_AFRL_AFSEL0_Bit3* = (0x00000008 shl GPIO_AFRL_AFSEL0_Pos) ## !< 0x00000008
  GPIO_AFRL_AFSEL1_Pos* = (4)
  GPIO_AFRL_AFSEL1_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL1_Pos) ## !< 0x000000F0
  GPIO_AFRL_AFSEL1* = GPIO_AFRL_AFSEL1_Msk
  GPIO_AFRL_AFSEL1_Bit0* = (0x00000001 shl GPIO_AFRL_AFSEL1_Pos) ## !< 0x00000010
  GPIO_AFRL_AFSEL1_Bit1* = (0x00000002 shl GPIO_AFRL_AFSEL1_Pos) ## !< 0x00000020
  GPIO_AFRL_AFSEL1_Bit2* = (0x00000004 shl GPIO_AFRL_AFSEL1_Pos) ## !< 0x00000040
  GPIO_AFRL_AFSEL1_Bit3* = (0x00000008 shl GPIO_AFRL_AFSEL1_Pos) ## !< 0x00000080
  GPIO_AFRL_AFSEL2_Pos* = (8)
  GPIO_AFRL_AFSEL2_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL2_Pos) ## !< 0x00000F00
  GPIO_AFRL_AFSEL2* = GPIO_AFRL_AFSEL2_Msk
  GPIO_AFRL_AFSEL2_Bit0* = (0x00000001 shl GPIO_AFRL_AFSEL2_Pos) ## !< 0x00000100
  GPIO_AFRL_AFSEL2_Bit1* = (0x00000002 shl GPIO_AFRL_AFSEL2_Pos) ## !< 0x00000200
  GPIO_AFRL_AFSEL2_Bit2* = (0x00000004 shl GPIO_AFRL_AFSEL2_Pos) ## !< 0x00000400
  GPIO_AFRL_AFSEL2_Bit3* = (0x00000008 shl GPIO_AFRL_AFSEL2_Pos) ## !< 0x00000800
  GPIO_AFRL_AFSEL3_Pos* = (12)
  GPIO_AFRL_AFSEL3_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL3_Pos) ## !< 0x0000F000
  GPIO_AFRL_AFSEL3* = GPIO_AFRL_AFSEL3_Msk
  GPIO_AFRL_AFSEL3_Bit0* = (0x00000001 shl GPIO_AFRL_AFSEL3_Pos) ## !< 0x00001000
  GPIO_AFRL_AFSEL3_Bit1* = (0x00000002 shl GPIO_AFRL_AFSEL3_Pos) ## !< 0x00002000
  GPIO_AFRL_AFSEL3_Bit2* = (0x00000004 shl GPIO_AFRL_AFSEL3_Pos) ## !< 0x00004000
  GPIO_AFRL_AFSEL3_Bit3* = (0x00000008 shl GPIO_AFRL_AFSEL3_Pos) ## !< 0x00008000
  GPIO_AFRL_AFSEL4_Pos* = (16)
  GPIO_AFRL_AFSEL4_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL4_Pos) ## !< 0x000F0000
  GPIO_AFRL_AFSEL4* = GPIO_AFRL_AFSEL4_Msk
  GPIO_AFRL_AFSEL4_Bit0* = (0x00000001 shl GPIO_AFRL_AFSEL4_Pos) ## !< 0x00010000
  GPIO_AFRL_AFSEL4_Bit1* = (0x00000002 shl GPIO_AFRL_AFSEL4_Pos) ## !< 0x00020000
  GPIO_AFRL_AFSEL4_Bit2* = (0x00000004 shl GPIO_AFRL_AFSEL4_Pos) ## !< 0x00040000
  GPIO_AFRL_AFSEL4_Bit3* = (0x00000008 shl GPIO_AFRL_AFSEL4_Pos) ## !< 0x00080000
  GPIO_AFRL_AFSEL5_Pos* = (20)
  GPIO_AFRL_AFSEL5_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL5_Pos) ## !< 0x00F00000
  GPIO_AFRL_AFSEL5* = GPIO_AFRL_AFSEL5_Msk
  GPIO_AFRL_AFSEL5_Bit0* = (0x00000001 shl GPIO_AFRL_AFSEL5_Pos) ## !< 0x00100000
  GPIO_AFRL_AFSEL5_Bit1* = (0x00000002 shl GPIO_AFRL_AFSEL5_Pos) ## !< 0x00200000
  GPIO_AFRL_AFSEL5_Bit2* = (0x00000004 shl GPIO_AFRL_AFSEL5_Pos) ## !< 0x00400000
  GPIO_AFRL_AFSEL5_Bit3* = (0x00000008 shl GPIO_AFRL_AFSEL5_Pos) ## !< 0x00800000
  GPIO_AFRL_AFSEL6_Pos* = (24)
  GPIO_AFRL_AFSEL6_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL6_Pos) ## !< 0x0F000000
  GPIO_AFRL_AFSEL6* = GPIO_AFRL_AFSEL6_Msk
  GPIO_AFRL_AFSEL6_Bit0* = (0x00000001 shl GPIO_AFRL_AFSEL6_Pos) ## !< 0x01000000
  GPIO_AFRL_AFSEL6_Bit1* = (0x00000002 shl GPIO_AFRL_AFSEL6_Pos) ## !< 0x02000000
  GPIO_AFRL_AFSEL6_Bit2* = (0x00000004 shl GPIO_AFRL_AFSEL6_Pos) ## !< 0x04000000
  GPIO_AFRL_AFSEL6_Bit3* = (0x00000008 shl GPIO_AFRL_AFSEL6_Pos) ## !< 0x08000000
  GPIO_AFRL_AFSEL7_Pos* = (28)
  GPIO_AFRL_AFSEL7_Msk* = (0x0000000F shl GPIO_AFRL_AFSEL7_Pos) ## !< 0xF0000000
  GPIO_AFRL_AFSEL7* = GPIO_AFRL_AFSEL7_Msk
  GPIO_AFRL_AFSEL7_Bit0* = (0x00000001 shl GPIO_AFRL_AFSEL7_Pos) ## !< 0x10000000
  GPIO_AFRL_AFSEL7_Bit1* = (0x00000002 shl GPIO_AFRL_AFSEL7_Pos) ## !< 0x20000000
  GPIO_AFRL_AFSEL7_Bit2* = (0x00000004 shl GPIO_AFRL_AFSEL7_Pos) ## !< 0x40000000
  GPIO_AFRL_AFSEL7_Bit3* = (0x00000008 shl GPIO_AFRL_AFSEL7_Pos) ## !< 0x80000000

##  Legacy defines

const
  GPIO_AFRL_AFRL0* = GPIO_AFRL_AFSEL0
  GPIO_AFRL_AFRL0_Bit0* = GPIO_AFRL_AFSEL0_Bit0
  GPIO_AFRL_AFRL0_Bit1* = GPIO_AFRL_AFSEL0_Bit1
  GPIO_AFRL_AFRL0_Bit2* = GPIO_AFRL_AFSEL0_Bit2
  GPIO_AFRL_AFRL0_Bit3* = GPIO_AFRL_AFSEL0_Bit3
  GPIO_AFRL_AFRL1* = GPIO_AFRL_AFSEL1
  GPIO_AFRL_AFRL1_Bit0* = GPIO_AFRL_AFSEL1_Bit0
  GPIO_AFRL_AFRL1_Bit1* = GPIO_AFRL_AFSEL1_Bit1
  GPIO_AFRL_AFRL1_Bit2* = GPIO_AFRL_AFSEL1_Bit2
  GPIO_AFRL_AFRL1_Bit3* = GPIO_AFRL_AFSEL1_Bit3
  GPIO_AFRL_AFRL2* = GPIO_AFRL_AFSEL2
  GPIO_AFRL_AFRL2_Bit0* = GPIO_AFRL_AFSEL2_Bit0
  GPIO_AFRL_AFRL2_Bit1* = GPIO_AFRL_AFSEL2_Bit1
  GPIO_AFRL_AFRL2_Bit2* = GPIO_AFRL_AFSEL2_Bit2
  GPIO_AFRL_AFRL2_Bit3* = GPIO_AFRL_AFSEL2_Bit3
  GPIO_AFRL_AFRL3* = GPIO_AFRL_AFSEL3
  GPIO_AFRL_AFRL3_Bit0* = GPIO_AFRL_AFSEL3_Bit0
  GPIO_AFRL_AFRL3_Bit1* = GPIO_AFRL_AFSEL3_Bit1
  GPIO_AFRL_AFRL3_Bit2* = GPIO_AFRL_AFSEL3_Bit2
  GPIO_AFRL_AFRL3_Bit3* = GPIO_AFRL_AFSEL3_Bit3
  GPIO_AFRL_AFRL4* = GPIO_AFRL_AFSEL4
  GPIO_AFRL_AFRL4_Bit0* = GPIO_AFRL_AFSEL4_Bit0
  GPIO_AFRL_AFRL4_Bit1* = GPIO_AFRL_AFSEL4_Bit1
  GPIO_AFRL_AFRL4_Bit2* = GPIO_AFRL_AFSEL4_Bit2
  GPIO_AFRL_AFRL4_Bit3* = GPIO_AFRL_AFSEL4_Bit3
  GPIO_AFRL_AFRL5* = GPIO_AFRL_AFSEL5
  GPIO_AFRL_AFRL5_Bit0* = GPIO_AFRL_AFSEL5_Bit0
  GPIO_AFRL_AFRL5_Bit1* = GPIO_AFRL_AFSEL5_Bit1
  GPIO_AFRL_AFRL5_Bit2* = GPIO_AFRL_AFSEL5_Bit2
  GPIO_AFRL_AFRL5_Bit3* = GPIO_AFRL_AFSEL5_Bit3
  GPIO_AFRL_AFRL6* = GPIO_AFRL_AFSEL6
  GPIO_AFRL_AFRL6_Bit0* = GPIO_AFRL_AFSEL6_Bit0
  GPIO_AFRL_AFRL6_Bit1* = GPIO_AFRL_AFSEL6_Bit1
  GPIO_AFRL_AFRL6_Bit2* = GPIO_AFRL_AFSEL6_Bit2
  GPIO_AFRL_AFRL6_Bit3* = GPIO_AFRL_AFSEL6_Bit3
  GPIO_AFRL_AFRL7* = GPIO_AFRL_AFSEL7
  GPIO_AFRL_AFRL7_Bit0* = GPIO_AFRL_AFSEL7_Bit0
  GPIO_AFRL_AFRL7_Bit1* = GPIO_AFRL_AFSEL7_Bit1
  GPIO_AFRL_AFRL7_Bit2* = GPIO_AFRL_AFSEL7_Bit2
  GPIO_AFRL_AFRL7_Bit3* = GPIO_AFRL_AFSEL7_Bit3

## ***************** Bit definition for GPIO_AFRH register ********************

const
  GPIO_AFRH_AFSEL8_Pos* = (0)
  GPIO_AFRH_AFSEL8_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL8_Pos) ## !< 0x0000000F
  GPIO_AFRH_AFSEL8* = GPIO_AFRH_AFSEL8_Msk
  GPIO_AFRH_AFSEL8_Bit0* = (0x00000001 shl GPIO_AFRH_AFSEL8_Pos) ## !< 0x00000001
  GPIO_AFRH_AFSEL8_Bit1* = (0x00000002 shl GPIO_AFRH_AFSEL8_Pos) ## !< 0x00000002
  GPIO_AFRH_AFSEL8_Bit2* = (0x00000004 shl GPIO_AFRH_AFSEL8_Pos) ## !< 0x00000004
  GPIO_AFRH_AFSEL8_Bit3* = (0x00000008 shl GPIO_AFRH_AFSEL8_Pos) ## !< 0x00000008
  GPIO_AFRH_AFSEL9_Pos* = (4)
  GPIO_AFRH_AFSEL9_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL9_Pos) ## !< 0x000000F0
  GPIO_AFRH_AFSEL9* = GPIO_AFRH_AFSEL9_Msk
  GPIO_AFRH_AFSEL9_Bit0* = (0x00000001 shl GPIO_AFRH_AFSEL9_Pos) ## !< 0x00000010
  GPIO_AFRH_AFSEL9_Bit1* = (0x00000002 shl GPIO_AFRH_AFSEL9_Pos) ## !< 0x00000020
  GPIO_AFRH_AFSEL9_Bit2* = (0x00000004 shl GPIO_AFRH_AFSEL9_Pos) ## !< 0x00000040
  GPIO_AFRH_AFSEL9_Bit3* = (0x00000008 shl GPIO_AFRH_AFSEL9_Pos) ## !< 0x00000080
  GPIO_AFRH_AFSEL10_Pos* = (8)
  GPIO_AFRH_AFSEL10_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL10_Pos) ## !< 0x00000F00
  GPIO_AFRH_AFSEL10* = GPIO_AFRH_AFSEL10_Msk
  GPIO_AFRH_AFSEL10_Bit0* = (0x00000001 shl GPIO_AFRH_AFSEL10_Pos) ## !< 0x00000100
  GPIO_AFRH_AFSEL10_Bit1* = (0x00000002 shl GPIO_AFRH_AFSEL10_Pos) ## !< 0x00000200
  GPIO_AFRH_AFSEL10_Bit2* = (0x00000004 shl GPIO_AFRH_AFSEL10_Pos) ## !< 0x00000400
  GPIO_AFRH_AFSEL10_Bit3* = (0x00000008 shl GPIO_AFRH_AFSEL10_Pos) ## !< 0x00000800
  GPIO_AFRH_AFSEL11_Pos* = (12)
  GPIO_AFRH_AFSEL11_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL11_Pos) ## !< 0x0000F000
  GPIO_AFRH_AFSEL11* = GPIO_AFRH_AFSEL11_Msk
  GPIO_AFRH_AFSEL11_Bit0* = (0x00000001 shl GPIO_AFRH_AFSEL11_Pos) ## !< 0x00001000
  GPIO_AFRH_AFSEL11_Bit1* = (0x00000002 shl GPIO_AFRH_AFSEL11_Pos) ## !< 0x00002000
  GPIO_AFRH_AFSEL11_Bit2* = (0x00000004 shl GPIO_AFRH_AFSEL11_Pos) ## !< 0x00004000
  GPIO_AFRH_AFSEL11_Bit3* = (0x00000008 shl GPIO_AFRH_AFSEL11_Pos) ## !< 0x00008000
  GPIO_AFRH_AFSEL12_Pos* = (16)
  GPIO_AFRH_AFSEL12_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL12_Pos) ## !< 0x000F0000
  GPIO_AFRH_AFSEL12* = GPIO_AFRH_AFSEL12_Msk
  GPIO_AFRH_AFSEL12_Bit0* = (0x00000001 shl GPIO_AFRH_AFSEL12_Pos) ## !< 0x00010000
  GPIO_AFRH_AFSEL12_Bit1* = (0x00000002 shl GPIO_AFRH_AFSEL12_Pos) ## !< 0x00020000
  GPIO_AFRH_AFSEL12_Bit2* = (0x00000004 shl GPIO_AFRH_AFSEL12_Pos) ## !< 0x00040000
  GPIO_AFRH_AFSEL12_Bit3* = (0x00000008 shl GPIO_AFRH_AFSEL12_Pos) ## !< 0x00080000
  GPIO_AFRH_AFSEL13_Pos* = (20)
  GPIO_AFRH_AFSEL13_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL13_Pos) ## !< 0x00F00000
  GPIO_AFRH_AFSEL13* = GPIO_AFRH_AFSEL13_Msk
  GPIO_AFRH_AFSEL13_Bit0* = (0x00000001 shl GPIO_AFRH_AFSEL13_Pos) ## !< 0x00100000
  GPIO_AFRH_AFSEL13_Bit1* = (0x00000002 shl GPIO_AFRH_AFSEL13_Pos) ## !< 0x00200000
  GPIO_AFRH_AFSEL13_Bit2* = (0x00000004 shl GPIO_AFRH_AFSEL13_Pos) ## !< 0x00400000
  GPIO_AFRH_AFSEL13_Bit3* = (0x00000008 shl GPIO_AFRH_AFSEL13_Pos) ## !< 0x00800000
  GPIO_AFRH_AFSEL14_Pos* = (24)
  GPIO_AFRH_AFSEL14_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL14_Pos) ## !< 0x0F000000
  GPIO_AFRH_AFSEL14* = GPIO_AFRH_AFSEL14_Msk
  GPIO_AFRH_AFSEL14_Bit0* = (0x00000001 shl GPIO_AFRH_AFSEL14_Pos) ## !< 0x01000000
  GPIO_AFRH_AFSEL14_Bit1* = (0x00000002 shl GPIO_AFRH_AFSEL14_Pos) ## !< 0x02000000
  GPIO_AFRH_AFSEL14_Bit2* = (0x00000004 shl GPIO_AFRH_AFSEL14_Pos) ## !< 0x04000000
  GPIO_AFRH_AFSEL14_Bit3* = (0x00000008 shl GPIO_AFRH_AFSEL14_Pos) ## !< 0x08000000
  GPIO_AFRH_AFSEL15_Pos* = (28)
  GPIO_AFRH_AFSEL15_Msk* = (0x0000000F shl GPIO_AFRH_AFSEL15_Pos) ## !< 0xF0000000
  GPIO_AFRH_AFSEL15* = GPIO_AFRH_AFSEL15_Msk
  GPIO_AFRH_AFSEL15_Bit0* = (0x00000001 shl GPIO_AFRH_AFSEL15_Pos) ## !< 0x10000000
  GPIO_AFRH_AFSEL15_Bit1* = (0x00000002 shl GPIO_AFRH_AFSEL15_Pos) ## !< 0x20000000
  GPIO_AFRH_AFSEL15_Bit2* = (0x00000004 shl GPIO_AFRH_AFSEL15_Pos) ## !< 0x40000000
  GPIO_AFRH_AFSEL15_Bit3* = (0x00000008 shl GPIO_AFRH_AFSEL15_Pos) ## !< 0x80000000

##  Legacy defines

const
  GPIO_AFRH_AFRH0* = GPIO_AFRH_AFSEL8
  GPIO_AFRH_AFRH0_Bit0* = GPIO_AFRH_AFSEL8_Bit0
  GPIO_AFRH_AFRH0_Bit1* = GPIO_AFRH_AFSEL8_Bit1
  GPIO_AFRH_AFRH0_Bit2* = GPIO_AFRH_AFSEL8_Bit2
  GPIO_AFRH_AFRH0_Bit3* = GPIO_AFRH_AFSEL8_Bit3
  GPIO_AFRH_AFRH1* = GPIO_AFRH_AFSEL9
  GPIO_AFRH_AFRH1_Bit0* = GPIO_AFRH_AFSEL9_Bit0
  GPIO_AFRH_AFRH1_Bit1* = GPIO_AFRH_AFSEL9_Bit1
  GPIO_AFRH_AFRH1_Bit2* = GPIO_AFRH_AFSEL9_Bit2
  GPIO_AFRH_AFRH1_Bit3* = GPIO_AFRH_AFSEL9_Bit3
  GPIO_AFRH_AFRH2* = GPIO_AFRH_AFSEL10
  GPIO_AFRH_AFRH2_Bit0* = GPIO_AFRH_AFSEL10_Bit0
  GPIO_AFRH_AFRH2_Bit1* = GPIO_AFRH_AFSEL10_Bit1
  GPIO_AFRH_AFRH2_Bit2* = GPIO_AFRH_AFSEL10_Bit2
  GPIO_AFRH_AFRH2_Bit3* = GPIO_AFRH_AFSEL10_Bit3
  GPIO_AFRH_AFRH3* = GPIO_AFRH_AFSEL11
  GPIO_AFRH_AFRH3_Bit0* = GPIO_AFRH_AFSEL11_Bit0
  GPIO_AFRH_AFRH3_Bit1* = GPIO_AFRH_AFSEL11_Bit1
  GPIO_AFRH_AFRH3_Bit2* = GPIO_AFRH_AFSEL11_Bit2
  GPIO_AFRH_AFRH3_Bit3* = GPIO_AFRH_AFSEL11_Bit3
  GPIO_AFRH_AFRH4* = GPIO_AFRH_AFSEL12
  GPIO_AFRH_AFRH4_Bit0* = GPIO_AFRH_AFSEL12_Bit0
  GPIO_AFRH_AFRH4_Bit1* = GPIO_AFRH_AFSEL12_Bit1
  GPIO_AFRH_AFRH4_Bit2* = GPIO_AFRH_AFSEL12_Bit2
  GPIO_AFRH_AFRH4_Bit3* = GPIO_AFRH_AFSEL12_Bit3
  GPIO_AFRH_AFRH5* = GPIO_AFRH_AFSEL13
  GPIO_AFRH_AFRH5_Bit0* = GPIO_AFRH_AFSEL13_Bit0
  GPIO_AFRH_AFRH5_Bit1* = GPIO_AFRH_AFSEL13_Bit1
  GPIO_AFRH_AFRH5_Bit2* = GPIO_AFRH_AFSEL13_Bit2
  GPIO_AFRH_AFRH5_Bit3* = GPIO_AFRH_AFSEL13_Bit3
  GPIO_AFRH_AFRH6* = GPIO_AFRH_AFSEL14
  GPIO_AFRH_AFRH6_Bit0* = GPIO_AFRH_AFSEL14_Bit0
  GPIO_AFRH_AFRH6_Bit1* = GPIO_AFRH_AFSEL14_Bit1
  GPIO_AFRH_AFRH6_Bit2* = GPIO_AFRH_AFSEL14_Bit2
  GPIO_AFRH_AFRH6_Bit3* = GPIO_AFRH_AFSEL14_Bit3
  GPIO_AFRH_AFRH7* = GPIO_AFRH_AFSEL15
  GPIO_AFRH_AFRH7_Bit0* = GPIO_AFRH_AFSEL15_Bit0
  GPIO_AFRH_AFRH7_Bit1* = GPIO_AFRH_AFSEL15_Bit1
  GPIO_AFRH_AFRH7_Bit2* = GPIO_AFRH_AFSEL15_Bit2
  GPIO_AFRH_AFRH7_Bit3* = GPIO_AFRH_AFSEL15_Bit3

## *****************  Bits definition for GPIO_BRR register  *****************

const
  GPIO_BRR_BR0_Pos* = (0)
  GPIO_BRR_BR0_Msk* = (0x00000001 shl GPIO_BRR_BR0_Pos) ## !< 0x00000001
  GPIO_BRR_BR0* = GPIO_BRR_BR0_Msk
  GPIO_BRR_BR1_Pos* = (1)
  GPIO_BRR_BR1_Msk* = (0x00000001 shl GPIO_BRR_BR1_Pos) ## !< 0x00000002
  GPIO_BRR_BR1* = GPIO_BRR_BR1_Msk
  GPIO_BRR_BR2_Pos* = (2)
  GPIO_BRR_BR2_Msk* = (0x00000001 shl GPIO_BRR_BR2_Pos) ## !< 0x00000004
  GPIO_BRR_BR2* = GPIO_BRR_BR2_Msk
  GPIO_BRR_BR3_Pos* = (3)
  GPIO_BRR_BR3_Msk* = (0x00000001 shl GPIO_BRR_BR3_Pos) ## !< 0x00000008
  GPIO_BRR_BR3* = GPIO_BRR_BR3_Msk
  GPIO_BRR_BR4_Pos* = (4)
  GPIO_BRR_BR4_Msk* = (0x00000001 shl GPIO_BRR_BR4_Pos) ## !< 0x00000010
  GPIO_BRR_BR4* = GPIO_BRR_BR4_Msk
  GPIO_BRR_BR5_Pos* = (5)
  GPIO_BRR_BR5_Msk* = (0x00000001 shl GPIO_BRR_BR5_Pos) ## !< 0x00000020
  GPIO_BRR_BR5* = GPIO_BRR_BR5_Msk
  GPIO_BRR_BR6_Pos* = (6)
  GPIO_BRR_BR6_Msk* = (0x00000001 shl GPIO_BRR_BR6_Pos) ## !< 0x00000040
  GPIO_BRR_BR6* = GPIO_BRR_BR6_Msk
  GPIO_BRR_BR7_Pos* = (7)
  GPIO_BRR_BR7_Msk* = (0x00000001 shl GPIO_BRR_BR7_Pos) ## !< 0x00000080
  GPIO_BRR_BR7* = GPIO_BRR_BR7_Msk
  GPIO_BRR_BR8_Pos* = (8)
  GPIO_BRR_BR8_Msk* = (0x00000001 shl GPIO_BRR_BR8_Pos) ## !< 0x00000100
  GPIO_BRR_BR8* = GPIO_BRR_BR8_Msk
  GPIO_BRR_BR9_Pos* = (9)
  GPIO_BRR_BR9_Msk* = (0x00000001 shl GPIO_BRR_BR9_Pos) ## !< 0x00000200
  GPIO_BRR_BR9* = GPIO_BRR_BR9_Msk
  GPIO_BRR_BR10_Pos* = (10)
  GPIO_BRR_BR10_Msk* = (0x00000001 shl GPIO_BRR_BR10_Pos) ## !< 0x00000400
  GPIO_BRR_BR10* = GPIO_BRR_BR10_Msk
  GPIO_BRR_BR11_Pos* = (11)
  GPIO_BRR_BR11_Msk* = (0x00000001 shl GPIO_BRR_BR11_Pos) ## !< 0x00000800
  GPIO_BRR_BR11* = GPIO_BRR_BR11_Msk
  GPIO_BRR_BR12_Pos* = (12)
  GPIO_BRR_BR12_Msk* = (0x00000001 shl GPIO_BRR_BR12_Pos) ## !< 0x00001000
  GPIO_BRR_BR12* = GPIO_BRR_BR12_Msk
  GPIO_BRR_BR13_Pos* = (13)
  GPIO_BRR_BR13_Msk* = (0x00000001 shl GPIO_BRR_BR13_Pos) ## !< 0x00002000
  GPIO_BRR_BR13* = GPIO_BRR_BR13_Msk
  GPIO_BRR_BR14_Pos* = (14)
  GPIO_BRR_BR14_Msk* = (0x00000001 shl GPIO_BRR_BR14_Pos) ## !< 0x00004000
  GPIO_BRR_BR14* = GPIO_BRR_BR14_Msk
  GPIO_BRR_BR15_Pos* = (15)
  GPIO_BRR_BR15_Msk* = (0x00000001 shl GPIO_BRR_BR15_Pos) ## !< 0x00008000
  GPIO_BRR_BR15* = GPIO_BRR_BR15_Msk

## ****************************************************************************
##
##                       Inter-integrated Circuit Interface
##
## ****************************************************************************
## ******************  Bit definition for I2C_CR1 register  *******************

const
  I2C_CR1_PE_Pos* = (0)
  I2C_CR1_PE_Msk* = (0x00000001 shl I2C_CR1_PE_Pos) ## !< 0x00000001
  I2C_CR1_PE* = I2C_CR1_PE_Msk
  I2C_CR1_SMBUS_Pos* = (1)
  I2C_CR1_SMBUS_Msk* = (0x00000001 shl I2C_CR1_SMBUS_Pos) ## !< 0x00000002
  I2C_CR1_SMBUS* = I2C_CR1_SMBUS_Msk
  I2C_CR1_SMBTYPE_Pos* = (3)
  I2C_CR1_SMBTYPE_Msk* = (0x00000001 shl I2C_CR1_SMBTYPE_Pos) ## !< 0x00000008
  I2C_CR1_SMBTYPE* = I2C_CR1_SMBTYPE_Msk
  I2C_CR1_ENARP_Pos* = (4)
  I2C_CR1_ENARP_Msk* = (0x00000001 shl I2C_CR1_ENARP_Pos) ## !< 0x00000010
  I2C_CR1_ENARP* = I2C_CR1_ENARP_Msk
  I2C_CR1_ENPEC_Pos* = (5)
  I2C_CR1_ENPEC_Msk* = (0x00000001 shl I2C_CR1_ENPEC_Pos) ## !< 0x00000020
  I2C_CR1_ENPEC* = I2C_CR1_ENPEC_Msk
  I2C_CR1_ENGC_Pos* = (6)
  I2C_CR1_ENGC_Msk* = (0x00000001 shl I2C_CR1_ENGC_Pos) ## !< 0x00000040
  I2C_CR1_ENGC* = I2C_CR1_ENGC_Msk
  I2C_CR1_NOSTRETCH_Pos* = (7)
  I2C_CR1_NOSTRETCH_Msk* = (0x00000001 shl I2C_CR1_NOSTRETCH_Pos) ## !< 0x00000080
  I2C_CR1_NOSTRETCH* = I2C_CR1_NOSTRETCH_Msk
  I2C_CR1_START_Pos* = (8)
  I2C_CR1_START_Msk* = (0x00000001 shl I2C_CR1_START_Pos) ## !< 0x00000100
  I2C_CR1_START* = I2C_CR1_START_Msk
  I2C_CR1_STOP_Pos* = (9)
  I2C_CR1_STOP_Msk* = (0x00000001 shl I2C_CR1_STOP_Pos) ## !< 0x00000200
  I2C_CR1_STOP* = I2C_CR1_STOP_Msk
  I2C_CR1_ACK_Pos* = (10)
  I2C_CR1_ACK_Msk* = (0x00000001 shl I2C_CR1_ACK_Pos) ## !< 0x00000400
  I2C_CR1_ACK* = I2C_CR1_ACK_Msk
  I2C_CR1_POS_Pos* = (11)
  I2C_CR1_POS_Msk* = (0x00000001 shl I2C_CR1_POS_Pos) ## !< 0x00000800
  I2C_CR1_POS* = I2C_CR1_POS_Msk
  I2C_CR1_PEC_Pos* = (12)
  I2C_CR1_PEC_Msk* = (0x00000001 shl I2C_CR1_PEC_Pos) ## !< 0x00001000
  I2C_CR1_PEC* = I2C_CR1_PEC_Msk
  I2C_CR1_ALERT_Pos* = (13)
  I2C_CR1_ALERT_Msk* = (0x00000001 shl I2C_CR1_ALERT_Pos) ## !< 0x00002000
  I2C_CR1_ALERT* = I2C_CR1_ALERT_Msk
  I2C_CR1_SWRST_Pos* = (15)
  I2C_CR1_SWRST_Msk* = (0x00000001 shl I2C_CR1_SWRST_Pos) ## !< 0x00008000
  I2C_CR1_SWRST* = I2C_CR1_SWRST_Msk

## ******************  Bit definition for I2C_CR2 register  *******************

const
  I2C_CR2_FREQ_Pos* = (0)
  I2C_CR2_FREQ_Msk* = (0x0000003F shl I2C_CR2_FREQ_Pos) ## !< 0x0000003F
  I2C_CR2_FREQ* = I2C_CR2_FREQ_Msk
  I2C_CR2_FREQ_Bit0* = (0x00000001 shl I2C_CR2_FREQ_Pos) ## !< 0x00000001
  I2C_CR2_FREQ_Bit1* = (0x00000002 shl I2C_CR2_FREQ_Pos) ## !< 0x00000002
  I2C_CR2_FREQ_Bit2* = (0x00000004 shl I2C_CR2_FREQ_Pos) ## !< 0x00000004
  I2C_CR2_FREQ_Bit3* = (0x00000008 shl I2C_CR2_FREQ_Pos) ## !< 0x00000008
  I2C_CR2_FREQ_Bit4* = (0x00000010 shl I2C_CR2_FREQ_Pos) ## !< 0x00000010
  I2C_CR2_FREQ_Bit5* = (0x00000020 shl I2C_CR2_FREQ_Pos) ## !< 0x00000020
  I2C_CR2_ITERREN_Pos* = (8)
  I2C_CR2_ITERREN_Msk* = (0x00000001 shl I2C_CR2_ITERREN_Pos) ## !< 0x00000100
  I2C_CR2_ITERREN* = I2C_CR2_ITERREN_Msk
  I2C_CR2_ITEVTEN_Pos* = (9)
  I2C_CR2_ITEVTEN_Msk* = (0x00000001 shl I2C_CR2_ITEVTEN_Pos) ## !< 0x00000200
  I2C_CR2_ITEVTEN* = I2C_CR2_ITEVTEN_Msk
  I2C_CR2_ITBUFEN_Pos* = (10)
  I2C_CR2_ITBUFEN_Msk* = (0x00000001 shl I2C_CR2_ITBUFEN_Pos) ## !< 0x00000400
  I2C_CR2_ITBUFEN* = I2C_CR2_ITBUFEN_Msk
  I2C_CR2_DMAEN_Pos* = (11)
  I2C_CR2_DMAEN_Msk* = (0x00000001 shl I2C_CR2_DMAEN_Pos) ## !< 0x00000800
  I2C_CR2_DMAEN* = I2C_CR2_DMAEN_Msk
  I2C_CR2_LAST_Pos* = (12)
  I2C_CR2_LAST_Msk* = (0x00000001 shl I2C_CR2_LAST_Pos) ## !< 0x00001000
  I2C_CR2_LAST* = I2C_CR2_LAST_Msk

## ******************  Bit definition for I2C_OAR1 register  ******************

const
  I2C_OAR1_ADD1_Bit7* = 0x000000FE
  I2C_OAR1_ADD8_Bit9* = 0x00000300
  I2C_OAR1_ADD0_Pos* = (0)
  I2C_OAR1_ADD0_Msk* = (0x00000001 shl I2C_OAR1_ADD0_Pos) ## !< 0x00000001
  I2C_OAR1_ADD0* = I2C_OAR1_ADD0_Msk
  I2C_OAR1_ADD1_Pos* = (1)
  I2C_OAR1_ADD1_Msk* = (0x00000001 shl I2C_OAR1_ADD1_Pos) ## !< 0x00000002
  I2C_OAR1_ADD1* = I2C_OAR1_ADD1_Msk
  I2C_OAR1_ADD2_Pos* = (2)
  I2C_OAR1_ADD2_Msk* = (0x00000001 shl I2C_OAR1_ADD2_Pos) ## !< 0x00000004
  I2C_OAR1_ADD2* = I2C_OAR1_ADD2_Msk
  I2C_OAR1_ADD3_Pos* = (3)
  I2C_OAR1_ADD3_Msk* = (0x00000001 shl I2C_OAR1_ADD3_Pos) ## !< 0x00000008
  I2C_OAR1_ADD3* = I2C_OAR1_ADD3_Msk
  I2C_OAR1_ADD4_Pos* = (4)
  I2C_OAR1_ADD4_Msk* = (0x00000001 shl I2C_OAR1_ADD4_Pos) ## !< 0x00000010
  I2C_OAR1_ADD4* = I2C_OAR1_ADD4_Msk
  I2C_OAR1_ADD5_Pos* = (5)
  I2C_OAR1_ADD5_Msk* = (0x00000001 shl I2C_OAR1_ADD5_Pos) ## !< 0x00000020
  I2C_OAR1_ADD5* = I2C_OAR1_ADD5_Msk
  I2C_OAR1_ADD6_Pos* = (6)
  I2C_OAR1_ADD6_Msk* = (0x00000001 shl I2C_OAR1_ADD6_Pos) ## !< 0x00000040
  I2C_OAR1_ADD6* = I2C_OAR1_ADD6_Msk
  I2C_OAR1_ADD7_Pos* = (7)
  I2C_OAR1_ADD7_Msk* = (0x00000001 shl I2C_OAR1_ADD7_Pos) ## !< 0x00000080
  I2C_OAR1_ADD7* = I2C_OAR1_ADD7_Msk
  I2C_OAR1_ADD8_Pos* = (8)
  I2C_OAR1_ADD8_Msk* = (0x00000001 shl I2C_OAR1_ADD8_Pos) ## !< 0x00000100
  I2C_OAR1_ADD8* = I2C_OAR1_ADD8_Msk
  I2C_OAR1_ADD9_Pos* = (9)
  I2C_OAR1_ADD9_Msk* = (0x00000001 shl I2C_OAR1_ADD9_Pos) ## !< 0x00000200
  I2C_OAR1_ADD9* = I2C_OAR1_ADD9_Msk
  I2C_OAR1_ADDMODE_Pos* = (15)
  I2C_OAR1_ADDMODE_Msk* = (0x00000001 shl I2C_OAR1_ADDMODE_Pos) ## !< 0x00008000
  I2C_OAR1_ADDMODE* = I2C_OAR1_ADDMODE_Msk

## ******************  Bit definition for I2C_OAR2 register  ******************

const
  I2C_OAR2_ENDUAL_Pos* = (0)
  I2C_OAR2_ENDUAL_Msk* = (0x00000001 shl I2C_OAR2_ENDUAL_Pos) ## !< 0x00000001
  I2C_OAR2_ENDUAL* = I2C_OAR2_ENDUAL_Msk
  I2C_OAR2_ADD2_Pos* = (1)
  I2C_OAR2_ADD2_Msk* = (0x0000007F shl I2C_OAR2_ADD2_Pos) ## !< 0x000000FE
  I2C_OAR2_ADD2* = I2C_OAR2_ADD2_Msk

## *******************  Bit definition for I2C_DR register  *******************

const
  I2C_DR_DR_Pos* = (0)
  I2C_DR_DR_Msk* = (0x000000FF shl I2C_DR_DR_Pos) ## !< 0x000000FF
  I2C_DR_DR* = I2C_DR_DR_Msk

## ******************  Bit definition for I2C_SR1 register  *******************

const
  I2C_SR1_SB_Pos* = (0)
  I2C_SR1_SB_Msk* = (0x00000001 shl I2C_SR1_SB_Pos) ## !< 0x00000001
  I2C_SR1_SB* = I2C_SR1_SB_Msk
  I2C_SR1_ADDR_Pos* = (1)
  I2C_SR1_ADDR_Msk* = (0x00000001 shl I2C_SR1_ADDR_Pos) ## !< 0x00000002
  I2C_SR1_ADDR* = I2C_SR1_ADDR_Msk
  I2C_SR1_BTF_Pos* = (2)
  I2C_SR1_BTF_Msk* = (0x00000001 shl I2C_SR1_BTF_Pos) ## !< 0x00000004
  I2C_SR1_BTF* = I2C_SR1_BTF_Msk
  I2C_SR1_ADD10_Pos* = (3)
  I2C_SR1_ADD10_Msk* = (0x00000001 shl I2C_SR1_ADD10_Pos) ## !< 0x00000008
  I2C_SR1_ADD10* = I2C_SR1_ADD10_Msk
  I2C_SR1_STOPF_Pos* = (4)
  I2C_SR1_STOPF_Msk* = (0x00000001 shl I2C_SR1_STOPF_Pos) ## !< 0x00000010
  I2C_SR1_STOPF* = I2C_SR1_STOPF_Msk
  I2C_SR1_RXNE_Pos* = (6)
  I2C_SR1_RXNE_Msk* = (0x00000001 shl I2C_SR1_RXNE_Pos) ## !< 0x00000040
  I2C_SR1_RXNE* = I2C_SR1_RXNE_Msk
  I2C_SR1_TXE_Pos* = (7)
  I2C_SR1_TXE_Msk* = (0x00000001 shl I2C_SR1_TXE_Pos) ## !< 0x00000080
  I2C_SR1_TXE* = I2C_SR1_TXE_Msk
  I2C_SR1_BERR_Pos* = (8)
  I2C_SR1_BERR_Msk* = (0x00000001 shl I2C_SR1_BERR_Pos) ## !< 0x00000100
  I2C_SR1_BERR* = I2C_SR1_BERR_Msk
  I2C_SR1_ARLO_Pos* = (9)
  I2C_SR1_ARLO_Msk* = (0x00000001 shl I2C_SR1_ARLO_Pos) ## !< 0x00000200
  I2C_SR1_ARLO* = I2C_SR1_ARLO_Msk
  I2C_SR1_AF_Pos* = (10)
  I2C_SR1_AF_Msk* = (0x00000001 shl I2C_SR1_AF_Pos) ## !< 0x00000400
  I2C_SR1_AF* = I2C_SR1_AF_Msk
  I2C_SR1_OVR_Pos* = (11)
  I2C_SR1_OVR_Msk* = (0x00000001 shl I2C_SR1_OVR_Pos) ## !< 0x00000800
  I2C_SR1_OVR* = I2C_SR1_OVR_Msk
  I2C_SR1_PECERR_Pos* = (12)
  I2C_SR1_PECERR_Msk* = (0x00000001 shl I2C_SR1_PECERR_Pos) ## !< 0x00001000
  I2C_SR1_PECERR* = I2C_SR1_PECERR_Msk
  I2C_SR1_TIMEOUT_Pos* = (14)
  I2C_SR1_TIMEOUT_Msk* = (0x00000001 shl I2C_SR1_TIMEOUT_Pos) ## !< 0x00004000
  I2C_SR1_TIMEOUT* = I2C_SR1_TIMEOUT_Msk
  I2C_SR1_SMBALERT_Pos* = (15)
  I2C_SR1_SMBALERT_Msk* = (0x00000001 shl I2C_SR1_SMBALERT_Pos) ## !< 0x00008000
  I2C_SR1_SMBALERT* = I2C_SR1_SMBALERT_Msk

## ******************  Bit definition for I2C_SR2 register  *******************

const
  I2C_SR2_MSL_Pos* = (0)
  I2C_SR2_MSL_Msk* = (0x00000001 shl I2C_SR2_MSL_Pos) ## !< 0x00000001
  I2C_SR2_MSL* = I2C_SR2_MSL_Msk
  I2C_SR2_BUSY_Pos* = (1)
  I2C_SR2_BUSY_Msk* = (0x00000001 shl I2C_SR2_BUSY_Pos) ## !< 0x00000002
  I2C_SR2_BUSY* = I2C_SR2_BUSY_Msk
  I2C_SR2_TRA_Pos* = (2)
  I2C_SR2_TRA_Msk* = (0x00000001 shl I2C_SR2_TRA_Pos) ## !< 0x00000004
  I2C_SR2_TRA* = I2C_SR2_TRA_Msk
  I2C_SR2_GENCALL_Pos* = (4)
  I2C_SR2_GENCALL_Msk* = (0x00000001 shl I2C_SR2_GENCALL_Pos) ## !< 0x00000010
  I2C_SR2_GENCALL* = I2C_SR2_GENCALL_Msk
  I2C_SR2_SMBDEFAULT_Pos* = (5)
  I2C_SR2_SMBDEFAULT_Msk* = (0x00000001 shl I2C_SR2_SMBDEFAULT_Pos) ## !< 0x00000020
  I2C_SR2_SMBDEFAULT* = I2C_SR2_SMBDEFAULT_Msk
  I2C_SR2_SMBHOST_Pos* = (6)
  I2C_SR2_SMBHOST_Msk* = (0x00000001 shl I2C_SR2_SMBHOST_Pos) ## !< 0x00000040
  I2C_SR2_SMBHOST* = I2C_SR2_SMBHOST_Msk
  I2C_SR2_DUALF_Pos* = (7)
  I2C_SR2_DUALF_Msk* = (0x00000001 shl I2C_SR2_DUALF_Pos) ## !< 0x00000080
  I2C_SR2_DUALF* = I2C_SR2_DUALF_Msk
  I2C_SR2_PEC_Pos* = (8)
  I2C_SR2_PEC_Msk* = (0x000000FF shl I2C_SR2_PEC_Pos) ## !< 0x0000FF00
  I2C_SR2_PEC* = I2C_SR2_PEC_Msk

## ******************  Bit definition for I2C_CCR register  *******************

const
  I2C_CCR_CCR_Pos* = (0)
  I2C_CCR_CCR_Msk* = (0x00000FFF shl I2C_CCR_CCR_Pos) ## !< 0x00000FFF
  I2C_CCR_CCR* = I2C_CCR_CCR_Msk
  I2C_CCR_DUTY_Pos* = (14)
  I2C_CCR_DUTY_Msk* = (0x00000001 shl I2C_CCR_DUTY_Pos) ## !< 0x00004000
  I2C_CCR_DUTY* = I2C_CCR_DUTY_Msk
  I2C_CCR_FS_Pos* = (15)
  I2C_CCR_FS_Msk* = (0x00000001 shl I2C_CCR_FS_Pos) ## !< 0x00008000
  I2C_CCR_FS* = I2C_CCR_FS_Msk

## *****************  Bit definition for I2C_TRISE register  ******************

const
  I2C_TRISE_TRISE_Pos* = (0)
  I2C_TRISE_TRISE_Msk* = (0x0000003F shl I2C_TRISE_TRISE_Pos) ## !< 0x0000003F
  I2C_TRISE_TRISE* = I2C_TRISE_TRISE_Msk

## *****************  Bit definition for I2C_FLTR register  ******************

const
  I2C_FLTR_DNF_Pos* = (0)
  I2C_FLTR_DNF_Msk* = (0x0000000F shl I2C_FLTR_DNF_Pos) ## !< 0x0000000F
  I2C_FLTR_DNF* = I2C_FLTR_DNF_Msk
  I2C_FLTR_ANOFF_Pos* = (4)
  I2C_FLTR_ANOFF_Msk* = (0x00000001 shl I2C_FLTR_ANOFF_Pos) ## !< 0x00000010
  I2C_FLTR_ANOFF* = I2C_FLTR_ANOFF_Msk

## ****************************************************************************
##
##         Fast Mode Plus Inter-integrated Circuit Interface (I2C)
##
## ****************************************************************************
## ******************  Bit definition for I2C_CR1 register  ******************

const
  FMPI2C_CR1_PE_Pos* = (0)
  FMPI2C_CR1_PE_Msk* = (0x00000001 shl FMPI2C_CR1_PE_Pos) ## !< 0x00000001
  FMPI2C_CR1_PE* = FMPI2C_CR1_PE_Msk
  FMPI2C_CR1_TXIE_Pos* = (1)
  FMPI2C_CR1_TXIE_Msk* = (0x00000001 shl FMPI2C_CR1_TXIE_Pos) ## !< 0x00000002
  FMPI2C_CR1_TXIE* = FMPI2C_CR1_TXIE_Msk
  FMPI2C_CR1_RXIE_Pos* = (2)
  FMPI2C_CR1_RXIE_Msk* = (0x00000001 shl FMPI2C_CR1_RXIE_Pos) ## !< 0x00000004
  FMPI2C_CR1_RXIE* = FMPI2C_CR1_RXIE_Msk
  FMPI2C_CR1_ADDRIE_Pos* = (3)
  FMPI2C_CR1_ADDRIE_Msk* = (0x00000001 shl FMPI2C_CR1_ADDRIE_Pos) ## !< 0x00000008
  FMPI2C_CR1_ADDRIE* = FMPI2C_CR1_ADDRIE_Msk
  FMPI2C_CR1_NACKIE_Pos* = (4)
  FMPI2C_CR1_NACKIE_Msk* = (0x00000001 shl FMPI2C_CR1_NACKIE_Pos) ## !< 0x00000010
  FMPI2C_CR1_NACKIE* = FMPI2C_CR1_NACKIE_Msk
  FMPI2C_CR1_STOPIE_Pos* = (5)
  FMPI2C_CR1_STOPIE_Msk* = (0x00000001 shl FMPI2C_CR1_STOPIE_Pos) ## !< 0x00000020
  FMPI2C_CR1_STOPIE* = FMPI2C_CR1_STOPIE_Msk
  FMPI2C_CR1_TCIE_Pos* = (6)
  FMPI2C_CR1_TCIE_Msk* = (0x00000001 shl FMPI2C_CR1_TCIE_Pos) ## !< 0x00000040
  FMPI2C_CR1_TCIE* = FMPI2C_CR1_TCIE_Msk
  FMPI2C_CR1_ERRIE_Pos* = (7)
  FMPI2C_CR1_ERRIE_Msk* = (0x00000001 shl FMPI2C_CR1_ERRIE_Pos) ## !< 0x00000080
  FMPI2C_CR1_ERRIE* = FMPI2C_CR1_ERRIE_Msk
  FMPI2C_CR1_DFN_Pos* = (8)
  FMPI2C_CR1_DFN_Msk* = (0x0000000F shl FMPI2C_CR1_DFN_Pos) ## !< 0x00000F00
  FMPI2C_CR1_DFN* = FMPI2C_CR1_DFN_Msk
  FMPI2C_CR1_ANFOFF_Pos* = (12)
  FMPI2C_CR1_ANFOFF_Msk* = (0x00000001 shl FMPI2C_CR1_ANFOFF_Pos) ## !< 0x00001000
  FMPI2C_CR1_ANFOFF* = FMPI2C_CR1_ANFOFF_Msk
  FMPI2C_CR1_TXDMAEN_Pos* = (14)
  FMPI2C_CR1_TXDMAEN_Msk* = (0x00000001 shl FMPI2C_CR1_TXDMAEN_Pos) ## !< 0x00004000
  FMPI2C_CR1_TXDMAEN* = FMPI2C_CR1_TXDMAEN_Msk
  FMPI2C_CR1_RXDMAEN_Pos* = (15)
  FMPI2C_CR1_RXDMAEN_Msk* = (0x00000001 shl FMPI2C_CR1_RXDMAEN_Pos) ## !< 0x00008000
  FMPI2C_CR1_RXDMAEN* = FMPI2C_CR1_RXDMAEN_Msk
  FMPI2C_CR1_SBC_Pos* = (16)
  FMPI2C_CR1_SBC_Msk* = (0x00000001 shl FMPI2C_CR1_SBC_Pos) ## !< 0x00010000
  FMPI2C_CR1_SBC* = FMPI2C_CR1_SBC_Msk
  FMPI2C_CR1_NOSTRETCH_Pos* = (17)
  FMPI2C_CR1_NOSTRETCH_Msk* = (0x00000001 shl FMPI2C_CR1_NOSTRETCH_Pos) ## !< 0x00020000
  FMPI2C_CR1_NOSTRETCH* = FMPI2C_CR1_NOSTRETCH_Msk
  FMPI2C_CR1_GCEN_Pos* = (19)
  FMPI2C_CR1_GCEN_Msk* = (0x00000001 shl FMPI2C_CR1_GCEN_Pos) ## !< 0x00080000
  FMPI2C_CR1_GCEN* = FMPI2C_CR1_GCEN_Msk
  FMPI2C_CR1_SMBHEN_Pos* = (20)
  FMPI2C_CR1_SMBHEN_Msk* = (0x00000001 shl FMPI2C_CR1_SMBHEN_Pos) ## !< 0x00100000
  FMPI2C_CR1_SMBHEN* = FMPI2C_CR1_SMBHEN_Msk
  FMPI2C_CR1_SMBDEN_Pos* = (21)
  FMPI2C_CR1_SMBDEN_Msk* = (0x00000001 shl FMPI2C_CR1_SMBDEN_Pos) ## !< 0x00200000
  FMPI2C_CR1_SMBDEN* = FMPI2C_CR1_SMBDEN_Msk
  FMPI2C_CR1_ALERTEN_Pos* = (22)
  FMPI2C_CR1_ALERTEN_Msk* = (0x00000001 shl FMPI2C_CR1_ALERTEN_Pos) ## !< 0x00400000
  FMPI2C_CR1_ALERTEN* = FMPI2C_CR1_ALERTEN_Msk
  FMPI2C_CR1_PECEN_Pos* = (23)
  FMPI2C_CR1_PECEN_Msk* = (0x00000001 shl FMPI2C_CR1_PECEN_Pos) ## !< 0x00800000
  FMPI2C_CR1_PECEN* = FMPI2C_CR1_PECEN_Msk

## *****************  Bit definition for I2C_CR2 register  *******************

const
  FMPI2C_CR2_SADD_Pos* = (0)
  FMPI2C_CR2_SADD_Msk* = (0x000003FF shl FMPI2C_CR2_SADD_Pos) ## !< 0x000003FF
  FMPI2C_CR2_SADD* = FMPI2C_CR2_SADD_Msk
  FMPI2C_CR2_RD_WRN_Pos* = (10)
  FMPI2C_CR2_RD_WRN_Msk* = (0x00000001 shl FMPI2C_CR2_RD_WRN_Pos) ## !< 0x00000400
  FMPI2C_CR2_RD_WRN* = FMPI2C_CR2_RD_WRN_Msk
  FMPI2C_CR2_ADD10_Pos* = (11)
  FMPI2C_CR2_ADD10_Msk* = (0x00000001 shl FMPI2C_CR2_ADD10_Pos) ## !< 0x00000800
  FMPI2C_CR2_ADD10* = FMPI2C_CR2_ADD10_Msk
  FMPI2C_CR2_HEAD10R_Pos* = (12)
  FMPI2C_CR2_HEAD10R_Msk* = (0x00000001 shl FMPI2C_CR2_HEAD10R_Pos) ## !< 0x00001000
  FMPI2C_CR2_HEAD10R* = FMPI2C_CR2_HEAD10R_Msk
  FMPI2C_CR2_START_Pos* = (13)
  FMPI2C_CR2_START_Msk* = (0x00000001 shl FMPI2C_CR2_START_Pos) ## !< 0x00002000
  FMPI2C_CR2_START* = FMPI2C_CR2_START_Msk
  FMPI2C_CR2_STOP_Pos* = (14)
  FMPI2C_CR2_STOP_Msk* = (0x00000001 shl FMPI2C_CR2_STOP_Pos) ## !< 0x00004000
  FMPI2C_CR2_STOP* = FMPI2C_CR2_STOP_Msk
  FMPI2C_CR2_NACK_Pos* = (15)
  FMPI2C_CR2_NACK_Msk* = (0x00000001 shl FMPI2C_CR2_NACK_Pos) ## !< 0x00008000
  FMPI2C_CR2_NACK* = FMPI2C_CR2_NACK_Msk
  FMPI2C_CR2_NBYTES_Pos* = (16)
  FMPI2C_CR2_NBYTES_Msk* = (0x000000FF shl FMPI2C_CR2_NBYTES_Pos) ## !< 0x00FF0000
  FMPI2C_CR2_NBYTES* = FMPI2C_CR2_NBYTES_Msk
  FMPI2C_CR2_RELOAD_Pos* = (24)
  FMPI2C_CR2_RELOAD_Msk* = (0x00000001 shl FMPI2C_CR2_RELOAD_Pos) ## !< 0x01000000
  FMPI2C_CR2_RELOAD* = FMPI2C_CR2_RELOAD_Msk
  FMPI2C_CR2_AUTOEND_Pos* = (25)
  FMPI2C_CR2_AUTOEND_Msk* = (0x00000001 shl FMPI2C_CR2_AUTOEND_Pos) ## !< 0x02000000
  FMPI2C_CR2_AUTOEND* = FMPI2C_CR2_AUTOEND_Msk
  FMPI2C_CR2_PECBYTE_Pos* = (26)
  FMPI2C_CR2_PECBYTE_Msk* = (0x00000001 shl FMPI2C_CR2_PECBYTE_Pos) ## !< 0x04000000
  FMPI2C_CR2_PECBYTE* = FMPI2C_CR2_PECBYTE_Msk

## ******************  Bit definition for I2C_OAR1 register  *****************

const
  FMPI2C_OAR1_OA1_Pos* = (0)
  FMPI2C_OAR1_OA1_Msk* = (0x000003FF shl FMPI2C_OAR1_OA1_Pos) ## !< 0x000003FF
  FMPI2C_OAR1_OA1* = FMPI2C_OAR1_OA1_Msk
  FMPI2C_OAR1_OA1MODE_Pos* = (10)
  FMPI2C_OAR1_OA1MODE_Msk* = (0x00000001 shl FMPI2C_OAR1_OA1MODE_Pos) ## !< 0x00000400
  FMPI2C_OAR1_OA1MODE* = FMPI2C_OAR1_OA1MODE_Msk
  FMPI2C_OAR1_OA1EN_Pos* = (15)
  FMPI2C_OAR1_OA1EN_Msk* = (0x00000001 shl FMPI2C_OAR1_OA1EN_Pos) ## !< 0x00008000
  FMPI2C_OAR1_OA1EN* = FMPI2C_OAR1_OA1EN_Msk

## ******************  Bit definition for I2C_OAR2 register  *****************

const
  FMPI2C_OAR2_OA2_Pos* = (1)
  FMPI2C_OAR2_OA2_Msk* = (0x0000007F shl FMPI2C_OAR2_OA2_Pos) ## !< 0x000000FE
  FMPI2C_OAR2_OA2* = FMPI2C_OAR2_OA2_Msk
  FMPI2C_OAR2_OA2MSK_Pos* = (8)
  FMPI2C_OAR2_OA2MSK_Msk* = (0x00000007 shl FMPI2C_OAR2_OA2MSK_Pos) ## !< 0x00000700
  FMPI2C_OAR2_OA2EN_Pos* = (15)
  FMPI2C_OAR2_OA2EN_Msk* = (0x00000001 shl FMPI2C_OAR2_OA2EN_Pos) ## !< 0x00008000
  FMPI2C_OAR2_OA2EN* = FMPI2C_OAR2_OA2EN_Msk

## ******************  Bit definition for I2C_TIMINGR register ******************

const
  FMPI2C_TIMINGR_SCLL_Pos* = (0)
  FMPI2C_TIMINGR_SCLL_Msk* = (0x000000FF shl FMPI2C_TIMINGR_SCLL_Pos) ## !< 0x000000FF
  FMPI2C_TIMINGR_SCLL* = FMPI2C_TIMINGR_SCLL_Msk
  FMPI2C_TIMINGR_SCLH_Pos* = (8)
  FMPI2C_TIMINGR_SCLH_Msk* = (0x000000FF shl FMPI2C_TIMINGR_SCLH_Pos) ## !< 0x0000FF00
  FMPI2C_TIMINGR_SCLH* = FMPI2C_TIMINGR_SCLH_Msk
  FMPI2C_TIMINGR_SDADEL_Pos* = (16)
  FMPI2C_TIMINGR_SDADEL_Msk* = (0x0000000F shl FMPI2C_TIMINGR_SDADEL_Pos) ## !< 0x000F0000
  FMPI2C_TIMINGR_SDADEL* = FMPI2C_TIMINGR_SDADEL_Msk
  FMPI2C_TIMINGR_SCLDEL_Pos* = (20)
  FMPI2C_TIMINGR_SCLDEL_Msk* = (0x0000000F shl FMPI2C_TIMINGR_SCLDEL_Pos) ## !< 0x00F00000
  FMPI2C_TIMINGR_SCLDEL* = FMPI2C_TIMINGR_SCLDEL_Msk
  FMPI2C_TIMINGR_PRESC_Pos* = (28)
  FMPI2C_TIMINGR_PRESC_Msk* = (0x0000000F shl FMPI2C_TIMINGR_PRESC_Pos) ## !< 0xF0000000
  FMPI2C_TIMINGR_PRESC* = FMPI2C_TIMINGR_PRESC_Msk

## ****************** Bit definition for I2C_TIMEOUTR register ******************

const
  FMPI2C_TIMEOUTR_TIMEOUTA_Pos* = (0)
  FMPI2C_TIMEOUTR_TIMEOUTA_Msk* = (0x00000FFF shl FMPI2C_TIMEOUTR_TIMEOUTA_Pos) ## !< 0x00000FFF
  FMPI2C_TIMEOUTR_TIMEOUTA* = FMPI2C_TIMEOUTR_TIMEOUTA_Msk
  FMPI2C_TIMEOUTR_TIDLE_Pos* = (12)
  FMPI2C_TIMEOUTR_TIDLE_Msk* = (0x00000001 shl FMPI2C_TIMEOUTR_TIDLE_Pos) ## !< 0x00001000
  FMPI2C_TIMEOUTR_TIDLE* = FMPI2C_TIMEOUTR_TIDLE_Msk
  FMPI2C_TIMEOUTR_TIMOUTEN_Pos* = (15)
  FMPI2C_TIMEOUTR_TIMOUTEN_Msk* = (0x00000001 shl FMPI2C_TIMEOUTR_TIMOUTEN_Pos) ## !< 0x00008000
  FMPI2C_TIMEOUTR_TIMOUTEN* = FMPI2C_TIMEOUTR_TIMOUTEN_Msk
  FMPI2C_TIMEOUTR_TIMEOUTB_Pos* = (16)
  FMPI2C_TIMEOUTR_TIMEOUTB_Msk* = (0x00000FFF shl FMPI2C_TIMEOUTR_TIMEOUTB_Pos) ## !< 0x0FFF0000
  FMPI2C_TIMEOUTR_TIMEOUTB* = FMPI2C_TIMEOUTR_TIMEOUTB_Msk
  FMPI2C_TIMEOUTR_TEXTEN_Pos* = (31)
  FMPI2C_TIMEOUTR_TEXTEN_Msk* = (0x00000001 shl FMPI2C_TIMEOUTR_TEXTEN_Pos) ## !< 0x80000000
  FMPI2C_TIMEOUTR_TEXTEN* = FMPI2C_TIMEOUTR_TEXTEN_Msk

## *****************  Bit definition for I2C_ISR register  ********************

const
  FMPI2C_ISR_TXE_Pos* = (0)
  FMPI2C_ISR_TXE_Msk* = (0x00000001 shl FMPI2C_ISR_TXE_Pos) ## !< 0x00000001
  FMPI2C_ISR_TXE* = FMPI2C_ISR_TXE_Msk
  FMPI2C_ISR_TXIS_Pos* = (1)
  FMPI2C_ISR_TXIS_Msk* = (0x00000001 shl FMPI2C_ISR_TXIS_Pos) ## !< 0x00000002
  FMPI2C_ISR_TXIS* = FMPI2C_ISR_TXIS_Msk
  FMPI2C_ISR_RXNE_Pos* = (2)
  FMPI2C_ISR_RXNE_Msk* = (0x00000001 shl FMPI2C_ISR_RXNE_Pos) ## !< 0x00000004
  FMPI2C_ISR_RXNE* = FMPI2C_ISR_RXNE_Msk
  FMPI2C_ISR_ADDR_Pos* = (3)
  FMPI2C_ISR_ADDR_Msk* = (0x00000001 shl FMPI2C_ISR_ADDR_Pos) ## !< 0x00000008
  FMPI2C_ISR_ADDR* = FMPI2C_ISR_ADDR_Msk
  FMPI2C_ISR_NACKF_Pos* = (4)
  FMPI2C_ISR_NACKF_Msk* = (0x00000001 shl FMPI2C_ISR_NACKF_Pos) ## !< 0x00000010
  FMPI2C_ISR_NACKF* = FMPI2C_ISR_NACKF_Msk
  FMPI2C_ISR_STOPF_Pos* = (5)
  FMPI2C_ISR_STOPF_Msk* = (0x00000001 shl FMPI2C_ISR_STOPF_Pos) ## !< 0x00000020
  FMPI2C_ISR_STOPF* = FMPI2C_ISR_STOPF_Msk
  FMPI2C_ISR_TC_Pos* = (6)
  FMPI2C_ISR_TC_Msk* = (0x00000001 shl FMPI2C_ISR_TC_Pos) ## !< 0x00000040
  FMPI2C_ISR_TC* = FMPI2C_ISR_TC_Msk
  FMPI2C_ISR_TCR_Pos* = (7)
  FMPI2C_ISR_TCR_Msk* = (0x00000001 shl FMPI2C_ISR_TCR_Pos) ## !< 0x00000080
  FMPI2C_ISR_TCR* = FMPI2C_ISR_TCR_Msk
  FMPI2C_ISR_BERR_Pos* = (8)
  FMPI2C_ISR_BERR_Msk* = (0x00000001 shl FMPI2C_ISR_BERR_Pos) ## !< 0x00000100
  FMPI2C_ISR_BERR* = FMPI2C_ISR_BERR_Msk
  FMPI2C_ISR_ARLO_Pos* = (9)
  FMPI2C_ISR_ARLO_Msk* = (0x00000001 shl FMPI2C_ISR_ARLO_Pos) ## !< 0x00000200
  FMPI2C_ISR_ARLO* = FMPI2C_ISR_ARLO_Msk
  FMPI2C_ISR_OVR_Pos* = (10)
  FMPI2C_ISR_OVR_Msk* = (0x00000001 shl FMPI2C_ISR_OVR_Pos) ## !< 0x00000400
  FMPI2C_ISR_OVR* = FMPI2C_ISR_OVR_Msk
  FMPI2C_ISR_PECERR_Pos* = (11)
  FMPI2C_ISR_PECERR_Msk* = (0x00000001 shl FMPI2C_ISR_PECERR_Pos) ## !< 0x00000800
  FMPI2C_ISR_PECERR* = FMPI2C_ISR_PECERR_Msk
  FMPI2C_ISR_TIMEOUT_Pos* = (12)
  FMPI2C_ISR_TIMEOUT_Msk* = (0x00000001 shl FMPI2C_ISR_TIMEOUT_Pos) ## !< 0x00001000
  FMPI2C_ISR_TIMEOUT* = FMPI2C_ISR_TIMEOUT_Msk
  FMPI2C_ISR_ALERT_Pos* = (13)
  FMPI2C_ISR_ALERT_Msk* = (0x00000001 shl FMPI2C_ISR_ALERT_Pos) ## !< 0x00002000
  FMPI2C_ISR_ALERT* = FMPI2C_ISR_ALERT_Msk
  FMPI2C_ISR_BUSY_Pos* = (15)
  FMPI2C_ISR_BUSY_Msk* = (0x00000001 shl FMPI2C_ISR_BUSY_Pos) ## !< 0x00008000
  FMPI2C_ISR_BUSY* = FMPI2C_ISR_BUSY_Msk
  FMPI2C_ISR_DIR_Pos* = (16)
  FMPI2C_ISR_DIR_Msk* = (0x00000001 shl FMPI2C_ISR_DIR_Pos) ## !< 0x00010000
  FMPI2C_ISR_DIR* = FMPI2C_ISR_DIR_Msk
  FMPI2C_ISR_ADDCODE_Pos* = (17)
  FMPI2C_ISR_ADDCODE_Msk* = (0x0000007F shl FMPI2C_ISR_ADDCODE_Pos) ## !< 0x00FE0000
  FMPI2C_ISR_ADDCODE* = FMPI2C_ISR_ADDCODE_Msk

## *****************  Bit definition for I2C_ICR register  ********************

const
  FMPI2C_ICR_ADDRCF_Pos* = (3)
  FMPI2C_ICR_ADDRCF_Msk* = (0x00000001 shl FMPI2C_ICR_ADDRCF_Pos) ## !< 0x00000008
  FMPI2C_ICR_ADDRCF* = FMPI2C_ICR_ADDRCF_Msk
  FMPI2C_ICR_NACKCF_Pos* = (4)
  FMPI2C_ICR_NACKCF_Msk* = (0x00000001 shl FMPI2C_ICR_NACKCF_Pos) ## !< 0x00000010
  FMPI2C_ICR_NACKCF* = FMPI2C_ICR_NACKCF_Msk
  FMPI2C_ICR_STOPCF_Pos* = (5)
  FMPI2C_ICR_STOPCF_Msk* = (0x00000001 shl FMPI2C_ICR_STOPCF_Pos) ## !< 0x00000020
  FMPI2C_ICR_STOPCF* = FMPI2C_ICR_STOPCF_Msk
  FMPI2C_ICR_BERRCF_Pos* = (8)
  FMPI2C_ICR_BERRCF_Msk* = (0x00000001 shl FMPI2C_ICR_BERRCF_Pos) ## !< 0x00000100
  FMPI2C_ICR_BERRCF* = FMPI2C_ICR_BERRCF_Msk
  FMPI2C_ICR_ARLOCF_Pos* = (9)
  FMPI2C_ICR_ARLOCF_Msk* = (0x00000001 shl FMPI2C_ICR_ARLOCF_Pos) ## !< 0x00000200
  FMPI2C_ICR_ARLOCF* = FMPI2C_ICR_ARLOCF_Msk
  FMPI2C_ICR_OVRCF_Pos* = (10)
  FMPI2C_ICR_OVRCF_Msk* = (0x00000001 shl FMPI2C_ICR_OVRCF_Pos) ## !< 0x00000400
  FMPI2C_ICR_OVRCF* = FMPI2C_ICR_OVRCF_Msk
  FMPI2C_ICR_PECCF_Pos* = (11)
  FMPI2C_ICR_PECCF_Msk* = (0x00000001 shl FMPI2C_ICR_PECCF_Pos) ## !< 0x00000800
  FMPI2C_ICR_PECCF* = FMPI2C_ICR_PECCF_Msk
  FMPI2C_ICR_TIMOUTCF_Pos* = (12)
  FMPI2C_ICR_TIMOUTCF_Msk* = (0x00000001 shl FMPI2C_ICR_TIMOUTCF_Pos) ## !< 0x00001000
  FMPI2C_ICR_TIMOUTCF* = FMPI2C_ICR_TIMOUTCF_Msk
  FMPI2C_ICR_ALERTCF_Pos* = (13)
  FMPI2C_ICR_ALERTCF_Msk* = (0x00000001 shl FMPI2C_ICR_ALERTCF_Pos) ## !< 0x00002000
  FMPI2C_ICR_ALERTCF* = FMPI2C_ICR_ALERTCF_Msk

## *****************  Bit definition for I2C_PECR register  ********************

const
  FMPI2C_PECR_PEC_Pos* = (0)
  FMPI2C_PECR_PEC_Msk* = (0x000000FF shl FMPI2C_PECR_PEC_Pos) ## !< 0x000000FF
  FMPI2C_PECR_PEC* = FMPI2C_PECR_PEC_Msk

## *****************  Bit definition for I2C_RXDR register  ********************

const
  FMPI2C_RXDR_RXDATA_Pos* = (0)
  FMPI2C_RXDR_RXDATA_Msk* = (0x000000FF shl FMPI2C_RXDR_RXDATA_Pos) ## !< 0x000000FF
  FMPI2C_RXDR_RXDATA* = FMPI2C_RXDR_RXDATA_Msk

## *****************  Bit definition for I2C_TXDR register  ********************

const
  FMPI2C_TXDR_TXDATA_Pos* = (0)
  FMPI2C_TXDR_TXDATA_Msk* = (0x000000FF shl FMPI2C_TXDR_TXDATA_Pos) ## !< 0x000000FF
  FMPI2C_TXDR_TXDATA* = FMPI2C_TXDR_TXDATA_Msk

## ****************************************************************************
##
##                            Independent WATCHDOG
##
## ****************************************************************************
## ******************  Bit definition for IWDG_KR register  *******************

const
  IWDG_KR_KEY_Pos* = (0)
  IWDG_KR_KEY_Msk* = (0x0000FFFF shl IWDG_KR_KEY_Pos) ## !< 0x0000FFFF
  IWDG_KR_KEY* = IWDG_KR_KEY_Msk

## ******************  Bit definition for IWDG_PR register  *******************

const
  IWDG_PR_PR_Pos* = (0)
  IWDG_PR_PR_Msk* = (0x00000007 shl IWDG_PR_PR_Pos) ## !< 0x00000007
  IWDG_PR_PR* = IWDG_PR_PR_Msk
  IWDG_PR_PR_Bit0* = (0x00000001 shl IWDG_PR_PR_Pos) ## !< 0x01
  IWDG_PR_PR_Bit1* = (0x00000002 shl IWDG_PR_PR_Pos) ## !< 0x02
  IWDG_PR_PR_Bit2* = (0x00000004 shl IWDG_PR_PR_Pos) ## !< 0x04

## ******************  Bit definition for IWDG_RLR register  ******************

const
  IWDG_RLR_RL_Pos* = (0)
  IWDG_RLR_RL_Msk* = (0x00000FFF shl IWDG_RLR_RL_Pos) ## !< 0x00000FFF
  IWDG_RLR_RL* = IWDG_RLR_RL_Msk

## ******************  Bit definition for IWDG_SR register  *******************

const
  IWDG_SR_PVU_Pos* = (0)
  IWDG_SR_PVU_Msk* = (0x00000001 shl IWDG_SR_PVU_Pos) ## !< 0x00000001
  IWDG_SR_PVU* = IWDG_SR_PVU_Msk
  IWDG_SR_RVU_Pos* = (1)
  IWDG_SR_RVU_Msk* = (0x00000001 shl IWDG_SR_RVU_Pos) ## !< 0x00000002
  IWDG_SR_RVU* = IWDG_SR_RVU_Msk

## ****************************************************************************
##
##                              Power Control
##
## ****************************************************************************
## *******************  Bit definition for PWR_CR register  *******************

const
  PWR_CR_LPDS_Pos* = (0)
  PWR_CR_LPDS_Msk* = (0x00000001 shl PWR_CR_LPDS_Pos) ## !< 0x00000001
  PWR_CR_LPDS* = PWR_CR_LPDS_Msk
  PWR_CR_PDDS_Pos* = (1)
  PWR_CR_PDDS_Msk* = (0x00000001 shl PWR_CR_PDDS_Pos) ## !< 0x00000002
  PWR_CR_PDDS* = PWR_CR_PDDS_Msk
  PWR_CR_CWUF_Pos* = (2)
  PWR_CR_CWUF_Msk* = (0x00000001 shl PWR_CR_CWUF_Pos) ## !< 0x00000004
  PWR_CR_CWUF* = PWR_CR_CWUF_Msk
  PWR_CR_CSBF_Pos* = (3)
  PWR_CR_CSBF_Msk* = (0x00000001 shl PWR_CR_CSBF_Pos) ## !< 0x00000008
  PWR_CR_CSBF* = PWR_CR_CSBF_Msk
  PWR_CR_PVDE_Pos* = (4)
  PWR_CR_PVDE_Msk* = (0x00000001 shl PWR_CR_PVDE_Pos) ## !< 0x00000010
  PWR_CR_PVDE* = PWR_CR_PVDE_Msk
  PWR_CR_PLS_Pos* = (5)
  PWR_CR_PLS_Msk* = (0x00000007 shl PWR_CR_PLS_Pos) ## !< 0x000000E0
  PWR_CR_PLS* = PWR_CR_PLS_Msk
  PWR_CR_PLS_Bit0* = (0x00000001 shl PWR_CR_PLS_Pos) ## !< 0x00000020
  PWR_CR_PLS_Bit1* = (0x00000002 shl PWR_CR_PLS_Pos) ## !< 0x00000040
  PWR_CR_PLS_Bit2* = (0x00000004 shl PWR_CR_PLS_Pos) ## !< 0x00000080

## !< PVD level configuration

const
  PWR_CR_PLS_LEV0* = 0x00000000
  PWR_CR_PLS_LEV1* = 0x00000020
  PWR_CR_PLS_LEV2* = 0x00000040
  PWR_CR_PLS_LEV3* = 0x00000060
  PWR_CR_PLS_LEV4* = 0x00000080
  PWR_CR_PLS_LEV5* = 0x000000A0
  PWR_CR_PLS_LEV6* = 0x000000C0
  PWR_CR_PLS_LEV7* = 0x000000E0
  PWR_CR_DBP_Pos* = (8)
  PWR_CR_DBP_Msk* = (0x00000001 shl PWR_CR_DBP_Pos) ## !< 0x00000100
  PWR_CR_DBP* = PWR_CR_DBP_Msk
  PWR_CR_FPDS_Pos* = (9)
  PWR_CR_FPDS_Msk* = (0x00000001 shl PWR_CR_FPDS_Pos) ## !< 0x00000200
  PWR_CR_FPDS* = PWR_CR_FPDS_Msk
  PWR_CR_LPLVDS_Pos* = (10)
  PWR_CR_LPLVDS_Msk* = (0x00000001 shl PWR_CR_LPLVDS_Pos) ## !< 0x00000400
  PWR_CR_LPLVDS* = PWR_CR_LPLVDS_Msk
  PWR_CR_MRLVDS_Pos* = (11)
  PWR_CR_MRLVDS_Msk* = (0x00000001 shl PWR_CR_MRLVDS_Pos) ## !< 0x00000800
  PWR_CR_MRLVDS* = PWR_CR_MRLVDS_Msk
  PWR_CR_ADCDC1_Pos* = (13)
  PWR_CR_ADCDC1_Msk* = (0x00000001 shl PWR_CR_ADCDC1_Pos) ## !< 0x00002000
  PWR_CR_ADCDC1* = PWR_CR_ADCDC1_Msk
  PWR_CR_VOS_Pos* = (14)
  PWR_CR_VOS_Msk* = (0x00000003 shl PWR_CR_VOS_Pos) ## !< 0x0000C000
  PWR_CR_VOS* = PWR_CR_VOS_Msk
  PWR_CR_VOS_Bit0* = 0x00004000
  PWR_CR_VOS_Bit1* = 0x00008000
  PWR_CR_FMSSR_Pos* = (20)
  PWR_CR_FMSSR_Msk* = (0x00000001 shl PWR_CR_FMSSR_Pos) ## !< 0x00100000
  PWR_CR_FMSSR* = PWR_CR_FMSSR_Msk
  PWR_CR_FISSR_Pos* = (21)
  PWR_CR_FISSR_Msk* = (0x00000001 shl PWR_CR_FISSR_Pos) ## !< 0x00200000
  PWR_CR_FISSR* = PWR_CR_FISSR_Msk

##  Legacy define

const
  PWR_CR_PMODE* = PWR_CR_VOS

## ******************  Bit definition for PWR_CSR register  *******************

const
  PWR_CSR_WUF_Pos* = (0)
  PWR_CSR_WUF_Msk* = (0x00000001 shl PWR_CSR_WUF_Pos) ## !< 0x00000001
  PWR_CSR_WUF* = PWR_CSR_WUF_Msk
  PWR_CSR_SBF_Pos* = (1)
  PWR_CSR_SBF_Msk* = (0x00000001 shl PWR_CSR_SBF_Pos) ## !< 0x00000002
  PWR_CSR_SBF* = PWR_CSR_SBF_Msk
  PWR_CSR_PVDO_Pos* = (2)
  PWR_CSR_PVDO_Msk* = (0x00000001 shl PWR_CSR_PVDO_Pos) ## !< 0x00000004
  PWR_CSR_PVDO* = PWR_CSR_PVDO_Msk
  PWR_CSR_BRR_Pos* = (3)
  PWR_CSR_BRR_Msk* = (0x00000001 shl PWR_CSR_BRR_Pos) ## !< 0x00000008
  PWR_CSR_BRR* = PWR_CSR_BRR_Msk
  PWR_CSR_EWUP3_Pos* = (6)
  PWR_CSR_EWUP3_Msk* = (0x00000001 shl PWR_CSR_EWUP1_Pos) ## !< 0x00000040
  PWR_CSR_EWUP3* = PWR_CSR_EWUP3_Msk
  PWR_CSR_EWUP2_Pos* = (7)
  PWR_CSR_EWUP2_Msk* = (0x00000001 shl PWR_CSR_EWUP2_Pos) ## !< 0x00000080
  PWR_CSR_EWUP2* = PWR_CSR_EWUP2_Msk
  PWR_CSR_EWUP1_Pos* = (8)
  PWR_CSR_EWUP1_Msk* = (0x00000001 shl PWR_CSR_EWUP1_Pos) ## !< 0x00000100
  PWR_CSR_EWUP1* = PWR_CSR_EWUP1_Msk
  PWR_CSR_BRE_Pos* = (9)
  PWR_CSR_BRE_Msk* = (0x00000001 shl PWR_CSR_BRE_Pos) ## !< 0x00000200
  PWR_CSR_BRE* = PWR_CSR_BRE_Msk
  PWR_CSR_VOSRDY_Pos* = (14)
  PWR_CSR_VOSRDY_Msk* = (0x00000001 shl PWR_CSR_VOSRDY_Pos) ## !< 0x00004000
  PWR_CSR_VOSRDY* = PWR_CSR_VOSRDY_Msk

##  Legacy define

const
  PWR_CSR_REGRDY* = PWR_CSR_VOSRDY

## ****************************************************************************
##
##                          Reset and Clock Control
##
## ****************************************************************************
## *******************  Bit definition for RCC_CR register  *******************

const
  RCC_CR_HSION_Pos* = (0)
  RCC_CR_HSION_Msk* = (0x00000001 shl RCC_CR_HSION_Pos) ## !< 0x00000001
  RCC_CR_HSION* = RCC_CR_HSION_Msk
  RCC_CR_HSIRDY_Pos* = (1)
  RCC_CR_HSIRDY_Msk* = (0x00000001 shl RCC_CR_HSIRDY_Pos) ## !< 0x00000002
  RCC_CR_HSIRDY* = RCC_CR_HSIRDY_Msk
  RCC_CR_HSITRIM_Pos* = (3)
  RCC_CR_HSITRIM_Msk* = (0x0000001F shl RCC_CR_HSITRIM_Pos) ## !< 0x000000F8
  RCC_CR_HSITRIM* = RCC_CR_HSITRIM_Msk
  RCC_CR_HSITRIM_Bit0* = (0x00000001 shl RCC_CR_HSITRIM_Pos) ## !< 0x00000008
  RCC_CR_HSITRIM_Bit1* = (0x00000002 shl RCC_CR_HSITRIM_Pos) ## !< 0x00000010
  RCC_CR_HSITRIM_Bit2* = (0x00000004 shl RCC_CR_HSITRIM_Pos) ## !< 0x00000020
  RCC_CR_HSITRIM_Bit3* = (0x00000008 shl RCC_CR_HSITRIM_Pos) ## !< 0x00000040
  RCC_CR_HSITRIM_Bit4* = (0x00000010 shl RCC_CR_HSITRIM_Pos) ## !< 0x00000080
  RCC_CR_HSICAL_Pos* = (8)
  RCC_CR_HSICAL_Msk* = (0x000000FF shl RCC_CR_HSICAL_Pos) ## !< 0x0000FF00
  RCC_CR_HSICAL* = RCC_CR_HSICAL_Msk
  RCC_CR_HSICAL_Bit0* = (0x00000001 shl RCC_CR_HSICAL_Pos) ## !< 0x00000100
  RCC_CR_HSICAL_Bit1* = (0x00000002 shl RCC_CR_HSICAL_Pos) ## !< 0x00000200
  RCC_CR_HSICAL_Bit2* = (0x00000004 shl RCC_CR_HSICAL_Pos) ## !< 0x00000400
  RCC_CR_HSICAL_Bit3* = (0x00000008 shl RCC_CR_HSICAL_Pos) ## !< 0x00000800
  RCC_CR_HSICAL_Bit4* = (0x00000010 shl RCC_CR_HSICAL_Pos) ## !< 0x00001000
  RCC_CR_HSICAL_Bit5* = (0x00000020 shl RCC_CR_HSICAL_Pos) ## !< 0x00002000
  RCC_CR_HSICAL_Bit6* = (0x00000040 shl RCC_CR_HSICAL_Pos) ## !< 0x00004000
  RCC_CR_HSICAL_Bit7* = (0x00000080 shl RCC_CR_HSICAL_Pos) ## !< 0x00008000
  RCC_CR_HSEON_Pos* = (16)
  RCC_CR_HSEON_Msk* = (0x00000001 shl RCC_CR_HSEON_Pos) ## !< 0x00010000
  RCC_CR_HSEON* = RCC_CR_HSEON_Msk
  RCC_CR_HSERDY_Pos* = (17)
  RCC_CR_HSERDY_Msk* = (0x00000001 shl RCC_CR_HSERDY_Pos) ## !< 0x00020000
  RCC_CR_HSERDY* = RCC_CR_HSERDY_Msk
  RCC_CR_HSEBYP_Pos* = (18)
  RCC_CR_HSEBYP_Msk* = (0x00000001 shl RCC_CR_HSEBYP_Pos) ## !< 0x00040000
  RCC_CR_HSEBYP* = RCC_CR_HSEBYP_Msk
  RCC_CR_CSSON_Pos* = (19)
  RCC_CR_CSSON_Msk* = (0x00000001 shl RCC_CR_CSSON_Pos) ## !< 0x00080000
  RCC_CR_CSSON* = RCC_CR_CSSON_Msk
  RCC_CR_PLLON_Pos* = (24)
  RCC_CR_PLLON_Msk* = (0x00000001 shl RCC_CR_PLLON_Pos) ## !< 0x01000000
  RCC_CR_PLLON* = RCC_CR_PLLON_Msk
  RCC_CR_PLLRDY_Pos* = (25)
  RCC_CR_PLLRDY_Msk* = (0x00000001 shl RCC_CR_PLLRDY_Pos) ## !< 0x02000000
  RCC_CR_PLLRDY* = RCC_CR_PLLRDY_Msk

## *******************  Bit definition for RCC_PLLCFGR register  **************

const
  RCC_PLLCFGR_PLLM_Pos* = (0)
  RCC_PLLCFGR_PLLM_Msk* = (0x0000003F shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x0000003F
  RCC_PLLCFGR_PLLM* = RCC_PLLCFGR_PLLM_Msk
  RCC_PLLCFGR_PLLM_Bit0* = (0x00000001 shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x00000001
  RCC_PLLCFGR_PLLM_Bit1* = (0x00000002 shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x00000002
  RCC_PLLCFGR_PLLM_Bit2* = (0x00000004 shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x00000004
  RCC_PLLCFGR_PLLM_Bit3* = (0x00000008 shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x00000008
  RCC_PLLCFGR_PLLM_Bit4* = (0x00000010 shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x00000010
  RCC_PLLCFGR_PLLM_Bit5* = (0x00000020 shl RCC_PLLCFGR_PLLM_Pos) ## !< 0x00000020
  RCC_PLLCFGR_PLLN_Pos* = (6)
  RCC_PLLCFGR_PLLN_Msk* = (0x000001FF shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00007FC0
  RCC_PLLCFGR_PLLN* = RCC_PLLCFGR_PLLN_Msk
  RCC_PLLCFGR_PLLN_Bit0* = (0x00000001 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00000040
  RCC_PLLCFGR_PLLN_Bit1* = (0x00000002 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00000080
  RCC_PLLCFGR_PLLN_Bit2* = (0x00000004 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00000100
  RCC_PLLCFGR_PLLN_Bit3* = (0x00000008 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00000200
  RCC_PLLCFGR_PLLN_Bit4* = (0x00000010 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00000400
  RCC_PLLCFGR_PLLN_Bit5* = (0x00000020 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00000800
  RCC_PLLCFGR_PLLN_Bit6* = (0x00000040 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00001000
  RCC_PLLCFGR_PLLN_Bit7* = (0x00000080 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00002000
  RCC_PLLCFGR_PLLN_Bit8* = (0x00000100 shl RCC_PLLCFGR_PLLN_Pos) ## !< 0x00004000
  RCC_PLLCFGR_PLLP_Pos* = (16)
  RCC_PLLCFGR_PLLP_Msk* = (0x00000003 shl RCC_PLLCFGR_PLLP_Pos) ## !< 0x00030000
  RCC_PLLCFGR_PLLP* = RCC_PLLCFGR_PLLP_Msk
  RCC_PLLCFGR_PLLP_Bit0* = (0x00000001 shl RCC_PLLCFGR_PLLP_Pos) ## !< 0x00010000
  RCC_PLLCFGR_PLLP_Bit1* = (0x00000002 shl RCC_PLLCFGR_PLLP_Pos) ## !< 0x00020000
  RCC_PLLCFGR_PLLSRC_Pos* = (22)
  RCC_PLLCFGR_PLLSRC_Msk* = (0x00000001 shl RCC_PLLCFGR_PLLSRC_Pos) ## !< 0x00400000
  RCC_PLLCFGR_PLLSRC* = RCC_PLLCFGR_PLLSRC_Msk
  RCC_PLLCFGR_PLLSRC_HSE_Pos* = (22)
  RCC_PLLCFGR_PLLSRC_HSE_Msk* = (0x00000001 shl RCC_PLLCFGR_PLLSRC_HSE_Pos) ## !< 0x00400000
  RCC_PLLCFGR_PLLSRC_HSE* = RCC_PLLCFGR_PLLSRC_HSE_Msk
  RCC_PLLCFGR_PLLSRC_HSI* = 0x00000000
  RCC_PLLCFGR_PLLQ_Pos* = (24)
  RCC_PLLCFGR_PLLQ_Msk* = (0x0000000F shl RCC_PLLCFGR_PLLQ_Pos) ## !< 0x0F000000
  RCC_PLLCFGR_PLLQ* = RCC_PLLCFGR_PLLQ_Msk
  RCC_PLLCFGR_PLLQ_Bit0* = (0x00000001 shl RCC_PLLCFGR_PLLQ_Pos) ## !< 0x01000000
  RCC_PLLCFGR_PLLQ_Bit1* = (0x00000002 shl RCC_PLLCFGR_PLLQ_Pos) ## !< 0x02000000
  RCC_PLLCFGR_PLLQ_Bit2* = (0x00000004 shl RCC_PLLCFGR_PLLQ_Pos) ## !< 0x04000000
  RCC_PLLCFGR_PLLQ_Bit3* = (0x00000008 shl RCC_PLLCFGR_PLLQ_Pos) ## !< 0x08000000

##
##  @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
##

const
  RCC_PLLR_I2S_CLKSOURCE_SUPPORT* = true ## !< Support PLLR clock as I2S clock source
  RCC_PLLCFGR_PLLR_Pos* = (28)
  RCC_PLLCFGR_PLLR_Msk* = (0x00000007 shl RCC_PLLCFGR_PLLR_Pos) ## !< 0x70000000
  RCC_PLLCFGR_PLLR* = RCC_PLLCFGR_PLLR_Msk
  RCC_PLLCFGR_PLLR_Bit0* = (0x00000001 shl RCC_PLLCFGR_PLLR_Pos) ## !< 0x10000000
  RCC_PLLCFGR_PLLR_Bit1* = (0x00000002 shl RCC_PLLCFGR_PLLR_Pos) ## !< 0x20000000
  RCC_PLLCFGR_PLLR_Bit2* = (0x00000004 shl RCC_PLLCFGR_PLLR_Pos) ## !< 0x40000000

## *******************  Bit definition for RCC_CFGR register  *****************
## !< SW configuration

const
  RCC_CFGR_SW_Pos* = (0)
  RCC_CFGR_SW_Msk* = (0x00000003 shl RCC_CFGR_SW_Pos) ## !< 0x00000003
  RCC_CFGR_SW* = RCC_CFGR_SW_Msk
  RCC_CFGR_SW_Bit0* = (0x00000001 shl RCC_CFGR_SW_Pos) ## !< 0x00000001
  RCC_CFGR_SW_Bit1* = (0x00000002 shl RCC_CFGR_SW_Pos) ## !< 0x00000002
  RCC_CFGR_SW_HSI* = 0x00000000
  RCC_CFGR_SW_HSE* = 0x00000001
  RCC_CFGR_SW_PLL* = 0x00000002

## !< SWS configuration

const
  RCC_CFGR_SWS_Pos* = (2)
  RCC_CFGR_SWS_Msk* = (0x00000003 shl RCC_CFGR_SWS_Pos) ## !< 0x0000000C
  RCC_CFGR_SWS* = RCC_CFGR_SWS_Msk
  RCC_CFGR_SWS_Bit0* = (0x00000001 shl RCC_CFGR_SWS_Pos) ## !< 0x00000004
  RCC_CFGR_SWS_Bit1* = (0x00000002 shl RCC_CFGR_SWS_Pos) ## !< 0x00000008
  RCC_CFGR_SWS_HSI* = 0x00000000
  RCC_CFGR_SWS_HSE* = 0x00000004
  RCC_CFGR_SWS_PLL* = 0x00000008

## !< HPRE configuration

const
  RCC_CFGR_HPRE_Pos* = (4)
  RCC_CFGR_HPRE_Msk* = (0x0000000F shl RCC_CFGR_HPRE_Pos) ## !< 0x000000F0
  RCC_CFGR_HPRE* = RCC_CFGR_HPRE_Msk
  RCC_CFGR_HPRE_Bit0* = (0x00000001 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000010
  RCC_CFGR_HPRE_Bit1* = (0x00000002 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000020
  RCC_CFGR_HPRE_Bit2* = (0x00000004 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000040
  RCC_CFGR_HPRE_Bit3* = (0x00000008 shl RCC_CFGR_HPRE_Pos) ## !< 0x00000080
  RCC_CFGR_HPRE_DIV1* = 0x00000000
  RCC_CFGR_HPRE_DIV2* = 0x00000080
  RCC_CFGR_HPRE_DIV4* = 0x00000090
  RCC_CFGR_HPRE_DIV8* = 0x000000A0
  RCC_CFGR_HPRE_DIV16* = 0x000000B0
  RCC_CFGR_HPRE_DIV64* = 0x000000C0
  RCC_CFGR_HPRE_DIV128* = 0x000000D0
  RCC_CFGR_HPRE_DIV256* = 0x000000E0
  RCC_CFGR_HPRE_DIV512* = 0x000000F0

## !< MCO1EN configuration

const
  RCC_CFGR_MCO1EN_Pos* = (8)
  RCC_CFGR_MCO1EN_Msk* = (0x00000001 shl RCC_CFGR_MCO1EN_Pos) ## !< 0x00000100
  RCC_CFGR_MCO1EN* = RCC_CFGR_MCO1EN_Msk

## !< PPRE1 configuration

const
  RCC_CFGR_PPRE1_Pos* = (10)
  RCC_CFGR_PPRE1_Msk* = (0x00000007 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00001C00
  RCC_CFGR_PPRE1* = RCC_CFGR_PPRE1_Msk
  RCC_CFGR_PPRE1_Bit0* = (0x00000001 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00000400
  RCC_CFGR_PPRE1_Bit1* = (0x00000002 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00000800
  RCC_CFGR_PPRE1_Bit2* = (0x00000004 shl RCC_CFGR_PPRE1_Pos) ## !< 0x00001000
  RCC_CFGR_PPRE1_DIV1* = 0x00000000
  RCC_CFGR_PPRE1_DIV2* = 0x00001000
  RCC_CFGR_PPRE1_DIV4* = 0x00001400
  RCC_CFGR_PPRE1_DIV8* = 0x00001800
  RCC_CFGR_PPRE1_DIV16* = 0x00001C00

## !< PPRE2 configuration

const
  RCC_CFGR_PPRE2_Pos* = (13)
  RCC_CFGR_PPRE2_Msk* = (0x00000007 shl RCC_CFGR_PPRE2_Pos) ## !< 0x0000E000
  RCC_CFGR_PPRE2* = RCC_CFGR_PPRE2_Msk
  RCC_CFGR_PPRE2_Bit0* = (0x00000001 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00002000
  RCC_CFGR_PPRE2_Bit1* = (0x00000002 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00004000
  RCC_CFGR_PPRE2_Bit2* = (0x00000004 shl RCC_CFGR_PPRE2_Pos) ## !< 0x00008000
  RCC_CFGR_PPRE2_DIV1* = 0x00000000
  RCC_CFGR_PPRE2_DIV2* = 0x00008000
  RCC_CFGR_PPRE2_DIV4* = 0x0000A000
  RCC_CFGR_PPRE2_DIV8* = 0x0000C000
  RCC_CFGR_PPRE2_DIV16* = 0x0000E000

## !< RTCPRE configuration

const
  RCC_CFGR_RTCPRE_Pos* = (16)
  RCC_CFGR_RTCPRE_Msk* = (0x0000001F shl RCC_CFGR_RTCPRE_Pos) ## !< 0x001F0000
  RCC_CFGR_RTCPRE* = RCC_CFGR_RTCPRE_Msk
  RCC_CFGR_RTCPRE_Bit0* = (0x00000001 shl RCC_CFGR_RTCPRE_Pos) ## !< 0x00010000
  RCC_CFGR_RTCPRE_Bit1* = (0x00000002 shl RCC_CFGR_RTCPRE_Pos) ## !< 0x00020000
  RCC_CFGR_RTCPRE_Bit2* = (0x00000004 shl RCC_CFGR_RTCPRE_Pos) ## !< 0x00040000
  RCC_CFGR_RTCPRE_Bit3* = (0x00000008 shl RCC_CFGR_RTCPRE_Pos) ## !< 0x00080000
  RCC_CFGR_RTCPRE_Bit4* = (0x00000010 shl RCC_CFGR_RTCPRE_Pos) ## !< 0x00100000

## !< MCO1 configuration

const
  RCC_CFGR_MCO1_Pos* = (21)
  RCC_CFGR_MCO1_Msk* = (0x00000003 shl RCC_CFGR_MCO1_Pos) ## !< 0x00600000
  RCC_CFGR_MCO1* = RCC_CFGR_MCO1_Msk
  RCC_CFGR_MCO1_Bit0* = (0x00000001 shl RCC_CFGR_MCO1_Pos) ## !< 0x00200000
  RCC_CFGR_MCO1_Bit1* = (0x00000002 shl RCC_CFGR_MCO1_Pos) ## !< 0x00400000
  RCC_CFGR_MCO1PRE_Pos* = (24)
  RCC_CFGR_MCO1PRE_Msk* = (0x00000007 shl RCC_CFGR_MCO1PRE_Pos) ## !< 0x07000000
  RCC_CFGR_MCO1PRE* = RCC_CFGR_MCO1PRE_Msk
  RCC_CFGR_MCO1PRE_Bit0* = (0x00000001 shl RCC_CFGR_MCO1PRE_Pos) ## !< 0x01000000
  RCC_CFGR_MCO1PRE_Bit1* = (0x00000002 shl RCC_CFGR_MCO1PRE_Pos) ## !< 0x02000000
  RCC_CFGR_MCO1PRE_Bit2* = (0x00000004 shl RCC_CFGR_MCO1PRE_Pos) ## !< 0x04000000
  RCC_CFGR_MCO2PRE_Pos* = (27)
  RCC_CFGR_MCO2PRE_Msk* = (0x00000007 shl RCC_CFGR_MCO2PRE_Pos) ## !< 0x38000000
  RCC_CFGR_MCO2PRE* = RCC_CFGR_MCO2PRE_Msk
  RCC_CFGR_MCO2PRE_Bit0* = (0x00000001 shl RCC_CFGR_MCO2PRE_Pos) ## !< 0x08000000
  RCC_CFGR_MCO2PRE_Bit1* = (0x00000002 shl RCC_CFGR_MCO2PRE_Pos) ## !< 0x10000000
  RCC_CFGR_MCO2PRE_Bit2* = (0x00000004 shl RCC_CFGR_MCO2PRE_Pos) ## !< 0x20000000
  RCC_CFGR_MCO2_Pos* = (30)
  RCC_CFGR_MCO2_Msk* = (0x00000003 shl RCC_CFGR_MCO2_Pos) ## !< 0xC0000000
  RCC_CFGR_MCO2* = RCC_CFGR_MCO2_Msk
  RCC_CFGR_MCO2_Bit0* = (0x00000001 shl RCC_CFGR_MCO2_Pos) ## !< 0x40000000
  RCC_CFGR_MCO2_Bit1* = (0x00000002 shl RCC_CFGR_MCO2_Pos) ## !< 0x80000000

## *******************  Bit definition for RCC_CIR register  ******************

const
  RCC_CIR_LSIRDYF_Pos* = (0)
  RCC_CIR_LSIRDYF_Msk* = (0x00000001 shl RCC_CIR_LSIRDYF_Pos) ## !< 0x00000001
  RCC_CIR_LSIRDYF* = RCC_CIR_LSIRDYF_Msk
  RCC_CIR_LSERDYF_Pos* = (1)
  RCC_CIR_LSERDYF_Msk* = (0x00000001 shl RCC_CIR_LSERDYF_Pos) ## !< 0x00000002
  RCC_CIR_LSERDYF* = RCC_CIR_LSERDYF_Msk
  RCC_CIR_HSIRDYF_Pos* = (2)
  RCC_CIR_HSIRDYF_Msk* = (0x00000001 shl RCC_CIR_HSIRDYF_Pos) ## !< 0x00000004
  RCC_CIR_HSIRDYF* = RCC_CIR_HSIRDYF_Msk
  RCC_CIR_HSERDYF_Pos* = (3)
  RCC_CIR_HSERDYF_Msk* = (0x00000001 shl RCC_CIR_HSERDYF_Pos) ## !< 0x00000008
  RCC_CIR_HSERDYF* = RCC_CIR_HSERDYF_Msk
  RCC_CIR_PLLRDYF_Pos* = (4)
  RCC_CIR_PLLRDYF_Msk* = (0x00000001 shl RCC_CIR_PLLRDYF_Pos) ## !< 0x00000010
  RCC_CIR_PLLRDYF* = RCC_CIR_PLLRDYF_Msk
  RCC_CIR_CSSF_Pos* = (7)
  RCC_CIR_CSSF_Msk* = (0x00000001 shl RCC_CIR_CSSF_Pos) ## !< 0x00000080
  RCC_CIR_CSSF* = RCC_CIR_CSSF_Msk
  RCC_CIR_LSIRDYIE_Pos* = (8)
  RCC_CIR_LSIRDYIE_Msk* = (0x00000001 shl RCC_CIR_LSIRDYIE_Pos) ## !< 0x00000100
  RCC_CIR_LSIRDYIE* = RCC_CIR_LSIRDYIE_Msk
  RCC_CIR_LSERDYIE_Pos* = (9)
  RCC_CIR_LSERDYIE_Msk* = (0x00000001 shl RCC_CIR_LSERDYIE_Pos) ## !< 0x00000200
  RCC_CIR_LSERDYIE* = RCC_CIR_LSERDYIE_Msk
  RCC_CIR_HSIRDYIE_Pos* = (10)
  RCC_CIR_HSIRDYIE_Msk* = (0x00000001 shl RCC_CIR_HSIRDYIE_Pos) ## !< 0x00000400
  RCC_CIR_HSIRDYIE* = RCC_CIR_HSIRDYIE_Msk
  RCC_CIR_HSERDYIE_Pos* = (11)
  RCC_CIR_HSERDYIE_Msk* = (0x00000001 shl RCC_CIR_HSERDYIE_Pos) ## !< 0x00000800
  RCC_CIR_HSERDYIE* = RCC_CIR_HSERDYIE_Msk
  RCC_CIR_PLLRDYIE_Pos* = (12)
  RCC_CIR_PLLRDYIE_Msk* = (0x00000001 shl RCC_CIR_PLLRDYIE_Pos) ## !< 0x00001000
  RCC_CIR_PLLRDYIE* = RCC_CIR_PLLRDYIE_Msk
  RCC_CIR_LSIRDYC_Pos* = (16)
  RCC_CIR_LSIRDYC_Msk* = (0x00000001 shl RCC_CIR_LSIRDYC_Pos) ## !< 0x00010000
  RCC_CIR_LSIRDYC* = RCC_CIR_LSIRDYC_Msk
  RCC_CIR_LSERDYC_Pos* = (17)
  RCC_CIR_LSERDYC_Msk* = (0x00000001 shl RCC_CIR_LSERDYC_Pos) ## !< 0x00020000
  RCC_CIR_LSERDYC* = RCC_CIR_LSERDYC_Msk
  RCC_CIR_HSIRDYC_Pos* = (18)
  RCC_CIR_HSIRDYC_Msk* = (0x00000001 shl RCC_CIR_HSIRDYC_Pos) ## !< 0x00040000
  RCC_CIR_HSIRDYC* = RCC_CIR_HSIRDYC_Msk
  RCC_CIR_HSERDYC_Pos* = (19)
  RCC_CIR_HSERDYC_Msk* = (0x00000001 shl RCC_CIR_HSERDYC_Pos) ## !< 0x00080000
  RCC_CIR_HSERDYC* = RCC_CIR_HSERDYC_Msk
  RCC_CIR_PLLRDYC_Pos* = (20)
  RCC_CIR_PLLRDYC_Msk* = (0x00000001 shl RCC_CIR_PLLRDYC_Pos) ## !< 0x00100000
  RCC_CIR_PLLRDYC* = RCC_CIR_PLLRDYC_Msk
  RCC_CIR_CSSC_Pos* = (23)
  RCC_CIR_CSSC_Msk* = (0x00000001 shl RCC_CIR_CSSC_Pos) ## !< 0x00800000
  RCC_CIR_CSSC* = RCC_CIR_CSSC_Msk

## *******************  Bit definition for RCC_AHB1RSTR register  *************

const
  RCC_AHB1RSTR_GPIOARST_Pos* = (0)
  RCC_AHB1RSTR_GPIOARST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIOARST_Pos) ## !< 0x00000001
  RCC_AHB1RSTR_GPIOARST* = RCC_AHB1RSTR_GPIOARST_Msk
  RCC_AHB1RSTR_GPIOBRST_Pos* = (1)
  RCC_AHB1RSTR_GPIOBRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIOBRST_Pos) ## !< 0x00000002
  RCC_AHB1RSTR_GPIOBRST* = RCC_AHB1RSTR_GPIOBRST_Msk
  RCC_AHB1RSTR_GPIOCRST_Pos* = (2)
  RCC_AHB1RSTR_GPIOCRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIOCRST_Pos) ## !< 0x00000004
  RCC_AHB1RSTR_GPIOCRST* = RCC_AHB1RSTR_GPIOCRST_Msk
  RCC_AHB1RSTR_GPIOHRST_Pos* = (7)
  RCC_AHB1RSTR_GPIOHRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_GPIOHRST_Pos) ## !< 0x00000080
  RCC_AHB1RSTR_GPIOHRST* = RCC_AHB1RSTR_GPIOHRST_Msk
  RCC_AHB1RSTR_CRCRST_Pos* = (12)
  RCC_AHB1RSTR_CRCRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_CRCRST_Pos) ## !< 0x00001000
  RCC_AHB1RSTR_CRCRST* = RCC_AHB1RSTR_CRCRST_Msk
  RCC_AHB1RSTR_DMA1RST_Pos* = (21)
  RCC_AHB1RSTR_DMA1RST_Msk* = (0x00000001 shl RCC_AHB1RSTR_DMA1RST_Pos) ## !< 0x00200000
  RCC_AHB1RSTR_DMA1RST* = RCC_AHB1RSTR_DMA1RST_Msk
  RCC_AHB1RSTR_DMA2RST_Pos* = (22)
  RCC_AHB1RSTR_DMA2RST_Msk* = (0x00000001 shl RCC_AHB1RSTR_DMA2RST_Pos) ## !< 0x00400000
  RCC_AHB1RSTR_DMA2RST* = RCC_AHB1RSTR_DMA2RST_Msk
  RCC_AHB1RSTR_RNGRST_Pos* = (31)
  RCC_AHB1RSTR_RNGRST_Msk* = (0x00000001 shl RCC_AHB1RSTR_RNGRST_Pos) ## !< 0x80000000
  RCC_AHB1RSTR_RNGRST* = RCC_AHB1RSTR_RNGRST_Msk

## *******************  Bit definition for RCC_APB1RSTR register  *************

const
  RCC_APB1RSTR_TIM5RST_Pos* = (3)
  RCC_APB1RSTR_TIM5RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM5RST_Pos) ## !< 0x00000008
  RCC_APB1RSTR_TIM5RST* = RCC_APB1RSTR_TIM5RST_Msk
  RCC_APB1RSTR_TIM6RST_Pos* = (4)
  RCC_APB1RSTR_TIM6RST_Msk* = (0x00000001 shl RCC_APB1RSTR_TIM6RST_Pos) ## !< 0x00000010
  RCC_APB1RSTR_TIM6RST* = RCC_APB1RSTR_TIM6RST_Msk
  RCC_APB1RSTR_LPTIM1RST_Pos* = (9)
  RCC_APB1RSTR_LPTIM1RST_Msk* = (0x00000001 shl RCC_APB1RSTR_LPTIM1RST_Pos) ## !< 0x00000200
  RCC_APB1RSTR_LPTIM1RST* = RCC_APB1RSTR_LPTIM1RST_Msk
  RCC_APB1RSTR_WWDGRST_Pos* = (11)
  RCC_APB1RSTR_WWDGRST_Msk* = (0x00000001 shl RCC_APB1RSTR_WWDGRST_Pos) ## !< 0x00000800
  RCC_APB1RSTR_WWDGRST* = RCC_APB1RSTR_WWDGRST_Msk
  RCC_APB1RSTR_SPI2RST_Pos* = (14)
  RCC_APB1RSTR_SPI2RST_Msk* = (0x00000001 shl RCC_APB1RSTR_SPI2RST_Pos) ## !< 0x00004000
  RCC_APB1RSTR_SPI2RST* = RCC_APB1RSTR_SPI2RST_Msk
  RCC_APB1RSTR_USART2RST_Pos* = (17)
  RCC_APB1RSTR_USART2RST_Msk* = (0x00000001 shl RCC_APB1RSTR_USART2RST_Pos) ## !< 0x00020000
  RCC_APB1RSTR_USART2RST* = RCC_APB1RSTR_USART2RST_Msk
  RCC_APB1RSTR_I2C1RST_Pos* = (21)
  RCC_APB1RSTR_I2C1RST_Msk* = (0x00000001 shl RCC_APB1RSTR_I2C1RST_Pos) ## !< 0x00200000
  RCC_APB1RSTR_I2C1RST* = RCC_APB1RSTR_I2C1RST_Msk
  RCC_APB1RSTR_I2C2RST_Pos* = (22)
  RCC_APB1RSTR_I2C2RST_Msk* = (0x00000001 shl RCC_APB1RSTR_I2C2RST_Pos) ## !< 0x00400000
  RCC_APB1RSTR_I2C2RST* = RCC_APB1RSTR_I2C2RST_Msk
  RCC_APB1RSTR_FMPI2C1RST_Pos* = (24)
  RCC_APB1RSTR_FMPI2C1RST_Msk* = (0x00000001 shl RCC_APB1RSTR_FMPI2C1RST_Pos) ## !< 0x01000000
  RCC_APB1RSTR_FMPI2C1RST* = RCC_APB1RSTR_FMPI2C1RST_Msk
  RCC_APB1RSTR_PWRRST_Pos* = (28)
  RCC_APB1RSTR_PWRRST_Msk* = (0x00000001 shl RCC_APB1RSTR_PWRRST_Pos) ## !< 0x10000000
  RCC_APB1RSTR_PWRRST* = RCC_APB1RSTR_PWRRST_Msk
  RCC_APB1RSTR_DACRST_Pos* = (29)
  RCC_APB1RSTR_DACRST_Msk* = (0x00000001 shl RCC_APB1RSTR_DACRST_Pos) ## !< 0x20000000
  RCC_APB1RSTR_DACRST* = RCC_APB1RSTR_DACRST_Msk

## *******************  Bit definition for RCC_APB2RSTR register  *************

const
  RCC_APB2RSTR_TIM1RST_Pos* = (0)
  RCC_APB2RSTR_TIM1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM1RST_Pos) ## !< 0x00000001
  RCC_APB2RSTR_TIM1RST* = RCC_APB2RSTR_TIM1RST_Msk
  RCC_APB2RSTR_USART1RST_Pos* = (4)
  RCC_APB2RSTR_USART1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_USART1RST_Pos) ## !< 0x00000010
  RCC_APB2RSTR_USART1RST* = RCC_APB2RSTR_USART1RST_Msk
  RCC_APB2RSTR_USART6RST_Pos* = (5)
  RCC_APB2RSTR_USART6RST_Msk* = (0x00000001 shl RCC_APB2RSTR_USART6RST_Pos) ## !< 0x00000020
  RCC_APB2RSTR_USART6RST* = RCC_APB2RSTR_USART6RST_Msk
  RCC_APB2RSTR_ADCRST_Pos* = (8)
  RCC_APB2RSTR_ADCRST_Msk* = (0x00000001 shl RCC_APB2RSTR_ADCRST_Pos) ## !< 0x00000100
  RCC_APB2RSTR_ADCRST* = RCC_APB2RSTR_ADCRST_Msk
  RCC_APB2RSTR_SPI1RST_Pos* = (12)
  RCC_APB2RSTR_SPI1RST_Msk* = (0x00000001 shl RCC_APB2RSTR_SPI1RST_Pos) ## !< 0x00001000
  RCC_APB2RSTR_SPI1RST* = RCC_APB2RSTR_SPI1RST_Msk
  RCC_APB2RSTR_SYSCFGRST_Pos* = (14)
  RCC_APB2RSTR_SYSCFGRST_Msk* = (0x00000001 shl RCC_APB2RSTR_SYSCFGRST_Pos) ## !< 0x00004000
  RCC_APB2RSTR_SYSCFGRST* = RCC_APB2RSTR_SYSCFGRST_Msk
  RCC_APB2RSTR_TIM9RST_Pos* = (16)
  RCC_APB2RSTR_TIM9RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM9RST_Pos) ## !< 0x00010000
  RCC_APB2RSTR_TIM9RST* = RCC_APB2RSTR_TIM9RST_Msk
  RCC_APB2RSTR_TIM11RST_Pos* = (18)
  RCC_APB2RSTR_TIM11RST_Msk* = (0x00000001 shl RCC_APB2RSTR_TIM11RST_Pos) ## !< 0x00040000
  RCC_APB2RSTR_TIM11RST* = RCC_APB2RSTR_TIM11RST_Msk
  RCC_APB2RSTR_SPI5RST_Pos* = (20)
  RCC_APB2RSTR_SPI5RST_Msk* = (0x00000001 shl RCC_APB2RSTR_SPI5RST_Pos) ## !< 0x00100000
  RCC_APB2RSTR_SPI5RST* = RCC_APB2RSTR_SPI5RST_Msk

## *******************  Bit definition for RCC_AHB1ENR register  **************

const
  RCC_AHB1ENR_GPIOAEN_Pos* = (0)
  RCC_AHB1ENR_GPIOAEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIOAEN_Pos) ## !< 0x00000001
  RCC_AHB1ENR_GPIOAEN* = RCC_AHB1ENR_GPIOAEN_Msk
  RCC_AHB1ENR_GPIOBEN_Pos* = (1)
  RCC_AHB1ENR_GPIOBEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIOBEN_Pos) ## !< 0x00000002
  RCC_AHB1ENR_GPIOBEN* = RCC_AHB1ENR_GPIOBEN_Msk
  RCC_AHB1ENR_GPIOCEN_Pos* = (2)
  RCC_AHB1ENR_GPIOCEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIOCEN_Pos) ## !< 0x00000004
  RCC_AHB1ENR_GPIOCEN* = RCC_AHB1ENR_GPIOCEN_Msk
  RCC_AHB1ENR_GPIOHEN_Pos* = (7)
  RCC_AHB1ENR_GPIOHEN_Msk* = (0x00000001 shl RCC_AHB1ENR_GPIOHEN_Pos) ## !< 0x00000080
  RCC_AHB1ENR_GPIOHEN* = RCC_AHB1ENR_GPIOHEN_Msk
  RCC_AHB1ENR_CRCEN_Pos* = (12)
  RCC_AHB1ENR_CRCEN_Msk* = (0x00000001 shl RCC_AHB1ENR_CRCEN_Pos) ## !< 0x00001000
  RCC_AHB1ENR_CRCEN* = RCC_AHB1ENR_CRCEN_Msk
  RCC_AHB1ENR_DMA1EN_Pos* = (21)
  RCC_AHB1ENR_DMA1EN_Msk* = (0x00000001 shl RCC_AHB1ENR_DMA1EN_Pos) ## !< 0x00200000
  RCC_AHB1ENR_DMA1EN* = RCC_AHB1ENR_DMA1EN_Msk
  RCC_AHB1ENR_DMA2EN_Pos* = (22)
  RCC_AHB1ENR_DMA2EN_Msk* = (0x00000001 shl RCC_AHB1ENR_DMA2EN_Pos) ## !< 0x00400000
  RCC_AHB1ENR_DMA2EN* = RCC_AHB1ENR_DMA2EN_Msk
  RCC_AHB1ENR_RNGEN_Pos* = (31)
  RCC_AHB1ENR_RNGEN_Msk* = (0x00000001 shl RCC_AHB1ENR_RNGEN_Pos) ## !< 0x80000000
  RCC_AHB1ENR_RNGEN* = RCC_AHB1ENR_RNGEN_Msk

## *******************  Bit definition for RCC_APB1ENR register  **************

const
  RCC_APB1ENR_TIM5EN_Pos* = (3)
  RCC_APB1ENR_TIM5EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM5EN_Pos) ## !< 0x00000008
  RCC_APB1ENR_TIM5EN* = RCC_APB1ENR_TIM5EN_Msk
  RCC_APB1ENR_TIM6EN_Pos* = (4)
  RCC_APB1ENR_TIM6EN_Msk* = (0x00000001 shl RCC_APB1ENR_TIM6EN_Pos) ## !< 0x00000010
  RCC_APB1ENR_TIM6EN* = RCC_APB1ENR_TIM6EN_Msk
  RCC_APB1ENR_LPTIM1EN_Pos* = (9)
  RCC_APB1ENR_LPTIM1EN_Msk* = (0x00000001 shl RCC_APB1ENR_LPTIM1EN_Pos) ## !< 0x00000200
  RCC_APB1ENR_LPTIM1EN* = RCC_APB1ENR_LPTIM1EN_Msk
  RCC_APB1ENR_RTCAPBEN_Pos* = (10)
  RCC_APB1ENR_RTCAPBEN_Msk* = (0x00000001 shl RCC_APB1ENR_RTCAPBEN_Pos) ## !< 0x00000400
  RCC_APB1ENR_RTCAPBEN* = RCC_APB1ENR_RTCAPBEN_Msk
  RCC_APB1ENR_WWDGEN_Pos* = (11)
  RCC_APB1ENR_WWDGEN_Msk* = (0x00000001 shl RCC_APB1ENR_WWDGEN_Pos) ## !< 0x00000800
  RCC_APB1ENR_WWDGEN* = RCC_APB1ENR_WWDGEN_Msk
  RCC_APB1ENR_SPI2EN_Pos* = (14)
  RCC_APB1ENR_SPI2EN_Msk* = (0x00000001 shl RCC_APB1ENR_SPI2EN_Pos) ## !< 0x00004000
  RCC_APB1ENR_SPI2EN* = RCC_APB1ENR_SPI2EN_Msk
  RCC_APB1ENR_USART2EN_Pos* = (17)
  RCC_APB1ENR_USART2EN_Msk* = (0x00000001 shl RCC_APB1ENR_USART2EN_Pos) ## !< 0x00020000
  RCC_APB1ENR_USART2EN* = RCC_APB1ENR_USART2EN_Msk
  RCC_APB1ENR_I2C1EN_Pos* = (21)
  RCC_APB1ENR_I2C1EN_Msk* = (0x00000001 shl RCC_APB1ENR_I2C1EN_Pos) ## !< 0x00200000
  RCC_APB1ENR_I2C1EN* = RCC_APB1ENR_I2C1EN_Msk
  RCC_APB1ENR_I2C2EN_Pos* = (22)
  RCC_APB1ENR_I2C2EN_Msk* = (0x00000001 shl RCC_APB1ENR_I2C2EN_Pos) ## !< 0x00400000
  RCC_APB1ENR_I2C2EN* = RCC_APB1ENR_I2C2EN_Msk
  RCC_APB1ENR_FMPI2C1EN_Pos* = (24)
  RCC_APB1ENR_FMPI2C1EN_Msk* = (0x00000001 shl RCC_APB1ENR_FMPI2C1EN_Pos) ## !< 0x01000000
  RCC_APB1ENR_FMPI2C1EN* = RCC_APB1ENR_FMPI2C1EN_Msk
  RCC_APB1ENR_PWREN_Pos* = (28)
  RCC_APB1ENR_PWREN_Msk* = (0x00000001 shl RCC_APB1ENR_PWREN_Pos) ## !< 0x10000000
  RCC_APB1ENR_PWREN* = RCC_APB1ENR_PWREN_Msk
  RCC_APB1ENR_DACEN_Pos* = (29)
  RCC_APB1ENR_DACEN_Msk* = (0x00000001 shl RCC_APB1ENR_DACEN_Pos) ## !< 0x20000000
  RCC_APB1ENR_DACEN* = RCC_APB1ENR_DACEN_Msk

## *******************  Bit definition for RCC_APB2ENR register  **************

const
  RCC_APB2ENR_TIM1EN_Pos* = (0)
  RCC_APB2ENR_TIM1EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM1EN_Pos) ## !< 0x00000001
  RCC_APB2ENR_TIM1EN* = RCC_APB2ENR_TIM1EN_Msk
  RCC_APB2ENR_USART1EN_Pos* = (4)
  RCC_APB2ENR_USART1EN_Msk* = (0x00000001 shl RCC_APB2ENR_USART1EN_Pos) ## !< 0x00000010
  RCC_APB2ENR_USART1EN* = RCC_APB2ENR_USART1EN_Msk
  RCC_APB2ENR_USART6EN_Pos* = (5)
  RCC_APB2ENR_USART6EN_Msk* = (0x00000001 shl RCC_APB2ENR_USART6EN_Pos) ## !< 0x00000020
  RCC_APB2ENR_USART6EN* = RCC_APB2ENR_USART6EN_Msk
  RCC_APB2ENR_ADC1EN_Pos* = (8)
  RCC_APB2ENR_ADC1EN_Msk* = (0x00000001 shl RCC_APB2ENR_ADC1EN_Pos) ## !< 0x00000100
  RCC_APB2ENR_ADC1EN* = RCC_APB2ENR_ADC1EN_Msk
  RCC_APB2ENR_SPI1EN_Pos* = (12)
  RCC_APB2ENR_SPI1EN_Msk* = (0x00000001 shl RCC_APB2ENR_SPI1EN_Pos) ## !< 0x00001000
  RCC_APB2ENR_SPI1EN* = RCC_APB2ENR_SPI1EN_Msk
  RCC_APB2ENR_SYSCFGEN_Pos* = (14)
  RCC_APB2ENR_SYSCFGEN_Msk* = (0x00000001 shl RCC_APB2ENR_SYSCFGEN_Pos) ## !< 0x00004000
  RCC_APB2ENR_SYSCFGEN* = RCC_APB2ENR_SYSCFGEN_Msk
  RCC_APB2ENR_EXTITEN_Pos* = (15)
  RCC_APB2ENR_EXTITEN_Msk* = (0x00000001 shl RCC_APB2ENR_EXTITEN_Pos) ## !< 0x00008000
  RCC_APB2ENR_EXTITEN* = RCC_APB2ENR_EXTITEN_Msk
  RCC_APB2ENR_TIM9EN_Pos* = (16)
  RCC_APB2ENR_TIM9EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM9EN_Pos) ## !< 0x00010000
  RCC_APB2ENR_TIM9EN* = RCC_APB2ENR_TIM9EN_Msk
  RCC_APB2ENR_TIM11EN_Pos* = (18)
  RCC_APB2ENR_TIM11EN_Msk* = (0x00000001 shl RCC_APB2ENR_TIM11EN_Pos) ## !< 0x00040000
  RCC_APB2ENR_TIM11EN* = RCC_APB2ENR_TIM11EN_Msk
  RCC_APB2ENR_SPI5EN_Pos* = (20)
  RCC_APB2ENR_SPI5EN_Msk* = (0x00000001 shl RCC_APB2ENR_SPI5EN_Pos) ## !< 0x00100000
  RCC_APB2ENR_SPI5EN* = RCC_APB2ENR_SPI5EN_Msk

## *******************  Bit definition for RCC_AHB1LPENR register  ************

const
  RCC_AHB1LPENR_GPIOALPEN_Pos* = (0)
  RCC_AHB1LPENR_GPIOALPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIOALPEN_Pos) ## !< 0x00000001
  RCC_AHB1LPENR_GPIOALPEN* = RCC_AHB1LPENR_GPIOALPEN_Msk
  RCC_AHB1LPENR_GPIOBLPEN_Pos* = (1)
  RCC_AHB1LPENR_GPIOBLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIOBLPEN_Pos) ## !< 0x00000002
  RCC_AHB1LPENR_GPIOBLPEN* = RCC_AHB1LPENR_GPIOBLPEN_Msk
  RCC_AHB1LPENR_GPIOCLPEN_Pos* = (2)
  RCC_AHB1LPENR_GPIOCLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIOCLPEN_Pos) ## !< 0x00000004
  RCC_AHB1LPENR_GPIOCLPEN* = RCC_AHB1LPENR_GPIOCLPEN_Msk
  RCC_AHB1LPENR_GPIOHLPEN_Pos* = (7)
  RCC_AHB1LPENR_GPIOHLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_GPIOHLPEN_Pos) ## !< 0x00000080
  RCC_AHB1LPENR_GPIOHLPEN* = RCC_AHB1LPENR_GPIOHLPEN_Msk
  RCC_AHB1LPENR_CRCLPEN_Pos* = (12)
  RCC_AHB1LPENR_CRCLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_CRCLPEN_Pos) ## !< 0x00001000
  RCC_AHB1LPENR_CRCLPEN* = RCC_AHB1LPENR_CRCLPEN_Msk
  RCC_AHB1LPENR_FLITFLPEN_Pos* = (15)
  RCC_AHB1LPENR_FLITFLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_FLITFLPEN_Pos) ## !< 0x00008000
  RCC_AHB1LPENR_FLITFLPEN* = RCC_AHB1LPENR_FLITFLPEN_Msk
  RCC_AHB1LPENR_SRAM1LPEN_Pos* = (16)
  RCC_AHB1LPENR_SRAM1LPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_SRAM1LPEN_Pos) ## !< 0x00010000
  RCC_AHB1LPENR_SRAM1LPEN* = RCC_AHB1LPENR_SRAM1LPEN_Msk
  RCC_AHB1LPENR_DMA1LPEN_Pos* = (21)
  RCC_AHB1LPENR_DMA1LPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_DMA1LPEN_Pos) ## !< 0x00200000
  RCC_AHB1LPENR_DMA1LPEN* = RCC_AHB1LPENR_DMA1LPEN_Msk
  RCC_AHB1LPENR_DMA2LPEN_Pos* = (22)
  RCC_AHB1LPENR_DMA2LPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_DMA2LPEN_Pos) ## !< 0x00400000
  RCC_AHB1LPENR_DMA2LPEN* = RCC_AHB1LPENR_DMA2LPEN_Msk
  RCC_AHB1LPENR_RNGLPEN_Pos* = (31)
  RCC_AHB1LPENR_RNGLPEN_Msk* = (0x00000001 shl RCC_AHB1LPENR_RNGLPEN_Pos) ## !< 0x80000000
  RCC_AHB1LPENR_RNGLPEN* = RCC_AHB1LPENR_RNGLPEN_Msk

## *******************  Bit definition for RCC_APB1LPENR register  ************

const
  RCC_APB1LPENR_TIM5LPEN_Pos* = (3)
  RCC_APB1LPENR_TIM5LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_TIM5LPEN_Pos) ## !< 0x00000008
  RCC_APB1LPENR_TIM5LPEN* = RCC_APB1LPENR_TIM5LPEN_Msk
  RCC_APB1LPENR_TIM6LPEN_Pos* = (4)
  RCC_APB1LPENR_TIM6LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_TIM6LPEN_Pos) ## !< 0x00000010
  RCC_APB1LPENR_TIM6LPEN* = RCC_APB1LPENR_TIM6LPEN_Msk
  RCC_APB1LPENR_LPTIM1LPEN_Pos* = (9)
  RCC_APB1LPENR_LPTIM1LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_LPTIM1LPEN_Pos) ## !< 0x00000200
  RCC_APB1LPENR_LPTIM1LPEN* = RCC_APB1LPENR_LPTIM1LPEN_Msk
  RCC_APB1LPENR_RTCAPBLPEN_Pos* = (10)
  RCC_APB1LPENR_RTCAPBLPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_RTCAPBLPEN_Pos) ## !< 0x00000400
  RCC_APB1LPENR_RTCAPBLPEN* = RCC_APB1LPENR_RTCAPBLPEN_Msk
  RCC_APB1LPENR_WWDGLPEN_Pos* = (11)
  RCC_APB1LPENR_WWDGLPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_WWDGLPEN_Pos) ## !< 0x00000800
  RCC_APB1LPENR_WWDGLPEN* = RCC_APB1LPENR_WWDGLPEN_Msk
  RCC_APB1LPENR_SPI2LPEN_Pos* = (14)
  RCC_APB1LPENR_SPI2LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_SPI2LPEN_Pos) ## !< 0x00004000
  RCC_APB1LPENR_SPI2LPEN* = RCC_APB1LPENR_SPI2LPEN_Msk
  RCC_APB1LPENR_USART2LPEN_Pos* = (17)
  RCC_APB1LPENR_USART2LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_USART2LPEN_Pos) ## !< 0x00020000
  RCC_APB1LPENR_USART2LPEN* = RCC_APB1LPENR_USART2LPEN_Msk
  RCC_APB1LPENR_I2C1LPEN_Pos* = (21)
  RCC_APB1LPENR_I2C1LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_I2C1LPEN_Pos) ## !< 0x00200000
  RCC_APB1LPENR_I2C1LPEN* = RCC_APB1LPENR_I2C1LPEN_Msk
  RCC_APB1LPENR_I2C2LPEN_Pos* = (22)
  RCC_APB1LPENR_I2C2LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_I2C2LPEN_Pos) ## !< 0x00400000
  RCC_APB1LPENR_I2C2LPEN* = RCC_APB1LPENR_I2C2LPEN_Msk
  RCC_APB1LPENR_FMPI2C1LPEN_Pos* = (24)
  RCC_APB1LPENR_FMPI2C1LPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_FMPI2C1LPEN_Pos) ## !< 0x01000000
  RCC_APB1LPENR_FMPI2C1LPEN* = RCC_APB1LPENR_FMPI2C1LPEN_Msk
  RCC_APB1LPENR_PWRLPEN_Pos* = (28)
  RCC_APB1LPENR_PWRLPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_PWRLPEN_Pos) ## !< 0x10000000
  RCC_APB1LPENR_PWRLPEN* = RCC_APB1LPENR_PWRLPEN_Msk
  RCC_APB1LPENR_DACLPEN_Pos* = (29)
  RCC_APB1LPENR_DACLPEN_Msk* = (0x00000001 shl RCC_APB1LPENR_DACLPEN_Pos) ## !< 0x20000000
  RCC_APB1LPENR_DACLPEN* = RCC_APB1LPENR_DACLPEN_Msk

## *******************  Bit definition for RCC_APB2LPENR register  ************

const
  RCC_APB2LPENR_TIM1LPEN_Pos* = (0)
  RCC_APB2LPENR_TIM1LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_TIM1LPEN_Pos) ## !< 0x00000001
  RCC_APB2LPENR_TIM1LPEN* = RCC_APB2LPENR_TIM1LPEN_Msk
  RCC_APB2LPENR_USART1LPEN_Pos* = (4)
  RCC_APB2LPENR_USART1LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_USART1LPEN_Pos) ## !< 0x00000010
  RCC_APB2LPENR_USART1LPEN* = RCC_APB2LPENR_USART1LPEN_Msk
  RCC_APB2LPENR_USART6LPEN_Pos* = (5)
  RCC_APB2LPENR_USART6LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_USART6LPEN_Pos) ## !< 0x00000020
  RCC_APB2LPENR_USART6LPEN* = RCC_APB2LPENR_USART6LPEN_Msk
  RCC_APB2LPENR_ADC1LPEN_Pos* = (8)
  RCC_APB2LPENR_ADC1LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_ADC1LPEN_Pos) ## !< 0x00000100
  RCC_APB2LPENR_ADC1LPEN* = RCC_APB2LPENR_ADC1LPEN_Msk
  RCC_APB2LPENR_SPI1LPEN_Pos* = (12)
  RCC_APB2LPENR_SPI1LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_SPI1LPEN_Pos) ## !< 0x00001000
  RCC_APB2LPENR_SPI1LPEN* = RCC_APB2LPENR_SPI1LPEN_Msk
  RCC_APB2LPENR_SYSCFGLPEN_Pos* = (14)
  RCC_APB2LPENR_SYSCFGLPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_SYSCFGLPEN_Pos) ## !< 0x00004000
  RCC_APB2LPENR_SYSCFGLPEN* = RCC_APB2LPENR_SYSCFGLPEN_Msk
  RCC_APB2LPENR_EXTITLPEN_Pos* = (15)
  RCC_APB2LPENR_EXTITLPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_EXTITLPEN_Pos) ## !< 0x00008000
  RCC_APB2LPENR_EXTITLPEN* = RCC_APB2LPENR_EXTITLPEN_Msk
  RCC_APB2LPENR_TIM9LPEN_Pos* = (16)
  RCC_APB2LPENR_TIM9LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_TIM9LPEN_Pos) ## !< 0x00010000
  RCC_APB2LPENR_TIM9LPEN* = RCC_APB2LPENR_TIM9LPEN_Msk
  RCC_APB2LPENR_TIM11LPEN_Pos* = (18)
  RCC_APB2LPENR_TIM11LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_TIM11LPEN_Pos) ## !< 0x00040000
  RCC_APB2LPENR_TIM11LPEN* = RCC_APB2LPENR_TIM11LPEN_Msk
  RCC_APB2LPENR_SPI5LPEN_Pos* = (20)
  RCC_APB2LPENR_SPI5LPEN_Msk* = (0x00000001 shl RCC_APB2LPENR_SPI5LPEN_Pos) ## !< 0x00100000
  RCC_APB2LPENR_SPI5LPEN* = RCC_APB2LPENR_SPI5LPEN_Msk

## *******************  Bit definition for RCC_BDCR register  *****************

const
  RCC_BDCR_LSEON_Pos* = (0)
  RCC_BDCR_LSEON_Msk* = (0x00000001 shl RCC_BDCR_LSEON_Pos) ## !< 0x00000001
  RCC_BDCR_LSEON* = RCC_BDCR_LSEON_Msk
  RCC_BDCR_LSERDY_Pos* = (1)
  RCC_BDCR_LSERDY_Msk* = (0x00000001 shl RCC_BDCR_LSERDY_Pos) ## !< 0x00000002
  RCC_BDCR_LSERDY* = RCC_BDCR_LSERDY_Msk
  RCC_BDCR_LSEBYP_Pos* = (2)
  RCC_BDCR_LSEBYP_Msk* = (0x00000001 shl RCC_BDCR_LSEBYP_Pos) ## !< 0x00000004
  RCC_BDCR_LSEBYP* = RCC_BDCR_LSEBYP_Msk
  RCC_BDCR_LSEMOD_Pos* = (3)
  RCC_BDCR_LSEMOD_Msk* = (0x00000001 shl RCC_BDCR_LSEMOD_Pos) ## !< 0x00000008
  RCC_BDCR_LSEMOD* = RCC_BDCR_LSEMOD_Msk
  RCC_BDCR_RTCSEL_Pos* = (8)
  RCC_BDCR_RTCSEL_Msk* = (0x00000003 shl RCC_BDCR_RTCSEL_Pos) ## !< 0x00000300
  RCC_BDCR_RTCSEL* = RCC_BDCR_RTCSEL_Msk
  RCC_BDCR_RTCSEL_Bit0* = (0x00000001 shl RCC_BDCR_RTCSEL_Pos) ## !< 0x00000100
  RCC_BDCR_RTCSEL_Bit1* = (0x00000002 shl RCC_BDCR_RTCSEL_Pos) ## !< 0x00000200
  RCC_BDCR_RTCEN_Pos* = (15)
  RCC_BDCR_RTCEN_Msk* = (0x00000001 shl RCC_BDCR_RTCEN_Pos) ## !< 0x00008000
  RCC_BDCR_RTCEN* = RCC_BDCR_RTCEN_Msk
  RCC_BDCR_BDRST_Pos* = (16)
  RCC_BDCR_BDRST_Msk* = (0x00000001 shl RCC_BDCR_BDRST_Pos) ## !< 0x00010000
  RCC_BDCR_BDRST* = RCC_BDCR_BDRST_Msk

## *******************  Bit definition for RCC_CSR register  ******************

const
  RCC_CSR_LSION_Pos* = (0)
  RCC_CSR_LSION_Msk* = (0x00000001 shl RCC_CSR_LSION_Pos) ## !< 0x00000001
  RCC_CSR_LSION* = RCC_CSR_LSION_Msk
  RCC_CSR_LSIRDY_Pos* = (1)
  RCC_CSR_LSIRDY_Msk* = (0x00000001 shl RCC_CSR_LSIRDY_Pos) ## !< 0x00000002
  RCC_CSR_LSIRDY* = RCC_CSR_LSIRDY_Msk
  RCC_CSR_RMVF_Pos* = (24)
  RCC_CSR_RMVF_Msk* = (0x00000001 shl RCC_CSR_RMVF_Pos) ## !< 0x01000000
  RCC_CSR_RMVF* = RCC_CSR_RMVF_Msk
  RCC_CSR_BORRSTF_Pos* = (25)
  RCC_CSR_BORRSTF_Msk* = (0x00000001 shl RCC_CSR_BORRSTF_Pos) ## !< 0x02000000
  RCC_CSR_BORRSTF* = RCC_CSR_BORRSTF_Msk
  RCC_CSR_PINRSTF_Pos* = (26)
  RCC_CSR_PINRSTF_Msk* = (0x00000001 shl RCC_CSR_PINRSTF_Pos) ## !< 0x04000000
  RCC_CSR_PINRSTF* = RCC_CSR_PINRSTF_Msk
  RCC_CSR_PORRSTF_Pos* = (27)
  RCC_CSR_PORRSTF_Msk* = (0x00000001 shl RCC_CSR_PORRSTF_Pos) ## !< 0x08000000
  RCC_CSR_PORRSTF* = RCC_CSR_PORRSTF_Msk
  RCC_CSR_SFTRSTF_Pos* = (28)
  RCC_CSR_SFTRSTF_Msk* = (0x00000001 shl RCC_CSR_SFTRSTF_Pos) ## !< 0x10000000
  RCC_CSR_SFTRSTF* = RCC_CSR_SFTRSTF_Msk
  RCC_CSR_IWDGRSTF_Pos* = (29)
  RCC_CSR_IWDGRSTF_Msk* = (0x00000001 shl RCC_CSR_IWDGRSTF_Pos) ## !< 0x20000000
  RCC_CSR_IWDGRSTF* = RCC_CSR_IWDGRSTF_Msk
  RCC_CSR_WWDGRSTF_Pos* = (30)
  RCC_CSR_WWDGRSTF_Msk* = (0x00000001 shl RCC_CSR_WWDGRSTF_Pos) ## !< 0x40000000
  RCC_CSR_WWDGRSTF* = RCC_CSR_WWDGRSTF_Msk
  RCC_CSR_LPWRRSTF_Pos* = (31)
  RCC_CSR_LPWRRSTF_Msk* = (0x00000001 shl RCC_CSR_LPWRRSTF_Pos) ## !< 0x80000000
  RCC_CSR_LPWRRSTF* = RCC_CSR_LPWRRSTF_Msk

##  Legacy defines

const
  RCC_CSR_PADRSTF* = RCC_CSR_PINRSTF
  RCC_CSR_WDGRSTF* = RCC_CSR_IWDGRSTF

## *******************  Bit definition for RCC_SSCGR register  ****************

const
  RCC_SSCGR_MODPER_Pos* = (0)
  RCC_SSCGR_MODPER_Msk* = (0x00001FFF shl RCC_SSCGR_MODPER_Pos) ## !< 0x00001FFF
  RCC_SSCGR_MODPER* = RCC_SSCGR_MODPER_Msk
  RCC_SSCGR_INCSTEP_Pos* = (13)
  RCC_SSCGR_INCSTEP_Msk* = (0x00007FFF shl RCC_SSCGR_INCSTEP_Pos) ## !< 0x0FFFE000
  RCC_SSCGR_INCSTEP* = RCC_SSCGR_INCSTEP_Msk
  RCC_SSCGR_SPREADSEL_Pos* = (30)
  RCC_SSCGR_SPREADSEL_Msk* = (0x00000001 shl RCC_SSCGR_SPREADSEL_Pos) ## !< 0x40000000
  RCC_SSCGR_SPREADSEL* = RCC_SSCGR_SPREADSEL_Msk
  RCC_SSCGR_SSCGEN_Pos* = (31)
  RCC_SSCGR_SSCGEN_Msk* = (0x00000001 shl RCC_SSCGR_SSCGEN_Pos) ## !< 0x80000000
  RCC_SSCGR_SSCGEN* = RCC_SSCGR_SSCGEN_Msk

## *******************  Bit definition for RCC_DCKCFGR register  **************

const
  RCC_DCKCFGR_TIMPRE_Pos* = (24)
  RCC_DCKCFGR_TIMPRE_Msk* = (0x00000001 shl RCC_DCKCFGR_TIMPRE_Pos) ## !< 0x01000000
  RCC_DCKCFGR_TIMPRE* = RCC_DCKCFGR_TIMPRE_Msk
  RCC_DCKCFGR_I2SSRC_Pos* = (25)
  RCC_DCKCFGR_I2SSRC_Msk* = (0x00000003 shl RCC_DCKCFGR_I2SSRC_Pos) ## !< 0x06000000
  RCC_DCKCFGR_I2SSRC* = RCC_DCKCFGR_I2SSRC_Msk
  RCC_DCKCFGR_I2SSRC_Bit0* = (0x00000001 shl RCC_DCKCFGR_I2SSRC_Pos) ## !< 0x02000000
  RCC_DCKCFGR_I2SSRC_Bit1* = (0x00000002 shl RCC_DCKCFGR_I2SSRC_Pos) ## !< 0x04000000

## *******************  Bit definition for RCC_DCKCFGR2 register  **************

const
  RCC_DCKCFGR2_FMPI2C1SEL_Pos* = (22)
  RCC_DCKCFGR2_FMPI2C1SEL_Msk* = (0x00000003 shl RCC_DCKCFGR2_FMPI2C1SEL_Pos) ## !< 0x00C00000
  RCC_DCKCFGR2_FMPI2C1SEL* = RCC_DCKCFGR2_FMPI2C1SEL_Msk
  RCC_DCKCFGR2_FMPI2C1SEL_Bit0* = (0x00000001 shl RCC_DCKCFGR2_FMPI2C1SEL_Pos) ## !< 0x00400000
  RCC_DCKCFGR2_FMPI2C1SEL_Bit1* = (0x00000002 shl RCC_DCKCFGR2_FMPI2C1SEL_Pos) ## !< 0x00800000
  RCC_DCKCFGR2_LPTIM1SEL_Pos* = (30)
  RCC_DCKCFGR2_LPTIM1SEL_Msk* = (0x00000003 shl RCC_DCKCFGR2_LPTIM1SEL_Pos) ## !< 0xC0000000
  RCC_DCKCFGR2_LPTIM1SEL* = RCC_DCKCFGR2_LPTIM1SEL_Msk
  RCC_DCKCFGR2_LPTIM1SEL_Bit0* = (0x00000001 shl RCC_DCKCFGR2_LPTIM1SEL_Pos) ## !< 0x40000000
  RCC_DCKCFGR2_LPTIM1SEL_Bit1* = (0x00000002 shl RCC_DCKCFGR2_LPTIM1SEL_Pos) ## !< 0x80000000

## ****************************************************************************
##
##                                     RNG
##
## ****************************************************************************
## *******************  Bits definition for RNG_CR register  ******************

const
  RNG_CR_RNGEN_Pos* = (2)
  RNG_CR_RNGEN_Msk* = (0x00000001 shl RNG_CR_RNGEN_Pos) ## !< 0x00000004
  RNG_CR_RNGEN* = RNG_CR_RNGEN_Msk
  RNG_CR_IE_Pos* = (3)
  RNG_CR_IE_Msk* = (0x00000001 shl RNG_CR_IE_Pos) ## !< 0x00000008
  RNG_CR_IE* = RNG_CR_IE_Msk

## *******************  Bits definition for RNG_SR register  ******************

const
  RNG_SR_DRDY_Pos* = (0)
  RNG_SR_DRDY_Msk* = (0x00000001 shl RNG_SR_DRDY_Pos) ## !< 0x00000001
  RNG_SR_DRDY* = RNG_SR_DRDY_Msk
  RNG_SR_CECS_Pos* = (1)
  RNG_SR_CECS_Msk* = (0x00000001 shl RNG_SR_CECS_Pos) ## !< 0x00000002
  RNG_SR_CECS* = RNG_SR_CECS_Msk
  RNG_SR_SECS_Pos* = (2)
  RNG_SR_SECS_Msk* = (0x00000001 shl RNG_SR_SECS_Pos) ## !< 0x00000004
  RNG_SR_SECS* = RNG_SR_SECS_Msk
  RNG_SR_CEIS_Pos* = (5)
  RNG_SR_CEIS_Msk* = (0x00000001 shl RNG_SR_CEIS_Pos) ## !< 0x00000020
  RNG_SR_CEIS* = RNG_SR_CEIS_Msk
  RNG_SR_SEIS_Pos* = (6)
  RNG_SR_SEIS_Msk* = (0x00000001 shl RNG_SR_SEIS_Pos) ## !< 0x00000040
  RNG_SR_SEIS* = RNG_SR_SEIS_Msk

## ****************************************************************************
##
##                            Real-Time Clock (RTC)
##
## ****************************************************************************
##
##  @brief Specific device feature definitions  (not present on all devices in the STM32F4 serie)
##

const
  RTC_TAMPER2_SUPPORT* = true   ## !< TAMPER 2 feature support

## *******************  Bits definition for RTC_TR register  ******************

const
  RTC_TR_PM_Pos* = (22)
  RTC_TR_PM_Msk* = (0x00000001 shl RTC_TR_PM_Pos) ## !< 0x00400000
  RTC_TR_PM* = RTC_TR_PM_Msk
  RTC_TR_HT_Pos* = (20)
  RTC_TR_HT_Msk* = (0x00000003 shl RTC_TR_HT_Pos) ## !< 0x00300000
  RTC_TR_HT* = RTC_TR_HT_Msk
  RTC_TR_HT_Bit0* = (0x00000001 shl RTC_TR_HT_Pos) ## !< 0x00100000
  RTC_TR_HT_Bit1* = (0x00000002 shl RTC_TR_HT_Pos) ## !< 0x00200000
  RTC_TR_HU_Pos* = (16)
  RTC_TR_HU_Msk* = (0x0000000F shl RTC_TR_HU_Pos) ## !< 0x000F0000
  RTC_TR_HU* = RTC_TR_HU_Msk
  RTC_TR_HU_Bit0* = (0x00000001 shl RTC_TR_HU_Pos) ## !< 0x00010000
  RTC_TR_HU_Bit1* = (0x00000002 shl RTC_TR_HU_Pos) ## !< 0x00020000
  RTC_TR_HU_Bit2* = (0x00000004 shl RTC_TR_HU_Pos) ## !< 0x00040000
  RTC_TR_HU_Bit3* = (0x00000008 shl RTC_TR_HU_Pos) ## !< 0x00080000
  RTC_TR_MNT_Pos* = (12)
  RTC_TR_MNT_Msk* = (0x00000007 shl RTC_TR_MNT_Pos) ## !< 0x00007000
  RTC_TR_MNT* = RTC_TR_MNT_Msk
  RTC_TR_MNT_Bit0* = (0x00000001 shl RTC_TR_MNT_Pos) ## !< 0x00001000
  RTC_TR_MNT_Bit1* = (0x00000002 shl RTC_TR_MNT_Pos) ## !< 0x00002000
  RTC_TR_MNT_Bit2* = (0x00000004 shl RTC_TR_MNT_Pos) ## !< 0x00004000
  RTC_TR_MNU_Pos* = (8)
  RTC_TR_MNU_Msk* = (0x0000000F shl RTC_TR_MNU_Pos) ## !< 0x00000F00
  RTC_TR_MNU* = RTC_TR_MNU_Msk
  RTC_TR_MNU_Bit0* = (0x00000001 shl RTC_TR_MNU_Pos) ## !< 0x00000100
  RTC_TR_MNU_Bit1* = (0x00000002 shl RTC_TR_MNU_Pos) ## !< 0x00000200
  RTC_TR_MNU_Bit2* = (0x00000004 shl RTC_TR_MNU_Pos) ## !< 0x00000400
  RTC_TR_MNU_Bit3* = (0x00000008 shl RTC_TR_MNU_Pos) ## !< 0x00000800
  RTC_TR_ST_Pos* = (4)
  RTC_TR_ST_Msk* = (0x00000007 shl RTC_TR_ST_Pos) ## !< 0x00000070
  RTC_TR_ST* = RTC_TR_ST_Msk
  RTC_TR_ST_Bit0* = (0x00000001 shl RTC_TR_ST_Pos) ## !< 0x00000010
  RTC_TR_ST_Bit1* = (0x00000002 shl RTC_TR_ST_Pos) ## !< 0x00000020
  RTC_TR_ST_Bit2* = (0x00000004 shl RTC_TR_ST_Pos) ## !< 0x00000040
  RTC_TR_SU_Pos* = (0)
  RTC_TR_SU_Msk* = (0x0000000F shl RTC_TR_SU_Pos) ## !< 0x0000000F
  RTC_TR_SU* = RTC_TR_SU_Msk
  RTC_TR_SU_Bit0* = (0x00000001 shl RTC_TR_SU_Pos) ## !< 0x00000001
  RTC_TR_SU_Bit1* = (0x00000002 shl RTC_TR_SU_Pos) ## !< 0x00000002
  RTC_TR_SU_Bit2* = (0x00000004 shl RTC_TR_SU_Pos) ## !< 0x00000004
  RTC_TR_SU_Bit3* = (0x00000008 shl RTC_TR_SU_Pos) ## !< 0x00000008

## *******************  Bits definition for RTC_DR register  ******************

const
  RTC_DR_YT_Pos* = (20)
  RTC_DR_YT_Msk* = (0x0000000F shl RTC_DR_YT_Pos) ## !< 0x00F00000
  RTC_DR_YT* = RTC_DR_YT_Msk
  RTC_DR_YT_Bit0* = (0x00000001 shl RTC_DR_YT_Pos) ## !< 0x00100000
  RTC_DR_YT_Bit1* = (0x00000002 shl RTC_DR_YT_Pos) ## !< 0x00200000
  RTC_DR_YT_Bit2* = (0x00000004 shl RTC_DR_YT_Pos) ## !< 0x00400000
  RTC_DR_YT_Bit3* = (0x00000008 shl RTC_DR_YT_Pos) ## !< 0x00800000
  RTC_DR_YU_Pos* = (16)
  RTC_DR_YU_Msk* = (0x0000000F shl RTC_DR_YU_Pos) ## !< 0x000F0000
  RTC_DR_YU* = RTC_DR_YU_Msk
  RTC_DR_YU_Bit0* = (0x00000001 shl RTC_DR_YU_Pos) ## !< 0x00010000
  RTC_DR_YU_Bit1* = (0x00000002 shl RTC_DR_YU_Pos) ## !< 0x00020000
  RTC_DR_YU_Bit2* = (0x00000004 shl RTC_DR_YU_Pos) ## !< 0x00040000
  RTC_DR_YU_Bit3* = (0x00000008 shl RTC_DR_YU_Pos) ## !< 0x00080000
  RTC_DR_WDU_Pos* = (13)
  RTC_DR_WDU_Msk* = (0x00000007 shl RTC_DR_WDU_Pos) ## !< 0x0000E000
  RTC_DR_WDU* = RTC_DR_WDU_Msk
  RTC_DR_WDU_Bit0* = (0x00000001 shl RTC_DR_WDU_Pos) ## !< 0x00002000
  RTC_DR_WDU_Bit1* = (0x00000002 shl RTC_DR_WDU_Pos) ## !< 0x00004000
  RTC_DR_WDU_Bit2* = (0x00000004 shl RTC_DR_WDU_Pos) ## !< 0x00008000
  RTC_DR_MT_Pos* = (12)
  RTC_DR_MT_Msk* = (0x00000001 shl RTC_DR_MT_Pos) ## !< 0x00001000
  RTC_DR_MT* = RTC_DR_MT_Msk
  RTC_DR_MU_Pos* = (8)
  RTC_DR_MU_Msk* = (0x0000000F shl RTC_DR_MU_Pos) ## !< 0x00000F00
  RTC_DR_MU* = RTC_DR_MU_Msk
  RTC_DR_MU_Bit0* = (0x00000001 shl RTC_DR_MU_Pos) ## !< 0x00000100
  RTC_DR_MU_Bit1* = (0x00000002 shl RTC_DR_MU_Pos) ## !< 0x00000200
  RTC_DR_MU_Bit2* = (0x00000004 shl RTC_DR_MU_Pos) ## !< 0x00000400
  RTC_DR_MU_Bit3* = (0x00000008 shl RTC_DR_MU_Pos) ## !< 0x00000800
  RTC_DR_DT_Pos* = (4)
  RTC_DR_DT_Msk* = (0x00000003 shl RTC_DR_DT_Pos) ## !< 0x00000030
  RTC_DR_DT* = RTC_DR_DT_Msk
  RTC_DR_DT_Bit0* = (0x00000001 shl RTC_DR_DT_Pos) ## !< 0x00000010
  RTC_DR_DT_Bit1* = (0x00000002 shl RTC_DR_DT_Pos) ## !< 0x00000020
  RTC_DR_DU_Pos* = (0)
  RTC_DR_DU_Msk* = (0x0000000F shl RTC_DR_DU_Pos) ## !< 0x0000000F
  RTC_DR_DU* = RTC_DR_DU_Msk
  RTC_DR_DU_Bit0* = (0x00000001 shl RTC_DR_DU_Pos) ## !< 0x00000001
  RTC_DR_DU_Bit1* = (0x00000002 shl RTC_DR_DU_Pos) ## !< 0x00000002
  RTC_DR_DU_Bit2* = (0x00000004 shl RTC_DR_DU_Pos) ## !< 0x00000004
  RTC_DR_DU_Bit3* = (0x00000008 shl RTC_DR_DU_Pos) ## !< 0x00000008

## *******************  Bits definition for RTC_CR register  ******************

const
  RTC_CR_COE_Pos* = (23)
  RTC_CR_COE_Msk* = (0x00000001 shl RTC_CR_COE_Pos) ## !< 0x00800000
  RTC_CR_COE* = RTC_CR_COE_Msk
  RTC_CR_OSEL_Pos* = (21)
  RTC_CR_OSEL_Msk* = (0x00000003 shl RTC_CR_OSEL_Pos) ## !< 0x00600000
  RTC_CR_OSEL* = RTC_CR_OSEL_Msk
  RTC_CR_OSEL_Bit0* = (0x00000001 shl RTC_CR_OSEL_Pos) ## !< 0x00200000
  RTC_CR_OSEL_Bit1* = (0x00000002 shl RTC_CR_OSEL_Pos) ## !< 0x00400000
  RTC_CR_POL_Pos* = (20)
  RTC_CR_POL_Msk* = (0x00000001 shl RTC_CR_POL_Pos) ## !< 0x00100000
  RTC_CR_POL* = RTC_CR_POL_Msk
  RTC_CR_COSEL_Pos* = (19)
  RTC_CR_COSEL_Msk* = (0x00000001 shl RTC_CR_COSEL_Pos) ## !< 0x00080000
  RTC_CR_COSEL* = RTC_CR_COSEL_Msk
  RTC_CR_BKP_Pos* = (18)
  RTC_CR_BKP_Msk* = (0x00000001 shl RTC_CR_BKP_Pos) ## !< 0x00040000
  RTC_CR_BKP* = RTC_CR_BKP_Msk
  RTC_CR_SUB1H_Pos* = (17)
  RTC_CR_SUB1H_Msk* = (0x00000001 shl RTC_CR_SUB1H_Pos) ## !< 0x00020000
  RTC_CR_SUB1H* = RTC_CR_SUB1H_Msk
  RTC_CR_ADD1H_Pos* = (16)
  RTC_CR_ADD1H_Msk* = (0x00000001 shl RTC_CR_ADD1H_Pos) ## !< 0x00010000
  RTC_CR_ADD1H* = RTC_CR_ADD1H_Msk
  RTC_CR_TSIE_Pos* = (15)
  RTC_CR_TSIE_Msk* = (0x00000001 shl RTC_CR_TSIE_Pos) ## !< 0x00008000
  RTC_CR_TSIE* = RTC_CR_TSIE_Msk
  RTC_CR_WUTIE_Pos* = (14)
  RTC_CR_WUTIE_Msk* = (0x00000001 shl RTC_CR_WUTIE_Pos) ## !< 0x00004000
  RTC_CR_WUTIE* = RTC_CR_WUTIE_Msk
  RTC_CR_ALRBIE_Pos* = (13)
  RTC_CR_ALRBIE_Msk* = (0x00000001 shl RTC_CR_ALRBIE_Pos) ## !< 0x00002000
  RTC_CR_ALRBIE* = RTC_CR_ALRBIE_Msk
  RTC_CR_ALRAIE_Pos* = (12)
  RTC_CR_ALRAIE_Msk* = (0x00000001 shl RTC_CR_ALRAIE_Pos) ## !< 0x00001000
  RTC_CR_ALRAIE* = RTC_CR_ALRAIE_Msk
  RTC_CR_TSE_Pos* = (11)
  RTC_CR_TSE_Msk* = (0x00000001 shl RTC_CR_TSE_Pos) ## !< 0x00000800
  RTC_CR_TSE* = RTC_CR_TSE_Msk
  RTC_CR_WUTE_Pos* = (10)
  RTC_CR_WUTE_Msk* = (0x00000001 shl RTC_CR_WUTE_Pos) ## !< 0x00000400
  RTC_CR_WUTE* = RTC_CR_WUTE_Msk
  RTC_CR_ALRBE_Pos* = (9)
  RTC_CR_ALRBE_Msk* = (0x00000001 shl RTC_CR_ALRBE_Pos) ## !< 0x00000200
  RTC_CR_ALRBE* = RTC_CR_ALRBE_Msk
  RTC_CR_ALRAE_Pos* = (8)
  RTC_CR_ALRAE_Msk* = (0x00000001 shl RTC_CR_ALRAE_Pos) ## !< 0x00000100
  RTC_CR_ALRAE* = RTC_CR_ALRAE_Msk
  RTC_CR_DCE_Pos* = (7)
  RTC_CR_DCE_Msk* = (0x00000001 shl RTC_CR_DCE_Pos) ## !< 0x00000080
  RTC_CR_DCE* = RTC_CR_DCE_Msk
  RTC_CR_FMT_Pos* = (6)
  RTC_CR_FMT_Msk* = (0x00000001 shl RTC_CR_FMT_Pos) ## !< 0x00000040
  RTC_CR_FMT* = RTC_CR_FMT_Msk
  RTC_CR_BYPSHAD_Pos* = (5)
  RTC_CR_BYPSHAD_Msk* = (0x00000001 shl RTC_CR_BYPSHAD_Pos) ## !< 0x00000020
  RTC_CR_BYPSHAD* = RTC_CR_BYPSHAD_Msk
  RTC_CR_REFCKON_Pos* = (4)
  RTC_CR_REFCKON_Msk* = (0x00000001 shl RTC_CR_REFCKON_Pos) ## !< 0x00000010
  RTC_CR_REFCKON* = RTC_CR_REFCKON_Msk
  RTC_CR_TSEDGE_Pos* = (3)
  RTC_CR_TSEDGE_Msk* = (0x00000001 shl RTC_CR_TSEDGE_Pos) ## !< 0x00000008
  RTC_CR_TSEDGE* = RTC_CR_TSEDGE_Msk
  RTC_CR_WUCKSEL_Pos* = (0)
  RTC_CR_WUCKSEL_Msk* = (0x00000007 shl RTC_CR_WUCKSEL_Pos) ## !< 0x00000007
  RTC_CR_WUCKSEL* = RTC_CR_WUCKSEL_Msk
  RTC_CR_WUCKSEL_Bit0* = (0x00000001 shl RTC_CR_WUCKSEL_Pos) ## !< 0x00000001
  RTC_CR_WUCKSEL_Bit1* = (0x00000002 shl RTC_CR_WUCKSEL_Pos) ## !< 0x00000002
  RTC_CR_WUCKSEL_Bit2* = (0x00000004 shl RTC_CR_WUCKSEL_Pos) ## !< 0x00000004

##  Legacy defines

const
  RTC_CR_BCK* = RTC_CR_BKP

## *******************  Bits definition for RTC_ISR register  *****************

const
  RTC_ISR_RECALPF_Pos* = (16)
  RTC_ISR_RECALPF_Msk* = (0x00000001 shl RTC_ISR_RECALPF_Pos) ## !< 0x00010000
  RTC_ISR_RECALPF* = RTC_ISR_RECALPF_Msk
  RTC_ISR_TAMP1F_Pos* = (13)
  RTC_ISR_TAMP1F_Msk* = (0x00000001 shl RTC_ISR_TAMP1F_Pos) ## !< 0x00002000
  RTC_ISR_TAMP1F* = RTC_ISR_TAMP1F_Msk
  RTC_ISR_TAMP2F_Pos* = (14)
  RTC_ISR_TAMP2F_Msk* = (0x00000001 shl RTC_ISR_TAMP2F_Pos) ## !< 0x00004000
  RTC_ISR_TAMP2F* = RTC_ISR_TAMP2F_Msk
  RTC_ISR_TSOVF_Pos* = (12)
  RTC_ISR_TSOVF_Msk* = (0x00000001 shl RTC_ISR_TSOVF_Pos) ## !< 0x00001000
  RTC_ISR_TSOVF* = RTC_ISR_TSOVF_Msk
  RTC_ISR_TSF_Pos* = (11)
  RTC_ISR_TSF_Msk* = (0x00000001 shl RTC_ISR_TSF_Pos) ## !< 0x00000800
  RTC_ISR_TSF* = RTC_ISR_TSF_Msk
  RTC_ISR_WUTF_Pos* = (10)
  RTC_ISR_WUTF_Msk* = (0x00000001 shl RTC_ISR_WUTF_Pos) ## !< 0x00000400
  RTC_ISR_WUTF* = RTC_ISR_WUTF_Msk
  RTC_ISR_ALRBF_Pos* = (9)
  RTC_ISR_ALRBF_Msk* = (0x00000001 shl RTC_ISR_ALRBF_Pos) ## !< 0x00000200
  RTC_ISR_ALRBF* = RTC_ISR_ALRBF_Msk
  RTC_ISR_ALRAF_Pos* = (8)
  RTC_ISR_ALRAF_Msk* = (0x00000001 shl RTC_ISR_ALRAF_Pos) ## !< 0x00000100
  RTC_ISR_ALRAF* = RTC_ISR_ALRAF_Msk
  RTC_ISR_INIT_Pos* = (7)
  RTC_ISR_INIT_Msk* = (0x00000001 shl RTC_ISR_INIT_Pos) ## !< 0x00000080
  RTC_ISR_INIT* = RTC_ISR_INIT_Msk
  RTC_ISR_INITF_Pos* = (6)
  RTC_ISR_INITF_Msk* = (0x00000001 shl RTC_ISR_INITF_Pos) ## !< 0x00000040
  RTC_ISR_INITF* = RTC_ISR_INITF_Msk
  RTC_ISR_RSF_Pos* = (5)
  RTC_ISR_RSF_Msk* = (0x00000001 shl RTC_ISR_RSF_Pos) ## !< 0x00000020
  RTC_ISR_RSF* = RTC_ISR_RSF_Msk
  RTC_ISR_INITS_Pos* = (4)
  RTC_ISR_INITS_Msk* = (0x00000001 shl RTC_ISR_INITS_Pos) ## !< 0x00000010
  RTC_ISR_INITS* = RTC_ISR_INITS_Msk
  RTC_ISR_SHPF_Pos* = (3)
  RTC_ISR_SHPF_Msk* = (0x00000001 shl RTC_ISR_SHPF_Pos) ## !< 0x00000008
  RTC_ISR_SHPF* = RTC_ISR_SHPF_Msk
  RTC_ISR_WUTWF_Pos* = (2)
  RTC_ISR_WUTWF_Msk* = (0x00000001 shl RTC_ISR_WUTWF_Pos) ## !< 0x00000004
  RTC_ISR_WUTWF* = RTC_ISR_WUTWF_Msk
  RTC_ISR_ALRBWF_Pos* = (1)
  RTC_ISR_ALRBWF_Msk* = (0x00000001 shl RTC_ISR_ALRBWF_Pos) ## !< 0x00000002
  RTC_ISR_ALRBWF* = RTC_ISR_ALRBWF_Msk
  RTC_ISR_ALRAWF_Pos* = (0)
  RTC_ISR_ALRAWF_Msk* = (0x00000001 shl RTC_ISR_ALRAWF_Pos) ## !< 0x00000001
  RTC_ISR_ALRAWF* = RTC_ISR_ALRAWF_Msk

## *******************  Bits definition for RTC_PRER register  ****************

const
  RTC_PRER_PREDIV_A_Pos* = (16)
  RTC_PRER_PREDIV_A_Msk* = (0x0000007F shl RTC_PRER_PREDIV_A_Pos) ## !< 0x007F0000
  RTC_PRER_PREDIV_A* = RTC_PRER_PREDIV_A_Msk
  RTC_PRER_PREDIV_S_Pos* = (0)
  RTC_PRER_PREDIV_S_Msk* = (0x00007FFF shl RTC_PRER_PREDIV_S_Pos) ## !< 0x00007FFF
  RTC_PRER_PREDIV_S* = RTC_PRER_PREDIV_S_Msk

## *******************  Bits definition for RTC_WUTR register  ****************

const
  RTC_WUTR_WUT_Pos* = (0)
  RTC_WUTR_WUT_Msk* = (0x0000FFFF shl RTC_WUTR_WUT_Pos) ## !< 0x0000FFFF
  RTC_WUTR_WUT* = RTC_WUTR_WUT_Msk

## *******************  Bits definition for RTC_CALIBR register  **************

const
  RTC_CALIBR_DCS_Pos* = (7)
  RTC_CALIBR_DCS_Msk* = (0x00000001 shl RTC_CALIBR_DCS_Pos) ## !< 0x00000080
  RTC_CALIBR_DCS* = RTC_CALIBR_DCS_Msk
  RTC_CALIBR_DC_Pos* = (0)
  RTC_CALIBR_DC_Msk* = (0x0000001F shl RTC_CALIBR_DC_Pos) ## !< 0x0000001F
  RTC_CALIBR_DC* = RTC_CALIBR_DC_Msk

## *******************  Bits definition for RTC_ALRMAR register  **************

const
  RTC_ALRMAR_MSK4_Pos* = (31)
  RTC_ALRMAR_MSK4_Msk* = (0x00000001 shl RTC_ALRMAR_MSK4_Pos) ## !< 0x80000000
  RTC_ALRMAR_MSK4* = RTC_ALRMAR_MSK4_Msk
  RTC_ALRMAR_WDSEL_Pos* = (30)
  RTC_ALRMAR_WDSEL_Msk* = (0x00000001 shl RTC_ALRMAR_WDSEL_Pos) ## !< 0x40000000
  RTC_ALRMAR_WDSEL* = RTC_ALRMAR_WDSEL_Msk
  RTC_ALRMAR_DT_Pos* = (28)
  RTC_ALRMAR_DT_Msk* = (0x00000003 shl RTC_ALRMAR_DT_Pos) ## !< 0x30000000
  RTC_ALRMAR_DT* = RTC_ALRMAR_DT_Msk
  RTC_ALRMAR_DT_Bit0* = (0x00000001 shl RTC_ALRMAR_DT_Pos) ## !< 0x10000000
  RTC_ALRMAR_DT_Bit1* = (0x00000002 shl RTC_ALRMAR_DT_Pos) ## !< 0x20000000
  RTC_ALRMAR_DU_Pos* = (24)
  RTC_ALRMAR_DU_Msk* = (0x0000000F shl RTC_ALRMAR_DU_Pos) ## !< 0x0F000000
  RTC_ALRMAR_DU* = RTC_ALRMAR_DU_Msk
  RTC_ALRMAR_DU_Bit0* = (0x00000001 shl RTC_ALRMAR_DU_Pos) ## !< 0x01000000
  RTC_ALRMAR_DU_Bit1* = (0x00000002 shl RTC_ALRMAR_DU_Pos) ## !< 0x02000000
  RTC_ALRMAR_DU_Bit2* = (0x00000004 shl RTC_ALRMAR_DU_Pos) ## !< 0x04000000
  RTC_ALRMAR_DU_Bit3* = (0x00000008 shl RTC_ALRMAR_DU_Pos) ## !< 0x08000000
  RTC_ALRMAR_MSK3_Pos* = (23)
  RTC_ALRMAR_MSK3_Msk* = (0x00000001 shl RTC_ALRMAR_MSK3_Pos) ## !< 0x00800000
  RTC_ALRMAR_MSK3* = RTC_ALRMAR_MSK3_Msk
  RTC_ALRMAR_PM_Pos* = (22)
  RTC_ALRMAR_PM_Msk* = (0x00000001 shl RTC_ALRMAR_PM_Pos) ## !< 0x00400000
  RTC_ALRMAR_PM* = RTC_ALRMAR_PM_Msk
  RTC_ALRMAR_HT_Pos* = (20)
  RTC_ALRMAR_HT_Msk* = (0x00000003 shl RTC_ALRMAR_HT_Pos) ## !< 0x00300000
  RTC_ALRMAR_HT* = RTC_ALRMAR_HT_Msk
  RTC_ALRMAR_HT_Bit0* = (0x00000001 shl RTC_ALRMAR_HT_Pos) ## !< 0x00100000
  RTC_ALRMAR_HT_Bit1* = (0x00000002 shl RTC_ALRMAR_HT_Pos) ## !< 0x00200000
  RTC_ALRMAR_HU_Pos* = (16)
  RTC_ALRMAR_HU_Msk* = (0x0000000F shl RTC_ALRMAR_HU_Pos) ## !< 0x000F0000
  RTC_ALRMAR_HU* = RTC_ALRMAR_HU_Msk
  RTC_ALRMAR_HU_Bit0* = (0x00000001 shl RTC_ALRMAR_HU_Pos) ## !< 0x00010000
  RTC_ALRMAR_HU_Bit1* = (0x00000002 shl RTC_ALRMAR_HU_Pos) ## !< 0x00020000
  RTC_ALRMAR_HU_Bit2* = (0x00000004 shl RTC_ALRMAR_HU_Pos) ## !< 0x00040000
  RTC_ALRMAR_HU_Bit3* = (0x00000008 shl RTC_ALRMAR_HU_Pos) ## !< 0x00080000
  RTC_ALRMAR_MSK2_Pos* = (15)
  RTC_ALRMAR_MSK2_Msk* = (0x00000001 shl RTC_ALRMAR_MSK2_Pos) ## !< 0x00008000
  RTC_ALRMAR_MSK2* = RTC_ALRMAR_MSK2_Msk
  RTC_ALRMAR_MNT_Pos* = (12)
  RTC_ALRMAR_MNT_Msk* = (0x00000007 shl RTC_ALRMAR_MNT_Pos) ## !< 0x00007000
  RTC_ALRMAR_MNT* = RTC_ALRMAR_MNT_Msk
  RTC_ALRMAR_MNT_Bit0* = (0x00000001 shl RTC_ALRMAR_MNT_Pos) ## !< 0x00001000
  RTC_ALRMAR_MNT_Bit1* = (0x00000002 shl RTC_ALRMAR_MNT_Pos) ## !< 0x00002000
  RTC_ALRMAR_MNT_Bit2* = (0x00000004 shl RTC_ALRMAR_MNT_Pos) ## !< 0x00004000
  RTC_ALRMAR_MNU_Pos* = (8)
  RTC_ALRMAR_MNU_Msk* = (0x0000000F shl RTC_ALRMAR_MNU_Pos) ## !< 0x00000F00
  RTC_ALRMAR_MNU* = RTC_ALRMAR_MNU_Msk
  RTC_ALRMAR_MNU_Bit0* = (0x00000001 shl RTC_ALRMAR_MNU_Pos) ## !< 0x00000100
  RTC_ALRMAR_MNU_Bit1* = (0x00000002 shl RTC_ALRMAR_MNU_Pos) ## !< 0x00000200
  RTC_ALRMAR_MNU_Bit2* = (0x00000004 shl RTC_ALRMAR_MNU_Pos) ## !< 0x00000400
  RTC_ALRMAR_MNU_Bit3* = (0x00000008 shl RTC_ALRMAR_MNU_Pos) ## !< 0x00000800
  RTC_ALRMAR_MSK1_Pos* = (7)
  RTC_ALRMAR_MSK1_Msk* = (0x00000001 shl RTC_ALRMAR_MSK1_Pos) ## !< 0x00000080
  RTC_ALRMAR_MSK1* = RTC_ALRMAR_MSK1_Msk
  RTC_ALRMAR_ST_Pos* = (4)
  RTC_ALRMAR_ST_Msk* = (0x00000007 shl RTC_ALRMAR_ST_Pos) ## !< 0x00000070
  RTC_ALRMAR_ST* = RTC_ALRMAR_ST_Msk
  RTC_ALRMAR_ST_Bit0* = (0x00000001 shl RTC_ALRMAR_ST_Pos) ## !< 0x00000010
  RTC_ALRMAR_ST_Bit1* = (0x00000002 shl RTC_ALRMAR_ST_Pos) ## !< 0x00000020
  RTC_ALRMAR_ST_Bit2* = (0x00000004 shl RTC_ALRMAR_ST_Pos) ## !< 0x00000040
  RTC_ALRMAR_SU_Pos* = (0)
  RTC_ALRMAR_SU_Msk* = (0x0000000F shl RTC_ALRMAR_SU_Pos) ## !< 0x0000000F
  RTC_ALRMAR_SU* = RTC_ALRMAR_SU_Msk
  RTC_ALRMAR_SU_Bit0* = (0x00000001 shl RTC_ALRMAR_SU_Pos) ## !< 0x00000001
  RTC_ALRMAR_SU_Bit1* = (0x00000002 shl RTC_ALRMAR_SU_Pos) ## !< 0x00000002
  RTC_ALRMAR_SU_Bit2* = (0x00000004 shl RTC_ALRMAR_SU_Pos) ## !< 0x00000004
  RTC_ALRMAR_SU_Bit3* = (0x00000008 shl RTC_ALRMAR_SU_Pos) ## !< 0x00000008

## *******************  Bits definition for RTC_ALRMBR register  **************

const
  RTC_ALRMBR_MSK4_Pos* = (31)
  RTC_ALRMBR_MSK4_Msk* = (0x00000001 shl RTC_ALRMBR_MSK4_Pos) ## !< 0x80000000
  RTC_ALRMBR_MSK4* = RTC_ALRMBR_MSK4_Msk
  RTC_ALRMBR_WDSEL_Pos* = (30)
  RTC_ALRMBR_WDSEL_Msk* = (0x00000001 shl RTC_ALRMBR_WDSEL_Pos) ## !< 0x40000000
  RTC_ALRMBR_WDSEL* = RTC_ALRMBR_WDSEL_Msk
  RTC_ALRMBR_DT_Pos* = (28)
  RTC_ALRMBR_DT_Msk* = (0x00000003 shl RTC_ALRMBR_DT_Pos) ## !< 0x30000000
  RTC_ALRMBR_DT* = RTC_ALRMBR_DT_Msk
  RTC_ALRMBR_DT_Bit0* = (0x00000001 shl RTC_ALRMBR_DT_Pos) ## !< 0x10000000
  RTC_ALRMBR_DT_Bit1* = (0x00000002 shl RTC_ALRMBR_DT_Pos) ## !< 0x20000000
  RTC_ALRMBR_DU_Pos* = (24)
  RTC_ALRMBR_DU_Msk* = (0x0000000F shl RTC_ALRMBR_DU_Pos) ## !< 0x0F000000
  RTC_ALRMBR_DU* = RTC_ALRMBR_DU_Msk
  RTC_ALRMBR_DU_Bit0* = (0x00000001 shl RTC_ALRMBR_DU_Pos) ## !< 0x01000000
  RTC_ALRMBR_DU_Bit1* = (0x00000002 shl RTC_ALRMBR_DU_Pos) ## !< 0x02000000
  RTC_ALRMBR_DU_Bit2* = (0x00000004 shl RTC_ALRMBR_DU_Pos) ## !< 0x04000000
  RTC_ALRMBR_DU_Bit3* = (0x00000008 shl RTC_ALRMBR_DU_Pos) ## !< 0x08000000
  RTC_ALRMBR_MSK3_Pos* = (23)
  RTC_ALRMBR_MSK3_Msk* = (0x00000001 shl RTC_ALRMBR_MSK3_Pos) ## !< 0x00800000
  RTC_ALRMBR_MSK3* = RTC_ALRMBR_MSK3_Msk
  RTC_ALRMBR_PM_Pos* = (22)
  RTC_ALRMBR_PM_Msk* = (0x00000001 shl RTC_ALRMBR_PM_Pos) ## !< 0x00400000
  RTC_ALRMBR_PM* = RTC_ALRMBR_PM_Msk
  RTC_ALRMBR_HT_Pos* = (20)
  RTC_ALRMBR_HT_Msk* = (0x00000003 shl RTC_ALRMBR_HT_Pos) ## !< 0x00300000
  RTC_ALRMBR_HT* = RTC_ALRMBR_HT_Msk
  RTC_ALRMBR_HT_Bit0* = (0x00000001 shl RTC_ALRMBR_HT_Pos) ## !< 0x00100000
  RTC_ALRMBR_HT_Bit1* = (0x00000002 shl RTC_ALRMBR_HT_Pos) ## !< 0x00200000
  RTC_ALRMBR_HU_Pos* = (16)
  RTC_ALRMBR_HU_Msk* = (0x0000000F shl RTC_ALRMBR_HU_Pos) ## !< 0x000F0000
  RTC_ALRMBR_HU* = RTC_ALRMBR_HU_Msk
  RTC_ALRMBR_HU_Bit0* = (0x00000001 shl RTC_ALRMBR_HU_Pos) ## !< 0x00010000
  RTC_ALRMBR_HU_Bit1* = (0x00000002 shl RTC_ALRMBR_HU_Pos) ## !< 0x00020000
  RTC_ALRMBR_HU_Bit2* = (0x00000004 shl RTC_ALRMBR_HU_Pos) ## !< 0x00040000
  RTC_ALRMBR_HU_Bit3* = (0x00000008 shl RTC_ALRMBR_HU_Pos) ## !< 0x00080000
  RTC_ALRMBR_MSK2_Pos* = (15)
  RTC_ALRMBR_MSK2_Msk* = (0x00000001 shl RTC_ALRMBR_MSK2_Pos) ## !< 0x00008000
  RTC_ALRMBR_MSK2* = RTC_ALRMBR_MSK2_Msk
  RTC_ALRMBR_MNT_Pos* = (12)
  RTC_ALRMBR_MNT_Msk* = (0x00000007 shl RTC_ALRMBR_MNT_Pos) ## !< 0x00007000
  RTC_ALRMBR_MNT* = RTC_ALRMBR_MNT_Msk
  RTC_ALRMBR_MNT_Bit0* = (0x00000001 shl RTC_ALRMBR_MNT_Pos) ## !< 0x00001000
  RTC_ALRMBR_MNT_Bit1* = (0x00000002 shl RTC_ALRMBR_MNT_Pos) ## !< 0x00002000
  RTC_ALRMBR_MNT_Bit2* = (0x00000004 shl RTC_ALRMBR_MNT_Pos) ## !< 0x00004000
  RTC_ALRMBR_MNU_Pos* = (8)
  RTC_ALRMBR_MNU_Msk* = (0x0000000F shl RTC_ALRMBR_MNU_Pos) ## !< 0x00000F00
  RTC_ALRMBR_MNU* = RTC_ALRMBR_MNU_Msk
  RTC_ALRMBR_MNU_Bit0* = (0x00000001 shl RTC_ALRMBR_MNU_Pos) ## !< 0x00000100
  RTC_ALRMBR_MNU_Bit1* = (0x00000002 shl RTC_ALRMBR_MNU_Pos) ## !< 0x00000200
  RTC_ALRMBR_MNU_Bit2* = (0x00000004 shl RTC_ALRMBR_MNU_Pos) ## !< 0x00000400
  RTC_ALRMBR_MNU_Bit3* = (0x00000008 shl RTC_ALRMBR_MNU_Pos) ## !< 0x00000800
  RTC_ALRMBR_MSK1_Pos* = (7)
  RTC_ALRMBR_MSK1_Msk* = (0x00000001 shl RTC_ALRMBR_MSK1_Pos) ## !< 0x00000080
  RTC_ALRMBR_MSK1* = RTC_ALRMBR_MSK1_Msk
  RTC_ALRMBR_ST_Pos* = (4)
  RTC_ALRMBR_ST_Msk* = (0x00000007 shl RTC_ALRMBR_ST_Pos) ## !< 0x00000070
  RTC_ALRMBR_ST* = RTC_ALRMBR_ST_Msk
  RTC_ALRMBR_ST_Bit0* = (0x00000001 shl RTC_ALRMBR_ST_Pos) ## !< 0x00000010
  RTC_ALRMBR_ST_Bit1* = (0x00000002 shl RTC_ALRMBR_ST_Pos) ## !< 0x00000020
  RTC_ALRMBR_ST_Bit2* = (0x00000004 shl RTC_ALRMBR_ST_Pos) ## !< 0x00000040
  RTC_ALRMBR_SU_Pos* = (0)
  RTC_ALRMBR_SU_Msk* = (0x0000000F shl RTC_ALRMBR_SU_Pos) ## !< 0x0000000F
  RTC_ALRMBR_SU* = RTC_ALRMBR_SU_Msk
  RTC_ALRMBR_SU_Bit0* = (0x00000001 shl RTC_ALRMBR_SU_Pos) ## !< 0x00000001
  RTC_ALRMBR_SU_Bit1* = (0x00000002 shl RTC_ALRMBR_SU_Pos) ## !< 0x00000002
  RTC_ALRMBR_SU_Bit2* = (0x00000004 shl RTC_ALRMBR_SU_Pos) ## !< 0x00000004
  RTC_ALRMBR_SU_Bit3* = (0x00000008 shl RTC_ALRMBR_SU_Pos) ## !< 0x00000008

## *******************  Bits definition for RTC_WPR register  *****************

const
  RTC_WPR_KEY_Pos* = (0)
  RTC_WPR_KEY_Msk* = (0x000000FF shl RTC_WPR_KEY_Pos) ## !< 0x000000FF
  RTC_WPR_KEY* = RTC_WPR_KEY_Msk

## *******************  Bits definition for RTC_SSR register  *****************

const
  RTC_SSR_SS_Pos* = (0)
  RTC_SSR_SS_Msk* = (0x0000FFFF shl RTC_SSR_SS_Pos) ## !< 0x0000FFFF
  RTC_SSR_SS* = RTC_SSR_SS_Msk

## *******************  Bits definition for RTC_SHIFTR register  **************

const
  RTC_SHIFTR_SUBFS_Pos* = (0)
  RTC_SHIFTR_SUBFS_Msk* = (0x00007FFF shl RTC_SHIFTR_SUBFS_Pos) ## !< 0x00007FFF
  RTC_SHIFTR_SUBFS* = RTC_SHIFTR_SUBFS_Msk
  RTC_SHIFTR_ADD1S_Pos* = (31)
  RTC_SHIFTR_ADD1S_Msk* = (0x00000001 shl RTC_SHIFTR_ADD1S_Pos) ## !< 0x80000000
  RTC_SHIFTR_ADD1S* = RTC_SHIFTR_ADD1S_Msk

## *******************  Bits definition for RTC_TSTR register  ****************

const
  RTC_TSTR_PM_Pos* = (22)
  RTC_TSTR_PM_Msk* = (0x00000001 shl RTC_TSTR_PM_Pos) ## !< 0x00400000
  RTC_TSTR_PM* = RTC_TSTR_PM_Msk
  RTC_TSTR_HT_Pos* = (20)
  RTC_TSTR_HT_Msk* = (0x00000003 shl RTC_TSTR_HT_Pos) ## !< 0x00300000
  RTC_TSTR_HT* = RTC_TSTR_HT_Msk
  RTC_TSTR_HT_Bit0* = (0x00000001 shl RTC_TSTR_HT_Pos) ## !< 0x00100000
  RTC_TSTR_HT_Bit1* = (0x00000002 shl RTC_TSTR_HT_Pos) ## !< 0x00200000
  RTC_TSTR_HU_Pos* = (16)
  RTC_TSTR_HU_Msk* = (0x0000000F shl RTC_TSTR_HU_Pos) ## !< 0x000F0000
  RTC_TSTR_HU* = RTC_TSTR_HU_Msk
  RTC_TSTR_HU_Bit0* = (0x00000001 shl RTC_TSTR_HU_Pos) ## !< 0x00010000
  RTC_TSTR_HU_Bit1* = (0x00000002 shl RTC_TSTR_HU_Pos) ## !< 0x00020000
  RTC_TSTR_HU_Bit2* = (0x00000004 shl RTC_TSTR_HU_Pos) ## !< 0x00040000
  RTC_TSTR_HU_Bit3* = (0x00000008 shl RTC_TSTR_HU_Pos) ## !< 0x00080000
  RTC_TSTR_MNT_Pos* = (12)
  RTC_TSTR_MNT_Msk* = (0x00000007 shl RTC_TSTR_MNT_Pos) ## !< 0x00007000
  RTC_TSTR_MNT* = RTC_TSTR_MNT_Msk
  RTC_TSTR_MNT_Bit0* = (0x00000001 shl RTC_TSTR_MNT_Pos) ## !< 0x00001000
  RTC_TSTR_MNT_Bit1* = (0x00000002 shl RTC_TSTR_MNT_Pos) ## !< 0x00002000
  RTC_TSTR_MNT_Bit2* = (0x00000004 shl RTC_TSTR_MNT_Pos) ## !< 0x00004000
  RTC_TSTR_MNU_Pos* = (8)
  RTC_TSTR_MNU_Msk* = (0x0000000F shl RTC_TSTR_MNU_Pos) ## !< 0x00000F00
  RTC_TSTR_MNU* = RTC_TSTR_MNU_Msk
  RTC_TSTR_MNU_Bit0* = (0x00000001 shl RTC_TSTR_MNU_Pos) ## !< 0x00000100
  RTC_TSTR_MNU_Bit1* = (0x00000002 shl RTC_TSTR_MNU_Pos) ## !< 0x00000200
  RTC_TSTR_MNU_Bit2* = (0x00000004 shl RTC_TSTR_MNU_Pos) ## !< 0x00000400
  RTC_TSTR_MNU_Bit3* = (0x00000008 shl RTC_TSTR_MNU_Pos) ## !< 0x00000800
  RTC_TSTR_ST_Pos* = (4)
  RTC_TSTR_ST_Msk* = (0x00000007 shl RTC_TSTR_ST_Pos) ## !< 0x00000070
  RTC_TSTR_ST* = RTC_TSTR_ST_Msk
  RTC_TSTR_ST_Bit0* = (0x00000001 shl RTC_TSTR_ST_Pos) ## !< 0x00000010
  RTC_TSTR_ST_Bit1* = (0x00000002 shl RTC_TSTR_ST_Pos) ## !< 0x00000020
  RTC_TSTR_ST_Bit2* = (0x00000004 shl RTC_TSTR_ST_Pos) ## !< 0x00000040
  RTC_TSTR_SU_Pos* = (0)
  RTC_TSTR_SU_Msk* = (0x0000000F shl RTC_TSTR_SU_Pos) ## !< 0x0000000F
  RTC_TSTR_SU* = RTC_TSTR_SU_Msk
  RTC_TSTR_SU_Bit0* = (0x00000001 shl RTC_TSTR_SU_Pos) ## !< 0x00000001
  RTC_TSTR_SU_Bit1* = (0x00000002 shl RTC_TSTR_SU_Pos) ## !< 0x00000002
  RTC_TSTR_SU_Bit2* = (0x00000004 shl RTC_TSTR_SU_Pos) ## !< 0x00000004
  RTC_TSTR_SU_Bit3* = (0x00000008 shl RTC_TSTR_SU_Pos) ## !< 0x00000008

## *******************  Bits definition for RTC_TSDR register  ****************

const
  RTC_TSDR_WDU_Pos* = (13)
  RTC_TSDR_WDU_Msk* = (0x00000007 shl RTC_TSDR_WDU_Pos) ## !< 0x0000E000
  RTC_TSDR_WDU* = RTC_TSDR_WDU_Msk
  RTC_TSDR_WDU_Bit0* = (0x00000001 shl RTC_TSDR_WDU_Pos) ## !< 0x00002000
  RTC_TSDR_WDU_Bit1* = (0x00000002 shl RTC_TSDR_WDU_Pos) ## !< 0x00004000
  RTC_TSDR_WDU_Bit2* = (0x00000004 shl RTC_TSDR_WDU_Pos) ## !< 0x00008000
  RTC_TSDR_MT_Pos* = (12)
  RTC_TSDR_MT_Msk* = (0x00000001 shl RTC_TSDR_MT_Pos) ## !< 0x00001000
  RTC_TSDR_MT* = RTC_TSDR_MT_Msk
  RTC_TSDR_MU_Pos* = (8)
  RTC_TSDR_MU_Msk* = (0x0000000F shl RTC_TSDR_MU_Pos) ## !< 0x00000F00
  RTC_TSDR_MU* = RTC_TSDR_MU_Msk
  RTC_TSDR_MU_Bit0* = (0x00000001 shl RTC_TSDR_MU_Pos) ## !< 0x00000100
  RTC_TSDR_MU_Bit1* = (0x00000002 shl RTC_TSDR_MU_Pos) ## !< 0x00000200
  RTC_TSDR_MU_Bit2* = (0x00000004 shl RTC_TSDR_MU_Pos) ## !< 0x00000400
  RTC_TSDR_MU_Bit3* = (0x00000008 shl RTC_TSDR_MU_Pos) ## !< 0x00000800
  RTC_TSDR_DT_Pos* = (4)
  RTC_TSDR_DT_Msk* = (0x00000003 shl RTC_TSDR_DT_Pos) ## !< 0x00000030
  RTC_TSDR_DT* = RTC_TSDR_DT_Msk
  RTC_TSDR_DT_Bit0* = (0x00000001 shl RTC_TSDR_DT_Pos) ## !< 0x00000010
  RTC_TSDR_DT_Bit1* = (0x00000002 shl RTC_TSDR_DT_Pos) ## !< 0x00000020
  RTC_TSDR_DU_Pos* = (0)
  RTC_TSDR_DU_Msk* = (0x0000000F shl RTC_TSDR_DU_Pos) ## !< 0x0000000F
  RTC_TSDR_DU* = RTC_TSDR_DU_Msk
  RTC_TSDR_DU_Bit0* = (0x00000001 shl RTC_TSDR_DU_Pos) ## !< 0x00000001
  RTC_TSDR_DU_Bit1* = (0x00000002 shl RTC_TSDR_DU_Pos) ## !< 0x00000002
  RTC_TSDR_DU_Bit2* = (0x00000004 shl RTC_TSDR_DU_Pos) ## !< 0x00000004
  RTC_TSDR_DU_Bit3* = (0x00000008 shl RTC_TSDR_DU_Pos) ## !< 0x00000008

## *******************  Bits definition for RTC_TSSSR register  ***************

const
  RTC_TSSSR_SS_Pos* = (0)
  RTC_TSSSR_SS_Msk* = (0x0000FFFF shl RTC_TSSSR_SS_Pos) ## !< 0x0000FFFF
  RTC_TSSSR_SS* = RTC_TSSSR_SS_Msk

## *******************  Bits definition for RTC_CAL register  ****************

const
  RTC_CALR_CALP_Pos* = (15)
  RTC_CALR_CALP_Msk* = (0x00000001 shl RTC_CALR_CALP_Pos) ## !< 0x00008000
  RTC_CALR_CALP* = RTC_CALR_CALP_Msk
  RTC_CALR_CALW8_Pos* = (14)
  RTC_CALR_CALW8_Msk* = (0x00000001 shl RTC_CALR_CALW8_Pos) ## !< 0x00004000
  RTC_CALR_CALW8* = RTC_CALR_CALW8_Msk
  RTC_CALR_CALW16_Pos* = (13)
  RTC_CALR_CALW16_Msk* = (0x00000001 shl RTC_CALR_CALW16_Pos) ## !< 0x00002000
  RTC_CALR_CALW16* = RTC_CALR_CALW16_Msk
  RTC_CALR_CALM_Pos* = (0)
  RTC_CALR_CALM_Msk* = (0x000001FF shl RTC_CALR_CALM_Pos) ## !< 0x000001FF
  RTC_CALR_CALM* = RTC_CALR_CALM_Msk
  RTC_CALR_CALM_Bit0* = (0x00000001 shl RTC_CALR_CALM_Pos) ## !< 0x00000001
  RTC_CALR_CALM_Bit1* = (0x00000002 shl RTC_CALR_CALM_Pos) ## !< 0x00000002
  RTC_CALR_CALM_Bit2* = (0x00000004 shl RTC_CALR_CALM_Pos) ## !< 0x00000004
  RTC_CALR_CALM_Bit3* = (0x00000008 shl RTC_CALR_CALM_Pos) ## !< 0x00000008
  RTC_CALR_CALM_Bit4* = (0x00000010 shl RTC_CALR_CALM_Pos) ## !< 0x00000010
  RTC_CALR_CALM_Bit5* = (0x00000020 shl RTC_CALR_CALM_Pos) ## !< 0x00000020
  RTC_CALR_CALM_Bit6* = (0x00000040 shl RTC_CALR_CALM_Pos) ## !< 0x00000040
  RTC_CALR_CALM_Bit7* = (0x00000080 shl RTC_CALR_CALM_Pos) ## !< 0x00000080
  RTC_CALR_CALM_Bit8* = (0x00000100 shl RTC_CALR_CALM_Pos) ## !< 0x00000100

## *******************  Bits definition for RTC_TAFCR register  ***************

const
  RTC_TAFCR_ALARMOUTTYPE_Pos* = (18)
  RTC_TAFCR_ALARMOUTTYPE_Msk* = (0x00000001 shl RTC_TAFCR_ALARMOUTTYPE_Pos) ## !< 0x00040000
  RTC_TAFCR_ALARMOUTTYPE* = RTC_TAFCR_ALARMOUTTYPE_Msk
  RTC_TAFCR_TSINSEL_Pos* = (17)
  RTC_TAFCR_TSINSEL_Msk* = (0x00000001 shl RTC_TAFCR_TSINSEL_Pos) ## !< 0x00020000
  RTC_TAFCR_TSINSEL* = RTC_TAFCR_TSINSEL_Msk
  RTC_TAFCR_TAMP1INSEL_Pos* = (16)
  RTC_TAFCR_TAMP1INSEL_Msk* = (0x00000001 shl RTC_TAFCR_TAMP1INSEL_Pos) ## !< 0x00010000
  RTC_TAFCR_TAMP1INSEL* = RTC_TAFCR_TAMP1INSEL_Msk
  RTC_TAFCR_TAMPPUDIS_Pos* = (15)
  RTC_TAFCR_TAMPPUDIS_Msk* = (0x00000001 shl RTC_TAFCR_TAMPPUDIS_Pos) ## !< 0x00008000
  RTC_TAFCR_TAMPPUDIS* = RTC_TAFCR_TAMPPUDIS_Msk
  RTC_TAFCR_TAMPPRCH_Pos* = (13)
  RTC_TAFCR_TAMPPRCH_Msk* = (0x00000003 shl RTC_TAFCR_TAMPPRCH_Pos) ## !< 0x00006000
  RTC_TAFCR_TAMPPRCH* = RTC_TAFCR_TAMPPRCH_Msk
  RTC_TAFCR_TAMPPRCH_Bit0* = (0x00000001 shl RTC_TAFCR_TAMPPRCH_Pos) ## !< 0x00002000
  RTC_TAFCR_TAMPPRCH_Bit1* = (0x00000002 shl RTC_TAFCR_TAMPPRCH_Pos) ## !< 0x00004000
  RTC_TAFCR_TAMPFLT_Pos* = (11)
  RTC_TAFCR_TAMPFLT_Msk* = (0x00000003 shl RTC_TAFCR_TAMPFLT_Pos) ## !< 0x00001800
  RTC_TAFCR_TAMPFLT* = RTC_TAFCR_TAMPFLT_Msk
  RTC_TAFCR_TAMPFLT_Bit0* = (0x00000001 shl RTC_TAFCR_TAMPFLT_Pos) ## !< 0x00000800
  RTC_TAFCR_TAMPFLT_Bit1* = (0x00000002 shl RTC_TAFCR_TAMPFLT_Pos) ## !< 0x00001000
  RTC_TAFCR_TAMPFREQ_Pos* = (8)
  RTC_TAFCR_TAMPFREQ_Msk* = (0x00000007 shl RTC_TAFCR_TAMPFREQ_Pos) ## !< 0x00000700
  RTC_TAFCR_TAMPFREQ* = RTC_TAFCR_TAMPFREQ_Msk
  RTC_TAFCR_TAMPFREQ_Bit0* = (0x00000001 shl RTC_TAFCR_TAMPFREQ_Pos) ## !< 0x00000100
  RTC_TAFCR_TAMPFREQ_Bit1* = (0x00000002 shl RTC_TAFCR_TAMPFREQ_Pos) ## !< 0x00000200
  RTC_TAFCR_TAMPFREQ_Bit2* = (0x00000004 shl RTC_TAFCR_TAMPFREQ_Pos) ## !< 0x00000400
  RTC_TAFCR_TAMPTS_Pos* = (7)
  RTC_TAFCR_TAMPTS_Msk* = (0x00000001 shl RTC_TAFCR_TAMPTS_Pos) ## !< 0x00000080
  RTC_TAFCR_TAMPTS* = RTC_TAFCR_TAMPTS_Msk
  RTC_TAFCR_TAMP2TRG_Pos* = (4)
  RTC_TAFCR_TAMP2TRG_Msk* = (0x00000001 shl RTC_TAFCR_TAMP2TRG_Pos) ## !< 0x00000010
  RTC_TAFCR_TAMP2TRG* = RTC_TAFCR_TAMP2TRG_Msk
  RTC_TAFCR_TAMP2E_Pos* = (3)
  RTC_TAFCR_TAMP2E_Msk* = (0x00000001 shl RTC_TAFCR_TAMP2E_Pos) ## !< 0x00000008
  RTC_TAFCR_TAMP2E* = RTC_TAFCR_TAMP2E_Msk
  RTC_TAFCR_TAMPIE_Pos* = (2)
  RTC_TAFCR_TAMPIE_Msk* = (0x00000001 shl RTC_TAFCR_TAMPIE_Pos) ## !< 0x00000004
  RTC_TAFCR_TAMPIE* = RTC_TAFCR_TAMPIE_Msk
  RTC_TAFCR_TAMP1TRG_Pos* = (1)
  RTC_TAFCR_TAMP1TRG_Msk* = (0x00000001 shl RTC_TAFCR_TAMP1TRG_Pos) ## !< 0x00000002
  RTC_TAFCR_TAMP1TRG* = RTC_TAFCR_TAMP1TRG_Msk
  RTC_TAFCR_TAMP1E_Pos* = (0)
  RTC_TAFCR_TAMP1E_Msk* = (0x00000001 shl RTC_TAFCR_TAMP1E_Pos) ## !< 0x00000001
  RTC_TAFCR_TAMP1E* = RTC_TAFCR_TAMP1E_Msk

##  Legacy defines

const
  RTC_TAFCR_TAMPINSEL* = RTC_TAFCR_TAMP1INSEL

## *******************  Bits definition for RTC_ALRMASSR register  ************

const
  RTC_ALRMASSR_MASKSS_Pos* = (24)
  RTC_ALRMASSR_MASKSS_Msk* = (0x0000000F shl RTC_ALRMASSR_MASKSS_Pos) ## !< 0x0F000000
  RTC_ALRMASSR_MASKSS* = RTC_ALRMASSR_MASKSS_Msk
  RTC_ALRMASSR_MASKSS_Bit0* = (0x00000001 shl RTC_ALRMASSR_MASKSS_Pos) ## !< 0x01000000
  RTC_ALRMASSR_MASKSS_Bit1* = (0x00000002 shl RTC_ALRMASSR_MASKSS_Pos) ## !< 0x02000000
  RTC_ALRMASSR_MASKSS_Bit2* = (0x00000004 shl RTC_ALRMASSR_MASKSS_Pos) ## !< 0x04000000
  RTC_ALRMASSR_MASKSS_Bit3* = (0x00000008 shl RTC_ALRMASSR_MASKSS_Pos) ## !< 0x08000000
  RTC_ALRMASSR_SS_Pos* = (0)
  RTC_ALRMASSR_SS_Msk* = (0x00007FFF shl RTC_ALRMASSR_SS_Pos) ## !< 0x00007FFF
  RTC_ALRMASSR_SS* = RTC_ALRMASSR_SS_Msk

## *******************  Bits definition for RTC_ALRMBSSR register  ************

const
  RTC_ALRMBSSR_MASKSS_Pos* = (24)
  RTC_ALRMBSSR_MASKSS_Msk* = (0x0000000F shl RTC_ALRMBSSR_MASKSS_Pos) ## !< 0x0F000000
  RTC_ALRMBSSR_MASKSS* = RTC_ALRMBSSR_MASKSS_Msk
  RTC_ALRMBSSR_MASKSS_Bit0* = (0x00000001 shl RTC_ALRMBSSR_MASKSS_Pos) ## !< 0x01000000
  RTC_ALRMBSSR_MASKSS_Bit1* = (0x00000002 shl RTC_ALRMBSSR_MASKSS_Pos) ## !< 0x02000000
  RTC_ALRMBSSR_MASKSS_Bit2* = (0x00000004 shl RTC_ALRMBSSR_MASKSS_Pos) ## !< 0x04000000
  RTC_ALRMBSSR_MASKSS_Bit3* = (0x00000008 shl RTC_ALRMBSSR_MASKSS_Pos) ## !< 0x08000000
  RTC_ALRMBSSR_SS_Pos* = (0)
  RTC_ALRMBSSR_SS_Msk* = (0x00007FFF shl RTC_ALRMBSSR_SS_Pos) ## !< 0x00007FFF
  RTC_ALRMBSSR_SS* = RTC_ALRMBSSR_SS_Msk

## *******************  Bits definition for RTC_BKP0R register  ***************

const
  RTC_BKP0R_Pos* = (0)
  RTC_BKP0R_Msk* = (0xFFFFFFFF shl RTC_BKP0R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP0R* = RTC_BKP0R_Msk

## *******************  Bits definition for RTC_BKP1R register  ***************

const
  RTC_BKP1R_Pos* = (0)
  RTC_BKP1R_Msk* = (0xFFFFFFFF shl RTC_BKP1R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP1R* = RTC_BKP1R_Msk

## *******************  Bits definition for RTC_BKP2R register  ***************

const
  RTC_BKP2R_Pos* = (0)
  RTC_BKP2R_Msk* = (0xFFFFFFFF shl RTC_BKP2R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP2R* = RTC_BKP2R_Msk

## *******************  Bits definition for RTC_BKP3R register  ***************

const
  RTC_BKP3R_Pos* = (0)
  RTC_BKP3R_Msk* = (0xFFFFFFFF shl RTC_BKP3R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP3R* = RTC_BKP3R_Msk

## *******************  Bits definition for RTC_BKP4R register  ***************

const
  RTC_BKP4R_Pos* = (0)
  RTC_BKP4R_Msk* = (0xFFFFFFFF shl RTC_BKP4R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP4R* = RTC_BKP4R_Msk

## *******************  Bits definition for RTC_BKP5R register  ***************

const
  RTC_BKP5R_Pos* = (0)
  RTC_BKP5R_Msk* = (0xFFFFFFFF shl RTC_BKP5R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP5R* = RTC_BKP5R_Msk

## *******************  Bits definition for RTC_BKP6R register  ***************

const
  RTC_BKP6R_Pos* = (0)
  RTC_BKP6R_Msk* = (0xFFFFFFFF shl RTC_BKP6R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP6R* = RTC_BKP6R_Msk

## *******************  Bits definition for RTC_BKP7R register  ***************

const
  RTC_BKP7R_Pos* = (0)
  RTC_BKP7R_Msk* = (0xFFFFFFFF shl RTC_BKP7R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP7R* = RTC_BKP7R_Msk

## *******************  Bits definition for RTC_BKP8R register  ***************

const
  RTC_BKP8R_Pos* = (0)
  RTC_BKP8R_Msk* = (0xFFFFFFFF shl RTC_BKP8R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP8R* = RTC_BKP8R_Msk

## *******************  Bits definition for RTC_BKP9R register  ***************

const
  RTC_BKP9R_Pos* = (0)
  RTC_BKP9R_Msk* = (0xFFFFFFFF shl RTC_BKP9R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP9R* = RTC_BKP9R_Msk

## *******************  Bits definition for RTC_BKP10R register  **************

const
  RTC_BKP10R_Pos* = (0)
  RTC_BKP10R_Msk* = (0xFFFFFFFF shl RTC_BKP10R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP10R* = RTC_BKP10R_Msk

## *******************  Bits definition for RTC_BKP11R register  **************

const
  RTC_BKP11R_Pos* = (0)
  RTC_BKP11R_Msk* = (0xFFFFFFFF shl RTC_BKP11R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP11R* = RTC_BKP11R_Msk

## *******************  Bits definition for RTC_BKP12R register  **************

const
  RTC_BKP12R_Pos* = (0)
  RTC_BKP12R_Msk* = (0xFFFFFFFF shl RTC_BKP12R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP12R* = RTC_BKP12R_Msk

## *******************  Bits definition for RTC_BKP13R register  **************

const
  RTC_BKP13R_Pos* = (0)
  RTC_BKP13R_Msk* = (0xFFFFFFFF shl RTC_BKP13R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP13R* = RTC_BKP13R_Msk

## *******************  Bits definition for RTC_BKP14R register  **************

const
  RTC_BKP14R_Pos* = (0)
  RTC_BKP14R_Msk* = (0xFFFFFFFF shl RTC_BKP14R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP14R* = RTC_BKP14R_Msk

## *******************  Bits definition for RTC_BKP15R register  **************

const
  RTC_BKP15R_Pos* = (0)
  RTC_BKP15R_Msk* = (0xFFFFFFFF shl RTC_BKP15R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP15R* = RTC_BKP15R_Msk

## *******************  Bits definition for RTC_BKP16R register  **************

const
  RTC_BKP16R_Pos* = (0)
  RTC_BKP16R_Msk* = (0xFFFFFFFF shl RTC_BKP16R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP16R* = RTC_BKP16R_Msk

## *******************  Bits definition for RTC_BKP17R register  **************

const
  RTC_BKP17R_Pos* = (0)
  RTC_BKP17R_Msk* = (0xFFFFFFFF shl RTC_BKP17R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP17R* = RTC_BKP17R_Msk

## *******************  Bits definition for RTC_BKP18R register  **************

const
  RTC_BKP18R_Pos* = (0)
  RTC_BKP18R_Msk* = (0xFFFFFFFF shl RTC_BKP18R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP18R* = RTC_BKP18R_Msk

## *******************  Bits definition for RTC_BKP19R register  **************

const
  RTC_BKP19R_Pos* = (0)
  RTC_BKP19R_Msk* = (0xFFFFFFFF shl RTC_BKP19R_Pos) ## !< 0xFFFFFFFF
  RTC_BKP19R* = RTC_BKP19R_Msk

## ******************* Number of backup registers *****************************

const
  RTC_BKP_NUMBER* = 0x0000000000000000'i64

## ****************************************************************************
##
##                         Serial Peripheral Interface
##
## ****************************************************************************
## ******************  Bit definition for SPI_CR1 register  *******************

const
  SPI_CR1_CPHA_Pos* = (0)
  SPI_CR1_CPHA_Msk* = (0x00000001 shl SPI_CR1_CPHA_Pos) ## !< 0x00000001
  SPI_CR1_CPHA* = SPI_CR1_CPHA_Msk
  SPI_CR1_CPOL_Pos* = (1)
  SPI_CR1_CPOL_Msk* = (0x00000001 shl SPI_CR1_CPOL_Pos) ## !< 0x00000002
  SPI_CR1_CPOL* = SPI_CR1_CPOL_Msk
  SPI_CR1_MSTR_Pos* = (2)
  SPI_CR1_MSTR_Msk* = (0x00000001 shl SPI_CR1_MSTR_Pos) ## !< 0x00000004
  SPI_CR1_MSTR* = SPI_CR1_MSTR_Msk
  SPI_CR1_BR_Pos* = (3)
  SPI_CR1_BR_Msk* = (0x00000007 shl SPI_CR1_BR_Pos) ## !< 0x00000038
  SPI_CR1_BR* = SPI_CR1_BR_Msk
  SPI_CR1_BR_Bit0* = (0x00000001 shl SPI_CR1_BR_Pos) ## !< 0x00000008
  SPI_CR1_BR_Bit1* = (0x00000002 shl SPI_CR1_BR_Pos) ## !< 0x00000010
  SPI_CR1_BR_Bit2* = (0x00000004 shl SPI_CR1_BR_Pos) ## !< 0x00000020
  SPI_CR1_SPE_Pos* = (6)
  SPI_CR1_SPE_Msk* = (0x00000001 shl SPI_CR1_SPE_Pos) ## !< 0x00000040
  SPI_CR1_SPE* = SPI_CR1_SPE_Msk
  SPI_CR1_LSBFIRST_Pos* = (7)
  SPI_CR1_LSBFIRST_Msk* = (0x00000001 shl SPI_CR1_LSBFIRST_Pos) ## !< 0x00000080
  SPI_CR1_LSBFIRST* = SPI_CR1_LSBFIRST_Msk
  SPI_CR1_SSI_Pos* = (8)
  SPI_CR1_SSI_Msk* = (0x00000001 shl SPI_CR1_SSI_Pos) ## !< 0x00000100
  SPI_CR1_SSI* = SPI_CR1_SSI_Msk
  SPI_CR1_SSM_Pos* = (9)
  SPI_CR1_SSM_Msk* = (0x00000001 shl SPI_CR1_SSM_Pos) ## !< 0x00000200
  SPI_CR1_SSM* = SPI_CR1_SSM_Msk
  SPI_CR1_RXONLY_Pos* = (10)
  SPI_CR1_RXONLY_Msk* = (0x00000001 shl SPI_CR1_RXONLY_Pos) ## !< 0x00000400
  SPI_CR1_RXONLY* = SPI_CR1_RXONLY_Msk
  SPI_CR1_DFF_Pos* = (11)
  SPI_CR1_DFF_Msk* = (0x00000001 shl SPI_CR1_DFF_Pos) ## !< 0x00000800
  SPI_CR1_DFF* = SPI_CR1_DFF_Msk
  SPI_CR1_CRCNEXT_Pos* = (12)
  SPI_CR1_CRCNEXT_Msk* = (0x00000001 shl SPI_CR1_CRCNEXT_Pos) ## !< 0x00001000
  SPI_CR1_CRCNEXT* = SPI_CR1_CRCNEXT_Msk
  SPI_CR1_CRCEN_Pos* = (13)
  SPI_CR1_CRCEN_Msk* = (0x00000001 shl SPI_CR1_CRCEN_Pos) ## !< 0x00002000
  SPI_CR1_CRCEN* = SPI_CR1_CRCEN_Msk
  SPI_CR1_BIDIOE_Pos* = (14)
  SPI_CR1_BIDIOE_Msk* = (0x00000001 shl SPI_CR1_BIDIOE_Pos) ## !< 0x00004000
  SPI_CR1_BIDIOE* = SPI_CR1_BIDIOE_Msk
  SPI_CR1_BIDIMODE_Pos* = (15)
  SPI_CR1_BIDIMODE_Msk* = (0x00000001 shl SPI_CR1_BIDIMODE_Pos) ## !< 0x00008000
  SPI_CR1_BIDIMODE* = SPI_CR1_BIDIMODE_Msk

## ******************  Bit definition for SPI_CR2 register  *******************

const
  SPI_CR2_RXDMAEN_Pos* = (0)
  SPI_CR2_RXDMAEN_Msk* = (0x00000001 shl SPI_CR2_RXDMAEN_Pos) ## !< 0x00000001
  SPI_CR2_RXDMAEN* = SPI_CR2_RXDMAEN_Msk
  SPI_CR2_TXDMAEN_Pos* = (1)
  SPI_CR2_TXDMAEN_Msk* = (0x00000001 shl SPI_CR2_TXDMAEN_Pos) ## !< 0x00000002
  SPI_CR2_TXDMAEN* = SPI_CR2_TXDMAEN_Msk
  SPI_CR2_SSOE_Pos* = (2)
  SPI_CR2_SSOE_Msk* = (0x00000001 shl SPI_CR2_SSOE_Pos) ## !< 0x00000004
  SPI_CR2_SSOE* = SPI_CR2_SSOE_Msk
  SPI_CR2_FRF_Pos* = (4)
  SPI_CR2_FRF_Msk* = (0x00000001 shl SPI_CR2_FRF_Pos) ## !< 0x00000010
  SPI_CR2_FRF* = SPI_CR2_FRF_Msk
  SPI_CR2_ERRIE_Pos* = (5)
  SPI_CR2_ERRIE_Msk* = (0x00000001 shl SPI_CR2_ERRIE_Pos) ## !< 0x00000020
  SPI_CR2_ERRIE* = SPI_CR2_ERRIE_Msk
  SPI_CR2_RXNEIE_Pos* = (6)
  SPI_CR2_RXNEIE_Msk* = (0x00000001 shl SPI_CR2_RXNEIE_Pos) ## !< 0x00000040
  SPI_CR2_RXNEIE* = SPI_CR2_RXNEIE_Msk
  SPI_CR2_TXEIE_Pos* = (7)
  SPI_CR2_TXEIE_Msk* = (0x00000001 shl SPI_CR2_TXEIE_Pos) ## !< 0x00000080
  SPI_CR2_TXEIE* = SPI_CR2_TXEIE_Msk

## *******************  Bit definition for SPI_SR register  *******************

const
  SPI_SR_RXNE_Pos* = (0)
  SPI_SR_RXNE_Msk* = (0x00000001 shl SPI_SR_RXNE_Pos) ## !< 0x00000001
  SPI_SR_RXNE* = SPI_SR_RXNE_Msk
  SPI_SR_TXE_Pos* = (1)
  SPI_SR_TXE_Msk* = (0x00000001 shl SPI_SR_TXE_Pos) ## !< 0x00000002
  SPI_SR_TXE* = SPI_SR_TXE_Msk
  SPI_SR_CHSIDE_Pos* = (2)
  SPI_SR_CHSIDE_Msk* = (0x00000001 shl SPI_SR_CHSIDE_Pos) ## !< 0x00000004
  SPI_SR_CHSIDE* = SPI_SR_CHSIDE_Msk
  SPI_SR_UDR_Pos* = (3)
  SPI_SR_UDR_Msk* = (0x00000001 shl SPI_SR_UDR_Pos) ## !< 0x00000008
  SPI_SR_UDR* = SPI_SR_UDR_Msk
  SPI_SR_CRCERR_Pos* = (4)
  SPI_SR_CRCERR_Msk* = (0x00000001 shl SPI_SR_CRCERR_Pos) ## !< 0x00000010
  SPI_SR_CRCERR* = SPI_SR_CRCERR_Msk
  SPI_SR_MODF_Pos* = (5)
  SPI_SR_MODF_Msk* = (0x00000001 shl SPI_SR_MODF_Pos) ## !< 0x00000020
  SPI_SR_MODF* = SPI_SR_MODF_Msk
  SPI_SR_OVR_Pos* = (6)
  SPI_SR_OVR_Msk* = (0x00000001 shl SPI_SR_OVR_Pos) ## !< 0x00000040
  SPI_SR_OVR* = SPI_SR_OVR_Msk
  SPI_SR_BSY_Pos* = (7)
  SPI_SR_BSY_Msk* = (0x00000001 shl SPI_SR_BSY_Pos) ## !< 0x00000080
  SPI_SR_BSY* = SPI_SR_BSY_Msk
  SPI_SR_FRE_Pos* = (8)
  SPI_SR_FRE_Msk* = (0x00000001 shl SPI_SR_FRE_Pos) ## !< 0x00000100
  SPI_SR_FRE* = SPI_SR_FRE_Msk

## *******************  Bit definition for SPI_DR register  *******************

const
  SPI_DR_DR_Pos* = (0)
  SPI_DR_DR_Msk* = (0x0000FFFF shl SPI_DR_DR_Pos) ## !< 0x0000FFFF
  SPI_DR_DR* = SPI_DR_DR_Msk

## ******************  Bit definition for SPI_CRCPR register  *****************

const
  SPI_CRCPR_CRCPOLY_Pos* = (0)
  SPI_CRCPR_CRCPOLY_Msk* = (0x0000FFFF shl SPI_CRCPR_CRCPOLY_Pos) ## !< 0x0000FFFF
  SPI_CRCPR_CRCPOLY* = SPI_CRCPR_CRCPOLY_Msk

## *****************  Bit definition for SPI_RXCRCR register  *****************

const
  SPI_RXCRCR_RXCRC_Pos* = (0)
  SPI_RXCRCR_RXCRC_Msk* = (0x0000FFFF shl SPI_RXCRCR_RXCRC_Pos) ## !< 0x0000FFFF
  SPI_RXCRCR_RXCRC* = SPI_RXCRCR_RXCRC_Msk

## *****************  Bit definition for SPI_TXCRCR register  *****************

const
  SPI_TXCRCR_TXCRC_Pos* = (0)
  SPI_TXCRCR_TXCRC_Msk* = (0x0000FFFF shl SPI_TXCRCR_TXCRC_Pos) ## !< 0x0000FFFF
  SPI_TXCRCR_TXCRC* = SPI_TXCRCR_TXCRC_Msk

## *****************  Bit definition for SPI_I2SCFGR register  ****************

const
  SPI_I2SCFGR_CHLEN_Pos* = (0)
  SPI_I2SCFGR_CHLEN_Msk* = (0x00000001 shl SPI_I2SCFGR_CHLEN_Pos) ## !< 0x00000001
  SPI_I2SCFGR_CHLEN* = SPI_I2SCFGR_CHLEN_Msk
  SPI_I2SCFGR_DATLEN_Pos* = (1)
  SPI_I2SCFGR_DATLEN_Msk* = (0x00000003 shl SPI_I2SCFGR_DATLEN_Pos) ## !< 0x00000006
  SPI_I2SCFGR_DATLEN* = SPI_I2SCFGR_DATLEN_Msk
  SPI_I2SCFGR_DATLEN_Bit0* = (0x00000001 shl SPI_I2SCFGR_DATLEN_Pos) ## !< 0x00000002
  SPI_I2SCFGR_DATLEN_Bit1* = (0x00000002 shl SPI_I2SCFGR_DATLEN_Pos) ## !< 0x00000004
  SPI_I2SCFGR_CKPOL_Pos* = (3)
  SPI_I2SCFGR_CKPOL_Msk* = (0x00000001 shl SPI_I2SCFGR_CKPOL_Pos) ## !< 0x00000008
  SPI_I2SCFGR_CKPOL* = SPI_I2SCFGR_CKPOL_Msk
  SPI_I2SCFGR_I2SSTD_Pos* = (4)
  SPI_I2SCFGR_I2SSTD_Msk* = (0x00000003 shl SPI_I2SCFGR_I2SSTD_Pos) ## !< 0x00000030
  SPI_I2SCFGR_I2SSTD* = SPI_I2SCFGR_I2SSTD_Msk
  SPI_I2SCFGR_I2SSTD_Bit0* = (0x00000001 shl SPI_I2SCFGR_I2SSTD_Pos) ## !< 0x00000010
  SPI_I2SCFGR_I2SSTD_Bit1* = (0x00000002 shl SPI_I2SCFGR_I2SSTD_Pos) ## !< 0x00000020
  SPI_I2SCFGR_PCMSYNC_Pos* = (7)
  SPI_I2SCFGR_PCMSYNC_Msk* = (0x00000001 shl SPI_I2SCFGR_PCMSYNC_Pos) ## !< 0x00000080
  SPI_I2SCFGR_PCMSYNC* = SPI_I2SCFGR_PCMSYNC_Msk
  SPI_I2SCFGR_I2SCFG_Pos* = (8)
  SPI_I2SCFGR_I2SCFG_Msk* = (0x00000003 shl SPI_I2SCFGR_I2SCFG_Pos) ## !< 0x00000300
  SPI_I2SCFGR_I2SCFG* = SPI_I2SCFGR_I2SCFG_Msk
  SPI_I2SCFGR_I2SCFG_Bit0* = (0x00000001 shl SPI_I2SCFGR_I2SCFG_Pos) ## !< 0x00000100
  SPI_I2SCFGR_I2SCFG_Bit1* = (0x00000002 shl SPI_I2SCFGR_I2SCFG_Pos) ## !< 0x00000200
  SPI_I2SCFGR_I2SE_Pos* = (10)
  SPI_I2SCFGR_I2SE_Msk* = (0x00000001 shl SPI_I2SCFGR_I2SE_Pos) ## !< 0x00000400
  SPI_I2SCFGR_I2SE* = SPI_I2SCFGR_I2SE_Msk
  SPI_I2SCFGR_I2SMOD_Pos* = (11)
  SPI_I2SCFGR_I2SMOD_Msk* = (0x00000001 shl SPI_I2SCFGR_I2SMOD_Pos) ## !< 0x00000800
  SPI_I2SCFGR_I2SMOD* = SPI_I2SCFGR_I2SMOD_Msk

## *****************  Bit definition for SPI_I2SPR register  ******************

const
  SPI_I2SPR_I2SDIV_Pos* = (0)
  SPI_I2SPR_I2SDIV_Msk* = (0x000000FF shl SPI_I2SPR_I2SDIV_Pos) ## !< 0x000000FF
  SPI_I2SPR_I2SDIV* = SPI_I2SPR_I2SDIV_Msk
  SPI_I2SPR_ODD_Pos* = (8)
  SPI_I2SPR_ODD_Msk* = (0x00000001 shl SPI_I2SPR_ODD_Pos) ## !< 0x00000100
  SPI_I2SPR_ODD* = SPI_I2SPR_ODD_Msk
  SPI_I2SPR_MCKOE_Pos* = (9)
  SPI_I2SPR_MCKOE_Msk* = (0x00000001 shl SPI_I2SPR_MCKOE_Pos) ## !< 0x00000200
  SPI_I2SPR_MCKOE* = SPI_I2SPR_MCKOE_Msk

## ****************************************************************************
##
##                                  SYSCFG
##
## ****************************************************************************
## *****************  Bit definition for SYSCFG_MEMRMP register  **************

const
  SYSCFG_MEMRMP_MEM_MODE_Pos* = (0)
  SYSCFG_MEMRMP_MEM_MODE_Msk* = (0x00000003 shl SYSCFG_MEMRMP_MEM_MODE_Pos) ## !< 0x00000003
  SYSCFG_MEMRMP_MEM_MODE* = SYSCFG_MEMRMP_MEM_MODE_Msk
  SYSCFG_MEMRMP_MEM_MODE_Bit0* = (0x00000001 shl SYSCFG_MEMRMP_MEM_MODE_Pos) ## !< 0x00000001
  SYSCFG_MEMRMP_MEM_MODE_Bit1* = (0x00000002 shl SYSCFG_MEMRMP_MEM_MODE_Pos) ## !< 0x00000002

## *****************  Bit definition for SYSCFG_PMC register  *****************

const
  SYSCFG_PMC_ADC1DC2_Pos* = (16)
  SYSCFG_PMC_ADC1DC2_Msk* = (0x00000001 shl SYSCFG_PMC_ADC1DC2_Pos) ## !< 0x00010000
  SYSCFG_PMC_ADC1DC2* = SYSCFG_PMC_ADC1DC2_Msk

## ****************  Bit definition for SYSCFG_EXTICR1 register  **************

const
  SYSCFG_EXTICR1_EXTI0_Pos* = (0)
  SYSCFG_EXTICR1_EXTI0_Msk* = (0x0000000F shl SYSCFG_EXTICR1_EXTI0_Pos) ## !< 0x0000000F
  SYSCFG_EXTICR1_EXTI0* = SYSCFG_EXTICR1_EXTI0_Msk
  SYSCFG_EXTICR1_EXTI1_Pos* = (4)
  SYSCFG_EXTICR1_EXTI1_Msk* = (0x0000000F shl SYSCFG_EXTICR1_EXTI1_Pos) ## !< 0x000000F0
  SYSCFG_EXTICR1_EXTI1* = SYSCFG_EXTICR1_EXTI1_Msk
  SYSCFG_EXTICR1_EXTI2_Pos* = (8)
  SYSCFG_EXTICR1_EXTI2_Msk* = (0x0000000F shl SYSCFG_EXTICR1_EXTI2_Pos) ## !< 0x00000F00
  SYSCFG_EXTICR1_EXTI2* = SYSCFG_EXTICR1_EXTI2_Msk
  SYSCFG_EXTICR1_EXTI3_Pos* = (12)
  SYSCFG_EXTICR1_EXTI3_Msk* = (0x0000000F shl SYSCFG_EXTICR1_EXTI3_Pos) ## !< 0x0000F000
  SYSCFG_EXTICR1_EXTI3* = SYSCFG_EXTICR1_EXTI3_Msk

## *
##  @brief   EXTI0 configuration
##

const
  SYSCFG_EXTICR1_EXTI0_PA* = 0x00000000
  SYSCFG_EXTICR1_EXTI0_PB* = 0x00000001
  SYSCFG_EXTICR1_EXTI0_PC* = 0x00000002
  SYSCFG_EXTICR1_EXTI0_PH* = 0x00000007

## *
##  @brief   EXTI1 configuration
##

const
  SYSCFG_EXTICR1_EXTI1_PA* = 0x00000000
  SYSCFG_EXTICR1_EXTI1_PB* = 0x00000010
  SYSCFG_EXTICR1_EXTI1_PC* = 0x00000020
  SYSCFG_EXTICR1_EXTI1_PH* = 0x00000070

## *
##  @brief   EXTI2 configuration
##

const
  SYSCFG_EXTICR1_EXTI2_PA* = 0x00000000
  SYSCFG_EXTICR1_EXTI2_PB* = 0x00000100
  SYSCFG_EXTICR1_EXTI2_PC* = 0x00000200
  SYSCFG_EXTICR1_EXTI2_PH* = 0x00000700

## *
##  @brief   EXTI3 configuration
##

const
  SYSCFG_EXTICR1_EXTI3_PA* = 0x00000000
  SYSCFG_EXTICR1_EXTI3_PB* = 0x00001000
  SYSCFG_EXTICR1_EXTI3_PC* = 0x00002000
  SYSCFG_EXTICR1_EXTI3_PH* = 0x00007000

## ****************  Bit definition for SYSCFG_EXTICR2 register  **************

const
  SYSCFG_EXTICR2_EXTI4_Pos* = (0)
  SYSCFG_EXTICR2_EXTI4_Msk* = (0x0000000F shl SYSCFG_EXTICR2_EXTI4_Pos) ## !< 0x0000000F
  SYSCFG_EXTICR2_EXTI4* = SYSCFG_EXTICR2_EXTI4_Msk
  SYSCFG_EXTICR2_EXTI5_Pos* = (4)
  SYSCFG_EXTICR2_EXTI5_Msk* = (0x0000000F shl SYSCFG_EXTICR2_EXTI5_Pos) ## !< 0x000000F0
  SYSCFG_EXTICR2_EXTI5* = SYSCFG_EXTICR2_EXTI5_Msk
  SYSCFG_EXTICR2_EXTI6_Pos* = (8)
  SYSCFG_EXTICR2_EXTI6_Msk* = (0x0000000F shl SYSCFG_EXTICR2_EXTI6_Pos) ## !< 0x00000F00
  SYSCFG_EXTICR2_EXTI6* = SYSCFG_EXTICR2_EXTI6_Msk
  SYSCFG_EXTICR2_EXTI7_Pos* = (12)
  SYSCFG_EXTICR2_EXTI7_Msk* = (0x0000000F shl SYSCFG_EXTICR2_EXTI7_Pos) ## !< 0x0000F000
  SYSCFG_EXTICR2_EXTI7* = SYSCFG_EXTICR2_EXTI7_Msk

## *
##  @brief   EXTI4 configuration
##

const
  SYSCFG_EXTICR2_EXTI4_PA* = 0x00000000
  SYSCFG_EXTICR2_EXTI4_PB* = 0x00000001
  SYSCFG_EXTICR2_EXTI4_PC* = 0x00000002
  SYSCFG_EXTICR2_EXTI4_PH* = 0x00000007

## *
##  @brief   EXTI5 configuration
##

const
  SYSCFG_EXTICR2_EXTI5_PA* = 0x00000000
  SYSCFG_EXTICR2_EXTI5_PB* = 0x00000010
  SYSCFG_EXTICR2_EXTI5_PC* = 0x00000020
  SYSCFG_EXTICR2_EXTI5_PH* = 0x00000070

## *
##  @brief   EXTI6 configuration
##

const
  SYSCFG_EXTICR2_EXTI6_PA* = 0x00000000
  SYSCFG_EXTICR2_EXTI6_PB* = 0x00000100
  SYSCFG_EXTICR2_EXTI6_PC* = 0x00000200
  SYSCFG_EXTICR2_EXTI6_PH* = 0x00000700

## *
##  @brief   EXTI7 configuration
##

const
  SYSCFG_EXTICR2_EXTI7_PA* = 0x00000000
  SYSCFG_EXTICR2_EXTI7_PB* = 0x00001000
  SYSCFG_EXTICR2_EXTI7_PC* = 0x00002000
  SYSCFG_EXTICR2_EXTI7_PH* = 0x00007000

## ****************  Bit definition for SYSCFG_EXTICR3 register  **************

const
  SYSCFG_EXTICR3_EXTI8_Pos* = (0)
  SYSCFG_EXTICR3_EXTI8_Msk* = (0x0000000F shl SYSCFG_EXTICR3_EXTI8_Pos) ## !< 0x0000000F
  SYSCFG_EXTICR3_EXTI8* = SYSCFG_EXTICR3_EXTI8_Msk
  SYSCFG_EXTICR3_EXTI9_Pos* = (4)
  SYSCFG_EXTICR3_EXTI9_Msk* = (0x0000000F shl SYSCFG_EXTICR3_EXTI9_Pos) ## !< 0x000000F0
  SYSCFG_EXTICR3_EXTI9* = SYSCFG_EXTICR3_EXTI9_Msk
  SYSCFG_EXTICR3_EXTI10_Pos* = (8)
  SYSCFG_EXTICR3_EXTI10_Msk* = (0x0000000F shl SYSCFG_EXTICR3_EXTI10_Pos) ## !< 0x00000F00
  SYSCFG_EXTICR3_EXTI10* = SYSCFG_EXTICR3_EXTI10_Msk
  SYSCFG_EXTICR3_EXTI11_Pos* = (12)
  SYSCFG_EXTICR3_EXTI11_Msk* = (0x0000000F shl SYSCFG_EXTICR3_EXTI11_Pos) ## !< 0x0000F000
  SYSCFG_EXTICR3_EXTI11* = SYSCFG_EXTICR3_EXTI11_Msk

## *
##  @brief   EXTI8 configuration
##

const
  SYSCFG_EXTICR3_EXTI8_PA* = 0x00000000
  SYSCFG_EXTICR3_EXTI8_PB* = 0x00000001
  SYSCFG_EXTICR3_EXTI8_PC* = 0x00000002
  SYSCFG_EXTICR3_EXTI8_PH* = 0x00000007

## *
##  @brief   EXTI9 configuration
##

const
  SYSCFG_EXTICR3_EXTI9_PA* = 0x00000000
  SYSCFG_EXTICR3_EXTI9_PB* = 0x00000010
  SYSCFG_EXTICR3_EXTI9_PC* = 0x00000020
  SYSCFG_EXTICR3_EXTI9_PH* = 0x00000070

## *
##  @brief   EXTI10 configuration
##

const
  SYSCFG_EXTICR3_EXTI10_PA* = 0x00000000
  SYSCFG_EXTICR3_EXTI10_PB* = 0x00000100
  SYSCFG_EXTICR3_EXTI10_PC* = 0x00000200
  SYSCFG_EXTICR3_EXTI10_PH* = 0x00000700

## *
##  @brief   EXTI11 configuration
##

const
  SYSCFG_EXTICR3_EXTI11_PA* = 0x00000000
  SYSCFG_EXTICR3_EXTI11_PB* = 0x00001000
  SYSCFG_EXTICR3_EXTI11_PC* = 0x00002000
  SYSCFG_EXTICR3_EXTI11_PH* = 0x00007000

## ****************  Bit definition for SYSCFG_EXTICR4 register  **************

const
  SYSCFG_EXTICR4_EXTI12_Pos* = (0)
  SYSCFG_EXTICR4_EXTI12_Msk* = (0x0000000F shl SYSCFG_EXTICR4_EXTI12_Pos) ## !< 0x0000000F
  SYSCFG_EXTICR4_EXTI12* = SYSCFG_EXTICR4_EXTI12_Msk
  SYSCFG_EXTICR4_EXTI13_Pos* = (4)
  SYSCFG_EXTICR4_EXTI13_Msk* = (0x0000000F shl SYSCFG_EXTICR4_EXTI13_Pos) ## !< 0x000000F0
  SYSCFG_EXTICR4_EXTI13* = SYSCFG_EXTICR4_EXTI13_Msk
  SYSCFG_EXTICR4_EXTI14_Pos* = (8)
  SYSCFG_EXTICR4_EXTI14_Msk* = (0x0000000F shl SYSCFG_EXTICR4_EXTI14_Pos) ## !< 0x00000F00
  SYSCFG_EXTICR4_EXTI14* = SYSCFG_EXTICR4_EXTI14_Msk
  SYSCFG_EXTICR4_EXTI15_Pos* = (12)
  SYSCFG_EXTICR4_EXTI15_Msk* = (0x0000000F shl SYSCFG_EXTICR4_EXTI15_Pos) ## !< 0x0000F000
  SYSCFG_EXTICR4_EXTI15* = SYSCFG_EXTICR4_EXTI15_Msk

## *
##  @brief   EXTI12 configuration
##

const
  SYSCFG_EXTICR4_EXTI12_PA* = 0x00000000
  SYSCFG_EXTICR4_EXTI12_PB* = 0x00000001
  SYSCFG_EXTICR4_EXTI12_PC* = 0x00000002
  SYSCFG_EXTICR4_EXTI12_PH* = 0x00000007

## *
##  @brief   EXTI13 configuration
##

const
  SYSCFG_EXTICR4_EXTI13_PA* = 0x00000000
  SYSCFG_EXTICR4_EXTI13_PB* = 0x00000010
  SYSCFG_EXTICR4_EXTI13_PC* = 0x00000020
  SYSCFG_EXTICR4_EXTI13_PH* = 0x00000070

## *
##  @brief   EXTI14 configuration
##

const
  SYSCFG_EXTICR4_EXTI14_PA* = 0x00000000
  SYSCFG_EXTICR4_EXTI14_PB* = 0x00000100
  SYSCFG_EXTICR4_EXTI14_PC* = 0x00000200
  SYSCFG_EXTICR4_EXTI14_PH* = 0x00000700

## *
##  @brief   EXTI15 configuration
##

const
  SYSCFG_EXTICR4_EXTI15_PA* = 0x00000000
  SYSCFG_EXTICR4_EXTI15_PB* = 0x00001000
  SYSCFG_EXTICR4_EXTI15_PC* = 0x00002000
  SYSCFG_EXTICR4_EXTI15_PH* = 0x00007000

## *****************  Bit definition for SYSCFG_CMPCR register  ***************

const
  SYSCFG_CMPCR_CMP_PD_Pos* = (0)
  SYSCFG_CMPCR_CMP_PD_Msk* = (0x00000001 shl SYSCFG_CMPCR_CMP_PD_Pos) ## !< 0x00000001
  SYSCFG_CMPCR_CMP_PD* = SYSCFG_CMPCR_CMP_PD_Msk
  SYSCFG_CMPCR_READY_Pos* = (8)
  SYSCFG_CMPCR_READY_Msk* = (0x00000001 shl SYSCFG_CMPCR_READY_Pos) ## !< 0x00000100
  SYSCFG_CMPCR_READY* = SYSCFG_CMPCR_READY_Msk

## *****************  Bit definition for SYSCFG_CFGR register  ****************

const
  SYSCFG_CFGR_FMPI2C1_SCL_Pos* = (0)
  SYSCFG_CFGR_FMPI2C1_SCL_Msk* = (0x00000001 shl SYSCFG_CFGR_FMPI2C1_SCL_Pos) ## !< 0x00000001
  SYSCFG_CFGR_FMPI2C1_SCL* = SYSCFG_CFGR_FMPI2C1_SCL_Msk
  SYSCFG_CFGR_FMPI2C1_SDA_Pos* = (1)
  SYSCFG_CFGR_FMPI2C1_SDA_Msk* = (0x00000001 shl SYSCFG_CFGR_FMPI2C1_SDA_Pos) ## !< 0x00000002
  SYSCFG_CFGR_FMPI2C1_SDA* = SYSCFG_CFGR_FMPI2C1_SDA_Msk

## *****************  Bit definition for SYSCFG_CFGR2 register  ****************

const
  SYSCFG_CFGR2_LOCKUP_LOCK_Pos* = (0)
  SYSCFG_CFGR2_LOCKUP_LOCK_Msk* = (0x00000001 shl SYSCFG_CFGR2_LOCKUP_LOCK_Pos) ## !< 0x00000001
  SYSCFG_CFGR2_LOCKUP_LOCK* = SYSCFG_CFGR2_LOCKUP_LOCK_Msk
  SYSCFG_CFGR2_PVD_LOCK_Pos* = (2)
  SYSCFG_CFGR2_PVD_LOCK_Msk* = (0x00000001 shl SYSCFG_CFGR2_PVD_LOCK_Pos) ## !< 0x00000004
  SYSCFG_CFGR2_PVD_LOCK* = SYSCFG_CFGR2_PVD_LOCK_Msk

## ****************************************************************************
##
##                                     TIM
##
## ****************************************************************************
## ******************  Bit definition for TIM_CR1 register  *******************

const
  TIM_CR1_CEN_Pos* = (0)
  TIM_CR1_CEN_Msk* = (0x00000001 shl TIM_CR1_CEN_Pos) ## !< 0x00000001
  TIM_CR1_CEN* = TIM_CR1_CEN_Msk
  TIM_CR1_UDIS_Pos* = (1)
  TIM_CR1_UDIS_Msk* = (0x00000001 shl TIM_CR1_UDIS_Pos) ## !< 0x00000002
  TIM_CR1_UDIS* = TIM_CR1_UDIS_Msk
  TIM_CR1_URS_Pos* = (2)
  TIM_CR1_URS_Msk* = (0x00000001 shl TIM_CR1_URS_Pos) ## !< 0x00000004
  TIM_CR1_URS* = TIM_CR1_URS_Msk
  TIM_CR1_OPM_Pos* = (3)
  TIM_CR1_OPM_Msk* = (0x00000001 shl TIM_CR1_OPM_Pos) ## !< 0x00000008
  TIM_CR1_OPM* = TIM_CR1_OPM_Msk
  TIM_CR1_DIR_Pos* = (4)
  TIM_CR1_DIR_Msk* = (0x00000001 shl TIM_CR1_DIR_Pos) ## !< 0x00000010
  TIM_CR1_DIR* = TIM_CR1_DIR_Msk
  TIM_CR1_CMS_Pos* = (5)
  TIM_CR1_CMS_Msk* = (0x00000003 shl TIM_CR1_CMS_Pos) ## !< 0x00000060
  TIM_CR1_CMS* = TIM_CR1_CMS_Msk
  TIM_CR1_CMS_Bit0* = (0x00000001 shl TIM_CR1_CMS_Pos) ## !< 0x0020
  TIM_CR1_CMS_Bit1* = (0x00000002 shl TIM_CR1_CMS_Pos) ## !< 0x0040
  TIM_CR1_ARPE_Pos* = (7)
  TIM_CR1_ARPE_Msk* = (0x00000001 shl TIM_CR1_ARPE_Pos) ## !< 0x00000080
  TIM_CR1_ARPE* = TIM_CR1_ARPE_Msk
  TIM_CR1_CKD_Pos* = (8)
  TIM_CR1_CKD_Msk* = (0x00000003 shl TIM_CR1_CKD_Pos) ## !< 0x00000300
  TIM_CR1_CKD* = TIM_CR1_CKD_Msk
  TIM_CR1_CKD_Bit0* = (0x00000001 shl TIM_CR1_CKD_Pos) ## !< 0x0100
  TIM_CR1_CKD_Bit1* = (0x00000002 shl TIM_CR1_CKD_Pos) ## !< 0x0200

## ******************  Bit definition for TIM_CR2 register  *******************

const
  TIM_CR2_CCPC_Pos* = (0)
  TIM_CR2_CCPC_Msk* = (0x00000001 shl TIM_CR2_CCPC_Pos) ## !< 0x00000001
  TIM_CR2_CCPC* = TIM_CR2_CCPC_Msk
  TIM_CR2_CCUS_Pos* = (2)
  TIM_CR2_CCUS_Msk* = (0x00000001 shl TIM_CR2_CCUS_Pos) ## !< 0x00000004
  TIM_CR2_CCUS* = TIM_CR2_CCUS_Msk
  TIM_CR2_CCDS_Pos* = (3)
  TIM_CR2_CCDS_Msk* = (0x00000001 shl TIM_CR2_CCDS_Pos) ## !< 0x00000008
  TIM_CR2_CCDS* = TIM_CR2_CCDS_Msk
  TIM_CR2_MMS_Pos* = (4)
  TIM_CR2_MMS_Msk* = (0x00000007 shl TIM_CR2_MMS_Pos) ## !< 0x00000070
  TIM_CR2_MMS* = TIM_CR2_MMS_Msk
  TIM_CR2_MMS_Bit0* = (0x00000001 shl TIM_CR2_MMS_Pos) ## !< 0x0010
  TIM_CR2_MMS_Bit1* = (0x00000002 shl TIM_CR2_MMS_Pos) ## !< 0x0020
  TIM_CR2_MMS_Bit2* = (0x00000004 shl TIM_CR2_MMS_Pos) ## !< 0x0040
  TIM_CR2_TI1S_Pos* = (7)
  TIM_CR2_TI1S_Msk* = (0x00000001 shl TIM_CR2_TI1S_Pos) ## !< 0x00000080
  TIM_CR2_TI1S* = TIM_CR2_TI1S_Msk
  TIM_CR2_OIS1_Pos* = (8)
  TIM_CR2_OIS1_Msk* = (0x00000001 shl TIM_CR2_OIS1_Pos) ## !< 0x00000100
  TIM_CR2_OIS1* = TIM_CR2_OIS1_Msk
  TIM_CR2_OIS1N_Pos* = (9)
  TIM_CR2_OIS1N_Msk* = (0x00000001 shl TIM_CR2_OIS1N_Pos) ## !< 0x00000200
  TIM_CR2_OIS1N* = TIM_CR2_OIS1N_Msk
  TIM_CR2_OIS2_Pos* = (10)
  TIM_CR2_OIS2_Msk* = (0x00000001 shl TIM_CR2_OIS2_Pos) ## !< 0x00000400
  TIM_CR2_OIS2* = TIM_CR2_OIS2_Msk
  TIM_CR2_OIS2N_Pos* = (11)
  TIM_CR2_OIS2N_Msk* = (0x00000001 shl TIM_CR2_OIS2N_Pos) ## !< 0x00000800
  TIM_CR2_OIS2N* = TIM_CR2_OIS2N_Msk
  TIM_CR2_OIS3_Pos* = (12)
  TIM_CR2_OIS3_Msk* = (0x00000001 shl TIM_CR2_OIS3_Pos) ## !< 0x00001000
  TIM_CR2_OIS3* = TIM_CR2_OIS3_Msk
  TIM_CR2_OIS3N_Pos* = (13)
  TIM_CR2_OIS3N_Msk* = (0x00000001 shl TIM_CR2_OIS3N_Pos) ## !< 0x00002000
  TIM_CR2_OIS3N* = TIM_CR2_OIS3N_Msk
  TIM_CR2_OIS4_Pos* = (14)
  TIM_CR2_OIS4_Msk* = (0x00000001 shl TIM_CR2_OIS4_Pos) ## !< 0x00004000
  TIM_CR2_OIS4* = TIM_CR2_OIS4_Msk

## ******************  Bit definition for TIM_SMCR register  ******************

const
  TIM_SMCR_SMS_Pos* = (0)
  TIM_SMCR_SMS_Msk* = (0x00000007 shl TIM_SMCR_SMS_Pos) ## !< 0x00000007
  TIM_SMCR_SMS* = TIM_SMCR_SMS_Msk
  TIM_SMCR_SMS_Bit0* = (0x00000001 shl TIM_SMCR_SMS_Pos) ## !< 0x0001
  TIM_SMCR_SMS_Bit1* = (0x00000002 shl TIM_SMCR_SMS_Pos) ## !< 0x0002
  TIM_SMCR_SMS_Bit2* = (0x00000004 shl TIM_SMCR_SMS_Pos) ## !< 0x0004
  TIM_SMCR_TS_Pos* = (4)
  TIM_SMCR_TS_Msk* = (0x00000007 shl TIM_SMCR_TS_Pos) ## !< 0x00000070
  TIM_SMCR_TS* = TIM_SMCR_TS_Msk
  TIM_SMCR_TS_Bit0* = (0x00000001 shl TIM_SMCR_TS_Pos) ## !< 0x0010
  TIM_SMCR_TS_Bit1* = (0x00000002 shl TIM_SMCR_TS_Pos) ## !< 0x0020
  TIM_SMCR_TS_Bit2* = (0x00000004 shl TIM_SMCR_TS_Pos) ## !< 0x0040
  TIM_SMCR_MSM_Pos* = (7)
  TIM_SMCR_MSM_Msk* = (0x00000001 shl TIM_SMCR_MSM_Pos) ## !< 0x00000080
  TIM_SMCR_MSM* = TIM_SMCR_MSM_Msk
  TIM_SMCR_ETF_Pos* = (8)
  TIM_SMCR_ETF_Msk* = (0x0000000F shl TIM_SMCR_ETF_Pos) ## !< 0x00000F00
  TIM_SMCR_ETF* = TIM_SMCR_ETF_Msk
  TIM_SMCR_ETF_Bit0* = (0x00000001 shl TIM_SMCR_ETF_Pos) ## !< 0x0100
  TIM_SMCR_ETF_Bit1* = (0x00000002 shl TIM_SMCR_ETF_Pos) ## !< 0x0200
  TIM_SMCR_ETF_Bit2* = (0x00000004 shl TIM_SMCR_ETF_Pos) ## !< 0x0400
  TIM_SMCR_ETF_Bit3* = (0x00000008 shl TIM_SMCR_ETF_Pos) ## !< 0x0800
  TIM_SMCR_ETPS_Pos* = (12)
  TIM_SMCR_ETPS_Msk* = (0x00000003 shl TIM_SMCR_ETPS_Pos) ## !< 0x00003000
  TIM_SMCR_ETPS* = TIM_SMCR_ETPS_Msk
  TIM_SMCR_ETPS_Bit0* = (0x00000001 shl TIM_SMCR_ETPS_Pos) ## !< 0x1000
  TIM_SMCR_ETPS_Bit1* = (0x00000002 shl TIM_SMCR_ETPS_Pos) ## !< 0x2000
  TIM_SMCR_ECE_Pos* = (14)
  TIM_SMCR_ECE_Msk* = (0x00000001 shl TIM_SMCR_ECE_Pos) ## !< 0x00004000
  TIM_SMCR_ECE* = TIM_SMCR_ECE_Msk
  TIM_SMCR_ETP_Pos* = (15)
  TIM_SMCR_ETP_Msk* = (0x00000001 shl TIM_SMCR_ETP_Pos) ## !< 0x00008000
  TIM_SMCR_ETP* = TIM_SMCR_ETP_Msk

## ******************  Bit definition for TIM_DIER register  ******************

const
  TIM_DIER_UIE_Pos* = (0)
  TIM_DIER_UIE_Msk* = (0x00000001 shl TIM_DIER_UIE_Pos) ## !< 0x00000001
  TIM_DIER_UIE* = TIM_DIER_UIE_Msk
  TIM_DIER_CC1IE_Pos* = (1)
  TIM_DIER_CC1IE_Msk* = (0x00000001 shl TIM_DIER_CC1IE_Pos) ## !< 0x00000002
  TIM_DIER_CC1IE* = TIM_DIER_CC1IE_Msk
  TIM_DIER_CC2IE_Pos* = (2)
  TIM_DIER_CC2IE_Msk* = (0x00000001 shl TIM_DIER_CC2IE_Pos) ## !< 0x00000004
  TIM_DIER_CC2IE* = TIM_DIER_CC2IE_Msk
  TIM_DIER_CC3IE_Pos* = (3)
  TIM_DIER_CC3IE_Msk* = (0x00000001 shl TIM_DIER_CC3IE_Pos) ## !< 0x00000008
  TIM_DIER_CC3IE* = TIM_DIER_CC3IE_Msk
  TIM_DIER_CC4IE_Pos* = (4)
  TIM_DIER_CC4IE_Msk* = (0x00000001 shl TIM_DIER_CC4IE_Pos) ## !< 0x00000010
  TIM_DIER_CC4IE* = TIM_DIER_CC4IE_Msk
  TIM_DIER_COMIE_Pos* = (5)
  TIM_DIER_COMIE_Msk* = (0x00000001 shl TIM_DIER_COMIE_Pos) ## !< 0x00000020
  TIM_DIER_COMIE* = TIM_DIER_COMIE_Msk
  TIM_DIER_TIE_Pos* = (6)
  TIM_DIER_TIE_Msk* = (0x00000001 shl TIM_DIER_TIE_Pos) ## !< 0x00000040
  TIM_DIER_TIE* = TIM_DIER_TIE_Msk
  TIM_DIER_BIE_Pos* = (7)
  TIM_DIER_BIE_Msk* = (0x00000001 shl TIM_DIER_BIE_Pos) ## !< 0x00000080
  TIM_DIER_BIE* = TIM_DIER_BIE_Msk
  TIM_DIER_UDE_Pos* = (8)
  TIM_DIER_UDE_Msk* = (0x00000001 shl TIM_DIER_UDE_Pos) ## !< 0x00000100
  TIM_DIER_UDE* = TIM_DIER_UDE_Msk
  TIM_DIER_CC1DE_Pos* = (9)
  TIM_DIER_CC1DE_Msk* = (0x00000001 shl TIM_DIER_CC1DE_Pos) ## !< 0x00000200
  TIM_DIER_CC1DE* = TIM_DIER_CC1DE_Msk
  TIM_DIER_CC2DE_Pos* = (10)
  TIM_DIER_CC2DE_Msk* = (0x00000001 shl TIM_DIER_CC2DE_Pos) ## !< 0x00000400
  TIM_DIER_CC2DE* = TIM_DIER_CC2DE_Msk
  TIM_DIER_CC3DE_Pos* = (11)
  TIM_DIER_CC3DE_Msk* = (0x00000001 shl TIM_DIER_CC3DE_Pos) ## !< 0x00000800
  TIM_DIER_CC3DE* = TIM_DIER_CC3DE_Msk
  TIM_DIER_CC4DE_Pos* = (12)
  TIM_DIER_CC4DE_Msk* = (0x00000001 shl TIM_DIER_CC4DE_Pos) ## !< 0x00001000
  TIM_DIER_CC4DE* = TIM_DIER_CC4DE_Msk
  TIM_DIER_COMDE_Pos* = (13)
  TIM_DIER_COMDE_Msk* = (0x00000001 shl TIM_DIER_COMDE_Pos) ## !< 0x00002000
  TIM_DIER_COMDE* = TIM_DIER_COMDE_Msk
  TIM_DIER_TDE_Pos* = (14)
  TIM_DIER_TDE_Msk* = (0x00000001 shl TIM_DIER_TDE_Pos) ## !< 0x00004000
  TIM_DIER_TDE* = TIM_DIER_TDE_Msk

## *******************  Bit definition for TIM_SR register  *******************

const
  TIM_SR_UIF_Pos* = (0)
  TIM_SR_UIF_Msk* = (0x00000001 shl TIM_SR_UIF_Pos) ## !< 0x00000001
  TIM_SR_UIF* = TIM_SR_UIF_Msk
  TIM_SR_CC1IF_Pos* = (1)
  TIM_SR_CC1IF_Msk* = (0x00000001 shl TIM_SR_CC1IF_Pos) ## !< 0x00000002
  TIM_SR_CC1IF* = TIM_SR_CC1IF_Msk
  TIM_SR_CC2IF_Pos* = (2)
  TIM_SR_CC2IF_Msk* = (0x00000001 shl TIM_SR_CC2IF_Pos) ## !< 0x00000004
  TIM_SR_CC2IF* = TIM_SR_CC2IF_Msk
  TIM_SR_CC3IF_Pos* = (3)
  TIM_SR_CC3IF_Msk* = (0x00000001 shl TIM_SR_CC3IF_Pos) ## !< 0x00000008
  TIM_SR_CC3IF* = TIM_SR_CC3IF_Msk
  TIM_SR_CC4IF_Pos* = (4)
  TIM_SR_CC4IF_Msk* = (0x00000001 shl TIM_SR_CC4IF_Pos) ## !< 0x00000010
  TIM_SR_CC4IF* = TIM_SR_CC4IF_Msk
  TIM_SR_COMIF_Pos* = (5)
  TIM_SR_COMIF_Msk* = (0x00000001 shl TIM_SR_COMIF_Pos) ## !< 0x00000020
  TIM_SR_COMIF* = TIM_SR_COMIF_Msk
  TIM_SR_TIF_Pos* = (6)
  TIM_SR_TIF_Msk* = (0x00000001 shl TIM_SR_TIF_Pos) ## !< 0x00000040
  TIM_SR_TIF* = TIM_SR_TIF_Msk
  TIM_SR_BIF_Pos* = (7)
  TIM_SR_BIF_Msk* = (0x00000001 shl TIM_SR_BIF_Pos) ## !< 0x00000080
  TIM_SR_BIF* = TIM_SR_BIF_Msk
  TIM_SR_CC1OF_Pos* = (9)
  TIM_SR_CC1OF_Msk* = (0x00000001 shl TIM_SR_CC1OF_Pos) ## !< 0x00000200
  TIM_SR_CC1OF* = TIM_SR_CC1OF_Msk
  TIM_SR_CC2OF_Pos* = (10)
  TIM_SR_CC2OF_Msk* = (0x00000001 shl TIM_SR_CC2OF_Pos) ## !< 0x00000400
  TIM_SR_CC2OF* = TIM_SR_CC2OF_Msk
  TIM_SR_CC3OF_Pos* = (11)
  TIM_SR_CC3OF_Msk* = (0x00000001 shl TIM_SR_CC3OF_Pos) ## !< 0x00000800
  TIM_SR_CC3OF* = TIM_SR_CC3OF_Msk
  TIM_SR_CC4OF_Pos* = (12)
  TIM_SR_CC4OF_Msk* = (0x00000001 shl TIM_SR_CC4OF_Pos) ## !< 0x00001000
  TIM_SR_CC4OF* = TIM_SR_CC4OF_Msk

## ******************  Bit definition for TIM_EGR register  *******************

const
  TIM_EGR_UG_Pos* = (0)
  TIM_EGR_UG_Msk* = (0x00000001 shl TIM_EGR_UG_Pos) ## !< 0x00000001
  TIM_EGR_UG* = TIM_EGR_UG_Msk
  TIM_EGR_CC1G_Pos* = (1)
  TIM_EGR_CC1G_Msk* = (0x00000001 shl TIM_EGR_CC1G_Pos) ## !< 0x00000002
  TIM_EGR_CC1G* = TIM_EGR_CC1G_Msk
  TIM_EGR_CC2G_Pos* = (2)
  TIM_EGR_CC2G_Msk* = (0x00000001 shl TIM_EGR_CC2G_Pos) ## !< 0x00000004
  TIM_EGR_CC2G* = TIM_EGR_CC2G_Msk
  TIM_EGR_CC3G_Pos* = (3)
  TIM_EGR_CC3G_Msk* = (0x00000001 shl TIM_EGR_CC3G_Pos) ## !< 0x00000008
  TIM_EGR_CC3G* = TIM_EGR_CC3G_Msk
  TIM_EGR_CC4G_Pos* = (4)
  TIM_EGR_CC4G_Msk* = (0x00000001 shl TIM_EGR_CC4G_Pos) ## !< 0x00000010
  TIM_EGR_CC4G* = TIM_EGR_CC4G_Msk
  TIM_EGR_COMG_Pos* = (5)
  TIM_EGR_COMG_Msk* = (0x00000001 shl TIM_EGR_COMG_Pos) ## !< 0x00000020
  TIM_EGR_COMG* = TIM_EGR_COMG_Msk
  TIM_EGR_TG_Pos* = (6)
  TIM_EGR_TG_Msk* = (0x00000001 shl TIM_EGR_TG_Pos) ## !< 0x00000040
  TIM_EGR_TG* = TIM_EGR_TG_Msk
  TIM_EGR_BG_Pos* = (7)
  TIM_EGR_BG_Msk* = (0x00000001 shl TIM_EGR_BG_Pos) ## !< 0x00000080
  TIM_EGR_BG* = TIM_EGR_BG_Msk

## *****************  Bit definition for TIM_CCMR1 register  ******************

const
  TIM_CCMR1_CC1S_Pos* = (0)
  TIM_CCMR1_CC1S_Msk* = (0x00000003 shl TIM_CCMR1_CC1S_Pos) ## !< 0x00000003
  TIM_CCMR1_CC1S* = TIM_CCMR1_CC1S_Msk
  TIM_CCMR1_CC1S_Bit0* = (0x00000001 shl TIM_CCMR1_CC1S_Pos) ## !< 0x0001
  TIM_CCMR1_CC1S_Bit1* = (0x00000002 shl TIM_CCMR1_CC1S_Pos) ## !< 0x0002
  TIM_CCMR1_OC1FE_Pos* = (2)
  TIM_CCMR1_OC1FE_Msk* = (0x00000001 shl TIM_CCMR1_OC1FE_Pos) ## !< 0x00000004
  TIM_CCMR1_OC1FE* = TIM_CCMR1_OC1FE_Msk
  TIM_CCMR1_OC1PE_Pos* = (3)
  TIM_CCMR1_OC1PE_Msk* = (0x00000001 shl TIM_CCMR1_OC1PE_Pos) ## !< 0x00000008
  TIM_CCMR1_OC1PE* = TIM_CCMR1_OC1PE_Msk
  TIM_CCMR1_OC1M_Pos* = (4)
  TIM_CCMR1_OC1M_Msk* = (0x00000007 shl TIM_CCMR1_OC1M_Pos) ## !< 0x00000070
  TIM_CCMR1_OC1M* = TIM_CCMR1_OC1M_Msk
  TIM_CCMR1_OC1M_Bit0* = (0x00000001 shl TIM_CCMR1_OC1M_Pos) ## !< 0x0010
  TIM_CCMR1_OC1M_Bit1* = (0x00000002 shl TIM_CCMR1_OC1M_Pos) ## !< 0x0020
  TIM_CCMR1_OC1M_Bit2* = (0x00000004 shl TIM_CCMR1_OC1M_Pos) ## !< 0x0040
  TIM_CCMR1_OC1CE_Pos* = (7)
  TIM_CCMR1_OC1CE_Msk* = (0x00000001 shl TIM_CCMR1_OC1CE_Pos) ## !< 0x00000080
  TIM_CCMR1_OC1CE* = TIM_CCMR1_OC1CE_Msk
  TIM_CCMR1_CC2S_Pos* = (8)
  TIM_CCMR1_CC2S_Msk* = (0x00000003 shl TIM_CCMR1_CC2S_Pos) ## !< 0x00000300
  TIM_CCMR1_CC2S* = TIM_CCMR1_CC2S_Msk
  TIM_CCMR1_CC2S_Bit0* = (0x00000001 shl TIM_CCMR1_CC2S_Pos) ## !< 0x0100
  TIM_CCMR1_CC2S_Bit1* = (0x00000002 shl TIM_CCMR1_CC2S_Pos) ## !< 0x0200
  TIM_CCMR1_OC2FE_Pos* = (10)
  TIM_CCMR1_OC2FE_Msk* = (0x00000001 shl TIM_CCMR1_OC2FE_Pos) ## !< 0x00000400
  TIM_CCMR1_OC2FE* = TIM_CCMR1_OC2FE_Msk
  TIM_CCMR1_OC2PE_Pos* = (11)
  TIM_CCMR1_OC2PE_Msk* = (0x00000001 shl TIM_CCMR1_OC2PE_Pos) ## !< 0x00000800
  TIM_CCMR1_OC2PE* = TIM_CCMR1_OC2PE_Msk
  TIM_CCMR1_OC2M_Pos* = (12)
  TIM_CCMR1_OC2M_Msk* = (0x00000007 shl TIM_CCMR1_OC2M_Pos) ## !< 0x00007000
  TIM_CCMR1_OC2M* = TIM_CCMR1_OC2M_Msk
  TIM_CCMR1_OC2M_Bit0* = (0x00000001 shl TIM_CCMR1_OC2M_Pos) ## !< 0x1000
  TIM_CCMR1_OC2M_Bit1* = (0x00000002 shl TIM_CCMR1_OC2M_Pos) ## !< 0x2000
  TIM_CCMR1_OC2M_Bit2* = (0x00000004 shl TIM_CCMR1_OC2M_Pos) ## !< 0x4000
  TIM_CCMR1_OC2CE_Pos* = (15)
  TIM_CCMR1_OC2CE_Msk* = (0x00000001 shl TIM_CCMR1_OC2CE_Pos) ## !< 0x00008000
  TIM_CCMR1_OC2CE* = TIM_CCMR1_OC2CE_Msk

## ----------------------------------------------------------------------------

const
  TIM_CCMR1_IC1PSC_Pos* = (2)
  TIM_CCMR1_IC1PSC_Msk* = (0x00000003 shl TIM_CCMR1_IC1PSC_Pos) ## !< 0x0000000C
  TIM_CCMR1_IC1PSC* = TIM_CCMR1_IC1PSC_Msk
  TIM_CCMR1_IC1PSC_Bit0* = (0x00000001 shl TIM_CCMR1_IC1PSC_Pos) ## !< 0x0004
  TIM_CCMR1_IC1PSC_Bit1* = (0x00000002 shl TIM_CCMR1_IC1PSC_Pos) ## !< 0x0008
  TIM_CCMR1_IC1F_Pos* = (4)
  TIM_CCMR1_IC1F_Msk* = (0x0000000F shl TIM_CCMR1_IC1F_Pos) ## !< 0x000000F0
  TIM_CCMR1_IC1F* = TIM_CCMR1_IC1F_Msk
  TIM_CCMR1_IC1F_Bit0* = (0x00000001 shl TIM_CCMR1_IC1F_Pos) ## !< 0x0010
  TIM_CCMR1_IC1F_Bit1* = (0x00000002 shl TIM_CCMR1_IC1F_Pos) ## !< 0x0020
  TIM_CCMR1_IC1F_Bit2* = (0x00000004 shl TIM_CCMR1_IC1F_Pos) ## !< 0x0040
  TIM_CCMR1_IC1F_Bit3* = (0x00000008 shl TIM_CCMR1_IC1F_Pos) ## !< 0x0080
  TIM_CCMR1_IC2PSC_Pos* = (10)
  TIM_CCMR1_IC2PSC_Msk* = (0x00000003 shl TIM_CCMR1_IC2PSC_Pos) ## !< 0x00000C00
  TIM_CCMR1_IC2PSC* = TIM_CCMR1_IC2PSC_Msk
  TIM_CCMR1_IC2PSC_Bit0* = (0x00000001 shl TIM_CCMR1_IC2PSC_Pos) ## !< 0x0400
  TIM_CCMR1_IC2PSC_Bit1* = (0x00000002 shl TIM_CCMR1_IC2PSC_Pos) ## !< 0x0800
  TIM_CCMR1_IC2F_Pos* = (12)
  TIM_CCMR1_IC2F_Msk* = (0x0000000F shl TIM_CCMR1_IC2F_Pos) ## !< 0x0000F000
  TIM_CCMR1_IC2F* = TIM_CCMR1_IC2F_Msk
  TIM_CCMR1_IC2F_Bit0* = (0x00000001 shl TIM_CCMR1_IC2F_Pos) ## !< 0x1000
  TIM_CCMR1_IC2F_Bit1* = (0x00000002 shl TIM_CCMR1_IC2F_Pos) ## !< 0x2000
  TIM_CCMR1_IC2F_Bit2* = (0x00000004 shl TIM_CCMR1_IC2F_Pos) ## !< 0x4000
  TIM_CCMR1_IC2F_Bit3* = (0x00000008 shl TIM_CCMR1_IC2F_Pos) ## !< 0x8000

## *****************  Bit definition for TIM_CCMR2 register  ******************

const
  TIM_CCMR2_CC3S_Pos* = (0)
  TIM_CCMR2_CC3S_Msk* = (0x00000003 shl TIM_CCMR2_CC3S_Pos) ## !< 0x00000003
  TIM_CCMR2_CC3S* = TIM_CCMR2_CC3S_Msk
  TIM_CCMR2_CC3S_Bit0* = (0x00000001 shl TIM_CCMR2_CC3S_Pos) ## !< 0x0001
  TIM_CCMR2_CC3S_Bit1* = (0x00000002 shl TIM_CCMR2_CC3S_Pos) ## !< 0x0002
  TIM_CCMR2_OC3FE_Pos* = (2)
  TIM_CCMR2_OC3FE_Msk* = (0x00000001 shl TIM_CCMR2_OC3FE_Pos) ## !< 0x00000004
  TIM_CCMR2_OC3FE* = TIM_CCMR2_OC3FE_Msk
  TIM_CCMR2_OC3PE_Pos* = (3)
  TIM_CCMR2_OC3PE_Msk* = (0x00000001 shl TIM_CCMR2_OC3PE_Pos) ## !< 0x00000008
  TIM_CCMR2_OC3PE* = TIM_CCMR2_OC3PE_Msk
  TIM_CCMR2_OC3M_Pos* = (4)
  TIM_CCMR2_OC3M_Msk* = (0x00000007 shl TIM_CCMR2_OC3M_Pos) ## !< 0x00000070
  TIM_CCMR2_OC3M* = TIM_CCMR2_OC3M_Msk
  TIM_CCMR2_OC3M_Bit0* = (0x00000001 shl TIM_CCMR2_OC3M_Pos) ## !< 0x0010
  TIM_CCMR2_OC3M_Bit1* = (0x00000002 shl TIM_CCMR2_OC3M_Pos) ## !< 0x0020
  TIM_CCMR2_OC3M_Bit2* = (0x00000004 shl TIM_CCMR2_OC3M_Pos) ## !< 0x0040
  TIM_CCMR2_OC3CE_Pos* = (7)
  TIM_CCMR2_OC3CE_Msk* = (0x00000001 shl TIM_CCMR2_OC3CE_Pos) ## !< 0x00000080
  TIM_CCMR2_OC3CE* = TIM_CCMR2_OC3CE_Msk
  TIM_CCMR2_CC4S_Pos* = (8)
  TIM_CCMR2_CC4S_Msk* = (0x00000003 shl TIM_CCMR2_CC4S_Pos) ## !< 0x00000300
  TIM_CCMR2_CC4S* = TIM_CCMR2_CC4S_Msk
  TIM_CCMR2_CC4S_Bit0* = (0x00000001 shl TIM_CCMR2_CC4S_Pos) ## !< 0x0100
  TIM_CCMR2_CC4S_Bit1* = (0x00000002 shl TIM_CCMR2_CC4S_Pos) ## !< 0x0200
  TIM_CCMR2_OC4FE_Pos* = (10)
  TIM_CCMR2_OC4FE_Msk* = (0x00000001 shl TIM_CCMR2_OC4FE_Pos) ## !< 0x00000400
  TIM_CCMR2_OC4FE* = TIM_CCMR2_OC4FE_Msk
  TIM_CCMR2_OC4PE_Pos* = (11)
  TIM_CCMR2_OC4PE_Msk* = (0x00000001 shl TIM_CCMR2_OC4PE_Pos) ## !< 0x00000800
  TIM_CCMR2_OC4PE* = TIM_CCMR2_OC4PE_Msk
  TIM_CCMR2_OC4M_Pos* = (12)
  TIM_CCMR2_OC4M_Msk* = (0x00000007 shl TIM_CCMR2_OC4M_Pos) ## !< 0x00007000
  TIM_CCMR2_OC4M* = TIM_CCMR2_OC4M_Msk
  TIM_CCMR2_OC4M_Bit0* = (0x00000001 shl TIM_CCMR2_OC4M_Pos) ## !< 0x1000
  TIM_CCMR2_OC4M_Bit1* = (0x00000002 shl TIM_CCMR2_OC4M_Pos) ## !< 0x2000
  TIM_CCMR2_OC4M_Bit2* = (0x00000004 shl TIM_CCMR2_OC4M_Pos) ## !< 0x4000
  TIM_CCMR2_OC4CE_Pos* = (15)
  TIM_CCMR2_OC4CE_Msk* = (0x00000001 shl TIM_CCMR2_OC4CE_Pos) ## !< 0x00008000
  TIM_CCMR2_OC4CE* = TIM_CCMR2_OC4CE_Msk

## ----------------------------------------------------------------------------

const
  TIM_CCMR2_IC3PSC_Pos* = (2)
  TIM_CCMR2_IC3PSC_Msk* = (0x00000003 shl TIM_CCMR2_IC3PSC_Pos) ## !< 0x0000000C
  TIM_CCMR2_IC3PSC* = TIM_CCMR2_IC3PSC_Msk
  TIM_CCMR2_IC3PSC_Bit0* = (0x00000001 shl TIM_CCMR2_IC3PSC_Pos) ## !< 0x0004
  TIM_CCMR2_IC3PSC_Bit1* = (0x00000002 shl TIM_CCMR2_IC3PSC_Pos) ## !< 0x0008
  TIM_CCMR2_IC3F_Pos* = (4)
  TIM_CCMR2_IC3F_Msk* = (0x0000000F shl TIM_CCMR2_IC3F_Pos) ## !< 0x000000F0
  TIM_CCMR2_IC3F* = TIM_CCMR2_IC3F_Msk
  TIM_CCMR2_IC3F_Bit0* = (0x00000001 shl TIM_CCMR2_IC3F_Pos) ## !< 0x0010
  TIM_CCMR2_IC3F_Bit1* = (0x00000002 shl TIM_CCMR2_IC3F_Pos) ## !< 0x0020
  TIM_CCMR2_IC3F_Bit2* = (0x00000004 shl TIM_CCMR2_IC3F_Pos) ## !< 0x0040
  TIM_CCMR2_IC3F_Bit3* = (0x00000008 shl TIM_CCMR2_IC3F_Pos) ## !< 0x0080
  TIM_CCMR2_IC4PSC_Pos* = (10)
  TIM_CCMR2_IC4PSC_Msk* = (0x00000003 shl TIM_CCMR2_IC4PSC_Pos) ## !< 0x00000C00
  TIM_CCMR2_IC4PSC* = TIM_CCMR2_IC4PSC_Msk
  TIM_CCMR2_IC4PSC_Bit0* = (0x00000001 shl TIM_CCMR2_IC4PSC_Pos) ## !< 0x0400
  TIM_CCMR2_IC4PSC_Bit1* = (0x00000002 shl TIM_CCMR2_IC4PSC_Pos) ## !< 0x0800
  TIM_CCMR2_IC4F_Pos* = (12)
  TIM_CCMR2_IC4F_Msk* = (0x0000000F shl TIM_CCMR2_IC4F_Pos) ## !< 0x0000F000
  TIM_CCMR2_IC4F* = TIM_CCMR2_IC4F_Msk
  TIM_CCMR2_IC4F_Bit0* = (0x00000001 shl TIM_CCMR2_IC4F_Pos) ## !< 0x1000
  TIM_CCMR2_IC4F_Bit1* = (0x00000002 shl TIM_CCMR2_IC4F_Pos) ## !< 0x2000
  TIM_CCMR2_IC4F_Bit2* = (0x00000004 shl TIM_CCMR2_IC4F_Pos) ## !< 0x4000
  TIM_CCMR2_IC4F_Bit3* = (0x00000008 shl TIM_CCMR2_IC4F_Pos) ## !< 0x8000

## ******************  Bit definition for TIM_CCER register  ******************

const
  TIM_CCER_CC1E_Pos* = (0)
  TIM_CCER_CC1E_Msk* = (0x00000001 shl TIM_CCER_CC1E_Pos) ## !< 0x00000001
  TIM_CCER_CC1E* = TIM_CCER_CC1E_Msk
  TIM_CCER_CC1P_Pos* = (1)
  TIM_CCER_CC1P_Msk* = (0x00000001 shl TIM_CCER_CC1P_Pos) ## !< 0x00000002
  TIM_CCER_CC1P* = TIM_CCER_CC1P_Msk
  TIM_CCER_CC1NE_Pos* = (2)
  TIM_CCER_CC1NE_Msk* = (0x00000001 shl TIM_CCER_CC1NE_Pos) ## !< 0x00000004
  TIM_CCER_CC1NE* = TIM_CCER_CC1NE_Msk
  TIM_CCER_CC1NP_Pos* = (3)
  TIM_CCER_CC1NP_Msk* = (0x00000001 shl TIM_CCER_CC1NP_Pos) ## !< 0x00000008
  TIM_CCER_CC1NP* = TIM_CCER_CC1NP_Msk
  TIM_CCER_CC2E_Pos* = (4)
  TIM_CCER_CC2E_Msk* = (0x00000001 shl TIM_CCER_CC2E_Pos) ## !< 0x00000010
  TIM_CCER_CC2E* = TIM_CCER_CC2E_Msk
  TIM_CCER_CC2P_Pos* = (5)
  TIM_CCER_CC2P_Msk* = (0x00000001 shl TIM_CCER_CC2P_Pos) ## !< 0x00000020
  TIM_CCER_CC2P* = TIM_CCER_CC2P_Msk
  TIM_CCER_CC2NE_Pos* = (6)
  TIM_CCER_CC2NE_Msk* = (0x00000001 shl TIM_CCER_CC2NE_Pos) ## !< 0x00000040
  TIM_CCER_CC2NE* = TIM_CCER_CC2NE_Msk
  TIM_CCER_CC2NP_Pos* = (7)
  TIM_CCER_CC2NP_Msk* = (0x00000001 shl TIM_CCER_CC2NP_Pos) ## !< 0x00000080
  TIM_CCER_CC2NP* = TIM_CCER_CC2NP_Msk
  TIM_CCER_CC3E_Pos* = (8)
  TIM_CCER_CC3E_Msk* = (0x00000001 shl TIM_CCER_CC3E_Pos) ## !< 0x00000100
  TIM_CCER_CC3E* = TIM_CCER_CC3E_Msk
  TIM_CCER_CC3P_Pos* = (9)
  TIM_CCER_CC3P_Msk* = (0x00000001 shl TIM_CCER_CC3P_Pos) ## !< 0x00000200
  TIM_CCER_CC3P* = TIM_CCER_CC3P_Msk
  TIM_CCER_CC3NE_Pos* = (10)
  TIM_CCER_CC3NE_Msk* = (0x00000001 shl TIM_CCER_CC3NE_Pos) ## !< 0x00000400
  TIM_CCER_CC3NE* = TIM_CCER_CC3NE_Msk
  TIM_CCER_CC3NP_Pos* = (11)
  TIM_CCER_CC3NP_Msk* = (0x00000001 shl TIM_CCER_CC3NP_Pos) ## !< 0x00000800
  TIM_CCER_CC3NP* = TIM_CCER_CC3NP_Msk
  TIM_CCER_CC4E_Pos* = (12)
  TIM_CCER_CC4E_Msk* = (0x00000001 shl TIM_CCER_CC4E_Pos) ## !< 0x00001000
  TIM_CCER_CC4E* = TIM_CCER_CC4E_Msk
  TIM_CCER_CC4P_Pos* = (13)
  TIM_CCER_CC4P_Msk* = (0x00000001 shl TIM_CCER_CC4P_Pos) ## !< 0x00002000
  TIM_CCER_CC4P* = TIM_CCER_CC4P_Msk
  TIM_CCER_CC4NP_Pos* = (15)
  TIM_CCER_CC4NP_Msk* = (0x00000001 shl TIM_CCER_CC4NP_Pos) ## !< 0x00008000
  TIM_CCER_CC4NP* = TIM_CCER_CC4NP_Msk

## ******************  Bit definition for TIM_CNT register  *******************

const
  TIM_CNT_CNT_Pos* = (0)
  TIM_CNT_CNT_Msk* = (0xFFFFFFFF shl TIM_CNT_CNT_Pos) ## !< 0xFFFFFFFF
  TIM_CNT_CNT* = TIM_CNT_CNT_Msk

## ******************  Bit definition for TIM_PSC register  *******************

const
  TIM_PSC_PSC_Pos* = (0)
  TIM_PSC_PSC_Msk* = (0x0000FFFF shl TIM_PSC_PSC_Pos) ## !< 0x0000FFFF
  TIM_PSC_PSC* = TIM_PSC_PSC_Msk

## ******************  Bit definition for TIM_ARR register  *******************

const
  TIM_ARR_ARR_Pos* = (0)
  TIM_ARR_ARR_Msk* = (0xFFFFFFFF shl TIM_ARR_ARR_Pos) ## !< 0xFFFFFFFF
  TIM_ARR_ARR* = TIM_ARR_ARR_Msk

## ******************  Bit definition for TIM_RCR register  *******************

const
  TIM_RCR_REP_Pos* = (0)
  TIM_RCR_REP_Msk* = (0x000000FF shl TIM_RCR_REP_Pos) ## !< 0x000000FF
  TIM_RCR_REP* = TIM_RCR_REP_Msk

## ******************  Bit definition for TIM_CCR1 register  ******************

const
  TIM_CCR1_CCR1_Pos* = (0)
  TIM_CCR1_CCR1_Msk* = (0x0000FFFF shl TIM_CCR1_CCR1_Pos) ## !< 0x0000FFFF
  TIM_CCR1_CCR1* = TIM_CCR1_CCR1_Msk

## ******************  Bit definition for TIM_CCR2 register  ******************

const
  TIM_CCR2_CCR2_Pos* = (0)
  TIM_CCR2_CCR2_Msk* = (0x0000FFFF shl TIM_CCR2_CCR2_Pos) ## !< 0x0000FFFF
  TIM_CCR2_CCR2* = TIM_CCR2_CCR2_Msk

## ******************  Bit definition for TIM_CCR3 register  ******************

const
  TIM_CCR3_CCR3_Pos* = (0)
  TIM_CCR3_CCR3_Msk* = (0x0000FFFF shl TIM_CCR3_CCR3_Pos) ## !< 0x0000FFFF
  TIM_CCR3_CCR3* = TIM_CCR3_CCR3_Msk

## ******************  Bit definition for TIM_CCR4 register  ******************

const
  TIM_CCR4_CCR4_Pos* = (0)
  TIM_CCR4_CCR4_Msk* = (0x0000FFFF shl TIM_CCR4_CCR4_Pos) ## !< 0x0000FFFF
  TIM_CCR4_CCR4* = TIM_CCR4_CCR4_Msk

## ******************  Bit definition for TIM_BDTR register  ******************

const
  TIM_BDTR_DTG_Pos* = (0)
  TIM_BDTR_DTG_Msk* = (0x000000FF shl TIM_BDTR_DTG_Pos) ## !< 0x000000FF
  TIM_BDTR_DTG* = TIM_BDTR_DTG_Msk
  TIM_BDTR_DTG_Bit0* = (0x00000001 shl TIM_BDTR_DTG_Pos) ## !< 0x0001
  TIM_BDTR_DTG_Bit1* = (0x00000002 shl TIM_BDTR_DTG_Pos) ## !< 0x0002
  TIM_BDTR_DTG_Bit2* = (0x00000004 shl TIM_BDTR_DTG_Pos) ## !< 0x0004
  TIM_BDTR_DTG_Bit3* = (0x00000008 shl TIM_BDTR_DTG_Pos) ## !< 0x0008
  TIM_BDTR_DTG_Bit4* = (0x00000010 shl TIM_BDTR_DTG_Pos) ## !< 0x0010
  TIM_BDTR_DTG_Bit5* = (0x00000020 shl TIM_BDTR_DTG_Pos) ## !< 0x0020
  TIM_BDTR_DTG_Bit6* = (0x00000040 shl TIM_BDTR_DTG_Pos) ## !< 0x0040
  TIM_BDTR_DTG_Bit7* = (0x00000080 shl TIM_BDTR_DTG_Pos) ## !< 0x0080
  TIM_BDTR_LOCK_Pos* = (8)
  TIM_BDTR_LOCK_Msk* = (0x00000003 shl TIM_BDTR_LOCK_Pos) ## !< 0x00000300
  TIM_BDTR_LOCK* = TIM_BDTR_LOCK_Msk
  TIM_BDTR_LOCK_Bit0* = (0x00000001 shl TIM_BDTR_LOCK_Pos) ## !< 0x0100
  TIM_BDTR_LOCK_Bit1* = (0x00000002 shl TIM_BDTR_LOCK_Pos) ## !< 0x0200
  TIM_BDTR_OSSI_Pos* = (10)
  TIM_BDTR_OSSI_Msk* = (0x00000001 shl TIM_BDTR_OSSI_Pos) ## !< 0x00000400
  TIM_BDTR_OSSI* = TIM_BDTR_OSSI_Msk
  TIM_BDTR_OSSR_Pos* = (11)
  TIM_BDTR_OSSR_Msk* = (0x00000001 shl TIM_BDTR_OSSR_Pos) ## !< 0x00000800
  TIM_BDTR_OSSR* = TIM_BDTR_OSSR_Msk
  TIM_BDTR_BKE_Pos* = (12)
  TIM_BDTR_BKE_Msk* = (0x00000001 shl TIM_BDTR_BKE_Pos) ## !< 0x00001000
  TIM_BDTR_BKE* = TIM_BDTR_BKE_Msk
  TIM_BDTR_BKP_Pos* = (13)
  TIM_BDTR_BKP_Msk* = (0x00000001 shl TIM_BDTR_BKP_Pos) ## !< 0x00002000
  TIM_BDTR_BKP* = TIM_BDTR_BKP_Msk
  TIM_BDTR_AOE_Pos* = (14)
  TIM_BDTR_AOE_Msk* = (0x00000001 shl TIM_BDTR_AOE_Pos) ## !< 0x00004000
  TIM_BDTR_AOE* = TIM_BDTR_AOE_Msk
  TIM_BDTR_MOE_Pos* = (15)
  TIM_BDTR_MOE_Msk* = (0x00000001 shl TIM_BDTR_MOE_Pos) ## !< 0x00008000
  TIM_BDTR_MOE* = TIM_BDTR_MOE_Msk

## ******************  Bit definition for TIM_DCR register  *******************

const
  TIM_DCR_DBA_Pos* = (0)
  TIM_DCR_DBA_Msk* = (0x0000001F shl TIM_DCR_DBA_Pos) ## !< 0x0000001F
  TIM_DCR_DBA* = TIM_DCR_DBA_Msk
  TIM_DCR_DBA_Bit0* = (0x00000001 shl TIM_DCR_DBA_Pos) ## !< 0x0001
  TIM_DCR_DBA_Bit1* = (0x00000002 shl TIM_DCR_DBA_Pos) ## !< 0x0002
  TIM_DCR_DBA_Bit2* = (0x00000004 shl TIM_DCR_DBA_Pos) ## !< 0x0004
  TIM_DCR_DBA_Bit3* = (0x00000008 shl TIM_DCR_DBA_Pos) ## !< 0x0008
  TIM_DCR_DBA_Bit4* = (0x00000010 shl TIM_DCR_DBA_Pos) ## !< 0x0010
  TIM_DCR_DBL_Pos* = (8)
  TIM_DCR_DBL_Msk* = (0x0000001F shl TIM_DCR_DBL_Pos) ## !< 0x00001F00
  TIM_DCR_DBL* = TIM_DCR_DBL_Msk
  TIM_DCR_DBL_Bit0* = (0x00000001 shl TIM_DCR_DBL_Pos) ## !< 0x0100
  TIM_DCR_DBL_Bit1* = (0x00000002 shl TIM_DCR_DBL_Pos) ## !< 0x0200
  TIM_DCR_DBL_Bit2* = (0x00000004 shl TIM_DCR_DBL_Pos) ## !< 0x0400
  TIM_DCR_DBL_Bit3* = (0x00000008 shl TIM_DCR_DBL_Pos) ## !< 0x0800
  TIM_DCR_DBL_Bit4* = (0x00000010 shl TIM_DCR_DBL_Pos) ## !< 0x1000

## ******************  Bit definition for TIM_DMAR register  ******************

const
  TIM_DMAR_DMAB_Pos* = (0)
  TIM_DMAR_DMAB_Msk* = (0x0000FFFF shl TIM_DMAR_DMAB_Pos) ## !< 0x0000FFFF
  TIM_DMAR_DMAB* = TIM_DMAR_DMAB_Msk

## ******************  Bit definition for TIM_OR register  ********************

const
  TIM_OR_TI1_RMP_Pos* = (0)
  TIM_OR_TI1_RMP_Msk* = (0x00000003 shl TIM_OR_TI1_RMP_Pos) ## !< 0x00000003
  TIM_OR_TI1_RMP* = TIM_OR_TI1_RMP_Msk
  TIM_OR_TI1_RMP_Bit0* = (0x00000001 shl TIM_OR_TI1_RMP_Pos) ## !< 0x00000001
  TIM_OR_TI1_RMP_Bit1* = (0x00000002 shl TIM_OR_TI1_RMP_Pos) ## !< 0x00000002
  TIM_OR_TI4_RMP_Pos* = (6)
  TIM_OR_TI4_RMP_Msk* = (0x00000003 shl TIM_OR_TI4_RMP_Pos) ## !< 0x000000C0
  TIM_OR_TI4_RMP* = TIM_OR_TI4_RMP_Msk
  TIM_OR_TI4_RMP_Bit0* = (0x00000001 shl TIM_OR_TI4_RMP_Pos) ## !< 0x0040
  TIM_OR_TI4_RMP_Bit1* = (0x00000002 shl TIM_OR_TI4_RMP_Pos) ## !< 0x0080

## ****************************************************************************
##
##                          Low Power Timer (LPTIM)
##
## ****************************************************************************
## *****************  Bit definition for LPTIM_ISR register  ******************

const
  LPTIM_ISR_CMPM_Pos* = (0)
  LPTIM_ISR_CMPM_Msk* = (0x00000001 shl LPTIM_ISR_CMPM_Pos) ## !< 0x00000001
  LPTIM_ISR_CMPM* = LPTIM_ISR_CMPM_Msk
  LPTIM_ISR_ARRM_Pos* = (1)
  LPTIM_ISR_ARRM_Msk* = (0x00000001 shl LPTIM_ISR_ARRM_Pos) ## !< 0x00000002
  LPTIM_ISR_ARRM* = LPTIM_ISR_ARRM_Msk
  LPTIM_ISR_EXTTRIG_Pos* = (2)
  LPTIM_ISR_EXTTRIG_Msk* = (0x00000001 shl LPTIM_ISR_EXTTRIG_Pos) ## !< 0x00000004
  LPTIM_ISR_EXTTRIG* = LPTIM_ISR_EXTTRIG_Msk
  LPTIM_ISR_CMPOK_Pos* = (3)
  LPTIM_ISR_CMPOK_Msk* = (0x00000001 shl LPTIM_ISR_CMPOK_Pos) ## !< 0x00000008
  LPTIM_ISR_CMPOK* = LPTIM_ISR_CMPOK_Msk
  LPTIM_ISR_ARROK_Pos* = (4)
  LPTIM_ISR_ARROK_Msk* = (0x00000001 shl LPTIM_ISR_ARROK_Pos) ## !< 0x00000010
  LPTIM_ISR_ARROK* = LPTIM_ISR_ARROK_Msk
  LPTIM_ISR_UP_Pos* = (5)
  LPTIM_ISR_UP_Msk* = (0x00000001 shl LPTIM_ISR_UP_Pos) ## !< 0x00000020
  LPTIM_ISR_UP* = LPTIM_ISR_UP_Msk
  LPTIM_ISR_DOWN_Pos* = (6)
  LPTIM_ISR_DOWN_Msk* = (0x00000001 shl LPTIM_ISR_DOWN_Pos) ## !< 0x00000040
  LPTIM_ISR_DOWN* = LPTIM_ISR_DOWN_Msk

## *****************  Bit definition for LPTIM_ICR register  ******************

const
  LPTIM_ICR_CMPMCF_Pos* = (0)
  LPTIM_ICR_CMPMCF_Msk* = (0x00000001 shl LPTIM_ICR_CMPMCF_Pos) ## !< 0x00000001
  LPTIM_ICR_CMPMCF* = LPTIM_ICR_CMPMCF_Msk
  LPTIM_ICR_ARRMCF_Pos* = (1)
  LPTIM_ICR_ARRMCF_Msk* = (0x00000001 shl LPTIM_ICR_ARRMCF_Pos) ## !< 0x00000002
  LPTIM_ICR_ARRMCF* = LPTIM_ICR_ARRMCF_Msk
  LPTIM_ICR_EXTTRIGCF_Pos* = (2)
  LPTIM_ICR_EXTTRIGCF_Msk* = (0x00000001 shl LPTIM_ICR_EXTTRIGCF_Pos) ## !< 0x00000004
  LPTIM_ICR_EXTTRIGCF* = LPTIM_ICR_EXTTRIGCF_Msk
  LPTIM_ICR_CMPOKCF_Pos* = (3)
  LPTIM_ICR_CMPOKCF_Msk* = (0x00000001 shl LPTIM_ICR_CMPOKCF_Pos) ## !< 0x00000008
  LPTIM_ICR_CMPOKCF* = LPTIM_ICR_CMPOKCF_Msk
  LPTIM_ICR_ARROKCF_Pos* = (4)
  LPTIM_ICR_ARROKCF_Msk* = (0x00000001 shl LPTIM_ICR_ARROKCF_Pos) ## !< 0x00000010
  LPTIM_ICR_ARROKCF* = LPTIM_ICR_ARROKCF_Msk
  LPTIM_ICR_UPCF_Pos* = (5)
  LPTIM_ICR_UPCF_Msk* = (0x00000001 shl LPTIM_ICR_UPCF_Pos) ## !< 0x00000020
  LPTIM_ICR_UPCF* = LPTIM_ICR_UPCF_Msk
  LPTIM_ICR_DOWNCF_Pos* = (6)
  LPTIM_ICR_DOWNCF_Msk* = (0x00000001 shl LPTIM_ICR_DOWNCF_Pos) ## !< 0x00000040
  LPTIM_ICR_DOWNCF* = LPTIM_ICR_DOWNCF_Msk

## *****************  Bit definition for LPTIM_IER register *******************

const
  LPTIM_IER_CMPMIE_Pos* = (0)
  LPTIM_IER_CMPMIE_Msk* = (0x00000001 shl LPTIM_IER_CMPMIE_Pos) ## !< 0x00000001
  LPTIM_IER_CMPMIE* = LPTIM_IER_CMPMIE_Msk
  LPTIM_IER_ARRMIE_Pos* = (1)
  LPTIM_IER_ARRMIE_Msk* = (0x00000001 shl LPTIM_IER_ARRMIE_Pos) ## !< 0x00000002
  LPTIM_IER_ARRMIE* = LPTIM_IER_ARRMIE_Msk
  LPTIM_IER_EXTTRIGIE_Pos* = (2)
  LPTIM_IER_EXTTRIGIE_Msk* = (0x00000001 shl LPTIM_IER_EXTTRIGIE_Pos) ## !< 0x00000004
  LPTIM_IER_EXTTRIGIE* = LPTIM_IER_EXTTRIGIE_Msk
  LPTIM_IER_CMPOKIE_Pos* = (3)
  LPTIM_IER_CMPOKIE_Msk* = (0x00000001 shl LPTIM_IER_CMPOKIE_Pos) ## !< 0x00000008
  LPTIM_IER_CMPOKIE* = LPTIM_IER_CMPOKIE_Msk
  LPTIM_IER_ARROKIE_Pos* = (4)
  LPTIM_IER_ARROKIE_Msk* = (0x00000001 shl LPTIM_IER_ARROKIE_Pos) ## !< 0x00000010
  LPTIM_IER_ARROKIE* = LPTIM_IER_ARROKIE_Msk
  LPTIM_IER_UPIE_Pos* = (5)
  LPTIM_IER_UPIE_Msk* = (0x00000001 shl LPTIM_IER_UPIE_Pos) ## !< 0x00000020
  LPTIM_IER_UPIE* = LPTIM_IER_UPIE_Msk
  LPTIM_IER_DOWNIE_Pos* = (6)
  LPTIM_IER_DOWNIE_Msk* = (0x00000001 shl LPTIM_IER_DOWNIE_Pos) ## !< 0x00000040
  LPTIM_IER_DOWNIE* = LPTIM_IER_DOWNIE_Msk

## *****************  Bit definition for LPTIM_CFGR register ******************

const
  LPTIM_CFGR_CKSEL_Pos* = (0)
  LPTIM_CFGR_CKSEL_Msk* = (0x00000001 shl LPTIM_CFGR_CKSEL_Pos) ## !< 0x00000001
  LPTIM_CFGR_CKSEL* = LPTIM_CFGR_CKSEL_Msk
  LPTIM_CFGR_CKPOL_Pos* = (1)
  LPTIM_CFGR_CKPOL_Msk* = (0x00000003 shl LPTIM_CFGR_CKPOL_Pos) ## !< 0x00000006
  LPTIM_CFGR_CKPOL* = LPTIM_CFGR_CKPOL_Msk
  LPTIM_CFGR_CKPOL_Bit0* = (0x00000001 shl LPTIM_CFGR_CKPOL_Pos) ## !< 0x00000002
  LPTIM_CFGR_CKPOL_Bit1* = (0x00000002 shl LPTIM_CFGR_CKPOL_Pos) ## !< 0x00000004
  LPTIM_CFGR_CKFLT_Pos* = (3)
  LPTIM_CFGR_CKFLT_Msk* = (0x00000003 shl LPTIM_CFGR_CKFLT_Pos) ## !< 0x00000018
  LPTIM_CFGR_CKFLT* = LPTIM_CFGR_CKFLT_Msk
  LPTIM_CFGR_CKFLT_Bit0* = (0x00000001 shl LPTIM_CFGR_CKFLT_Pos) ## !< 0x00000008
  LPTIM_CFGR_CKFLT_Bit1* = (0x00000002 shl LPTIM_CFGR_CKFLT_Pos) ## !< 0x00000010
  LPTIM_CFGR_TRGFLT_Pos* = (6)
  LPTIM_CFGR_TRGFLT_Msk* = (0x00000003 shl LPTIM_CFGR_TRGFLT_Pos) ## !< 0x000000C0
  LPTIM_CFGR_TRGFLT* = LPTIM_CFGR_TRGFLT_Msk
  LPTIM_CFGR_TRGFLT_Bit0* = (0x00000001 shl LPTIM_CFGR_TRGFLT_Pos) ## !< 0x00000040
  LPTIM_CFGR_TRGFLT_Bit1* = (0x00000002 shl LPTIM_CFGR_TRGFLT_Pos) ## !< 0x00000080
  LPTIM_CFGR_PRESC_Pos* = (9)
  LPTIM_CFGR_PRESC_Msk* = (0x00000007 shl LPTIM_CFGR_PRESC_Pos) ## !< 0x00000E00
  LPTIM_CFGR_PRESC* = LPTIM_CFGR_PRESC_Msk
  LPTIM_CFGR_PRESC_Bit0* = (0x00000001 shl LPTIM_CFGR_PRESC_Pos) ## !< 0x00000200
  LPTIM_CFGR_PRESC_Bit1* = (0x00000002 shl LPTIM_CFGR_PRESC_Pos) ## !< 0x00000400
  LPTIM_CFGR_PRESC_Bit2* = (0x00000004 shl LPTIM_CFGR_PRESC_Pos) ## !< 0x00000800
  LPTIM_CFGR_TRIGSEL_Pos* = (13)
  LPTIM_CFGR_TRIGSEL_Msk* = (0x00000007 shl LPTIM_CFGR_TRIGSEL_Pos) ## !< 0x0000E000
  LPTIM_CFGR_TRIGSEL* = LPTIM_CFGR_TRIGSEL_Msk
  LPTIM_CFGR_TRIGSEL_Bit0* = (0x00000001 shl LPTIM_CFGR_TRIGSEL_Pos) ## !< 0x00002000
  LPTIM_CFGR_TRIGSEL_Bit1* = (0x00000002 shl LPTIM_CFGR_TRIGSEL_Pos) ## !< 0x00004000
  LPTIM_CFGR_TRIGSEL_Bit2* = (0x00000004 shl LPTIM_CFGR_TRIGSEL_Pos) ## !< 0x00008000
  LPTIM_CFGR_TRIGEN_Pos* = (17)
  LPTIM_CFGR_TRIGEN_Msk* = (0x00000003 shl LPTIM_CFGR_TRIGEN_Pos) ## !< 0x00060000
  LPTIM_CFGR_TRIGEN* = LPTIM_CFGR_TRIGEN_Msk
  LPTIM_CFGR_TRIGEN_Bit0* = (0x00000001 shl LPTIM_CFGR_TRIGEN_Pos) ## !< 0x00020000
  LPTIM_CFGR_TRIGEN_Bit1* = (0x00000002 shl LPTIM_CFGR_TRIGEN_Pos) ## !< 0x00040000
  LPTIM_CFGR_TIMOUT_Pos* = (19)
  LPTIM_CFGR_TIMOUT_Msk* = (0x00000001 shl LPTIM_CFGR_TIMOUT_Pos) ## !< 0x00080000
  LPTIM_CFGR_TIMOUT* = LPTIM_CFGR_TIMOUT_Msk
  LPTIM_CFGR_WAVE_Pos* = (20)
  LPTIM_CFGR_WAVE_Msk* = (0x00000001 shl LPTIM_CFGR_WAVE_Pos) ## !< 0x00100000
  LPTIM_CFGR_WAVE* = LPTIM_CFGR_WAVE_Msk
  LPTIM_CFGR_WAVPOL_Pos* = (21)
  LPTIM_CFGR_WAVPOL_Msk* = (0x00000001 shl LPTIM_CFGR_WAVPOL_Pos) ## !< 0x00200000
  LPTIM_CFGR_WAVPOL* = LPTIM_CFGR_WAVPOL_Msk
  LPTIM_CFGR_PRELOAD_Pos* = (22)
  LPTIM_CFGR_PRELOAD_Msk* = (0x00000001 shl LPTIM_CFGR_PRELOAD_Pos) ## !< 0x00400000
  LPTIM_CFGR_PRELOAD* = LPTIM_CFGR_PRELOAD_Msk
  LPTIM_CFGR_COUNTMODE_Pos* = (23)
  LPTIM_CFGR_COUNTMODE_Msk* = (0x00000001 shl LPTIM_CFGR_COUNTMODE_Pos) ## !< 0x00800000
  LPTIM_CFGR_COUNTMODE* = LPTIM_CFGR_COUNTMODE_Msk
  LPTIM_CFGR_ENC_Pos* = (24)
  LPTIM_CFGR_ENC_Msk* = (0x00000001 shl LPTIM_CFGR_ENC_Pos) ## !< 0x01000000
  LPTIM_CFGR_ENC* = LPTIM_CFGR_ENC_Msk

## *****************  Bit definition for LPTIM_CR register  *******************

const
  LPTIM_CR_ENABLE_Pos* = (0)
  LPTIM_CR_ENABLE_Msk* = (0x00000001 shl LPTIM_CR_ENABLE_Pos) ## !< 0x00000001
  LPTIM_CR_ENABLE* = LPTIM_CR_ENABLE_Msk
  LPTIM_CR_SNGSTRT_Pos* = (1)
  LPTIM_CR_SNGSTRT_Msk* = (0x00000001 shl LPTIM_CR_SNGSTRT_Pos) ## !< 0x00000002
  LPTIM_CR_SNGSTRT* = LPTIM_CR_SNGSTRT_Msk
  LPTIM_CR_CNTSTRT_Pos* = (2)
  LPTIM_CR_CNTSTRT_Msk* = (0x00000001 shl LPTIM_CR_CNTSTRT_Pos) ## !< 0x00000004
  LPTIM_CR_CNTSTRT* = LPTIM_CR_CNTSTRT_Msk

## *****************  Bit definition for LPTIM_CMP register  ******************

const
  LPTIM_CMP_CMP_Pos* = (0)
  LPTIM_CMP_CMP_Msk* = (0x0000FFFF shl LPTIM_CMP_CMP_Pos) ## !< 0x0000FFFF
  LPTIM_CMP_CMP* = LPTIM_CMP_CMP_Msk

## *****************  Bit definition for LPTIM_ARR register  ******************

const
  LPTIM_ARR_ARR_Pos* = (0)
  LPTIM_ARR_ARR_Msk* = (0x0000FFFF shl LPTIM_ARR_ARR_Pos) ## !< 0x0000FFFF
  LPTIM_ARR_ARR* = LPTIM_ARR_ARR_Msk

## *****************  Bit definition for LPTIM_CNT register  ******************

const
  LPTIM_CNT_CNT_Pos* = (0)
  LPTIM_CNT_CNT_Msk* = (0x0000FFFF shl LPTIM_CNT_CNT_Pos) ## !< 0x0000FFFF
  LPTIM_CNT_CNT* = LPTIM_CNT_CNT_Msk

## *****************  Bit definition for LPTIM_OR register  ******************

const
  LPTIM_OR_LPT_IN1_RMP_Pos* = (0)
  LPTIM_OR_LPT_IN1_RMP_Msk* = (0x00000003 shl LPTIM_OR_LPT_IN1_RMP_Pos) ## !< 0x00000003
  LPTIM_OR_LPT_IN1_RMP* = LPTIM_OR_LPT_IN1_RMP_Msk
  LPTIM_OR_LPT_IN1_RMP_Bit0* = (0x00000001 shl LPTIM_OR_LPT_IN1_RMP_Pos) ## !< 0x00000001
  LPTIM_OR_LPT_IN1_RMP_Bit1* = (0x00000002 shl LPTIM_OR_LPT_IN1_RMP_Pos) ## !< 0x00000002

##  Legacy Defines

const
  LPTIM_OR_OR* = LPTIM_OR_LPT_IN1_RMP
  LPTIM_OR_OR_Bit0* = LPTIM_OR_LPT_IN1_RMP_Bit0
  LPTIM_OR_OR_Bit1* = LPTIM_OR_LPT_IN1_RMP_Bit1

## ****************************************************************************
##
##          Universal Synchronous Asynchronous Receiver Transmitter
##
## ****************************************************************************
## ******************  Bit definition for USART_SR register  ******************

const
  USART_SR_PE_Pos* = (0)
  USART_SR_PE_Msk* = (0x00000001 shl USART_SR_PE_Pos) ## !< 0x00000001
  USART_SR_PE* = USART_SR_PE_Msk
  USART_SR_FE_Pos* = (1)
  USART_SR_FE_Msk* = (0x00000001 shl USART_SR_FE_Pos) ## !< 0x00000002
  USART_SR_FE* = USART_SR_FE_Msk
  USART_SR_NE_Pos* = (2)
  USART_SR_NE_Msk* = (0x00000001 shl USART_SR_NE_Pos) ## !< 0x00000004
  USART_SR_NE* = USART_SR_NE_Msk
  USART_SR_ORE_Pos* = (3)
  USART_SR_ORE_Msk* = (0x00000001 shl USART_SR_ORE_Pos) ## !< 0x00000008
  USART_SR_ORE* = USART_SR_ORE_Msk
  USART_SR_IDLE_Pos* = (4)
  USART_SR_IDLE_Msk* = (0x00000001 shl USART_SR_IDLE_Pos) ## !< 0x00000010
  USART_SR_IDLE* = USART_SR_IDLE_Msk
  USART_SR_RXNE_Pos* = (5)
  USART_SR_RXNE_Msk* = (0x00000001 shl USART_SR_RXNE_Pos) ## !< 0x00000020
  USART_SR_RXNE* = USART_SR_RXNE_Msk
  USART_SR_TC_Pos* = (6)
  USART_SR_TC_Msk* = (0x00000001 shl USART_SR_TC_Pos) ## !< 0x00000040
  USART_SR_TC* = USART_SR_TC_Msk
  USART_SR_TXE_Pos* = (7)
  USART_SR_TXE_Msk* = (0x00000001 shl USART_SR_TXE_Pos) ## !< 0x00000080
  USART_SR_TXE* = USART_SR_TXE_Msk
  USART_SR_LBD_Pos* = (8)
  USART_SR_LBD_Msk* = (0x00000001 shl USART_SR_LBD_Pos) ## !< 0x00000100
  USART_SR_LBD* = USART_SR_LBD_Msk
  USART_SR_CTS_Pos* = (9)
  USART_SR_CTS_Msk* = (0x00000001 shl USART_SR_CTS_Pos) ## !< 0x00000200
  USART_SR_CTS* = USART_SR_CTS_Msk

## ******************  Bit definition for USART_DR register  ******************

const
  USART_DR_DR_Pos* = (0)
  USART_DR_DR_Msk* = (0x000001FF shl USART_DR_DR_Pos) ## !< 0x000001FF
  USART_DR_DR* = USART_DR_DR_Msk

## *****************  Bit definition for USART_BRR register  ******************

const
  USART_BRR_DIV_Fraction_Pos* = (0)
  USART_BRR_DIV_Fraction_Msk* = (0x0000000F shl USART_BRR_DIV_Fraction_Pos) ## !< 0x0000000F
  USART_BRR_DIV_Fraction* = USART_BRR_DIV_Fraction_Msk
  USART_BRR_DIV_Mantissa_Pos* = (4)
  USART_BRR_DIV_Mantissa_Msk* = (0x00000FFF shl USART_BRR_DIV_Mantissa_Pos) ## !< 0x0000FFF0
  USART_BRR_DIV_Mantissa* = USART_BRR_DIV_Mantissa_Msk

## *****************  Bit definition for USART_CR1 register  ******************

const
  USART_CR1_SBK_Pos* = (0)
  USART_CR1_SBK_Msk* = (0x00000001 shl USART_CR1_SBK_Pos) ## !< 0x00000001
  USART_CR1_SBK* = USART_CR1_SBK_Msk
  USART_CR1_RWU_Pos* = (1)
  USART_CR1_RWU_Msk* = (0x00000001 shl USART_CR1_RWU_Pos) ## !< 0x00000002
  USART_CR1_RWU* = USART_CR1_RWU_Msk
  USART_CR1_RE_Pos* = (2)
  USART_CR1_RE_Msk* = (0x00000001 shl USART_CR1_RE_Pos) ## !< 0x00000004
  USART_CR1_RE* = USART_CR1_RE_Msk
  USART_CR1_TE_Pos* = (3)
  USART_CR1_TE_Msk* = (0x00000001 shl USART_CR1_TE_Pos) ## !< 0x00000008
  USART_CR1_TE* = USART_CR1_TE_Msk
  USART_CR1_IDLEIE_Pos* = (4)
  USART_CR1_IDLEIE_Msk* = (0x00000001 shl USART_CR1_IDLEIE_Pos) ## !< 0x00000010
  USART_CR1_IDLEIE* = USART_CR1_IDLEIE_Msk
  USART_CR1_RXNEIE_Pos* = (5)
  USART_CR1_RXNEIE_Msk* = (0x00000001 shl USART_CR1_RXNEIE_Pos) ## !< 0x00000020
  USART_CR1_RXNEIE* = USART_CR1_RXNEIE_Msk
  USART_CR1_TCIE_Pos* = (6)
  USART_CR1_TCIE_Msk* = (0x00000001 shl USART_CR1_TCIE_Pos) ## !< 0x00000040
  USART_CR1_TCIE* = USART_CR1_TCIE_Msk
  USART_CR1_TXEIE_Pos* = (7)
  USART_CR1_TXEIE_Msk* = (0x00000001 shl USART_CR1_TXEIE_Pos) ## !< 0x00000080
  USART_CR1_TXEIE* = USART_CR1_TXEIE_Msk
  USART_CR1_PEIE_Pos* = (8)
  USART_CR1_PEIE_Msk* = (0x00000001 shl USART_CR1_PEIE_Pos) ## !< 0x00000100
  USART_CR1_PEIE* = USART_CR1_PEIE_Msk
  USART_CR1_PS_Pos* = (9)
  USART_CR1_PS_Msk* = (0x00000001 shl USART_CR1_PS_Pos) ## !< 0x00000200
  USART_CR1_PS* = USART_CR1_PS_Msk
  USART_CR1_PCE_Pos* = (10)
  USART_CR1_PCE_Msk* = (0x00000001 shl USART_CR1_PCE_Pos) ## !< 0x00000400
  USART_CR1_PCE* = USART_CR1_PCE_Msk
  USART_CR1_WAKE_Pos* = (11)
  USART_CR1_WAKE_Msk* = (0x00000001 shl USART_CR1_WAKE_Pos) ## !< 0x00000800
  USART_CR1_WAKE* = USART_CR1_WAKE_Msk
  USART_CR1_M_Pos* = (12)
  USART_CR1_M_Msk* = (0x00000001 shl USART_CR1_M_Pos) ## !< 0x00001000
  USART_CR1_M* = USART_CR1_M_Msk
  USART_CR1_UE_Pos* = (13)
  USART_CR1_UE_Msk* = (0x00000001 shl USART_CR1_UE_Pos) ## !< 0x00002000
  USART_CR1_UE* = USART_CR1_UE_Msk
  USART_CR1_OVER8_Pos* = (15)
  USART_CR1_OVER8_Msk* = (0x00000001 shl USART_CR1_OVER8_Pos) ## !< 0x00008000
  USART_CR1_OVER8* = USART_CR1_OVER8_Msk

## *****************  Bit definition for USART_CR2 register  ******************

const
  USART_CR2_ADD_Pos* = (0)
  USART_CR2_ADD_Msk* = (0x0000000F shl USART_CR2_ADD_Pos) ## !< 0x0000000F
  USART_CR2_ADD* = USART_CR2_ADD_Msk
  USART_CR2_LBDL_Pos* = (5)
  USART_CR2_LBDL_Msk* = (0x00000001 shl USART_CR2_LBDL_Pos) ## !< 0x00000020
  USART_CR2_LBDL* = USART_CR2_LBDL_Msk
  USART_CR2_LBDIE_Pos* = (6)
  USART_CR2_LBDIE_Msk* = (0x00000001 shl USART_CR2_LBDIE_Pos) ## !< 0x00000040
  USART_CR2_LBDIE* = USART_CR2_LBDIE_Msk
  USART_CR2_LBCL_Pos* = (8)
  USART_CR2_LBCL_Msk* = (0x00000001 shl USART_CR2_LBCL_Pos) ## !< 0x00000100
  USART_CR2_LBCL* = USART_CR2_LBCL_Msk
  USART_CR2_CPHA_Pos* = (9)
  USART_CR2_CPHA_Msk* = (0x00000001 shl USART_CR2_CPHA_Pos) ## !< 0x00000200
  USART_CR2_CPHA* = USART_CR2_CPHA_Msk
  USART_CR2_CPOL_Pos* = (10)
  USART_CR2_CPOL_Msk* = (0x00000001 shl USART_CR2_CPOL_Pos) ## !< 0x00000400
  USART_CR2_CPOL* = USART_CR2_CPOL_Msk
  USART_CR2_CLKEN_Pos* = (11)
  USART_CR2_CLKEN_Msk* = (0x00000001 shl USART_CR2_CLKEN_Pos) ## !< 0x00000800
  USART_CR2_CLKEN* = USART_CR2_CLKEN_Msk
  USART_CR2_STOP_Pos* = (12)
  USART_CR2_STOP_Msk* = (0x00000003 shl USART_CR2_STOP_Pos) ## !< 0x00003000
  USART_CR2_STOP* = USART_CR2_STOP_Msk
  USART_CR2_STOP_Bit0* = (0x00000001 shl USART_CR2_STOP_Pos) ## !< 0x1000
  USART_CR2_STOP_Bit1* = (0x00000002 shl USART_CR2_STOP_Pos) ## !< 0x2000
  USART_CR2_LINEN_Pos* = (14)
  USART_CR2_LINEN_Msk* = (0x00000001 shl USART_CR2_LINEN_Pos) ## !< 0x00004000
  USART_CR2_LINEN* = USART_CR2_LINEN_Msk

## *****************  Bit definition for USART_CR3 register  ******************

const
  USART_CR3_EIE_Pos* = (0)
  USART_CR3_EIE_Msk* = (0x00000001 shl USART_CR3_EIE_Pos) ## !< 0x00000001
  USART_CR3_EIE* = USART_CR3_EIE_Msk
  USART_CR3_IREN_Pos* = (1)
  USART_CR3_IREN_Msk* = (0x00000001 shl USART_CR3_IREN_Pos) ## !< 0x00000002
  USART_CR3_IREN* = USART_CR3_IREN_Msk
  USART_CR3_IRLP_Pos* = (2)
  USART_CR3_IRLP_Msk* = (0x00000001 shl USART_CR3_IRLP_Pos) ## !< 0x00000004
  USART_CR3_IRLP* = USART_CR3_IRLP_Msk
  USART_CR3_HDSEL_Pos* = (3)
  USART_CR3_HDSEL_Msk* = (0x00000001 shl USART_CR3_HDSEL_Pos) ## !< 0x00000008
  USART_CR3_HDSEL* = USART_CR3_HDSEL_Msk
  USART_CR3_NACK_Pos* = (4)
  USART_CR3_NACK_Msk* = (0x00000001 shl USART_CR3_NACK_Pos) ## !< 0x00000010
  USART_CR3_NACK* = USART_CR3_NACK_Msk
  USART_CR3_SCEN_Pos* = (5)
  USART_CR3_SCEN_Msk* = (0x00000001 shl USART_CR3_SCEN_Pos) ## !< 0x00000020
  USART_CR3_SCEN* = USART_CR3_SCEN_Msk
  USART_CR3_DMAR_Pos* = (6)
  USART_CR3_DMAR_Msk* = (0x00000001 shl USART_CR3_DMAR_Pos) ## !< 0x00000040
  USART_CR3_DMAR* = USART_CR3_DMAR_Msk
  USART_CR3_DMAT_Pos* = (7)
  USART_CR3_DMAT_Msk* = (0x00000001 shl USART_CR3_DMAT_Pos) ## !< 0x00000080
  USART_CR3_DMAT* = USART_CR3_DMAT_Msk
  USART_CR3_RTSE_Pos* = (8)
  USART_CR3_RTSE_Msk* = (0x00000001 shl USART_CR3_RTSE_Pos) ## !< 0x00000100
  USART_CR3_RTSE* = USART_CR3_RTSE_Msk
  USART_CR3_CTSE_Pos* = (9)
  USART_CR3_CTSE_Msk* = (0x00000001 shl USART_CR3_CTSE_Pos) ## !< 0x00000200
  USART_CR3_CTSE* = USART_CR3_CTSE_Msk
  USART_CR3_CTSIE_Pos* = (10)
  USART_CR3_CTSIE_Msk* = (0x00000001 shl USART_CR3_CTSIE_Pos) ## !< 0x00000400
  USART_CR3_CTSIE* = USART_CR3_CTSIE_Msk
  USART_CR3_ONEBIT_Pos* = (11)
  USART_CR3_ONEBIT_Msk* = (0x00000001 shl USART_CR3_ONEBIT_Pos) ## !< 0x00000800
  USART_CR3_ONEBIT* = USART_CR3_ONEBIT_Msk

## *****************  Bit definition for USART_GTPR register  *****************

const
  USART_GTPR_PSC_Pos* = (0)
  USART_GTPR_PSC_Msk* = (0x000000FF shl USART_GTPR_PSC_Pos) ## !< 0x000000FF
  USART_GTPR_PSC* = USART_GTPR_PSC_Msk
  USART_GTPR_PSC_Bit0* = (0x00000001 shl USART_GTPR_PSC_Pos) ## !< 0x0001
  USART_GTPR_PSC_Bit1* = (0x00000002 shl USART_GTPR_PSC_Pos) ## !< 0x0002
  USART_GTPR_PSC_Bit2* = (0x00000004 shl USART_GTPR_PSC_Pos) ## !< 0x0004
  USART_GTPR_PSC_Bit3* = (0x00000008 shl USART_GTPR_PSC_Pos) ## !< 0x0008
  USART_GTPR_PSC_Bit4* = (0x00000010 shl USART_GTPR_PSC_Pos) ## !< 0x0010
  USART_GTPR_PSC_Bit5* = (0x00000020 shl USART_GTPR_PSC_Pos) ## !< 0x0020
  USART_GTPR_PSC_Bit6* = (0x00000040 shl USART_GTPR_PSC_Pos) ## !< 0x0040
  USART_GTPR_PSC_Bit7* = (0x00000080 shl USART_GTPR_PSC_Pos) ## !< 0x0080
  USART_GTPR_GT_Pos* = (8)
  USART_GTPR_GT_Msk* = (0x000000FF shl USART_GTPR_GT_Pos) ## !< 0x0000FF00
  USART_GTPR_GT* = USART_GTPR_GT_Msk

## ****************************************************************************
##
##                             Window WATCHDOG
##
## ****************************************************************************
## ******************  Bit definition for WWDG_CR register  *******************

const
  WWDG_CR_T_Pos* = (0)
  WWDG_CR_T_Msk* = (0x0000007F shl WWDG_CR_T_Pos) ## !< 0x0000007F
  WWDG_CR_T* = WWDG_CR_T_Msk
  WWDG_CR_T_Bit0* = (0x00000001 shl WWDG_CR_T_Pos) ## !< 0x01
  WWDG_CR_T_Bit1* = (0x00000002 shl WWDG_CR_T_Pos) ## !< 0x02
  WWDG_CR_T_Bit2* = (0x00000004 shl WWDG_CR_T_Pos) ## !< 0x04
  WWDG_CR_T_Bit3* = (0x00000008 shl WWDG_CR_T_Pos) ## !< 0x08
  WWDG_CR_T_Bit4* = (0x00000010 shl WWDG_CR_T_Pos) ## !< 0x10
  WWDG_CR_T_Bit5* = (0x00000020 shl WWDG_CR_T_Pos) ## !< 0x20
  WWDG_CR_T_Bit6* = (0x00000040 shl WWDG_CR_T_Pos) ## !< 0x40

##  Legacy defines

const
  WWDG_CR_T0* = WWDG_CR_T_Bit0
  WWDG_CR_T1* = WWDG_CR_T_Bit1
  WWDG_CR_T2* = WWDG_CR_T_Bit2
  WWDG_CR_T3* = WWDG_CR_T_Bit3
  WWDG_CR_T4* = WWDG_CR_T_Bit4
  WWDG_CR_T5* = WWDG_CR_T_Bit5
  WWDG_CR_T6* = WWDG_CR_T_Bit6
  WWDG_CR_WDGA_Pos* = (7)
  WWDG_CR_WDGA_Msk* = (0x00000001 shl WWDG_CR_WDGA_Pos) ## !< 0x00000080
  WWDG_CR_WDGA* = WWDG_CR_WDGA_Msk

## ******************  Bit definition for WWDG_CFR register  ******************

const
  WWDG_CFR_W_Pos* = (0)
  WWDG_CFR_W_Msk* = (0x0000007F shl WWDG_CFR_W_Pos) ## !< 0x0000007F
  WWDG_CFR_W* = WWDG_CFR_W_Msk
  WWDG_CFR_W_Bit0* = (0x00000001 shl WWDG_CFR_W_Pos) ## !< 0x0001
  WWDG_CFR_W_Bit1* = (0x00000002 shl WWDG_CFR_W_Pos) ## !< 0x0002
  WWDG_CFR_W_Bit2* = (0x00000004 shl WWDG_CFR_W_Pos) ## !< 0x0004
  WWDG_CFR_W_Bit3* = (0x00000008 shl WWDG_CFR_W_Pos) ## !< 0x0008
  WWDG_CFR_W_Bit4* = (0x00000010 shl WWDG_CFR_W_Pos) ## !< 0x0010
  WWDG_CFR_W_Bit5* = (0x00000020 shl WWDG_CFR_W_Pos) ## !< 0x0020
  WWDG_CFR_W_Bit6* = (0x00000040 shl WWDG_CFR_W_Pos) ## !< 0x0040

##  Legacy defines

const
  WWDG_CFR_W0* = WWDG_CFR_W_Bit0
  WWDG_CFR_W1* = WWDG_CFR_W_Bit1
  WWDG_CFR_W2* = WWDG_CFR_W_Bit2
  WWDG_CFR_W3* = WWDG_CFR_W_Bit3
  WWDG_CFR_W4* = WWDG_CFR_W_Bit4
  WWDG_CFR_W5* = WWDG_CFR_W_Bit5
  WWDG_CFR_W6* = WWDG_CFR_W_Bit6
  WWDG_CFR_WDGTB_Pos* = (7)
  WWDG_CFR_WDGTB_Msk* = (0x00000003 shl WWDG_CFR_WDGTB_Pos) ## !< 0x00000180
  WWDG_CFR_WDGTB* = WWDG_CFR_WDGTB_Msk
  WWDG_CFR_WDGTB_Bit0* = (0x00000001 shl WWDG_CFR_WDGTB_Pos) ## !< 0x0080
  WWDG_CFR_WDGTB_Bit1* = (0x00000002 shl WWDG_CFR_WDGTB_Pos) ## !< 0x0100

##  Legacy defines

const
  WWDG_CFR_WDGTB0* = WWDG_CFR_WDGTB_Bit0
  WWDG_CFR_WDGTB1* = WWDG_CFR_WDGTB_Bit1
  WWDG_CFR_EWI_Pos* = (9)
  WWDG_CFR_EWI_Msk* = (0x00000001 shl WWDG_CFR_EWI_Pos) ## !< 0x00000200
  WWDG_CFR_EWI* = WWDG_CFR_EWI_Msk

## ******************  Bit definition for WWDG_SR register  *******************

const
  WWDG_SR_EWIF_Pos* = (0)
  WWDG_SR_EWIF_Msk* = (0x00000001 shl WWDG_SR_EWIF_Pos) ## !< 0x00000001
  WWDG_SR_EWIF* = WWDG_SR_EWIF_Msk

## ****************************************************************************
##
##                                 DBG
##
## ****************************************************************************
## *******************  Bit definition for DBGMCU_IDCODE register  ************

const
  DBGMCU_IDCODE_DEV_ID_Pos* = (0)
  DBGMCU_IDCODE_DEV_ID_Msk* = (0x00000FFF shl DBGMCU_IDCODE_DEV_ID_Pos) ## !< 0x00000FFF
  DBGMCU_IDCODE_DEV_ID* = DBGMCU_IDCODE_DEV_ID_Msk
  DBGMCU_IDCODE_REV_ID_Pos* = (16)
  DBGMCU_IDCODE_REV_ID_Msk* = (0x0000FFFF shl DBGMCU_IDCODE_REV_ID_Pos) ## !< 0xFFFF0000
  DBGMCU_IDCODE_REV_ID* = DBGMCU_IDCODE_REV_ID_Msk

## *******************  Bit definition for DBGMCU_CR register  ****************

const
  DBGMCU_CR_DBG_SLEEP_Pos* = (0)
  DBGMCU_CR_DBG_SLEEP_Msk* = (0x00000001 shl DBGMCU_CR_DBG_SLEEP_Pos) ## !< 0x00000001
  DBGMCU_CR_DBG_SLEEP* = DBGMCU_CR_DBG_SLEEP_Msk
  DBGMCU_CR_DBG_STOP_Pos* = (1)
  DBGMCU_CR_DBG_STOP_Msk* = (0x00000001 shl DBGMCU_CR_DBG_STOP_Pos) ## !< 0x00000002
  DBGMCU_CR_DBG_STOP* = DBGMCU_CR_DBG_STOP_Msk
  DBGMCU_CR_DBG_STANDBY_Pos* = (2)
  DBGMCU_CR_DBG_STANDBY_Msk* = (0x00000001 shl DBGMCU_CR_DBG_STANDBY_Pos) ## !< 0x00000004
  DBGMCU_CR_DBG_STANDBY* = DBGMCU_CR_DBG_STANDBY_Msk
  DBGMCU_CR_TRACE_IOEN_Pos* = (5)
  DBGMCU_CR_TRACE_IOEN_Msk* = (0x00000001 shl DBGMCU_CR_TRACE_IOEN_Pos) ## !< 0x00000020
  DBGMCU_CR_TRACE_IOEN* = DBGMCU_CR_TRACE_IOEN_Msk
  DBGMCU_CR_TRACE_MODE_Pos* = (6)
  DBGMCU_CR_TRACE_MODE_Msk* = (0x00000003 shl DBGMCU_CR_TRACE_MODE_Pos) ## !< 0x000000C0
  DBGMCU_CR_TRACE_MODE* = DBGMCU_CR_TRACE_MODE_Msk
  DBGMCU_CR_TRACE_MODE_Bit0* = (0x00000001 shl DBGMCU_CR_TRACE_MODE_Pos) ## !< 0x00000040
  DBGMCU_CR_TRACE_MODE_Bit1* = (0x00000002 shl DBGMCU_CR_TRACE_MODE_Pos) ## !< 0x00000080

## *******************  Bit definition for DBGMCU_APB1_FZ register  ***********

const
  DBGMCU_APB1_FZ_DBG_TIM5_STOP_Pos* = (3)
  DBGMCU_APB1_FZ_DBG_TIM5_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM5_STOP_Pos) ## !< 0x00000008
  DBGMCU_APB1_FZ_DBG_TIM5_STOP* = DBGMCU_APB1_FZ_DBG_TIM5_STOP_Msk
  DBGMCU_APB1_FZ_DBG_TIM6_STOP_Pos* = (4)
  DBGMCU_APB1_FZ_DBG_TIM6_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_TIM6_STOP_Pos) ## !< 0x00000010
  DBGMCU_APB1_FZ_DBG_TIM6_STOP* = DBGMCU_APB1_FZ_DBG_TIM6_STOP_Msk
  DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos* = (10)
  DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_RTC_STOP_Pos) ## !< 0x00000400
  DBGMCU_APB1_FZ_DBG_RTC_STOP* = DBGMCU_APB1_FZ_DBG_RTC_STOP_Msk
  DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos* = (11)
  DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_WWDG_STOP_Pos) ## !< 0x00000800
  DBGMCU_APB1_FZ_DBG_WWDG_STOP* = DBGMCU_APB1_FZ_DBG_WWDG_STOP_Msk
  DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos* = (12)
  DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_IWDG_STOP_Pos) ## !< 0x00001000
  DBGMCU_APB1_FZ_DBG_IWDG_STOP* = DBGMCU_APB1_FZ_DBG_IWDG_STOP_Msk
  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos* = (21)
  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Pos) ## !< 0x00200000
  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT* = DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT_Msk
  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Pos* = (22)
  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Pos) ## !< 0x00400000
  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT* = DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT_Msk
  DBGMCU_APB1_FZ_DBG_I2C4_SMBUS_TIMEOUT_Pos* = (24)
  DBGMCU_APB1_FZ_DBG_I2C4_SMBUS_TIMEOUT_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_I2C4_SMBUS_TIMEOUT_Pos) ## !< 0x01000000
  DBGMCU_APB1_FZ_DBG_I2C4_SMBUS_TIMEOUT* = DBGMCU_APB1_FZ_DBG_I2C4_SMBUS_TIMEOUT_Msk
  DBGMCU_APB1_FZ_DBG_CAN1_STOP_Pos* = (25)
  DBGMCU_APB1_FZ_DBG_CAN1_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_CAN1_STOP_Pos) ## !< 0x02000000
  DBGMCU_APB1_FZ_DBG_CAN1_STOP* = DBGMCU_APB1_FZ_DBG_CAN1_STOP_Msk
  DBGMCU_APB1_FZ_DBG_CAN2_STOP_Pos* = (26)
  DBGMCU_APB1_FZ_DBG_CAN2_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB1_FZ_DBG_CAN2_STOP_Pos) ## !< 0x04000000
  DBGMCU_APB1_FZ_DBG_CAN2_STOP* = DBGMCU_APB1_FZ_DBG_CAN2_STOP_Msk

## *******************  Bit definition for DBGMCU_APB2_FZ register  ***********

const
  DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos* = (0)
  DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM1_STOP_Pos) ## !< 0x00000001
  DBGMCU_APB2_FZ_DBG_TIM1_STOP* = DBGMCU_APB2_FZ_DBG_TIM1_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM9_STOP_Pos* = (16)
  DBGMCU_APB2_FZ_DBG_TIM9_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM9_STOP_Pos) ## !< 0x00010000
  DBGMCU_APB2_FZ_DBG_TIM9_STOP* = DBGMCU_APB2_FZ_DBG_TIM9_STOP_Msk
  DBGMCU_APB2_FZ_DBG_TIM11_STOP_Pos* = (18)
  DBGMCU_APB2_FZ_DBG_TIM11_STOP_Msk* = (
    0x00000001 shl DBGMCU_APB2_FZ_DBG_TIM11_STOP_Pos) ## !< 0x00040000
  DBGMCU_APB2_FZ_DBG_TIM11_STOP* = DBGMCU_APB2_FZ_DBG_TIM11_STOP_Msk

## *
##  @}
##
## *
##  @}
##
## * @addtogroup Exported_macros
##  @{
##
## ****************************** ADC Instances *******************************

template IS_ADC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == ADC1)

template IS_ADC_COMMON_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == ADC1_COMMON)

## ****************************** CRC Instances *******************************

template IS_CRC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == CRC)

## ****************************** DAC Instances *******************************

template IS_DAC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == DAC1)

## ******************************* DMA Instances ******************************

template IS_DMA_STREAM_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == DMA1_Stream0) or ((INSTANCE) == DMA1_Stream1) or
      ((INSTANCE) == DMA1_Stream2) or ((INSTANCE) == DMA1_Stream3) or
      ((INSTANCE) == DMA1_Stream4) or ((INSTANCE) == DMA1_Stream5) or
      ((INSTANCE) == DMA1_Stream6) or ((INSTANCE) == DMA1_Stream7) or
      ((INSTANCE) == DMA2_Stream0) or ((INSTANCE) == DMA2_Stream1) or
      ((INSTANCE) == DMA2_Stream2) or ((INSTANCE) == DMA2_Stream3) or
      ((INSTANCE) == DMA2_Stream4) or ((INSTANCE) == DMA2_Stream5) or
      ((INSTANCE) == DMA2_Stream6) or ((INSTANCE) == DMA2_Stream7))

## ****************************** GPIO Instances ******************************

template IS_GPIO_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == GPIOA) or ((INSTANCE) == GPIOB) or ((INSTANCE) == GPIOC) or
      ((INSTANCE) == GPIOH))

## ******************************* I2C Instances ******************************

template IS_I2C_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == I2C1) or ((INSTANCE) == I2C2))

## ****************************** SMBUS Instances *****************************

const
  IS_SMBUS_ALL_INSTANCE* = IS_I2C_ALL_INSTANCE

## ******************************* I2S Instances ******************************

template IS_I2S_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == SPI1) or ((INSTANCE) == SPI2) or ((INSTANCE) == SPI5))

## ****************************** LPTIM Instances *****************************

template IS_LPTIM_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == LPTIM1)

## ****************************** RNG Instances *******************************

template IS_RNG_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == RNG)

## ***************************** RTC Instances ********************************

template IS_RTC_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == RTC)

## ******************************* SPI Instances ******************************

template IS_SPI_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == SPI1) or ((INSTANCE) == SPI2) or ((INSTANCE) == SPI5))

## ************************** SPI Extended Instances **************************

template IS_SPI_ALL_INSTANCE_EXT*(INSTANCE: untyped): untyped =
  (((INSTANCE) == SPI1) or ((INSTANCE) == SPI2) or ((INSTANCE) == SPI5))

## ***************** TIM Instances : All supported instances ******************

template IS_TIM_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM6) or
      ((INSTANCE) == TIM9) or ((INSTANCE) == TIM11))

## ************ TIM Instances : at least 1 capture/compare channel ************

template IS_TIM_CC1_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9) or
      ((INSTANCE) == TIM11))

## *********** TIM Instances : at least 2 capture/compare channels ************

template IS_TIM_CC2_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9))

## *********** TIM Instances : at least 3 capture/compare channels ************

template IS_TIM_CC3_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5))

## *********** TIM Instances : at least 4 capture/compare channels ************

template IS_TIM_CC4_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5))

## ******************* TIM Instances : Advanced-control timers ****************

template IS_TIM_ADVANCED_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == TIM1)

## ****************** TIM Instances : Timer input XOR function ****************

template IS_TIM_XOR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5))

## ***************** TIM Instances : DMA requests generation (UDE) ************

template IS_TIM_DMA_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM6))

## *********** TIM Instances : DMA requests generation (CCxDE) ****************

template IS_TIM_DMA_CC_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5))

## *********** TIM Instances : DMA requests generation (COMDE) ****************

template IS_TIM_CCDMA_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5))

## ******************* TIM Instances : DMA burst feature **********************

template IS_TIM_DMABURST_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5))

## ***** TIM Instances : master mode available (TIMx_CR2.MMS available )*******

template IS_TIM_MASTER_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM6))

## ********** TIM Instances : Slave mode available (TIMx_SMCR available )******

template IS_TIM_SLAVE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9))

## ********************* TIM Instances : 32 bit Counter ***********************

template IS_TIM_Bit32B_COUNTER_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == TIM5)

## **************** TIM Instances : external trigger input availabe ***********

template IS_TIM_ETR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5))

## ***************** TIM Instances : remapping capability *********************

template IS_TIM_REMAP_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM5) or ((INSTANCE) == TIM11))

## ****************** TIM Instances : output(s) available *********************

template IS_TIM_CCX_INSTANCE*(INSTANCE, CHANNEL: untyped): untyped =
  ((((INSTANCE) == TIM1) and
      (((CHANNEL) == TIM_CHANNEL_Bit1) or ((CHANNEL) == TIM_CHANNEL_Bit2) or
      ((CHANNEL) == TIM_CHANNEL_Bit3) or ((CHANNEL) == TIM_CHANNEL_Bit4))) or
      (((INSTANCE) == TIM5) and
      (((CHANNEL) == TIM_CHANNEL_Bit1) or ((CHANNEL) == TIM_CHANNEL_Bit2) or
      ((CHANNEL) == TIM_CHANNEL_Bit3) or ((CHANNEL) == TIM_CHANNEL_Bit4))) or
      (((INSTANCE) == TIM9) and
      (((CHANNEL) == TIM_CHANNEL_Bit1) or ((CHANNEL) == TIM_CHANNEL_Bit2))) or
      (((INSTANCE) == TIM11) and (((CHANNEL) == TIM_CHANNEL_Bit1))))

## *********** TIM Instances : complementary output(s) available **************

template IS_TIM_CCXN_INSTANCE*(INSTANCE, CHANNEL: untyped): untyped =
  ((((INSTANCE) == TIM1) and
      (((CHANNEL) == TIM_CHANNEL_Bit1) or ((CHANNEL) == TIM_CHANNEL_Bit2) or
      ((CHANNEL) == TIM_CHANNEL_Bit3))))

## ***************** TIM Instances : supporting counting mode selection *******

template IS_TIM_COUNTER_MODE_SELECT_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5))

## ***************** TIM Instances : supporting clock division ****************

template IS_TIM_CLOCK_DIVISION_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9) or
      ((INSTANCE) == TIM11))

## ***************** TIM Instances : supporting commutation event generation **

template IS_TIM_COMMUTATION_EVENT_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == TIM1)

## ***************** TIM Instances : supporting OCxREF clear ******************

template IS_TIM_OCXREF_CLEAR_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5))

## ***** TIM Instances : supporting external clock mode 1 for ETRF input ******

template IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9))

## ***** TIM Instances : supporting external clock mode 2 for ETRF input ******

template IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5))

## ***************** TIM Instances : supporting repetition counter ************

template IS_TIM_REPETITION_COUNTER_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1))

## ***************** TIM Instances : supporting encoder interface *************

template IS_TIM_ENCODER_INTERFACE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5) or ((INSTANCE) == TIM9))

## ***************** TIM Instances : supporting Hall sensor interface *********

template IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1) or ((INSTANCE) == TIM5))

## ***************** TIM Instances : supporting the break function ************

template IS_TIM_BREAK_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == TIM1))

## ******************* USART Instances : Synchronous mode *********************

template IS_USART_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART6))

## ******************* UART Instances : Half-Duplex mode *********************

template IS_UART_HALFDUPLEX_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART6))

##  Legacy defines

const
  IS_UART_INSTANCE* = IS_UART_HALFDUPLEX_INSTANCE

## ***************** UART Instances : Hardware Flow control *******************

template IS_UART_HWFLOW_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART6))

## ******************* UART Instances : LIN mode *********************

const
  IS_UART_LIN_INSTANCE* = IS_UART_HALFDUPLEX_INSTANCE

## ******************** UART Instances : Smart card mode **********************

template IS_SMARTCARD_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART6))

## ********************** UART Instances : IRDA mode **************************

template IS_IRDA_INSTANCE*(INSTANCE: untyped): untyped =
  (((INSTANCE) == USART1) or ((INSTANCE) == USART2) or ((INSTANCE) == USART6))

## ***************************** IWDG Instances *******************************

template IS_IWDG_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == IWDG)

## ***************************** WWDG Instances *******************************

template IS_WWDG_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == WWDG)

## **************************** FMPI2C Instances ******************************

template IS_FMPI2C_ALL_INSTANCE*(INSTANCE: untyped): untyped =
  ((INSTANCE) == FMPI2C1)

##
##  @brief Specific devices reset values definitions
##

const
  RCC_PLLCFGR_RST_VALUE* = 0x7F003010
  RCC_PLLI2SCFGR_RST_VALUE* = 0x24003000
  RCC_MAX_FREQUENCY* = 100000000
  RCC_MAX_FREQUENCY_SCALE1* = RCC_MAX_FREQUENCY
  RCC_MAX_FREQUENCY_SCALE2* = 84000000
  RCC_MAX_FREQUENCY_SCALE3* = 64000000
  RCC_PLLVCO_OUTPUT_MIN* = 100000000
  RCC_PLLVCO_INPUT_MIN* = 950000
  RCC_PLLVCO_INPUT_MAX* = 2100000
  RCC_PLLVCO_OUTPUT_MAX* = 432000000
  RCC_PLLN_MIN_VALUE* = 50
  RCC_PLLN_MAX_VALUE* = 432
  FLASH_SCALE1_LATENCY1_FREQ* = 30000000
  FLASH_SCALE1_LATENCY2_FREQ* = 64000000
  FLASH_SCALE1_LATENCY3_FREQ* = 90000000
  FLASH_SCALE2_LATENCY1_FREQ* = 30000000
  FLASH_SCALE2_LATENCY2_FREQ* = 64000000
  FLASH_SCALE3_LATENCY1_FREQ* = 30000000
  FLASH_SCALE3_LATENCY2_FREQ* = 64000000

## *
##  @}
##
## *
##  @}
##
## *
##  @}
##

## *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE***
