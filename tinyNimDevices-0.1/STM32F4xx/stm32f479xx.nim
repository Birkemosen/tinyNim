## *
## *****************************************************************************
##  @file    stm32f479xx.h
##  @author  MCD Application Team
##  @brief   CMSIS STM32F479xx Device Peripheral Access Layer Header File.
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
## * @addtogroup stm32f479xx
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
    ADC_IRQn = 18,              ## !< ADC1, ADC2 and ADC3 global Interrupts
    CAN1_TX_IRQn = 19,          ## !< CAN1 TX Interrupt
    CAN1_RX0_IRQn = 20,         ## !< CAN1 RX0 Interrupt
    CAN1_RX1_IRQn = 21,         ## !< CAN1 RX1 Interrupt
    CAN1_SCE_IRQn = 22,         ## !< CAN1 SCE Interrupt
    EXTI9_Bit5_IRQn = 23,       ## !< External Line[9:5] Interrupts
    TIM1_BRK_TIM9_IRQn = 24,    ## !< TIM1 Break interrupt and TIM9 global interrupt
    TIM1_UP_TIM10_IRQn = 25,    ## !< TIM1 Update Interrupt and TIM10 global interrupt
    TIM1_TRG_COM_TIM11_IRQn = 26, ## !< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
    TIM1_CC_IRQn = 27,          ## !< TIM1 Capture Compare Interrupt
    TIM2_IRQn = 28,             ## !< TIM2 global Interrupt
    TIM3_IRQn = 29,             ## !< TIM3 global Interrupt
    TIM4_IRQn = 30,             ## !< TIM4 global Interrupt
    I2C1_EV_IRQn = 31,          ## !< I2C1 Event Interrupt
    I2C1_ER_IRQn = 32,          ## !< I2C1 Error Interrupt
    I2C2_EV_IRQn = 33,          ## !< I2C2 Event Interrupt
    I2C2_ER_IRQn = 34,          ## !< I2C2 Error Interrupt
    SPI1_IRQn = 35,             ## !< SPI1 global Interrupt
    SPI2_IRQn = 36,             ## !< SPI2 global Interrupt
    USART1_IRQn = 37,           ## !< USART1 global Interrupt
    USART2_IRQn = 38,           ## !< USART2 global Interrupt
    USART3_IRQn = 39,           ## !< USART3 global Interrupt
    EXTI15_Bit10_IRQn = 40,     ## !< External Line[15:10] Interrupts
    RTC_Alarm_IRQn = 41,        ## !< RTC Alarm (A and B) through EXTI Line Interrupt
    OTG_FS_WKUP_IRQn = 42,      ## !< USB OTG FS Wakeup through EXTI line interrupt
    TIM8_BRK_TIM12_IRQn = 43,   ## !< TIM8 Break Interrupt and TIM12 global interrupt
    TIM8_UP_TIM13_IRQn = 44,    ## !< TIM8 Update Interrupt and TIM13 global interrupt
    TIM8_TRG_COM_TIM14_IRQn = 45, ## !< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt
    TIM8_CC_IRQn = 46,          ## !< TIM8 Capture Compare global interrupt
    DMA1_Stream7_IRQn = 47,     ## !< DMA1 Stream7 Interrupt
    FMC_IRQn = 48,              ## !< FMC global Interrupt
    SDIO_IRQn = 49,             ## !< SDIO global Interrupt
    TIM5_IRQn = 50,             ## !< TIM5 global Interrupt
    SPI3_IRQn = 51,             ## !< SPI3 global Interrupt
    UART4_IRQn = 52,            ## !< UART4 global Interrupt
    UART5_IRQn = 53,            ## !< UART5 global Interrupt
    TIM6_DAC_IRQn = 54,         ## !< TIM6 global and DAC1&2 underrun error  interrupts
    TIM7_IRQn = 55,             ## !< TIM7 global interrupt
    DMA2_Stream0_IRQn = 56,     ## !< DMA2 Stream 0 global Interrupt
    DMA2_Stream1_IRQn = 57,     ## !< DMA2 Stream 1 global Interrupt
    DMA2_Stream2_IRQn = 58,     ## !< DMA2 Stream 2 global Interrupt
    DMA2_Stream3_IRQn = 59,     ## !< DMA2 Stream 3 global Interrupt
    DMA2_Stream4_IRQn = 60,     ## !< DMA2 Stream 4 global Interrupt
    ETH_IRQn = 61,              ## !< Ethernet global Interrupt
    ETH_WKUP_IRQn = 62,         ## !< Ethernet Wakeup through EXTI line Interrupt
    CAN2_TX_IRQn = 63,          ## !< CAN2 TX Interrupt
    CAN2_RX0_IRQn = 64,         ## !< CAN2 RX0 Interrupt
    CAN2_RX1_IRQn = 65,         ## !< CAN2 RX1 Interrupt
    CAN2_SCE_IRQn = 66,         ## !< CAN2 SCE Interrupt
    OTG_FS_IRQn = 67,           ## !< USB OTG FS global Interrupt
    DMA2_Stream5_IRQn = 68,     ## !< DMA2 Stream 5 global interrupt
    DMA2_Stream6_IRQn = 69,     ## !< DMA2 Stream 6 global interrupt
    DMA2_Stream7_IRQn = 70,     ## !< DMA2 Stream 7 global interrupt
    USART6_IRQn = 71,           ## !< USART6 global interrupt
    I2C3_EV_IRQn = 72,          ## !< I2C3 event interrupt
    I2C3_ER_IRQn = 73,          ## !< I2C3 error interrupt
    OTG_HS_EP1_OUT_IRQn = 74,   ## !< USB OTG HS End Point 1 Out global interrupt
    OTG_HS_EP1_IN_IRQn = 75,    ## !< USB OTG HS End Point 1 In global interrupt
    OTG_HS_WKUP_IRQn = 76,      ## !< USB OTG HS Wakeup through EXTI interrupt
    OTG_HS_IRQn = 77,           ## !< USB OTG HS global interrupt
    DCMI_IRQn = 78,             ## !< DCMI global interrupt
    CRYP_IRQn = 79,             ## !< CRYP crypto global interrupt
    HASH_RNG_IRQn = 80,         ## !< Hash and Rng global interrupt
    FPU_IRQn = 81,              ## !< FPU global interrupt
    UART7_IRQn = 82,            ## !< UART7 global interrupt
    UART8_IRQn = 83,            ## !< UART8 global interrupt
    SPI4_IRQn = 84,             ## !< SPI4 global Interrupt
    SPI5_IRQn = 85,             ## !< SPI5 global Interrupt
    SPI6_IRQn = 86,             ## !< SPI6 global Interrupt
    SAI1_IRQn = 87,             ## !< SAI1 global Interrupt
    LTDC_IRQn = 88,             ## !< LTDC global Interrupt
    LTDC_ER_IRQn = 89,          ## !< LTDC Error global Interrupt
    DMA2D_IRQn = 90,            ## !< DMA2D global Interrupt
    QUADSPI_IRQn = 91,          ## !< QUADSPI global Interrupt
    DSI_IRQn = 92


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
##  @brief Controller Area Network TxMailBox
##

type
  CAN_TxMailBox_TypeDef* {.bycopy.} = object
    TIR*: uint32               ## !< CAN TX mailbox identifier register
    TDTR*: uint32              ## !< CAN mailbox data length control and time stamp register
    TDLR*: uint32              ## !< CAN mailbox data low register
    TDHR*: uint32              ## !< CAN mailbox data high register


## *
##  @brief Controller Area Network FIFOMailBox
##

type
  CAN_FIFOMailBox_TypeDef* {.bycopy.} = object
    RIR*: uint32               ## !< CAN receive FIFO mailbox identifier register
    RDTR*: uint32              ## !< CAN receive FIFO mailbox data length control and time stamp register
    RDLR*: uint32              ## !< CAN receive FIFO mailbox data low register
    RDHR*: uint32              ## !< CAN receive FIFO mailbox data high register


## *
##  @brief Controller Area Network FilterRegister
##

type
  CAN_FilterRegister_TypeDef* {.bycopy.} = object
    FR1*: uint32               ## !< CAN Filter bank register 1
    FR2*: uint32               ## !< CAN Filter bank register 1


## *
##  @brief Controller Area Network
##

type
  CAN_TypeDef* {.bycopy.} = object
    MCR*: uint32               ## !< CAN master control register,         Address offset: 0x00
    MSR*: uint32               ## !< CAN master status register,          Address offset: 0x04
    TSR*: uint32               ## !< CAN transmit status register,        Address offset: 0x08
    RF0R*: uint32              ## !< CAN receive FIFO 0 register,         Address offset: 0x0C
    RF1R*: uint32              ## !< CAN receive FIFO 1 register,         Address offset: 0x10
    IER*: uint32               ## !< CAN interrupt enable register,       Address offset: 0x14
    ESR*: uint32               ## !< CAN error status register,           Address offset: 0x18
    BTR*: uint32               ## !< CAN bit timing register,             Address offset: 0x1C
    RESERVED0*: array[88, uint32] ## !< Reserved, 0x020 - 0x17F
    sTxMailBox*: array[3, CAN_TxMailBox_TypeDef] ## !< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC
    sFIFOMailBox*: array[2, CAN_FIFOMailBox_TypeDef] ## !< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC
    RESERVED1*: array[12, uint32] ## !< Reserved, 0x1D0 - 0x1FF
    FMR*: uint32               ## !< CAN filter master register,          Address offset: 0x200
    FM1R*: uint32              ## !< CAN filter mode register,            Address offset: 0x204
    RESERVED2*: uint32         ## !< Reserved, 0x208
    FS1R*: uint32              ## !< CAN filter scale register,           Address offset: 0x20C
    RESERVED3*: uint32         ## !< Reserved, 0x210
    FFA1R*: uint32             ## !< CAN filter FIFO assignment register, Address offset: 0x214
    RESERVED4*: uint32         ## !< Reserved, 0x218
    FA1R*: uint32              ## !< CAN filter activation register,      Address offset: 0x21C
    RESERVED5*: array[8, uint32] ## !< Reserved, 0x220-0x23F
    sFilterRegister*: array[28, CAN_FilterRegister_TypeDef] ## !< CAN Filter Register,                 Address offset: 0x240-0x31C


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
##  @brief DCMI
##

type
  DCMI_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< DCMI control register 1,                       Address offset: 0x00
    SR*: uint32                ## !< DCMI status register,                          Address offset: 0x04
    RISR*: uint32              ## !< DCMI raw interrupt status register,            Address offset: 0x08
    IER*: uint32               ## !< DCMI interrupt enable register,                Address offset: 0x0C
    MISR*: uint32              ## !< DCMI masked interrupt status register,         Address offset: 0x10
    ICR*: uint32               ## !< DCMI interrupt clear register,                 Address offset: 0x14
    ESCR*: uint32              ## !< DCMI embedded synchronization code register,   Address offset: 0x18
    ESUR*: uint32              ## !< DCMI embedded synchronization unmask register, Address offset: 0x1C
    CWSTRTR*: uint32           ## !< DCMI crop window start,                        Address offset: 0x20
    CWSIZER*: uint32           ## !< DCMI crop window size,                         Address offset: 0x24
    DR*: uint32                ## !< DCMI data register,                            Address offset: 0x28


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
##  @brief DMA2D Controller
##

type
  DMA2D_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< DMA2D Control Register,                         Address offset: 0x00
    ISR*: uint32               ## !< DMA2D Interrupt Status Register,                Address offset: 0x04
    IFCR*: uint32              ## !< DMA2D Interrupt Flag Clear Register,            Address offset: 0x08
    FGMAR*: uint32             ## !< DMA2D Foreground Memory Address Register,       Address offset: 0x0C
    FGOR*: uint32              ## !< DMA2D Foreground Offset Register,               Address offset: 0x10
    BGMAR*: uint32             ## !< DMA2D Background Memory Address Register,       Address offset: 0x14
    BGOR*: uint32              ## !< DMA2D Background Offset Register,               Address offset: 0x18
    FGPFCCR*: uint32           ## !< DMA2D Foreground PFC Control Register,          Address offset: 0x1C
    FGCOLR*: uint32            ## !< DMA2D Foreground Color Register,                Address offset: 0x20
    BGPFCCR*: uint32           ## !< DMA2D Background PFC Control Register,          Address offset: 0x24
    BGCOLR*: uint32            ## !< DMA2D Background Color Register,                Address offset: 0x28
    FGCMAR*: uint32            ## !< DMA2D Foreground CLUT Memory Address Register,  Address offset: 0x2C
    BGCMAR*: uint32            ## !< DMA2D Background CLUT Memory Address Register,  Address offset: 0x30
    OPFCCR*: uint32            ## !< DMA2D Output PFC Control Register,              Address offset: 0x34
    OCOLR*: uint32             ## !< DMA2D Output Color Register,                    Address offset: 0x38
    OMAR*: uint32              ## !< DMA2D Output Memory Address Register,           Address offset: 0x3C
    OOR*: uint32               ## !< DMA2D Output Offset Register,                   Address offset: 0x40
    NLR*: uint32               ## !< DMA2D Number of Line Register,                  Address offset: 0x44
    LWR*: uint32               ## !< DMA2D Line Watermark Register,                  Address offset: 0x48
    AMTCR*: uint32             ## !< DMA2D AHB Master Timer Configuration Register,  Address offset: 0x4C
    RESERVED*: array[236, uint32] ## !< Reserved, 0x50-0x3FF
    FGCLUT*: array[256, uint32] ## !< DMA2D Foreground CLUT,                          Address offset:400-7FF
    BGCLUT*: array[256, uint32] ## !< DMA2D Background CLUT,                          Address offset:800-BFF


## *
##  @brief DSI Controller
##

type
  DSI_TypeDef* {.bycopy.} = object
    VR*: uint32                ## !< DSI Host Version Register,                                 Address offset: 0x00
    CR*: uint32                ## !< DSI Host Control Register,                                 Address offset: 0x04
    CCR*: uint32               ## !< DSI HOST Clock Control Register,                           Address offset: 0x08
    LVCIDR*: uint32            ## !< DSI Host LTDC VCID Register,                               Address offset: 0x0C
    LCOLCR*: uint32            ## !< DSI Host LTDC Color Coding Register,                       Address offset: 0x10
    LPCR*: uint32              ## !< DSI Host LTDC Polarity Configuration Register,             Address offset: 0x14
    LPMCR*: uint32             ## !< DSI Host Low-Power Mode Configuration Register,            Address offset: 0x18
    RESERVED0*: array[4, uint32] ## !< Reserved, 0x1C - 0x2B
    PCR*: uint32               ## !< DSI Host Protocol Configuration Register,                  Address offset: 0x2C
    GVCIDR*: uint32            ## !< DSI Host Generic VCID Register,                            Address offset: 0x30
    MCR*: uint32               ## !< DSI Host Mode Configuration Register,                      Address offset: 0x34
    VMCR*: uint32              ## !< DSI Host Video Mode Configuration Register,                Address offset: 0x38
    VPCR*: uint32              ## !< DSI Host Video Packet Configuration Register,              Address offset: 0x3C
    VCCR*: uint32              ## !< DSI Host Video Chunks Configuration Register,              Address offset: 0x40
    VNPCR*: uint32             ## !< DSI Host Video Null Packet Configuration Register,         Address offset: 0x44
    VHSACR*: uint32            ## !< DSI Host Video HSA Configuration Register,                 Address offset: 0x48
    VHBPCR*: uint32            ## !< DSI Host Video HBP Configuration Register,                 Address offset: 0x4C
    VLCR*: uint32              ## !< DSI Host Video Line Configuration Register,                Address offset: 0x50
    VVSACR*: uint32            ## !< DSI Host Video VSA Configuration Register,                 Address offset: 0x54
    VVBPCR*: uint32            ## !< DSI Host Video VBP Configuration Register,                 Address offset: 0x58
    VVFPCR*: uint32            ## !< DSI Host Video VFP Configuration Register,                 Address offset: 0x5C
    VVACR*: uint32             ## !< DSI Host Video VA Configuration Register,                  Address offset: 0x60
    LCCR*: uint32              ## !< DSI Host LTDC Command Configuration Register,              Address offset: 0x64
    CMCR*: uint32              ## !< DSI Host Command Mode Configuration Register,              Address offset: 0x68
    GHCR*: uint32              ## !< DSI Host Generic Header Configuration Register,            Address offset: 0x6C
    GPDR*: uint32              ## !< DSI Host Generic Payload Data Register,                    Address offset: 0x70
    GPSR*: uint32              ## !< DSI Host Generic Packet Status Register,                   Address offset: 0x74
    TCCR*: array[6, uint32]     ## !< DSI Host Timeout Counter Configuration Register,           Address offset: 0x78-0x8F
    TDCR*: uint32              ## !< DSI Host 3D Configuration Register,                        Address offset: 0x90
    CLCR*: uint32              ## !< DSI Host Clock Lane Configuration Register,                Address offset: 0x94
    CLTCR*: uint32             ## !< DSI Host Clock Lane Timer Configuration Register,          Address offset: 0x98
    DLTCR*: uint32             ## !< DSI Host Data Lane Timer Configuration Register,           Address offset: 0x9C
    PCTLR*: uint32             ## !< DSI Host PHY Control Register,                             Address offset: 0xA0
    PCONFR*: uint32            ## !< DSI Host PHY Configuration Register,                       Address offset: 0xA4
    PUCR*: uint32              ## !< DSI Host PHY ULPS Control Register,                        Address offset: 0xA8
    PTTCR*: uint32             ## !< DSI Host PHY TX Triggers Configuration Register,           Address offset: 0xAC
    PSR*: uint32               ## !< DSI Host PHY Status Register,                              Address offset: 0xB0
    RESERVED1*: array[2, uint32] ## !< Reserved, 0xB4 - 0xBB
    ISR*: array[2, uint32]      ## !< DSI Host Interrupt & Status Register,                      Address offset: 0xBC-0xC3
    IER*: array[2, uint32]      ## !< DSI Host Interrupt Enable Register,                        Address offset: 0xC4-0xCB
    RESERVED2*: array[3, uint32] ## !< Reserved, 0xD0 - 0xD7
    FIR*: array[2, uint32]      ## !< DSI Host Force Interrupt Register,                         Address offset: 0xD8-0xDF
    RESERVED3*: array[8, uint32] ## !< Reserved, 0xE0 - 0xFF
    VSCR*: uint32              ## !< DSI Host Video Shadow Control Register,                    Address offset: 0x100
    RESERVED4*: array[2, uint32] ## !< Reserved, 0x104 - 0x10B
    LCVCIDR*: uint32           ## !< DSI Host LTDC Current VCID Register,                       Address offset: 0x10C
    LCCCR*: uint32             ## !< DSI Host LTDC Current Color Coding Register,               Address offset: 0x110
    RESERVED5*: uint32         ## !< Reserved, 0x114
    LPMCCR*: uint32            ## !< DSI Host Low-power Mode Current Configuration Register,    Address offset: 0x118
    RESERVED6*: array[7, uint32] ## !< Reserved, 0x11C - 0x137
    VMCCR*: uint32             ## !< DSI Host Video Mode Current Configuration Register,        Address offset: 0x138
    VPCCR*: uint32             ## !< DSI Host Video Packet Current Configuration Register,      Address offset: 0x13C
    VCCCR*: uint32             ## !< DSI Host Video Chuncks Current Configuration Register,     Address offset: 0x140
    VNPCCR*: uint32            ## !< DSI Host Video Null Packet Current Configuration Register, Address offset: 0x144
    VHSACCR*: uint32           ## !< DSI Host Video HSA Current Configuration Register,         Address offset: 0x148
    VHBPCCR*: uint32           ## !< DSI Host Video HBP Current Configuration Register,         Address offset: 0x14C
    VLCCR*: uint32             ## !< DSI Host Video Line Current Configuration Register,        Address offset: 0x150
    VVSACCR*: uint32           ## !< DSI Host Video VSA Current Configuration Register,         Address offset: 0x154
    VVBPCCR*: uint32           ## !< DSI Host Video VBP Current Configuration Register,         Address offset: 0x158
    VVFPCCR*: uint32           ## !< DSI Host Video VFP Current Configuration Register,         Address offset: 0x15C
    VVACCR*: uint32            ## !< DSI Host Video VA Current Configuration Register,          Address offset: 0x160
    RESERVED7*: array[11, uint32] ## !< Reserved, 0x164 - 0x18F
    TDCCR*: uint32             ## !< DSI Host 3D Current Configuration Register,                Address offset: 0x190
    RESERVED8*: array[155, uint32] ## !< Reserved, 0x194 - 0x3FF
    WCFGR*: uint32             ## !< DSI Wrapper Configuration Register,                       Address offset: 0x400
    WCR*: uint32               ## !< DSI Wrapper Control Register,                             Address offset: 0x404
    WIER*: uint32              ## !< DSI Wrapper Interrupt Enable Register,                    Address offset: 0x408
    WISR*: uint32              ## !< DSI Wrapper Interrupt and Status Register,                Address offset: 0x40C
    WIFCR*: uint32             ## !< DSI Wrapper Interrupt Flag Clear Register,                Address offset: 0x410
    RESERVED9*: uint32         ## !< Reserved, 0x414
    WPCR*: array[5, uint32]     ## !< DSI Wrapper PHY Configuration Register,                   Address offset: 0x418-0x42B
    RESERVED10*: uint32        ## !< Reserved, 0x42C
    WRPCR*: uint32             ## !< DSI Wrapper Regulator and PLL Control Register, Address offset: 0x430


## *
##  @brief Ethernet MAC
##

type
  ETH_TypeDef* {.bycopy.} = object
    MACCR*: uint32
    MACFFR*: uint32
    MACHTHR*: uint32
    MACHTLR*: uint32
    MACMIIAR*: uint32
    MACMIIDR*: uint32
    MACFCR*: uint32
    MACVLANTR*: uint32         ##     8
    RESERVED0*: array[2, uint32]
    MACRWUFFR*: uint32         ##    11
    MACPMTCSR*: uint32
    RESERVED1*: uint32
    MACDBGR*: uint32
    MACSR*: uint32             ##    15
    MACIMR*: uint32
    MACA0HR*: uint32
    MACA0LR*: uint32
    MACA1HR*: uint32
    MACA1LR*: uint32
    MACA2HR*: uint32
    MACA2LR*: uint32
    MACA3HR*: uint32
    MACA3LR*: uint32           ##    24
    RESERVED2*: array[40, uint32]
    MMCCR*: uint32             ##    65
    MMCRIR*: uint32
    MMCTIR*: uint32
    MMCRIMR*: uint32
    MMCTIMR*: uint32           ##    69
    RESERVED3*: array[14, uint32]
    MMCTGFSCCR*: uint32        ##    84
    MMCTGFMSCCR*: uint32
    RESERVED4*: array[5, uint32]
    MMCTGFCR*: uint32
    RESERVED5*: array[10, uint32]
    MMCRFCECR*: uint32
    MMCRFAECR*: uint32
    RESERVED6*: array[10, uint32]
    MMCRGUFCR*: uint32
    RESERVED7*: array[334, uint32]
    PTPTSCR*: uint32
    PTPSSIR*: uint32
    PTPTSHR*: uint32
    PTPTSLR*: uint32
    PTPTSHUR*: uint32
    PTPTSLUR*: uint32
    PTPTSAR*: uint32
    PTPTTHR*: uint32
    PTPTTLR*: uint32
    RESERVED8*: uint32
    PTPTSSR*: uint32
    RESERVED9*: array[565, uint32]
    DMABMR*: uint32
    DMATPDR*: uint32
    DMARPDR*: uint32
    DMARDLAR*: uint32
    DMATDLAR*: uint32
    DMASR*: uint32
    DMAOMR*: uint32
    DMAIER*: uint32
    DMAMFBOCR*: uint32
    DMARSWTR*: uint32
    RESERVED10*: array[8, uint32]
    DMACHTDR*: uint32
    DMACHRDR*: uint32
    DMACHTBAR*: uint32
    DMACHRBAR*: uint32


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
##  @brief Flexible Memory Controller
##

type
  FMC_Bank1_TypeDef* {.bycopy.} = object
    BTCR*: array[8, uint32]     ## !< NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C


## *
##  @brief Flexible Memory Controller Bank1E
##

type
  FMC_Bank1E_TypeDef* {.bycopy.} = object
    BWTR*: array[7, uint32]     ## !< NOR/PSRAM write timing registers, Address offset: 0x104-0x11C


## *
##  @brief Flexible Memory Controller Bank3
##

type
  FMC_Bank3_TypeDef* {.bycopy.} = object
    PCR*: uint32               ## !< NAND Flash control register,                       Address offset: 0x80
    SR*: uint32                ## !< NAND Flash FIFO status and interrupt register,     Address offset: 0x84
    PMEM*: uint32              ## !< NAND Flash Common memory space timing register,    Address offset: 0x88
    PATT*: uint32              ## !< NAND Flash Attribute memory space timing register, Address offset: 0x8C
    RESERVED*: uint32          ## !< Reserved, 0x90
    ECCR*: uint32              ## !< NAND Flash ECC result registers,                   Address offset: 0x94


## *
##  @brief Flexible Memory Controller Bank5_6
##

type
  FMC_Bank5_Bit6_TypeDef* {.bycopy.} = object
    SDCR*: array[2, uint32]     ## !< SDRAM Control registers ,      Address offset: 0x140-0x144
    SDTR*: array[2, uint32]     ## !< SDRAM Timing registers ,       Address offset: 0x148-0x14C
    SDCMR*: uint32             ## !< SDRAM Command Mode register,   Address offset: 0x150
    SDRTR*: uint32             ## !< SDRAM Refresh Timer register,  Address offset: 0x154
    SDSR*: uint32              ## !< SDRAM Status register,         Address offset: 0x158


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
    RESERVED*: array[2, uint32] ## !< Reserved, 0x18-0x1C
    CMPCR*: uint32             ## !< SYSCFG Compensation cell control register,         Address offset: 0x20


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
##  @brief Independent WATCHDOG
##

type
  IWDG_TypeDef* {.bycopy.} = object
    KR*: uint32                ## !< IWDG Key register,       Address offset: 0x00
    PR*: uint32                ## !< IWDG Prescaler register, Address offset: 0x04
    RLR*: uint32               ## !< IWDG Reload register,    Address offset: 0x08
    SR*: uint32                ## !< IWDG Status register,    Address offset: 0x0C


## *
##  @brief LCD-TFT Display Controller
##

type
  LTDC_TypeDef* {.bycopy.} = object
    RESERVED0*: array[2, uint32] ## !< Reserved, 0x00-0x04
    SSCR*: uint32              ## !< LTDC Synchronization Size Configuration Register,    Address offset: 0x08
    BPCR*: uint32              ## !< LTDC Back Porch Configuration Register,              Address offset: 0x0C
    AWCR*: uint32              ## !< LTDC Active Width Configuration Register,            Address offset: 0x10
    TWCR*: uint32              ## !< LTDC Total Width Configuration Register,             Address offset: 0x14
    GCR*: uint32               ## !< LTDC Global Control Register,                        Address offset: 0x18
    RESERVED1*: array[2, uint32] ## !< Reserved, 0x1C-0x20
    SRCR*: uint32              ## !< LTDC Shadow Reload Configuration Register,           Address offset: 0x24
    RESERVED2*: array[1, uint32] ## !< Reserved, 0x28
    BCCR*: uint32              ## !< LTDC Background Color Configuration Register,        Address offset: 0x2C
    RESERVED3*: array[1, uint32] ## !< Reserved, 0x30
    IER*: uint32               ## !< LTDC Interrupt Enable Register,                      Address offset: 0x34
    ISR*: uint32               ## !< LTDC Interrupt Status Register,                      Address offset: 0x38
    ICR*: uint32               ## !< LTDC Interrupt Clear Register,                       Address offset: 0x3C
    LIPCR*: uint32             ## !< LTDC Line Interrupt Position Configuration Register, Address offset: 0x40
    CPSR*: uint32              ## !< LTDC Current Position Status Register,               Address offset: 0x44
    CDSR*: uint32              ## !< LTDC Current Display Status Register,                Address offset: 0x48


## *
##  @brief LCD-TFT Display layer x Controller
##

type
  LTDC_Layer_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< LTDC Layerx Control Register                                  Address offset: 0x84
    WHPCR*: uint32             ## !< LTDC Layerx Window Horizontal Position Configuration Register Address offset: 0x88
    WVPCR*: uint32             ## !< LTDC Layerx Window Vertical Position Configuration Register   Address offset: 0x8C
    CKCR*: uint32              ## !< LTDC Layerx Color Keying Configuration Register               Address offset: 0x90
    PFCR*: uint32              ## !< LTDC Layerx Pixel Format Configuration Register               Address offset: 0x94
    CACR*: uint32              ## !< LTDC Layerx Constant Alpha Configuration Register             Address offset: 0x98
    DCCR*: uint32              ## !< LTDC Layerx Default Color Configuration Register              Address offset: 0x9C
    BFCR*: uint32              ## !< LTDC Layerx Blending Factors Configuration Register           Address offset: 0xA0
    RESERVED0*: array[2, uint32] ## !< Reserved
    CFBAR*: uint32             ## !< LTDC Layerx Color Frame Buffer Address Register               Address offset: 0xAC
    CFBLR*: uint32             ## !< LTDC Layerx Color Frame Buffer Length Register                Address offset: 0xB0
    CFBLNR*: uint32            ## !< LTDC Layerx ColorFrame Buffer Line Number Register            Address offset: 0xB4
    RESERVED1*: array[3, uint32] ## !< Reserved
    CLUTWR*: uint32            ## !< LTDC Layerx CLUT Write Register                               Address offset: 0x144


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
    AHB2RSTR*: uint32          ## !< RCC AHB2 peripheral reset register,                          Address offset: 0x14
    AHB3RSTR*: uint32          ## !< RCC AHB3 peripheral reset register,                          Address offset: 0x18
    RESERVED0*: uint32         ## !< Reserved, 0x1C
    APB1RSTR*: uint32          ## !< RCC APB1 peripheral reset register,                          Address offset: 0x20
    APB2RSTR*: uint32          ## !< RCC APB2 peripheral reset register,                          Address offset: 0x24
    RESERVED1*: array[2, uint32] ## !< Reserved, 0x28-0x2C
    AHB1ENR*: uint32           ## !< RCC AHB1 peripheral clock register,                          Address offset: 0x30
    AHB2ENR*: uint32           ## !< RCC AHB2 peripheral clock register,                          Address offset: 0x34
    AHB3ENR*: uint32           ## !< RCC AHB3 peripheral clock register,                          Address offset: 0x38
    RESERVED2*: uint32         ## !< Reserved, 0x3C
    APB1ENR*: uint32           ## !< RCC APB1 peripheral clock enable register,                   Address offset: 0x40
    APB2ENR*: uint32           ## !< RCC APB2 peripheral clock enable register,                   Address offset: 0x44
    RESERVED3*: array[2, uint32] ## !< Reserved, 0x48-0x4C
    AHB1LPENR*: uint32         ## !< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50
    AHB2LPENR*: uint32         ## !< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54
    AHB3LPENR*: uint32         ## !< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58
    RESERVED4*: uint32         ## !< Reserved, 0x5C
    APB1LPENR*: uint32         ## !< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60
    APB2LPENR*: uint32         ## !< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64
    RESERVED5*: array[2, uint32] ## !< Reserved, 0x68-0x6C
    BDCR*: uint32              ## !< RCC Backup domain control register,                          Address offset: 0x70
    CSR*: uint32               ## !< RCC clock control & status register,                         Address offset: 0x74
    RESERVED6*: array[2, uint32] ## !< Reserved, 0x78-0x7C
    SSCGR*: uint32             ## !< RCC spread spectrum clock generation register,               Address offset: 0x80
    PLLI2SCFGR*: uint32        ## !< RCC PLLI2S configuration register,                           Address offset: 0x84
    PLLSAICFGR*: uint32        ## !< RCC PLLSAI configuration register,                           Address offset: 0x88
    DCKCFGR*: uint32           ## !< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C


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
##  @brief Serial Audio Interface
##

type
  SAI_TypeDef* {.bycopy.} = object
    GCR*: uint32               ## !< SAI global configuration register,        Address offset: 0x00

  SAI_Block_TypeDef* {.bycopy.} = object
    CR1*: uint32               ## !< SAI block x configuration register 1,     Address offset: 0x04
    CR2*: uint32               ## !< SAI block x configuration register 2,     Address offset: 0x08
    FRCR*: uint32              ## !< SAI block x frame configuration register, Address offset: 0x0C
    SLOTR*: uint32             ## !< SAI block x slot register,                Address offset: 0x10
    IMR*: uint32               ## !< SAI block x interrupt mask register,      Address offset: 0x14
    SR*: uint32                ## !< SAI block x status register,              Address offset: 0x18
    CLRFR*: uint32             ## !< SAI block x clear flag register,          Address offset: 0x1C
    DR*: uint32                ## !< SAI block x data register,                Address offset: 0x20


## *
##  @brief SD host Interface
##

type
  SDIO_TypeDef* {.bycopy.} = object
    POWER*: uint32             ## !< SDIO power control register,    Address offset: 0x00
    CLKCR*: uint32             ## !< SDI clock control register,     Address offset: 0x04
    ARG*: uint32               ## !< SDIO argument register,         Address offset: 0x08
    CMD*: uint32               ## !< SDIO command register,          Address offset: 0x0C
    RESPCMD*: uint32           ## !< SDIO command response register, Address offset: 0x10
    RESP1*: uint32             ## !< SDIO response 1 register,       Address offset: 0x14
    RESP2*: uint32             ## !< SDIO response 2 register,       Address offset: 0x18
    RESP3*: uint32             ## !< SDIO response 3 register,       Address offset: 0x1C
    RESP4*: uint32             ## !< SDIO response 4 register,       Address offset: 0x20
    DTIMER*: uint32            ## !< SDIO data timer register,       Address offset: 0x24
    DLEN*: uint32              ## !< SDIO data length register,      Address offset: 0x28
    DCTRL*: uint32             ## !< SDIO data control register,     Address offset: 0x2C
    DCOUNT*: uint32            ## !< SDIO data counter register,     Address offset: 0x30
    STA*: uint32               ## !< SDIO status register,           Address offset: 0x34
    ICR*: uint32               ## !< SDIO interrupt clear register,  Address offset: 0x38
    MASK*: uint32              ## !< SDIO mask register,             Address offset: 0x3C
    RESERVED0*: array[2, uint32] ## !< Reserved, 0x40-0x44
    FIFOCNT*: uint32           ## !< SDIO FIFO counter register,     Address offset: 0x48
    RESERVED1*: array[13, uint32] ## !< Reserved, 0x4C-0x7C
    FIFO*: uint32              ## !< SDIO data FIFO register,        Address offset: 0x80


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
##  @brief QUAD Serial Peripheral Interface
##

type
  QUADSPI_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< QUADSPI Control register,                           Address offset: 0x00
    DCR*: uint32               ## !< QUADSPI Device Configuration register,              Address offset: 0x04
    SR*: uint32                ## !< QUADSPI Status register,                            Address offset: 0x08
    FCR*: uint32               ## !< QUADSPI Flag Clear register,                        Address offset: 0x0C
    DLR*: uint32               ## !< QUADSPI Data Length register,                       Address offset: 0x10
    CCR*: uint32               ## !< QUADSPI Communication Configuration register,       Address offset: 0x14
    AR*: uint32                ## !< QUADSPI Address register,                           Address offset: 0x18
    ABR*: uint32               ## !< QUADSPI Alternate Bytes register,                   Address offset: 0x1C
    DR*: uint32                ## !< QUADSPI Data register,                              Address offset: 0x20
    PSMKR*: uint32             ## !< QUADSPI Polling Status Mask register,               Address offset: 0x24
    PSMAR*: uint32             ## !< QUADSPI Polling Status Match register,              Address offset: 0x28
    PIR*: uint32               ## !< QUADSPI Polling Interval register,                  Address offset: 0x2C
    LPTR*: uint32              ## !< QUADSPI Low Power Timeout register,                 Address offset: 0x30


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
##  @brief Crypto Processor
##

type
  CRYP_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< CRYP control register,                                    Address offset: 0x00
    SR*: uint32                ## !< CRYP status register,                                     Address offset: 0x04
    DR*: uint32                ## !< CRYP data input register,                                 Address offset: 0x08
    DOUT*: uint32              ## !< CRYP data output register,                                Address offset: 0x0C
    DMACR*: uint32             ## !< CRYP DMA control register,                                Address offset: 0x10
    IMSCR*: uint32             ## !< CRYP interrupt mask set/clear register,                   Address offset: 0x14
    RISR*: uint32              ## !< CRYP raw interrupt status register,                       Address offset: 0x18
    MISR*: uint32              ## !< CRYP masked interrupt status register,                    Address offset: 0x1C
    K0LR*: uint32              ## !< CRYP key left  register 0,                                Address offset: 0x20
    K0RR*: uint32              ## !< CRYP key right register 0,                                Address offset: 0x24
    K1LR*: uint32              ## !< CRYP key left  register 1,                                Address offset: 0x28
    K1RR*: uint32              ## !< CRYP key right register 1,                                Address offset: 0x2C
    K2LR*: uint32              ## !< CRYP key left  register 2,                                Address offset: 0x30
    K2RR*: uint32              ## !< CRYP key right register 2,                                Address offset: 0x34
    K3LR*: uint32              ## !< CRYP key left  register 3,                                Address offset: 0x38
    K3RR*: uint32              ## !< CRYP key right register 3,                                Address offset: 0x3C
    IV0LR*: uint32             ## !< CRYP initialization vector left-word  register 0,         Address offset: 0x40
    IV0RR*: uint32             ## !< CRYP initialization vector right-word register 0,         Address offset: 0x44
    IV1LR*: uint32             ## !< CRYP initialization vector left-word  register 1,         Address offset: 0x48
    IV1RR*: uint32             ## !< CRYP initialization vector right-word register 1,         Address offset: 0x4C
    CSGCMCCM0R*: uint32        ## !< CRYP GCM/GMAC or CCM/CMAC context swap register 0,        Address offset: 0x50
    CSGCMCCM1R*: uint32        ## !< CRYP GCM/GMAC or CCM/CMAC context swap register 1,        Address offset: 0x54
    CSGCMCCM2R*: uint32        ## !< CRYP GCM/GMAC or CCM/CMAC context swap register 2,        Address offset: 0x58
    CSGCMCCM3R*: uint32        ## !< CRYP GCM/GMAC or CCM/CMAC context swap register 3,        Address offset: 0x5C
    CSGCMCCM4R*: uint32        ## !< CRYP GCM/GMAC or CCM/CMAC context swap register 4,        Address offset: 0x60
    CSGCMCCM5R*: uint32        ## !< CRYP GCM/GMAC or CCM/CMAC context swap register 5,        Address offset: 0x64
    CSGCMCCM6R*: uint32        ## !< CRYP GCM/GMAC or CCM/CMAC context swap register 6,        Address offset: 0x68
    CSGCMCCM7R*: uint32        ## !< CRYP GCM/GMAC or CCM/CMAC context swap register 7,        Address offset: 0x6C
    CSGCM0R*: uint32           ## !< CRYP GCM/GMAC context swap register 0,                    Address offset: 0x70
    CSGCM1R*: uint32           ## !< CRYP GCM/GMAC context swap register 1,                    Address offset: 0x74
    CSGCM2R*: uint32           ## !< CRYP GCM/GMAC context swap register 2,                    Address offset: 0x78
    CSGCM3R*: uint32           ## !< CRYP GCM/GMAC context swap register 3,                    Address offset: 0x7C
    CSGCM4R*: uint32           ## !< CRYP GCM/GMAC context swap register 4,                    Address offset: 0x80
    CSGCM5R*: uint32           ## !< CRYP GCM/GMAC context swap register 5,                    Address offset: 0x84
    CSGCM6R*: uint32           ## !< CRYP GCM/GMAC context swap register 6,                    Address offset: 0x88
    CSGCM7R*: uint32           ## !< CRYP GCM/GMAC context swap register 7,                    Address offset: 0x8C


## *
##  @brief HASH
##

type
  HASH_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< HASH control register,          Address offset: 0x00
    DIN*: uint32               ## !< HASH data input register,       Address offset: 0x04
    STR*: uint32               ## !< HASH start register,            Address offset: 0x08
    HR*: array[5, uint32]       ## !< HASH digest registers,          Address offset: 0x0C-0x1C
    IMR*: uint32               ## !< HASH interrupt enable register, Address offset: 0x20
    SR*: uint32                ## !< HASH status register,           Address offset: 0x24
    RESERVED*: array[52, uint32] ## !< Reserved, 0x28-0xF4
    CSR*: array[54, uint32]     ## !< HASH context swap registers,    Address offset: 0x0F8-0x1CC


## *
##  @brief HASH_DIGEST
##

type
  HASH_DIGEST_TypeDef* {.bycopy.} = object
    HR*: array[8, uint32]       ## !< HASH digest registers,          Address offset: 0x310-0x32C


## *
##  @brief RNG
##

type
  RNG_TypeDef* {.bycopy.} = object
    CR*: uint32                ## !< RNG control register, Address offset: 0x00
    SR*: uint32                ## !< RNG status register,  Address offset: 0x04
    DR*: uint32                ## !< RNG data register,    Address offset: 0x08


## *
##  @brief USB_OTG_Core_Registers
##

type
  USB_OTG_GlobalTypeDef* {.bycopy.} = object
    GOTGCTL*: uint32           ## !< USB_OTG Control and Status Register          000h
    GOTGINT*: uint32           ## !< USB_OTG Interrupt Register                   004h
    GAHBCFG*: uint32           ## !< Core AHB Configuration Register              008h
    GUSBCFG*: uint32           ## !< Core USB Configuration Register              00Ch
    GRSTCTL*: uint32           ## !< Core Reset Register                          010h
    GINTSTS*: uint32           ## !< Core Interrupt Register                      014h
    GINTMSK*: uint32           ## !< Core Interrupt Mask Register                 018h
    GRXSTSR*: uint32           ## !< Receive Sts Q Read Register                  01Ch
    GRXSTSP*: uint32           ## !< Receive Sts Q Read & POP Register            020h
    GRXFSIZ*: uint32           ## !< Receive FIFO Size Register                   024h
    DIEPTXF0_HNPTXFSIZ*: uint32 ## !< EP0 / Non Periodic Tx FIFO Size Register     028h
    HNPTXSTS*: uint32          ## !< Non Periodic Tx FIFO/Queue Sts reg           02Ch
    Reserved30*: array[2, uint32] ## !< Reserved                                     030h
    GCCFG*: uint32             ## !< General Purpose IO Register                  038h
    CID*: uint32               ## !< User ID Register                             03Ch
    Reserved5*: array[3, uint32] ## !< Reserved                                040h-048h
    GHWCFG3*: uint32           ## !< User HW config3                              04Ch
    Reserved6*: uint32         ## !< Reserved                                     050h
    GLPMCFG*: uint32           ## !< LPM Register                                 054h
    Reserved*: uint32          ## !< Reserved                                     058h
    GDFIFOCFG*: uint32         ## !< DFIFO Software Config Register               05Ch
    Reserved43*: array[40, uint32] ## !< Reserved                                058h-0FFh
    HPTXFSIZ*: uint32          ## !< Host Periodic Tx FIFO Size Reg               100h
    DIEPTXF*: array[0x0000000F, uint32] ## !< dev Periodic Transmit FIFO


## *
##  @brief USB_OTG_device_Registers
##

type
  USB_OTG_DeviceTypeDef* {.bycopy.} = object
    DCFG*: uint32              ## !< dev Configuration Register   800h
    DCTL*: uint32              ## !< dev Control Register         804h
    DSTS*: uint32              ## !< dev Status Register (RO)     808h
    Reserved0C*: uint32        ## !< Reserved                     80Ch
    DIEPMSK*: uint32           ## !< dev IN Endpoint Mask         810h
    DOEPMSK*: uint32           ## !< dev OUT Endpoint Mask        814h
    DAINT*: uint32             ## !< dev All Endpoints Itr Reg    818h
    DAINTMSK*: uint32          ## !< dev All Endpoints Itr Mask   81Ch
    Reserved20*: uint32        ## !< Reserved                     820h
    Reserved9*: uint32         ## !< Reserved                     824h
    DVBUSDIS*: uint32          ## !< dev VBUS discharge Register  828h
    DVBUSPULSE*: uint32        ## !< dev VBUS Pulse Register      82Ch
    DTHRCTL*: uint32           ## !< dev threshold                830h
    DIEPEMPMSK*: uint32        ## !< dev empty msk                834h
    DEACHINT*: uint32          ## !< dedicated EP interrupt       838h
    DEACHMSK*: uint32          ## !< dedicated EP msk             83Ch
    Reserved40*: uint32        ## !< dedicated EP mask            840h
    DINEP1MSK*: uint32         ## !< dedicated EP mask            844h
    Reserved44*: array[15, uint32] ## !< Reserved                 844-87Ch
    DOUTEP1MSK*: uint32        ## !< dedicated EP msk             884h


## *
##  @brief USB_OTG_IN_Endpoint-Specific_Register
##

type
  USB_OTG_INEndpointTypeDef* {.bycopy.} = object
    DIEPCTL*: uint32           ## !< dev IN Endpoint Control Reg    900h + (ep_num * 20h) + 00h
    Reserved04*: uint32        ## !< Reserved                       900h + (ep_num * 20h) + 04h
    DIEPINT*: uint32           ## !< dev IN Endpoint Itr Reg        900h + (ep_num * 20h) + 08h
    Reserved0C*: uint32        ## !< Reserved                       900h + (ep_num * 20h) + 0Ch
    DIEPTSIZ*: uint32          ## !< IN Endpoint Txfer Size         900h + (ep_num * 20h) + 10h
    DIEPDMA*: uint32           ## !< IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h
    DTXFSTS*: uint32           ## !< IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h
    Reserved18*: uint32        ## !< Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch


## *
##  @brief USB_OTG_OUT_Endpoint-Specific_Registers
##

type
  USB_OTG_OUTEndpointTypeDef* {.bycopy.} = object
    DOEPCTL*: uint32           ## !< dev OUT Endpoint Control Reg           B00h + (ep_num * 20h) + 00h
    Reserved04*: uint32        ## !< Reserved                               B00h + (ep_num * 20h) + 04h
    DOEPINT*: uint32           ## !< dev OUT Endpoint Itr Reg               B00h + (ep_num * 20h) + 08h
    Reserved0C*: uint32        ## !< Reserved                               B00h + (ep_num * 20h) + 0Ch
    DOEPTSIZ*: uint32          ## !< dev OUT Endpoint Txfer Size            B00h + (ep_num * 20h) + 10h
    DOEPDMA*: uint32           ## !< dev OUT Endpoint DMA Address           B00h + (ep_num * 20h) + 14h
    Reserved18*: array[2, uint32] ## !< Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch


## *
##  @brief USB_OTG_Host_Mode_Register_Structures
##

type
  USB_OTG_HostTypeDef* {.bycopy.} = object
    HCFG*: uint32              ## !< Host Configuration Register          400h
    HFIR*: uint32              ## !< Host Frame Interval Register         404h
    HFNUM*: uint32             ## !< Host Frame Nbr/Frame Remaining       408h
    Reserved40C*: uint32       ## !< Reserved                             40Ch
    HPTXSTS*: uint32           ## !< Host Periodic Tx FIFO/ Queue Status  410h
    HAINT*: uint32             ## !< Host All Channels Interrupt Register 414h
    HAINTMSK*: uint32          ## !< Host All Channels Interrupt Mask     418h


## *
##  @brief USB_OTG_Host_Channel_Specific_Registers
##

type
  USB_OTG_HostChannelTypeDef* {.bycopy.} = object
    HCCHAR*: uint32            ## !< Host Channel Characteristics Register    500h
    HCSPLT*: uint32            ## !< Host Channel Split Control Register      504h
    HCINT*: uint32             ## !< Host Channel Interrupt Register          508h
    HCINTMSK*: uint32          ## !< Host Channel Interrupt Mask Register     50Ch
    HCTSIZ*: uint32            ## !< Host Channel Transfer Size Register      510h
    HCDMA*: uint32             ## !< Host Channel DMA Address Register        514h
    Reserved*: array[2, uint32] ## !< Reserved


## *
##  @}
##
## * @addtogroup Peripheral_memory_map
##  @{
##

const
  FLASH_BASE* = 0x08000000
  CCMDATARAM_BASE* = 0x10000000
  SRAM1_BASE* = 0x20000000
  SRAM2_BASE* = 0x20028000
  SRAM3_BASE* = 0x20030000
  PERIPH_BASE* = 0x40000000
  BKPSRAM_BASE* = 0x40024000
  FMC_R_BASE* = 0xA0000000
  QSPI_R_BASE* = 0xA0001000
  SRAM1_BB_BASE* = 0x22000000
  SRAM2_BB_BASE* = 0x22500000
  SRAM3_BB_BASE* = 0x22600000
  PERIPH_BB_BASE* = 0x42000000
  BKPSRAM_BB_BASE* = 0x42480000
  FLASH_END* = 0x081FFFFF
  FLASH_OTP_BASE* = 0x1FFF7800
  FLASH_OTP_END* = 0x1FFF7A0F
  CCMDATARAM_END* = 0x1000FFFF

##  Legacy defines

const
  SRAM_BASE* = SRAM1_BASE
  SRAM_BB_BASE* = SRAM1_BB_BASE

## !< Peripheral memory map

const
  APB1PERIPH_BASE* = PERIPH_BASE
  APB2PERIPH_BASE* = (PERIPH_BASE + 0x00010000)
  AHB1PERIPH_BASE* = (PERIPH_BASE + 0x00020000)
  AHB2PERIPH_BASE* = (PERIPH_BASE + 0x10000000)

## !< APB1 peripherals

const
  TIM2_BASE* = (APB1PERIPH_BASE + 0x00000000)
  TIM3_BASE* = (APB1PERIPH_BASE + 0x00000400)
  TIM4_BASE* = (APB1PERIPH_BASE + 0x00000800)
  TIM5_BASE* = (APB1PERIPH_BASE + 0x00000C00)
  TIM6_BASE* = (APB1PERIPH_BASE + 0x00001000)
  TIM7_BASE* = (APB1PERIPH_BASE + 0x00001400)
  TIM12_BASE* = (APB1PERIPH_BASE + 0x00001800)
  TIM13_BASE* = (APB1PERIPH_BASE + 0x00001C00)
  TIM14_BASE* = (APB1PERIPH_BASE + 0x00002000)
  RTC_BASE* = (APB1PERIPH_BASE + 0x00002800)
  WWDG_BASE* = (APB1PERIPH_BASE + 0x00002C00)
  IWDG_BASE* = (APB1PERIPH_BASE + 0x00003000)
  I2S2ext_BASE* = (APB1PERIPH_BASE + 0x00003400)
  SPI2_BASE* = (APB1PERIPH_BASE + 0x00003800)
  SPI3_BASE* = (APB1PERIPH_BASE + 0x00003C00)
  I2S3ext_BASE* = (APB1PERIPH_BASE + 0x00004000)
  USART2_BASE* = (APB1PERIPH_BASE + 0x00004400)
  USART3_BASE* = (APB1PERIPH_BASE + 0x00004800)
  UART4_BASE* = (APB1PERIPH_BASE + 0x00004C00)
  UART5_BASE* = (APB1PERIPH_BASE + 0x00005000)
  I2C1_BASE* = (APB1PERIPH_BASE + 0x00005400)
  I2C2_BASE* = (APB1PERIPH_BASE + 0x00005800)
  I2C3_BASE* = (APB1PERIPH_BASE + 0x00005C00)
  CAN1_BASE* = (APB1PERIPH_BASE + 0x00006400)
  CAN2_BASE* = (APB1PERIPH_BASE + 0x00006800)
  PWR_BASE* = (APB1PERIPH_BASE + 0x00007000)
  DAC_BASE* = (APB1PERIPH_BASE + 0x00007400)
  UART7_BASE* = (APB1PERIPH_BASE + 0x00007800)
  UART8_BASE* = (APB1PERIPH_BASE + 0x00007C00)

## !< APB2 peripherals

const
  TIM1_BASE* = (APB2PERIPH_BASE + 0x00000000)
  TIM8_BASE* = (APB2PERIPH_BASE + 0x00000400)
  USART1_BASE* = (APB2PERIPH_BASE + 0x00001000)
  USART6_BASE* = (APB2PERIPH_BASE + 0x00001400)
  ADC1_BASE* = (APB2PERIPH_BASE + 0x00002000)
  ADC2_BASE* = (APB2PERIPH_BASE + 0x00002100)
  ADC3_BASE* = (APB2PERIPH_BASE + 0x00002200)
  ADC123_COMMON_BASE* = (APB2PERIPH_BASE + 0x00002300)

##  Legacy define

const
  ADC_BASE* = ADC123_COMMON_BASE
  SDIO_BASE* = (APB2PERIPH_BASE + 0x00002C00)
  SPI1_BASE* = (APB2PERIPH_BASE + 0x00003000)
  SPI4_BASE* = (APB2PERIPH_BASE + 0x00003400)
  SYSCFG_BASE* = (APB2PERIPH_BASE + 0x00003800)
  EXTI_BASE* = (APB2PERIPH_BASE + 0x00003C00)
  TIM9_BASE* = (APB2PERIPH_BASE + 0x00004000)
  TIM10_BASE* = (APB2PERIPH_BASE + 0x00004400)
  TIM11_BASE* = (APB2PERIPH_BASE + 0x00004800)
  SPI5_BASE* = (APB2PERIPH_BASE + 0x00005000)
  SPI6_BASE* = (APB2PERIPH_BASE + 0x00005400)
  SAI1_BASE* = (APB2PERIPH_BASE + 0x00005800)
  SAI1_Block_A_BASE* = (SAI1_BASE + 0x00000004)
  SAI1_Block_B_BASE* = (SAI1_BASE + 0x00000024)
  LTDC_BASE* = (APB2PERIPH_BASE + 0x00006800)
  LTDC_Layer1_BASE* = (LTDC_BASE + 0x00000084)
  LTDC_Layer2_BASE* = (LTDC_BASE + 0x00000104)
  DSI_BASE* = (APB2PERIPH_BASE + 0x00006C00)

## !< AHB1 peripherals

const
  GPIOA_BASE* = (AHB1PERIPH_BASE + 0x00000000)
  GPIOB_BASE* = (AHB1PERIPH_BASE + 0x00000400)
  GPIOC_BASE* = (AHB1PERIPH_BASE + 0x00000800)
  GPIOD_BASE* = (AHB1PERIPH_BASE + 0x00000C00)
  GPIOE_BASE* = (AHB1PERIPH_BASE + 0x00001000)
  GPIOF_BASE* = (AHB1PERIPH_BASE + 0x00001400)
  GPIOG_BASE* = (AHB1PERIPH_BASE + 0x00001800)
  GPIOH_BASE* = (AHB1PERIPH_BASE + 0x00001C00)
  GPIOI_BASE* = (AHB1PERIPH_BASE + 0x00002000)
  GPIOJ_BASE* = (AHB1PERIPH_BASE + 0x00002400)
  GPIOK_BASE* = (AHB1PERIPH_BASE + 0x00002800)
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
  ETH_BASE* = (AHB1PERIPH_BASE + 0x00008000)
  ETH_MAC_BASE* = (ETH_BASE)
  ETH_MMC_BASE* = (ETH_BASE + 0x00000100)
  ETH_PTP_BASE* = (ETH_BASE + 0x00000700)
  ETH_DMA_BASE* = (ETH_BASE + 0x00001000)
  DMA2D_BASE* = (AHB1PERIPH_BASE + 0x0000B000)

## !< AHB2 peripherals

const
  DCMI_BASE* = (AHB2PERIPH_BASE + 0x00050000)
  CRYP_BASE* = (AHB2PERIPH_BASE + 0x00060000)
  HASH_BASE* = (AHB2PERIPH_BASE + 0x00060400)
  HASH_DIGEST_BASE* = (AHB2PERIPH_BASE + 0x00060710)
  RNG_BASE* = (AHB2PERIPH_BASE + 0x00060800)

## !< FMC Bankx registers base address

const
  FMC_Bank1_R_BASE* = (FMC_R_BASE + 0x00000000)
  FMC_Bank1E_R_BASE* = (FMC_R_BASE + 0x00000104)
  FMC_Bank3_R_BASE* = (FMC_R_BASE + 0x00000080)
  FMC_Bank5_Bit6_R_BASE* = (FMC_R_BASE + 0x00000140)

## !< Debug MCU registers base address

const
  DBGMCU_BASE* = 0xE0042000

## !< USB registers base address

const
  USB_OTG_HS_PERIPH_BASE* = 0x40040000
  USB_OTG_FS_PERIPH_BASE* = 0x50000000
  USB_OTG_GLOBAL_BASE* = 0x00000000
  USB_OTG_DEVICE_BASE* = 0x00000800
  USB_OTG_IN_ENDPOINT_BASE* = 0x00000900
  USB_OTG_OUT_ENDPOINT_BASE* = 0x00000B00
  USB_OTG_EP_REG_SIZE* = 0x00000020
  USB_OTG_HOST_BASE* = 0x00000400
  USB_OTG_HOST_PORT_BASE* = 0x00000440
  USB_OTG_HOST_CHANNEL_BASE* = 0x00000500
  USB_OTG_HOST_CHANNEL_SIZE* = 0x00000020
  USB_OTG_PCGCCTL_BASE* = 0x00000E00
  USB_OTG_FIFO_BASE* = 0x00001000
  USB_OTG_FIFO_SIZE* = 0x00001000
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
  TIM2* = (cast[ptr TIM_TypeDef](TIM2_BASE))
  TIM3* = (cast[ptr TIM_TypeDef](TIM3_BASE))
  TIM4* = (cast[ptr TIM_TypeDef](TIM4_BASE))
  TIM5* = (cast[ptr TIM_TypeDef](TIM5_BASE))
  TIM6* = (cast[ptr TIM_TypeDef](TIM6_BASE))
  TIM7* = (cast[ptr TIM_TypeDef](TIM7_BASE))
  TIM12* = (cast[ptr TIM_TypeDef](TIM12_BASE))
  TIM13* = (cast[ptr TIM_TypeDef](TIM13_BASE))
  TIM14* = (cast[ptr TIM_TypeDef](TIM14_BASE))
  RTC* = (cast[ptr RTC_TypeDef](RTC_BASE))
  WWDG* = (cast[ptr WWDG_TypeDef](WWDG_BASE))
  IWDG* = (cast[ptr IWDG_TypeDef](IWDG_BASE))
  I2S2ext* = (cast[ptr SPI_TypeDef](I2S2ext_BASE))
  SPI2* = (cast[ptr SPI_TypeDef](SPI2_BASE))
  SPI3* = (cast[ptr SPI_TypeDef](SPI3_BASE))
  I2S3ext* = (cast[ptr SPI_TypeDef](I2S3ext_BASE))
  USART2* = (cast[ptr USART_TypeDef](USART2_BASE))
  USART3* = (cast[ptr USART_TypeDef](USART3_BASE))
  UART4* = (cast[ptr USART_TypeDef](UART4_BASE))
  UART5* = (cast[ptr USART_TypeDef](UART5_BASE))
  I2C1* = (cast[ptr I2C_TypeDef](I2C1_BASE))
  I2C2* = (cast[ptr I2C_TypeDef](I2C2_BASE))
  I2C3* = (cast[ptr I2C_TypeDef](I2C3_BASE))
  CAN1* = (cast[ptr CAN_TypeDef](CAN1_BASE))
  CAN2* = (cast[ptr CAN_TypeDef](CAN2_BASE))
  PWR* = (cast[ptr PWR_TypeDef](PWR_BASE))
  DAC1* = (cast[ptr DAC_TypeDef](DAC_BASE))
  DAC* = (cast[ptr DAC_TypeDef](DAC_BASE)) ##  Kept for legacy purpose
  UART7* = (cast[ptr USART_TypeDef](UART7_BASE))
  UART8* = (cast[ptr USART_TypeDef](UART8_BASE))
  TIM1* = (cast[ptr TIM_TypeDef](TIM1_BASE))
  TIM8* = (cast[ptr TIM_TypeDef](TIM8_BASE))
  USART1* = (cast[ptr USART_TypeDef](USART1_BASE))
  USART6* = (cast[ptr USART_TypeDef](USART6_BASE))
  ADC1* = (cast[ptr ADC_TypeDef](ADC1_BASE))
  ADC2* = (cast[ptr ADC_TypeDef](ADC2_BASE))
  ADC3* = (cast[ptr ADC_TypeDef](ADC3_BASE))
  ADC123_COMMON* = (cast[ptr ADC_Common_TypeDef](ADC123_COMMON_BASE))

##  Legacy define

const
  ADC* = ADC123_COMMON
  SDIO* = (cast[ptr SDIO_TypeDef](SDIO_BASE))
  SPI1* = (cast[ptr SPI_TypeDef](SPI1_BASE))
  SPI4* = (cast[ptr SPI_TypeDef](SPI4_BASE))
  SYSCFG* = (cast[ptr SYSCFG_TypeDef](SYSCFG_BASE))
  EXTI* = (cast[ptr EXTI_TypeDef](EXTI_BASE))
  TIM9* = (cast[ptr TIM_TypeDef](TIM9_BASE))
  TIM10* = (cast[ptr TIM_TypeDef](TIM10_BASE))
  TIM11* = (cast[ptr TIM_TypeDef](TIM11_BASE))
  SPI5* = (cast[ptr SPI_TypeDef](SPI5_BASE))
  SPI6* = (cast[ptr SPI_TypeDef](SPI6_BASE))
  SAI1* = (cast[ptr SAI_TypeDef](SAI1_BASE))
  SAI1_Block_A* = (cast[ptr SAI_Block_TypeDef](SAI1_Block_A_BASE))
  SAI1_Block_B* = (cast[ptr SAI_Block_TypeDef](SAI1_Block_B_BASE))
  LTDC* = (cast[ptr LTDC_TypeDef](LTDC_BASE))
  LTDC_Layer1* = (cast[ptr LTDC_Layer_TypeDef](LTDC_Layer1_BASE))
  LTDC_Layer2* = (cast[ptr LTDC_Layer_TypeDef](LTDC_Layer2_BASE))
  DSI* = (cast[ptr DSI_TypeDef](DSI_BASE))
  GPIOA* = (cast[ptr GPIO_TypeDef](GPIOA_BASE))
  GPIOB* = (cast[ptr GPIO_TypeDef](GPIOB_BASE))
  GPIOC* = (cast[ptr GPIO_TypeDef](GPIOC_BASE))
  GPIOD* = (cast[ptr GPIO_TypeDef](GPIOD_BASE))
  GPIOE* = (cast[ptr GPIO_TypeDef](GPIOE_BASE))
  GPIOF* = (cast[ptr GPIO_TypeDef](GPIOF_BASE))
  GPIOG* = (cast[ptr GPIO_TypeDef](GPIOG_BASE))
  GPIOH* = (cast[ptr GPIO_TypeDef](GPIOH_BASE))
  GPIOI* = (cast[ptr GPIO_TypeDef](GPIOI_BASE))
  GPIOJ* = (cast[ptr GPIO_TypeDef](GPIOJ_BASE))
  GPIOK* = (cast[ptr GPIO_TypeDef](GPIOK_BASE))
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
  ETH* = (cast[ptr ETH_TypeDef](ETH_BASE))
  DMA2D* = (cast[ptr DMA2D_TypeDef](DMA2D_BASE))
  DCMI* = (cast[ptr DCMI_TypeDef](DCMI_BASE))
  CRYP* = (cast[ptr CRYP_TypeDef](CRYP_BASE))
  HASH* = (cast[ptr HASH_TypeDef](HASH_BASE))
  HASH_DIGEST* = (cast[ptr HASH_DIGEST_TypeDef](HASH_DIGEST_BASE))
  RNG* = (cast[ptr RNG_TypeDef](RNG_BASE))
  FMC_Bank1* = (cast[ptr FMC_Bank1_TypeDef](FMC_Bank1_R_BASE))
  FMC_Bank1E* = (cast[ptr FMC_Bank1E_TypeDef](FMC_Bank1E_R_BASE))
  FMC_Bank3* = (cast[ptr FMC_Bank3_TypeDef](FMC_Bank3_R_BASE))
  FMC_Bank5_Bit6* = (cast[ptr FMC_Bank5_Bit6_TypeDef](FMC_Bank5_Bit6_R_BASE))
  QUADSPI* = (cast[ptr QUADSPI_TypeDef](QSPI_R_BASE))
  DBGMCU* = (cast[ptr DBGMCU_TypeDef](DBGMCU_BASE))
  USB_OTG_FS* = (cast[ptr USB_OTG_GlobalTypeDef](USB_OTG_FS_PERIPH_BASE))
  USB_OTG_HS* = (cast[ptr USB_OTG_GlobalTypeDef](USB_OTG_HS_PERIPH_BASE))

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
##
##  @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
##

const
  ADC_MULTIMODE_SUPPORT* = true ## !<ADC Multimode feature available on specific devices

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
  ADC_CSR_AWD2_Pos* = (8)
  ADC_CSR_AWD2_Msk* = (0x00000001 shl ADC_CSR_AWD2_Pos) ## !< 0x00000100
  ADC_CSR_AWD2* = ADC_CSR_AWD2_Msk
  ADC_CSR_EOC2_Pos* = (9)
  ADC_CSR_EOC2_Msk* = (0x00000001 shl ADC_CSR_EOC2_Pos) ## !< 0x00000200
  ADC_CSR_EOC2* = ADC_CSR_EOC2_Msk
  ADC_CSR_JEOC2_Pos* = (10)
  ADC_CSR_JEOC2_Msk* = (0x00000001 shl ADC_CSR_JEOC2_Pos) ## !< 0x00000400
  ADC_CSR_JEOC2* = ADC_CSR_JEOC2_Msk
  ADC_CSR_JSTRT2_Pos* = (11)
  ADC_CSR_JSTRT2_Msk* = (0x00000001 shl ADC_CSR_JSTRT2_Pos) ## !< 0x00000800
  ADC_CSR_JSTRT2* = ADC_CSR_JSTRT2_Msk
  ADC_CSR_STRT2_Pos* = (12)
  ADC_CSR_STRT2_Msk* = (0x00000001 shl ADC_CSR_STRT2_Pos) ## !< 0x00001000
  ADC_CSR_STRT2* = ADC_CSR_STRT2_Msk
  ADC_CSR_OVR2_Pos* = (13)
  ADC_CSR_OVR2_Msk* = (0x00000001 shl ADC_CSR_OVR2_Pos) ## !< 0x00002000
  ADC_CSR_OVR2* = ADC_CSR_OVR2_Msk
  ADC_CSR_AWD3_Pos* = (16)
  ADC_CSR_AWD3_Msk* = (0x00000001 shl ADC_CSR_AWD3_Pos) ## !< 0x00010000
  ADC_CSR_AWD3* = ADC_CSR_AWD3_Msk
  ADC_CSR_EOC3_Pos* = (17)
  ADC_CSR_EOC3_Msk* = (0x00000001 shl ADC_CSR_EOC3_Pos) ## !< 0x00020000
  ADC_CSR_EOC3* = ADC_CSR_EOC3_Msk
  ADC_CSR_JEOC3_Pos* = (18)
  ADC_CSR_JEOC3_Msk* = (0x00000001 shl ADC_CSR_JEOC3_Pos) ## !< 0x00040000
  ADC_CSR_JEOC3* = ADC_CSR_JEOC3_Msk
  ADC_CSR_JSTRT3_Pos* = (19)
  ADC_CSR_JSTRT3_Msk* = (0x00000001 shl ADC_CSR_JSTRT3_Pos) ## !< 0x00080000
  ADC_CSR_JSTRT3* = ADC_CSR_JSTRT3_Msk
  ADC_CSR_STRT3_Pos* = (20)
  ADC_CSR_STRT3_Msk* = (0x00000001 shl ADC_CSR_STRT3_Pos) ## !< 0x00100000
  ADC_CSR_STRT3* = ADC_CSR_STRT3_Msk
  ADC_CSR_OVR3_Pos* = (21)
  ADC_CSR_OVR3_Msk* = (0x00000001 shl ADC_CSR_OVR3_Pos) ## !< 0x00200000
  ADC_CSR_OVR3* = ADC_CSR_OVR3_Msk

##  Legacy defines

const
  ADC_CSR_DOVR1* = ADC_CSR_OVR1
  ADC_CSR_DOVR2* = ADC_CSR_OVR2
  ADC_CSR_DOVR3* = ADC_CSR_OVR3

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
##                          Controller Area Network
##
## ****************************************************************************
## !<CAN control and status registers
## ******************  Bit definition for CAN_MCR register  *******************

const
  CAN_MCR_INRQ_Pos* = (0)
  CAN_MCR_INRQ_Msk* = (0x00000001 shl CAN_MCR_INRQ_Pos) ## !< 0x00000001
  CAN_MCR_INRQ* = CAN_MCR_INRQ_Msk
  CAN_MCR_SLEEP_Pos* = (1)
  CAN_MCR_SLEEP_Msk* = (0x00000001 shl CAN_MCR_SLEEP_Pos) ## !< 0x00000002
  CAN_MCR_SLEEP* = CAN_MCR_SLEEP_Msk
  CAN_MCR_TXFP_Pos* = (2)
  CAN_MCR_TXFP_Msk* = (0x00000001 shl CAN_MCR_TXFP_Pos) ## !< 0x00000004
  CAN_MCR_TXFP* = CAN_MCR_TXFP_Msk
  CAN_MCR_RFLM_Pos* = (3)
  CAN_MCR_RFLM_Msk* = (0x00000001 shl CAN_MCR_RFLM_Pos) ## !< 0x00000008
  CAN_MCR_RFLM* = CAN_MCR_RFLM_Msk
  CAN_MCR_NART_Pos* = (4)
  CAN_MCR_NART_Msk* = (0x00000001 shl CAN_MCR_NART_Pos) ## !< 0x00000010
  CAN_MCR_NART* = CAN_MCR_NART_Msk
  CAN_MCR_AWUM_Pos* = (5)
  CAN_MCR_AWUM_Msk* = (0x00000001 shl CAN_MCR_AWUM_Pos) ## !< 0x00000020
  CAN_MCR_AWUM* = CAN_MCR_AWUM_Msk
  CAN_MCR_ABOM_Pos* = (6)
  CAN_MCR_ABOM_Msk* = (0x00000001 shl CAN_MCR_ABOM_Pos) ## !< 0x00000040
  CAN_MCR_ABOM* = CAN_MCR_ABOM_Msk
  CAN_MCR_TTCM_Pos* = (7)
  CAN_MCR_TTCM_Msk* = (0x00000001 shl CAN_MCR_TTCM_Pos) ## !< 0x00000080
  CAN_MCR_TTCM* = CAN_MCR_TTCM_Msk
  CAN_MCR_RESET_Pos* = (15)
  CAN_MCR_RESET_Msk* = (0x00000001 shl CAN_MCR_RESET_Pos) ## !< 0x00008000
  CAN_MCR_RESET* = CAN_MCR_RESET_Msk
  CAN_MCR_DBF_Pos* = (16)
  CAN_MCR_DBF_Msk* = (0x00000001 shl CAN_MCR_DBF_Pos) ## !< 0x00010000
  CAN_MCR_DBF* = CAN_MCR_DBF_Msk

## ******************  Bit definition for CAN_MSR register  *******************

const
  CAN_MSR_INAK_Pos* = (0)
  CAN_MSR_INAK_Msk* = (0x00000001 shl CAN_MSR_INAK_Pos) ## !< 0x00000001
  CAN_MSR_INAK* = CAN_MSR_INAK_Msk
  CAN_MSR_SLAK_Pos* = (1)
  CAN_MSR_SLAK_Msk* = (0x00000001 shl CAN_MSR_SLAK_Pos) ## !< 0x00000002
  CAN_MSR_SLAK* = CAN_MSR_SLAK_Msk
  CAN_MSR_ERRI_Pos* = (2)
  CAN_MSR_ERRI_Msk* = (0x00000001 shl CAN_MSR_ERRI_Pos) ## !< 0x00000004
  CAN_MSR_ERRI* = CAN_MSR_ERRI_Msk
  CAN_MSR_WKUI_Pos* = (3)
  CAN_MSR_WKUI_Msk* = (0x00000001 shl CAN_MSR_WKUI_Pos) ## !< 0x00000008
  CAN_MSR_WKUI* = CAN_MSR_WKUI_Msk
  CAN_MSR_SLAKI_Pos* = (4)
  CAN_MSR_SLAKI_Msk* = (0x00000001 shl CAN_MSR_SLAKI_Pos) ## !< 0x00000010
  CAN_MSR_SLAKI* = CAN_MSR_SLAKI_Msk
  CAN_MSR_TXM_Pos* = (8)
  CAN_MSR_TXM_Msk* = (0x00000001 shl CAN_MSR_TXM_Pos) ## !< 0x00000100
  CAN_MSR_TXM* = CAN_MSR_TXM_Msk
  CAN_MSR_RXM_Pos* = (9)
  CAN_MSR_RXM_Msk* = (0x00000001 shl CAN_MSR_RXM_Pos) ## !< 0x00000200
  CAN_MSR_RXM* = CAN_MSR_RXM_Msk
  CAN_MSR_SAMP_Pos* = (10)
  CAN_MSR_SAMP_Msk* = (0x00000001 shl CAN_MSR_SAMP_Pos) ## !< 0x00000400
  CAN_MSR_SAMP* = CAN_MSR_SAMP_Msk
  CAN_MSR_RX_Pos* = (11)
  CAN_MSR_RX_Msk* = (0x00000001 shl CAN_MSR_RX_Pos) ## !< 0x00000800
  CAN_MSR_RX* = CAN_MSR_RX_Msk

## ******************  Bit definition for CAN_TSR register  *******************

const
  CAN_TSR_RQCP0_Pos* = (0)
  CAN_TSR_RQCP0_Msk* = (0x00000001 shl CAN_TSR_RQCP0_Pos) ## !< 0x00000001
  CAN_TSR_RQCP0* = CAN_TSR_RQCP0_Msk
  CAN_TSR_TXOK0_Pos* = (1)
  CAN_TSR_TXOK0_Msk* = (0x00000001 shl CAN_TSR_TXOK0_Pos) ## !< 0x00000002
  CAN_TSR_TXOK0* = CAN_TSR_TXOK0_Msk
  CAN_TSR_ALST0_Pos* = (2)
  CAN_TSR_ALST0_Msk* = (0x00000001 shl CAN_TSR_ALST0_Pos) ## !< 0x00000004
  CAN_TSR_ALST0* = CAN_TSR_ALST0_Msk
  CAN_TSR_TERR0_Pos* = (3)
  CAN_TSR_TERR0_Msk* = (0x00000001 shl CAN_TSR_TERR0_Pos) ## !< 0x00000008
  CAN_TSR_TERR0* = CAN_TSR_TERR0_Msk
  CAN_TSR_ABRQ0_Pos* = (7)
  CAN_TSR_ABRQ0_Msk* = (0x00000001 shl CAN_TSR_ABRQ0_Pos) ## !< 0x00000080
  CAN_TSR_ABRQ0* = CAN_TSR_ABRQ0_Msk
  CAN_TSR_RQCP1_Pos* = (8)
  CAN_TSR_RQCP1_Msk* = (0x00000001 shl CAN_TSR_RQCP1_Pos) ## !< 0x00000100
  CAN_TSR_RQCP1* = CAN_TSR_RQCP1_Msk
  CAN_TSR_TXOK1_Pos* = (9)
  CAN_TSR_TXOK1_Msk* = (0x00000001 shl CAN_TSR_TXOK1_Pos) ## !< 0x00000200
  CAN_TSR_TXOK1* = CAN_TSR_TXOK1_Msk
  CAN_TSR_ALST1_Pos* = (10)
  CAN_TSR_ALST1_Msk* = (0x00000001 shl CAN_TSR_ALST1_Pos) ## !< 0x00000400
  CAN_TSR_ALST1* = CAN_TSR_ALST1_Msk
  CAN_TSR_TERR1_Pos* = (11)
  CAN_TSR_TERR1_Msk* = (0x00000001 shl CAN_TSR_TERR1_Pos) ## !< 0x00000800
  CAN_TSR_TERR1* = CAN_TSR_TERR1_Msk
  CAN_TSR_ABRQ1_Pos* = (15)
  CAN_TSR_ABRQ1_Msk* = (0x00000001 shl CAN_TSR_ABRQ1_Pos) ## !< 0x00008000
  CAN_TSR_ABRQ1* = CAN_TSR_ABRQ1_Msk
  CAN_TSR_RQCP2_Pos* = (16)
  CAN_TSR_RQCP2_Msk* = (0x00000001 shl CAN_TSR_RQCP2_Pos) ## !< 0x00010000
  CAN_TSR_RQCP2* = CAN_TSR_RQCP2_Msk
  CAN_TSR_TXOK2_Pos* = (17)
  CAN_TSR_TXOK2_Msk* = (0x00000001 shl CAN_TSR_TXOK2_Pos) ## !< 0x00020000
  CAN_TSR_TXOK2* = CAN_TSR_TXOK2_Msk
  CAN_TSR_ALST2_Pos* = (18)
  CAN_TSR_ALST2_Msk* = (0x00000001 shl CAN_TSR_ALST2_Pos) ## !< 0x00040000
  CAN_TSR_ALST2* = CAN_TSR_ALST2_Msk
  CAN_TSR_TERR2_Pos* = (19)
  CAN_TSR_TERR2_Msk* = (0x00000001 shl CAN_TSR_TERR2_Pos) ## !< 0x00080000
  CAN_TSR_TERR2* = CAN_TSR_TERR2_Msk
  CAN_TSR_ABRQ2_Pos* = (23)
  CAN_TSR_ABRQ2_Msk* = (0x00000001 shl CAN_TSR_ABRQ2_Pos) ## !< 0x00800000
  CAN_TSR_ABRQ2* = CAN_TSR_ABRQ2_Msk
  CAN_TSR_CODE_Pos* = (24)
  CAN_TSR_CODE_Msk* = (0x00000003 shl CAN_TSR_CODE_Pos) ## !< 0x03000000
  CAN_TSR_CODE* = CAN_TSR_CODE_Msk
  CAN_TSR_TME_Pos* = (26)
  CAN_TSR_TME_Msk* = (0x00000007 shl CAN_TSR_TME_Pos) ## !< 0x1C000000
  CAN_TSR_TME* = CAN_TSR_TME_Msk
  CAN_TSR_TME0_Pos* = (26)
  CAN_TSR_TME0_Msk* = (0x00000001 shl CAN_TSR_TME0_Pos) ## !< 0x04000000
  CAN_TSR_TME0* = CAN_TSR_TME0_Msk
  CAN_TSR_TME1_Pos* = (27)
  CAN_TSR_TME1_Msk* = (0x00000001 shl CAN_TSR_TME1_Pos) ## !< 0x08000000
  CAN_TSR_TME1* = CAN_TSR_TME1_Msk
  CAN_TSR_TME2_Pos* = (28)
  CAN_TSR_TME2_Msk* = (0x00000001 shl CAN_TSR_TME2_Pos) ## !< 0x10000000
  CAN_TSR_TME2* = CAN_TSR_TME2_Msk
  CAN_TSR_LOW_Pos* = (29)
  CAN_TSR_LOW_Msk* = (0x00000007 shl CAN_TSR_LOW_Pos) ## !< 0xE0000000
  CAN_TSR_LOW* = CAN_TSR_LOW_Msk
  CAN_TSR_LOW0_Pos* = (29)
  CAN_TSR_LOW0_Msk* = (0x00000001 shl CAN_TSR_LOW0_Pos) ## !< 0x20000000
  CAN_TSR_LOW0* = CAN_TSR_LOW0_Msk
  CAN_TSR_LOW1_Pos* = (30)
  CAN_TSR_LOW1_Msk* = (0x00000001 shl CAN_TSR_LOW1_Pos) ## !< 0x40000000
  CAN_TSR_LOW1* = CAN_TSR_LOW1_Msk
  CAN_TSR_LOW2_Pos* = (31)
  CAN_TSR_LOW2_Msk* = (0x00000001 shl CAN_TSR_LOW2_Pos) ## !< 0x80000000
  CAN_TSR_LOW2* = CAN_TSR_LOW2_Msk

## ******************  Bit definition for CAN_RF0R register  ******************

const
  CAN_RF0R_FMP0_Pos* = (0)
  CAN_RF0R_FMP0_Msk* = (0x00000003 shl CAN_RF0R_FMP0_Pos) ## !< 0x00000003
  CAN_RF0R_FMP0* = CAN_RF0R_FMP0_Msk
  CAN_RF0R_FULL0_Pos* = (3)
  CAN_RF0R_FULL0_Msk* = (0x00000001 shl CAN_RF0R_FULL0_Pos) ## !< 0x00000008
  CAN_RF0R_FULL0* = CAN_RF0R_FULL0_Msk
  CAN_RF0R_FOVR0_Pos* = (4)
  CAN_RF0R_FOVR0_Msk* = (0x00000001 shl CAN_RF0R_FOVR0_Pos) ## !< 0x00000010
  CAN_RF0R_FOVR0* = CAN_RF0R_FOVR0_Msk
  CAN_RF0R_RFOM0_Pos* = (5)
  CAN_RF0R_RFOM0_Msk* = (0x00000001 shl CAN_RF0R_RFOM0_Pos) ## !< 0x00000020
  CAN_RF0R_RFOM0* = CAN_RF0R_RFOM0_Msk

## ******************  Bit definition for CAN_RF1R register  ******************

const
  CAN_RF1R_FMP1_Pos* = (0)
  CAN_RF1R_FMP1_Msk* = (0x00000003 shl CAN_RF1R_FMP1_Pos) ## !< 0x00000003
  CAN_RF1R_FMP1* = CAN_RF1R_FMP1_Msk
  CAN_RF1R_FULL1_Pos* = (3)
  CAN_RF1R_FULL1_Msk* = (0x00000001 shl CAN_RF1R_FULL1_Pos) ## !< 0x00000008
  CAN_RF1R_FULL1* = CAN_RF1R_FULL1_Msk
  CAN_RF1R_FOVR1_Pos* = (4)
  CAN_RF1R_FOVR1_Msk* = (0x00000001 shl CAN_RF1R_FOVR1_Pos) ## !< 0x00000010
  CAN_RF1R_FOVR1* = CAN_RF1R_FOVR1_Msk
  CAN_RF1R_RFOM1_Pos* = (5)
  CAN_RF1R_RFOM1_Msk* = (0x00000001 shl CAN_RF1R_RFOM1_Pos) ## !< 0x00000020
  CAN_RF1R_RFOM1* = CAN_RF1R_RFOM1_Msk

## *******************  Bit definition for CAN_IER register  ******************

const
  CAN_IER_TMEIE_Pos* = (0)
  CAN_IER_TMEIE_Msk* = (0x00000001 shl CAN_IER_TMEIE_Pos) ## !< 0x00000001
  CAN_IER_TMEIE* = CAN_IER_TMEIE_Msk
  CAN_IER_FMPIE0_Pos* = (1)
  CAN_IER_FMPIE0_Msk* = (0x00000001 shl CAN_IER_FMPIE0_Pos) ## !< 0x00000002
  CAN_IER_FMPIE0* = CAN_IER_FMPIE0_Msk
  CAN_IER_FFIE0_Pos* = (2)
  CAN_IER_FFIE0_Msk* = (0x00000001 shl CAN_IER_FFIE0_Pos) ## !< 0x00000004
  CAN_IER_FFIE0* = CAN_IER_FFIE0_Msk
  CAN_IER_FOVIE0_Pos* = (3)
  CAN_IER_FOVIE0_Msk* = (0x00000001 shl CAN_IER_FOVIE0_Pos) ## !< 0x00000008
  CAN_IER_FOVIE0* = CAN_IER_FOVIE0_Msk
  CAN_IER_FMPIE1_Pos* = (4)
  CAN_IER_FMPIE1_Msk* = (0x00000001 shl CAN_IER_FMPIE1_Pos) ## !< 0x00000010
  CAN_IER_FMPIE1* = CAN_IER_FMPIE1_Msk
  CAN_IER_FFIE1_Pos* = (5)
  CAN_IER_FFIE1_Msk* = (0x00000001 shl CAN_IER_FFIE1_Pos) ## !< 0x00000020
  CAN_IER_FFIE1* = CAN_IER_FFIE1_Msk
  CAN_IER_FOVIE1_Pos* = (6)
  CAN_IER_FOVIE1_Msk* = (0x00000001 shl CAN_IER_FOVIE1_Pos) ## !< 0x00000040
  CAN_IER_FOVIE1* = CAN_IER_FOVIE1_Msk
  CAN_IER_EWGIE_Pos* = (8)
  CAN_IER_EWGIE_Msk* = (0x00000001 shl CAN_IER_EWGIE_Pos) ## !< 0x00000100
  CAN_IER_EWGIE* = CAN_IER_EWGIE_Msk
  CAN_IER_EPVIE_Pos* = (9)
  CAN_IER_EPVIE_Msk* = (0x00000001 shl CAN_IER_EPVIE_Pos) ## !< 0x00000200
  CAN_IER_EPVIE* = CAN_IER_EPVIE_Msk
  CAN_IER_BOFIE_Pos* = (10)
  CAN_IER_BOFIE_Msk* = (0x00000001 shl CAN_IER_BOFIE_Pos) ## !< 0x00000400
  CAN_IER_BOFIE* = CAN_IER_BOFIE_Msk
  CAN_IER_LECIE_Pos* = (11)
  CAN_IER_LECIE_Msk* = (0x00000001 shl CAN_IER_LECIE_Pos) ## !< 0x00000800
  CAN_IER_LECIE* = CAN_IER_LECIE_Msk
  CAN_IER_ERRIE_Pos* = (15)
  CAN_IER_ERRIE_Msk* = (0x00000001 shl CAN_IER_ERRIE_Pos) ## !< 0x00008000
  CAN_IER_ERRIE* = CAN_IER_ERRIE_Msk
  CAN_IER_WKUIE_Pos* = (16)
  CAN_IER_WKUIE_Msk* = (0x00000001 shl CAN_IER_WKUIE_Pos) ## !< 0x00010000
  CAN_IER_WKUIE* = CAN_IER_WKUIE_Msk
  CAN_IER_SLKIE_Pos* = (17)
  CAN_IER_SLKIE_Msk* = (0x00000001 shl CAN_IER_SLKIE_Pos) ## !< 0x00020000
  CAN_IER_SLKIE* = CAN_IER_SLKIE_Msk

## *******************  Bit definition for CAN_ESR register  ******************

const
  CAN_ESR_EWGF_Pos* = (0)
  CAN_ESR_EWGF_Msk* = (0x00000001 shl CAN_ESR_EWGF_Pos) ## !< 0x00000001
  CAN_ESR_EWGF* = CAN_ESR_EWGF_Msk
  CAN_ESR_EPVF_Pos* = (1)
  CAN_ESR_EPVF_Msk* = (0x00000001 shl CAN_ESR_EPVF_Pos) ## !< 0x00000002
  CAN_ESR_EPVF* = CAN_ESR_EPVF_Msk
  CAN_ESR_BOFF_Pos* = (2)
  CAN_ESR_BOFF_Msk* = (0x00000001 shl CAN_ESR_BOFF_Pos) ## !< 0x00000004
  CAN_ESR_BOFF* = CAN_ESR_BOFF_Msk
  CAN_ESR_LEC_Pos* = (4)
  CAN_ESR_LEC_Msk* = (0x00000007 shl CAN_ESR_LEC_Pos) ## !< 0x00000070
  CAN_ESR_LEC* = CAN_ESR_LEC_Msk
  CAN_ESR_LEC_Bit0* = (0x00000001 shl CAN_ESR_LEC_Pos) ## !< 0x00000010
  CAN_ESR_LEC_Bit1* = (0x00000002 shl CAN_ESR_LEC_Pos) ## !< 0x00000020
  CAN_ESR_LEC_Bit2* = (0x00000004 shl CAN_ESR_LEC_Pos) ## !< 0x00000040
  CAN_ESR_TEC_Pos* = (16)
  CAN_ESR_TEC_Msk* = (0x000000FF shl CAN_ESR_TEC_Pos) ## !< 0x00FF0000
  CAN_ESR_TEC* = CAN_ESR_TEC_Msk
  CAN_ESR_REC_Pos* = (24)
  CAN_ESR_REC_Msk* = (0x000000FF shl CAN_ESR_REC_Pos) ## !< 0xFF000000
  CAN_ESR_REC* = CAN_ESR_REC_Msk

## ******************  Bit definition for CAN_BTR register  *******************

const
  CAN_BTR_BRP_Pos* = (0)
  CAN_BTR_BRP_Msk* = (0x000003FF shl CAN_BTR_BRP_Pos) ## !< 0x000003FF
  CAN_BTR_BRP* = CAN_BTR_BRP_Msk
  CAN_BTR_TS1_Pos* = (16)
  CAN_BTR_TS1_Msk* = (0x0000000F shl CAN_BTR_TS1_Pos) ## !< 0x000F0000
  CAN_BTR_TS1* = CAN_BTR_TS1_Msk
  CAN_BTR_TS1_Bit0* = (0x00000001 shl CAN_BTR_TS1_Pos) ## !< 0x00010000
  CAN_BTR_TS1_Bit1* = (0x00000002 shl CAN_BTR_TS1_Pos) ## !< 0x00020000
  CAN_BTR_TS1_Bit2* = (0x00000004 shl CAN_BTR_TS1_Pos) ## !< 0x00040000
  CAN_BTR_TS1_Bit3* = (0x00000008 shl CAN_BTR_TS1_Pos) ## !< 0x00080000
  CAN_BTR_TS2_Pos* = (20)
  CAN_BTR_TS2_Msk* = (0x00000007 shl CAN_BTR_TS2_Pos) ## !< 0x00700000
  CAN_BTR_TS2* = CAN_BTR_TS2_Msk
  CAN_BTR_TS2_Bit0* = (0x00000001 shl CAN_BTR_TS2_Pos) ## !< 0x00100000
  CAN_BTR_TS2_Bit1* = (0x00000002 shl CAN_BTR_TS2_Pos) ## !< 0x00200000
  CAN_BTR_TS2_Bit2* = (0x00000004 shl CAN_BTR_TS2_Pos) ## !< 0x00400000
  CAN_BTR_SJW_Pos* = (24)
  CAN_BTR_SJW_Msk* = (0x00000003 shl CAN_BTR_SJW_Pos) ## !< 0x03000000
  CAN_BTR_SJW* = CAN_BTR_SJW_Msk
  CAN_BTR_SJW_Bit0* = (0x00000001 shl CAN_BTR_SJW_Pos) ## !< 0x01000000
  CAN_BTR_SJW_Bit1* = (0x00000002 shl CAN_BTR_SJW_Pos) ## !< 0x02000000
  CAN_BTR_LBKM_Pos* = (30)
  CAN_BTR_LBKM_Msk* = (0x00000001 shl CAN_BTR_LBKM_Pos) ## !< 0x40000000
  CAN_BTR_LBKM* = CAN_BTR_LBKM_Msk
  CAN_BTR_SILM_Pos* = (31)
  CAN_BTR_SILM_Msk* = (0x00000001 shl CAN_BTR_SILM_Pos) ## !< 0x80000000
  CAN_BTR_SILM* = CAN_BTR_SILM_Msk

## !<Mailbox registers
## *****************  Bit definition for CAN_TI0R register  *******************

const
  CAN_TI0R_TXRQ_Pos* = (0)
  CAN_TI0R_TXRQ_Msk* = (0x00000001 shl CAN_TI0R_TXRQ_Pos) ## !< 0x00000001
  CAN_TI0R_TXRQ* = CAN_TI0R_TXRQ_Msk
  CAN_TI0R_RTR_Pos* = (1)
  CAN_TI0R_RTR_Msk* = (0x00000001 shl CAN_TI0R_RTR_Pos) ## !< 0x00000002
  CAN_TI0R_RTR* = CAN_TI0R_RTR_Msk
  CAN_TI0R_IDE_Pos* = (2)
  CAN_TI0R_IDE_Msk* = (0x00000001 shl CAN_TI0R_IDE_Pos) ## !< 0x00000004
  CAN_TI0R_IDE* = CAN_TI0R_IDE_Msk
  CAN_TI0R_EXID_Pos* = (3)
  CAN_TI0R_EXID_Msk* = (0x0003FFFF shl CAN_TI0R_EXID_Pos) ## !< 0x001FFFF8
  CAN_TI0R_EXID* = CAN_TI0R_EXID_Msk
  CAN_TI0R_STID_Pos* = (21)
  CAN_TI0R_STID_Msk* = (0x000007FF shl CAN_TI0R_STID_Pos) ## !< 0xFFE00000
  CAN_TI0R_STID* = CAN_TI0R_STID_Msk

## *****************  Bit definition for CAN_TDT0R register  ******************

const
  CAN_TDT0R_DLC_Pos* = (0)
  CAN_TDT0R_DLC_Msk* = (0x0000000F shl CAN_TDT0R_DLC_Pos) ## !< 0x0000000F
  CAN_TDT0R_DLC* = CAN_TDT0R_DLC_Msk
  CAN_TDT0R_TGT_Pos* = (8)
  CAN_TDT0R_TGT_Msk* = (0x00000001 shl CAN_TDT0R_TGT_Pos) ## !< 0x00000100
  CAN_TDT0R_TGT* = CAN_TDT0R_TGT_Msk
  CAN_TDT0R_TIME_Pos* = (16)
  CAN_TDT0R_TIME_Msk* = (0x0000FFFF shl CAN_TDT0R_TIME_Pos) ## !< 0xFFFF0000
  CAN_TDT0R_TIME* = CAN_TDT0R_TIME_Msk

## *****************  Bit definition for CAN_TDL0R register  ******************

const
  CAN_TDL0R_DATA0_Pos* = (0)
  CAN_TDL0R_DATA0_Msk* = (0x000000FF shl CAN_TDL0R_DATA0_Pos) ## !< 0x000000FF
  CAN_TDL0R_DATA0* = CAN_TDL0R_DATA0_Msk
  CAN_TDL0R_DATA1_Pos* = (8)
  CAN_TDL0R_DATA1_Msk* = (0x000000FF shl CAN_TDL0R_DATA1_Pos) ## !< 0x0000FF00
  CAN_TDL0R_DATA1* = CAN_TDL0R_DATA1_Msk
  CAN_TDL0R_DATA2_Pos* = (16)
  CAN_TDL0R_DATA2_Msk* = (0x000000FF shl CAN_TDL0R_DATA2_Pos) ## !< 0x00FF0000
  CAN_TDL0R_DATA2* = CAN_TDL0R_DATA2_Msk
  CAN_TDL0R_DATA3_Pos* = (24)
  CAN_TDL0R_DATA3_Msk* = (0x000000FF shl CAN_TDL0R_DATA3_Pos) ## !< 0xFF000000
  CAN_TDL0R_DATA3* = CAN_TDL0R_DATA3_Msk

## *****************  Bit definition for CAN_TDH0R register  ******************

const
  CAN_TDH0R_DATA4_Pos* = (0)
  CAN_TDH0R_DATA4_Msk* = (0x000000FF shl CAN_TDH0R_DATA4_Pos) ## !< 0x000000FF
  CAN_TDH0R_DATA4* = CAN_TDH0R_DATA4_Msk
  CAN_TDH0R_DATA5_Pos* = (8)
  CAN_TDH0R_DATA5_Msk* = (0x000000FF shl CAN_TDH0R_DATA5_Pos) ## !< 0x0000FF00
  CAN_TDH0R_DATA5* = CAN_TDH0R_DATA5_Msk
  CAN_TDH0R_DATA6_Pos* = (16)
  CAN_TDH0R_DATA6_Msk* = (0x000000FF shl CAN_TDH0R_DATA6_Pos) ## !< 0x00FF0000
  CAN_TDH0R_DATA6* = CAN_TDH0R_DATA6_Msk
  CAN_TDH0R_DATA7_Pos* = (24)
  CAN_TDH0R_DATA7_Msk* = (0x000000FF shl CAN_TDH0R_DATA7_Pos) ## !< 0xFF000000
  CAN_TDH0R_DATA7* = CAN_TDH0R_DATA7_Msk

## ******************  Bit definition for CAN_TI1R register  ******************

const
  CAN_TI1R_TXRQ_Pos* = (0)
  CAN_TI1R_TXRQ_Msk* = (0x00000001 shl CAN_TI1R_TXRQ_Pos) ## !< 0x00000001
  CAN_TI1R_TXRQ* = CAN_TI1R_TXRQ_Msk
  CAN_TI1R_RTR_Pos* = (1)
  CAN_TI1R_RTR_Msk* = (0x00000001 shl CAN_TI1R_RTR_Pos) ## !< 0x00000002
  CAN_TI1R_RTR* = CAN_TI1R_RTR_Msk
  CAN_TI1R_IDE_Pos* = (2)
  CAN_TI1R_IDE_Msk* = (0x00000001 shl CAN_TI1R_IDE_Pos) ## !< 0x00000004
  CAN_TI1R_IDE* = CAN_TI1R_IDE_Msk
  CAN_TI1R_EXID_Pos* = (3)
  CAN_TI1R_EXID_Msk* = (0x0003FFFF shl CAN_TI1R_EXID_Pos) ## !< 0x001FFFF8
  CAN_TI1R_EXID* = CAN_TI1R_EXID_Msk
  CAN_TI1R_STID_Pos* = (21)
  CAN_TI1R_STID_Msk* = (0x000007FF shl CAN_TI1R_STID_Pos) ## !< 0xFFE00000
  CAN_TI1R_STID* = CAN_TI1R_STID_Msk

## ******************  Bit definition for CAN_TDT1R register  *****************

const
  CAN_TDT1R_DLC_Pos* = (0)
  CAN_TDT1R_DLC_Msk* = (0x0000000F shl CAN_TDT1R_DLC_Pos) ## !< 0x0000000F
  CAN_TDT1R_DLC* = CAN_TDT1R_DLC_Msk
  CAN_TDT1R_TGT_Pos* = (8)
  CAN_TDT1R_TGT_Msk* = (0x00000001 shl CAN_TDT1R_TGT_Pos) ## !< 0x00000100
  CAN_TDT1R_TGT* = CAN_TDT1R_TGT_Msk
  CAN_TDT1R_TIME_Pos* = (16)
  CAN_TDT1R_TIME_Msk* = (0x0000FFFF shl CAN_TDT1R_TIME_Pos) ## !< 0xFFFF0000
  CAN_TDT1R_TIME* = CAN_TDT1R_TIME_Msk

## ******************  Bit definition for CAN_TDL1R register  *****************

const
  CAN_TDL1R_DATA0_Pos* = (0)
  CAN_TDL1R_DATA0_Msk* = (0x000000FF shl CAN_TDL1R_DATA0_Pos) ## !< 0x000000FF
  CAN_TDL1R_DATA0* = CAN_TDL1R_DATA0_Msk
  CAN_TDL1R_DATA1_Pos* = (8)
  CAN_TDL1R_DATA1_Msk* = (0x000000FF shl CAN_TDL1R_DATA1_Pos) ## !< 0x0000FF00
  CAN_TDL1R_DATA1* = CAN_TDL1R_DATA1_Msk
  CAN_TDL1R_DATA2_Pos* = (16)
  CAN_TDL1R_DATA2_Msk* = (0x000000FF shl CAN_TDL1R_DATA2_Pos) ## !< 0x00FF0000
  CAN_TDL1R_DATA2* = CAN_TDL1R_DATA2_Msk
  CAN_TDL1R_DATA3_Pos* = (24)
  CAN_TDL1R_DATA3_Msk* = (0x000000FF shl CAN_TDL1R_DATA3_Pos) ## !< 0xFF000000
  CAN_TDL1R_DATA3* = CAN_TDL1R_DATA3_Msk

## ******************  Bit definition for CAN_TDH1R register  *****************

const
  CAN_TDH1R_DATA4_Pos* = (0)
  CAN_TDH1R_DATA4_Msk* = (0x000000FF shl CAN_TDH1R_DATA4_Pos) ## !< 0x000000FF
  CAN_TDH1R_DATA4* = CAN_TDH1R_DATA4_Msk
  CAN_TDH1R_DATA5_Pos* = (8)
  CAN_TDH1R_DATA5_Msk* = (0x000000FF shl CAN_TDH1R_DATA5_Pos) ## !< 0x0000FF00
  CAN_TDH1R_DATA5* = CAN_TDH1R_DATA5_Msk
  CAN_TDH1R_DATA6_Pos* = (16)
  CAN_TDH1R_DATA6_Msk* = (0x000000FF shl CAN_TDH1R_DATA6_Pos) ## !< 0x00FF0000
  CAN_TDH1R_DATA6* = CAN_TDH1R_DATA6_Msk
  CAN_TDH1R_DATA7_Pos* = (24)
  CAN_TDH1R_DATA7_Msk* = (0x000000FF shl CAN_TDH1R_DATA7_Pos) ## !< 0xFF000000
  CAN_TDH1R_DATA7* = CAN_TDH1R_DATA7_Msk

## ******************  Bit definition for CAN_TI2R register  ******************

const
  CAN_TI2R_TXRQ_Pos* = (0)
  CAN_TI2R_TXRQ_Msk* = (0x00000001 shl CAN_TI2R_TXRQ_Pos) ## !< 0x00000001
  CAN_TI2R_TXRQ* = CAN_TI2R_TXRQ_Msk
  CAN_TI2R_RTR_Pos* = (1)
  CAN_TI2R_RTR_Msk* = (0x00000001 shl CAN_TI2R_RTR_Pos) ## !< 0x00000002
  CAN_TI2R_RTR* = CAN_TI2R_RTR_Msk
  CAN_TI2R_IDE_Pos* = (2)
  CAN_TI2R_IDE_Msk* = (0x00000001 shl CAN_TI2R_IDE_Pos) ## !< 0x00000004
  CAN_TI2R_IDE* = CAN_TI2R_IDE_Msk
  CAN_TI2R_EXID_Pos* = (3)
  CAN_TI2R_EXID_Msk* = (0x0003FFFF shl CAN_TI2R_EXID_Pos) ## !< 0x001FFFF8
  CAN_TI2R_EXID* = CAN_TI2R_EXID_Msk
  CAN_TI2R_STID_Pos* = (21)
  CAN_TI2R_STID_Msk* = (0x000007FF shl CAN_TI2R_STID_Pos) ## !< 0xFFE00000
  CAN_TI2R_STID* = CAN_TI2R_STID_Msk

## ******************  Bit definition for CAN_TDT2R register  *****************

const
  CAN_TDT2R_DLC_Pos* = (0)
  CAN_TDT2R_DLC_Msk* = (0x0000000F shl CAN_TDT2R_DLC_Pos) ## !< 0x0000000F
  CAN_TDT2R_DLC* = CAN_TDT2R_DLC_Msk
  CAN_TDT2R_TGT_Pos* = (8)
  CAN_TDT2R_TGT_Msk* = (0x00000001 shl CAN_TDT2R_TGT_Pos) ## !< 0x00000100
  CAN_TDT2R_TGT* = CAN_TDT2R_TGT_Msk
  CAN_TDT2R_TIME_Pos* = (16)
  CAN_TDT2R_TIME_Msk* = (0x0000FFFF shl CAN_TDT2R_TIME_Pos) ## !< 0xFFFF0000
  CAN_TDT2R_TIME* = CAN_TDT2R_TIME_Msk

## ******************  Bit definition for CAN_TDL2R register  *****************

const
  CAN_TDL2R_DATA0_Pos* = (0)
  CAN_TDL2R_DATA0_Msk* = (0x000000FF shl CAN_TDL2R_DATA0_Pos) ## !< 0x000000FF
  CAN_TDL2R_DATA0* = CAN_TDL2R_DATA0_Msk
  CAN_TDL2R_DATA1_Pos* = (8)
  CAN_TDL2R_DATA1_Msk* = (0x000000FF shl CAN_TDL2R_DATA1_Pos) ## !< 0x0000FF00
  CAN_TDL2R_DATA1* = CAN_TDL2R_DATA1_Msk
  CAN_TDL2R_DATA2_Pos* = (16)
  CAN_TDL2R_DATA2_Msk* = (0x000000FF shl CAN_TDL2R_DATA2_Pos) ## !< 0x00FF0000
  CAN_TDL2R_DATA2* = CAN_TDL2R_DATA2_Msk
  CAN_TDL2R_DATA3_Pos* = (24)
  CAN_TDL2R_DATA3_Msk* = (0x000000FF shl CAN_TDL2R_DATA3_Pos) ## !< 0xFF000000
  CAN_TDL2R_DATA3* = CAN_TDL2R_DATA3_Msk

## ******************  Bit definition for CAN_TDH2R register  *****************

const
  CAN_TDH2R_DATA4_Pos* = (0)
  CAN_TDH2R_DATA4_Msk* = (0x000000FF shl CAN_TDH2R_DATA4_Pos) ## !< 0x000000FF
  CAN_TDH2R_DATA4* = CAN_TDH2R_DATA4_Msk
  CAN_TDH2R_DATA5_Pos* = (8)
  CAN_TDH2R_DATA5_Msk* = (0x000000FF shl CAN_TDH2R_DATA5_Pos) ## !< 0x0000FF00
  CAN_TDH2R_DATA5* = CAN_TDH2R_DATA5_Msk
  CAN_TDH2R_DATA6_Pos* = (16)
  CAN_TDH2R_DATA6_Msk* = (0x000000FF shl CAN_TDH2R_DATA6_Pos) ## !< 0x00FF0000
  CAN_TDH2R_DATA6* = CAN_TDH2R_DATA6_Msk
  CAN_TDH2R_DATA7_Pos* = (24)
  CAN_TDH2R_DATA7_Msk* = (0x000000FF shl CAN_TDH2R_DATA7_Pos) ## !< 0xFF000000
  CAN_TDH2R_DATA7* = CAN_TDH2R_DATA7_Msk

## ******************  Bit definition for CAN_RI0R register  ******************

const
  CAN_RI0R_RTR_Pos* = (1)
  CAN_RI0R_RTR_Msk* = (0x00000001 shl CAN_RI0R_RTR_Pos) ## !< 0x00000002
  CAN_RI0R_RTR* = CAN_RI0R_RTR_Msk
  CAN_RI0R_IDE_Pos* = (2)
  CAN_RI0R_IDE_Msk* = (0x00000001 shl CAN_RI0R_IDE_Pos) ## !< 0x00000004
  CAN_RI0R_IDE* = CAN_RI0R_IDE_Msk
  CAN_RI0R_EXID_Pos* = (3)
  CAN_RI0R_EXID_Msk* = (0x0003FFFF shl CAN_RI0R_EXID_Pos) ## !< 0x001FFFF8
  CAN_RI0R_EXID* = CAN_RI0R_EXID_Msk
  CAN_RI0R_STID_Pos* = (21)
  CAN_RI0R_STID_Msk* = (0x000007FF shl CAN_RI0R_STID_Pos) ## !< 0xFFE00000
  CAN_RI0R_STID* = CAN_RI0R_STID_Msk

## ******************  Bit definition for CAN_RDT0R register  *****************

const
  CAN_RDT0R_DLC_Pos* = (0)
  CAN_RDT0R_DLC_Msk* = (0x0000000F shl CAN_RDT0R_DLC_Pos) ## !< 0x0000000F
  CAN_RDT0R_DLC* = CAN_RDT0R_DLC_Msk
  CAN_RDT0R_FMI_Pos* = (8)
  CAN_RDT0R_FMI_Msk* = (0x000000FF shl CAN_RDT0R_FMI_Pos) ## !< 0x0000FF00
  CAN_RDT0R_FMI* = CAN_RDT0R_FMI_Msk
  CAN_RDT0R_TIME_Pos* = (16)
  CAN_RDT0R_TIME_Msk* = (0x0000FFFF shl CAN_RDT0R_TIME_Pos) ## !< 0xFFFF0000
  CAN_RDT0R_TIME* = CAN_RDT0R_TIME_Msk

## ******************  Bit definition for CAN_RDL0R register  *****************

const
  CAN_RDL0R_DATA0_Pos* = (0)
  CAN_RDL0R_DATA0_Msk* = (0x000000FF shl CAN_RDL0R_DATA0_Pos) ## !< 0x000000FF
  CAN_RDL0R_DATA0* = CAN_RDL0R_DATA0_Msk
  CAN_RDL0R_DATA1_Pos* = (8)
  CAN_RDL0R_DATA1_Msk* = (0x000000FF shl CAN_RDL0R_DATA1_Pos) ## !< 0x0000FF00
  CAN_RDL0R_DATA1* = CAN_RDL0R_DATA1_Msk
  CAN_RDL0R_DATA2_Pos* = (16)
  CAN_RDL0R_DATA2_Msk* = (0x000000FF shl CAN_RDL0R_DATA2_Pos) ## !< 0x00FF0000
  CAN_RDL0R_DATA2* = CAN_RDL0R_DATA2_Msk
  CAN_RDL0R_DATA3_Pos* = (24)
  CAN_RDL0R_DATA3_Msk* = (0x000000FF shl CAN_RDL0R_DATA3_Pos) ## !< 0xFF000000
  CAN_RDL0R_DATA3* = CAN_RDL0R_DATA3_Msk

## ******************  Bit definition for CAN_RDH0R register  *****************

const
  CAN_RDH0R_DATA4_Pos* = (0)
  CAN_RDH0R_DATA4_Msk* = (0x000000FF shl CAN_RDH0R_DATA4_Pos) ## !< 0x000000FF
  CAN_RDH0R_DATA4* = CAN_RDH0R_DATA4_Msk
  CAN_RDH0R_DATA5_Pos* = (8)
  CAN_RDH0R_DATA5_Msk* = (0x000000FF shl CAN_RDH0R_DATA5_Pos) ## !< 0x0000FF00
  CAN_RDH0R_DATA5* = CAN_RDH0R_DATA5_Msk
  CAN_RDH0R_DATA6_Pos* = (16)
  CAN_RDH0R_DATA6_Msk* = (0x000000FF shl CAN_RDH0R_DATA6_Pos) ## !< 0x00FF0000
  CAN_RDH0R_DATA6* = CAN_RDH0R_DATA6_Msk
  CAN_RDH0R_DATA7_Pos* = (24)
  CAN_RDH0R_DATA7_Msk* = (0x000000FF shl CAN_RDH0R_DATA7_Pos) ## !< 0xFF000000
  CAN_RDH0R_DATA7* = CAN_RDH0R_DATA7_Msk

## ******************  Bit definition for CAN_RI1R register  ******************

const
  CAN_RI1R_RTR_Pos* = (1)
  CAN_RI1R_RTR_Msk* = (0x00000001 shl CAN_RI1R_RTR_Pos) ## !< 0x00000002
  CAN_RI1R_RTR* = CAN_RI1R_RTR_Msk
  CAN_RI1R_IDE_Pos* = (2)
  CAN_RI1R_IDE_Msk* = (0x00000001 shl CAN_RI1R_IDE_Pos) ## !< 0x00000004
  CAN_RI1R_IDE* = CAN_RI1R_IDE_Msk
  CAN_RI1R_EXID_Pos* = (3)
  CAN_RI1R_EXID_Msk* = (0x0003FFFF shl CAN_RI1R_EXID_Pos) ## !< 0x001FFFF8
  CAN_RI1R_EXID* = CAN_RI1R_EXID_Msk
  CAN_RI1R_STID_Pos* = (21)
  CAN_RI1R_STID_Msk* = (0x000007FF shl CAN_RI1R_STID_Pos) ## !< 0xFFE00000
  CAN_RI1R_STID* = CAN_RI1R_STID_Msk

## ******************  Bit definition for CAN_RDT1R register  *****************

const
  CAN_RDT1R_DLC_Pos* = (0)
  CAN_RDT1R_DLC_Msk* = (0x0000000F shl CAN_RDT1R_DLC_Pos) ## !< 0x0000000F
  CAN_RDT1R_DLC* = CAN_RDT1R_DLC_Msk
  CAN_RDT1R_FMI_Pos* = (8)
  CAN_RDT1R_FMI_Msk* = (0x000000FF shl CAN_RDT1R_FMI_Pos) ## !< 0x0000FF00
  CAN_RDT1R_FMI* = CAN_RDT1R_FMI_Msk
  CAN_RDT1R_TIME_Pos* = (16)
  CAN_RDT1R_TIME_Msk* = (0x0000FFFF shl CAN_RDT1R_TIME_Pos) ## !< 0xFFFF0000
  CAN_RDT1R_TIME* = CAN_RDT1R_TIME_Msk

## ******************  Bit definition for CAN_RDL1R register  *****************

const
  CAN_RDL1R_DATA0_Pos* = (0)
  CAN_RDL1R_DATA0_Msk* = (0x000000FF shl CAN_RDL1R_DATA0_Pos) ## !< 0x000000FF
  CAN_RDL1R_DATA0* = CAN_RDL1R_DATA0_Msk
  CAN_RDL1R_DATA1_Pos* = (8)
  CAN_RDL1R_DATA1_Msk* = (0x000000FF shl CAN_RDL1R_DATA1_Pos) ## !< 0x0000FF00
  CAN_RDL1R_DATA1* = CAN_RDL1R_DATA1_Msk
  CAN_RDL1R_DATA2_Pos* = (16)
  CAN_RDL1R_DATA2_Msk* = (0x000000FF shl CAN_RDL1R_DATA2_Pos) ## !< 0x00FF0000
  CAN_RDL1R_DATA2* = CAN_RDL1R_DATA2_Msk
  CAN_RDL1R_DATA3_Pos* = (24)
  CAN_RDL1R_DATA3_Msk* = (0x000000FF shl CAN_RDL1R_DATA3_Pos) ## !< 0xFF000000
  CAN_RDL1R_DATA3* = CAN_RDL1R_DATA3_Msk

## ******************  Bit definition for CAN_RDH1R register  *****************

const
  CAN_RDH1R_DATA4_Pos* = (0)
  CAN_RDH1R_DATA4_Msk* = (0x000000FF shl CAN_RDH1R_DATA4_Pos) ## !< 0x000000FF
  CAN_RDH1R_DATA4* = CAN_RDH1R_DATA4_Msk
  CAN_RDH1R_DATA5_Pos* = (8)
  CAN_RDH1R_DATA5_Msk* = (0x000000FF shl CAN_RDH1R_DATA5_Pos) ## !< 0x0000FF00
  CAN_RDH1R_DATA5* = CAN_RDH1R_DATA5_Msk
  CAN_RDH1R_DATA6_Pos* = (16)
  CAN_RDH1R_DATA6_Msk* = (0x000000FF shl CAN_RDH1R_DATA6_Pos) ## !< 0x00FF0000
  CAN_RDH1R_DATA6* = CAN_RDH1R_DATA6_Msk
  CAN_RDH1R_DATA7_Pos* = (24)
  CAN_RDH1R_DATA7_Msk* = (0x000000FF shl CAN_RDH1R_DATA7_Pos) ## !< 0xFF000000
  CAN_RDH1R_DATA7* = CAN_RDH1R_DATA7_Msk

## !<CAN filter registers
## ******************  Bit definition for CAN_FMR register  *******************

const
  CAN_FMR_FINIT_Pos* = (0)
  CAN_FMR_FINIT_Msk* = (0x00000001 shl CAN_FMR_FINIT_Pos) ## !< 0x00000001
  CAN_FMR_FINIT* = CAN_FMR_FINIT_Msk
  CAN_FMR_CAN2SB_Pos* = (8)
  CAN_FMR_CAN2SB_Msk* = (0x0000003F shl CAN_FMR_CAN2SB_Pos) ## !< 0x00003F00
  CAN_FMR_CAN2SB* = CAN_FMR_CAN2SB_Msk

## ******************  Bit definition for CAN_FM1R register  ******************

const
  CAN_FM1R_FBM_Pos* = (0)
  CAN_FM1R_FBM_Msk* = (0x0FFFFFFF shl CAN_FM1R_FBM_Pos) ## !< 0x0FFFFFFF
  CAN_FM1R_FBM* = CAN_FM1R_FBM_Msk
  CAN_FM1R_FBM0_Pos* = (0)
  CAN_FM1R_FBM0_Msk* = (0x00000001 shl CAN_FM1R_FBM0_Pos) ## !< 0x00000001
  CAN_FM1R_FBM0* = CAN_FM1R_FBM0_Msk
  CAN_FM1R_FBM1_Pos* = (1)
  CAN_FM1R_FBM1_Msk* = (0x00000001 shl CAN_FM1R_FBM1_Pos) ## !< 0x00000002
  CAN_FM1R_FBM1* = CAN_FM1R_FBM1_Msk
  CAN_FM1R_FBM2_Pos* = (2)
  CAN_FM1R_FBM2_Msk* = (0x00000001 shl CAN_FM1R_FBM2_Pos) ## !< 0x00000004
  CAN_FM1R_FBM2* = CAN_FM1R_FBM2_Msk
  CAN_FM1R_FBM3_Pos* = (3)
  CAN_FM1R_FBM3_Msk* = (0x00000001 shl CAN_FM1R_FBM3_Pos) ## !< 0x00000008
  CAN_FM1R_FBM3* = CAN_FM1R_FBM3_Msk
  CAN_FM1R_FBM4_Pos* = (4)
  CAN_FM1R_FBM4_Msk* = (0x00000001 shl CAN_FM1R_FBM4_Pos) ## !< 0x00000010
  CAN_FM1R_FBM4* = CAN_FM1R_FBM4_Msk
  CAN_FM1R_FBM5_Pos* = (5)
  CAN_FM1R_FBM5_Msk* = (0x00000001 shl CAN_FM1R_FBM5_Pos) ## !< 0x00000020
  CAN_FM1R_FBM5* = CAN_FM1R_FBM5_Msk
  CAN_FM1R_FBM6_Pos* = (6)
  CAN_FM1R_FBM6_Msk* = (0x00000001 shl CAN_FM1R_FBM6_Pos) ## !< 0x00000040
  CAN_FM1R_FBM6* = CAN_FM1R_FBM6_Msk
  CAN_FM1R_FBM7_Pos* = (7)
  CAN_FM1R_FBM7_Msk* = (0x00000001 shl CAN_FM1R_FBM7_Pos) ## !< 0x00000080
  CAN_FM1R_FBM7* = CAN_FM1R_FBM7_Msk
  CAN_FM1R_FBM8_Pos* = (8)
  CAN_FM1R_FBM8_Msk* = (0x00000001 shl CAN_FM1R_FBM8_Pos) ## !< 0x00000100
  CAN_FM1R_FBM8* = CAN_FM1R_FBM8_Msk
  CAN_FM1R_FBM9_Pos* = (9)
  CAN_FM1R_FBM9_Msk* = (0x00000001 shl CAN_FM1R_FBM9_Pos) ## !< 0x00000200
  CAN_FM1R_FBM9* = CAN_FM1R_FBM9_Msk
  CAN_FM1R_FBM10_Pos* = (10)
  CAN_FM1R_FBM10_Msk* = (0x00000001 shl CAN_FM1R_FBM10_Pos) ## !< 0x00000400
  CAN_FM1R_FBM10* = CAN_FM1R_FBM10_Msk
  CAN_FM1R_FBM11_Pos* = (11)
  CAN_FM1R_FBM11_Msk* = (0x00000001 shl CAN_FM1R_FBM11_Pos) ## !< 0x00000800
  CAN_FM1R_FBM11* = CAN_FM1R_FBM11_Msk
  CAN_FM1R_FBM12_Pos* = (12)
  CAN_FM1R_FBM12_Msk* = (0x00000001 shl CAN_FM1R_FBM12_Pos) ## !< 0x00001000
  CAN_FM1R_FBM12* = CAN_FM1R_FBM12_Msk
  CAN_FM1R_FBM13_Pos* = (13)
  CAN_FM1R_FBM13_Msk* = (0x00000001 shl CAN_FM1R_FBM13_Pos) ## !< 0x00002000
  CAN_FM1R_FBM13* = CAN_FM1R_FBM13_Msk
  CAN_FM1R_FBM14_Pos* = (14)
  CAN_FM1R_FBM14_Msk* = (0x00000001 shl CAN_FM1R_FBM14_Pos) ## !< 0x00004000
  CAN_FM1R_FBM14* = CAN_FM1R_FBM14_Msk
  CAN_FM1R_FBM15_Pos* = (15)
  CAN_FM1R_FBM15_Msk* = (0x00000001 shl CAN_FM1R_FBM15_Pos) ## !< 0x00008000
  CAN_FM1R_FBM15* = CAN_FM1R_FBM15_Msk
  CAN_FM1R_FBM16_Pos* = (16)
  CAN_FM1R_FBM16_Msk* = (0x00000001 shl CAN_FM1R_FBM16_Pos) ## !< 0x00010000
  CAN_FM1R_FBM16* = CAN_FM1R_FBM16_Msk
  CAN_FM1R_FBM17_Pos* = (17)
  CAN_FM1R_FBM17_Msk* = (0x00000001 shl CAN_FM1R_FBM17_Pos) ## !< 0x00020000
  CAN_FM1R_FBM17* = CAN_FM1R_FBM17_Msk
  CAN_FM1R_FBM18_Pos* = (18)
  CAN_FM1R_FBM18_Msk* = (0x00000001 shl CAN_FM1R_FBM18_Pos) ## !< 0x00040000
  CAN_FM1R_FBM18* = CAN_FM1R_FBM18_Msk
  CAN_FM1R_FBM19_Pos* = (19)
  CAN_FM1R_FBM19_Msk* = (0x00000001 shl CAN_FM1R_FBM19_Pos) ## !< 0x00080000
  CAN_FM1R_FBM19* = CAN_FM1R_FBM19_Msk
  CAN_FM1R_FBM20_Pos* = (20)
  CAN_FM1R_FBM20_Msk* = (0x00000001 shl CAN_FM1R_FBM20_Pos) ## !< 0x00100000
  CAN_FM1R_FBM20* = CAN_FM1R_FBM20_Msk
  CAN_FM1R_FBM21_Pos* = (21)
  CAN_FM1R_FBM21_Msk* = (0x00000001 shl CAN_FM1R_FBM21_Pos) ## !< 0x00200000
  CAN_FM1R_FBM21* = CAN_FM1R_FBM21_Msk
  CAN_FM1R_FBM22_Pos* = (22)
  CAN_FM1R_FBM22_Msk* = (0x00000001 shl CAN_FM1R_FBM22_Pos) ## !< 0x00400000
  CAN_FM1R_FBM22* = CAN_FM1R_FBM22_Msk
  CAN_FM1R_FBM23_Pos* = (23)
  CAN_FM1R_FBM23_Msk* = (0x00000001 shl CAN_FM1R_FBM23_Pos) ## !< 0x00800000
  CAN_FM1R_FBM23* = CAN_FM1R_FBM23_Msk
  CAN_FM1R_FBM24_Pos* = (24)
  CAN_FM1R_FBM24_Msk* = (0x00000001 shl CAN_FM1R_FBM24_Pos) ## !< 0x01000000
  CAN_FM1R_FBM24* = CAN_FM1R_FBM24_Msk
  CAN_FM1R_FBM25_Pos* = (25)
  CAN_FM1R_FBM25_Msk* = (0x00000001 shl CAN_FM1R_FBM25_Pos) ## !< 0x02000000
  CAN_FM1R_FBM25* = CAN_FM1R_FBM25_Msk
  CAN_FM1R_FBM26_Pos* = (26)
  CAN_FM1R_FBM26_Msk* = (0x00000001 shl CAN_FM1R_FBM26_Pos) ## !< 0x04000000
  CAN_FM1R_FBM26* = CAN_FM1R_FBM26_Msk
  CAN_FM1R_FBM27_Pos* = (27)
  CAN_FM1R_FBM27_Msk* = (0x00000001 shl CAN_FM1R_FBM27_Pos) ## !< 0x08000000
  CAN_FM1R_FBM27* = CAN_FM1R_FBM27_Msk

## ******************  Bit definition for CAN_FS1R register  ******************

const
  CAN_FS1R_FSC_Pos* = (0)
  CAN_FS1R_FSC_Msk* = (0x0FFFFFFF shl CAN_FS1R_FSC_Pos) ## !< 0x0FFFFFFF
  CAN_FS1R_FSC* = CAN_FS1R_FSC_Msk
  CAN_FS1R_FSC0_Pos* = (0)
  CAN_FS1R_FSC0_Msk* = (0x00000001 shl CAN_FS1R_FSC0_Pos) ## !< 0x00000001
  CAN_FS1R_FSC0* = CAN_FS1R_FSC0_Msk
  CAN_FS1R_FSC1_Pos* = (1)
  CAN_FS1R_FSC1_Msk* = (0x00000001 shl CAN_FS1R_FSC1_Pos) ## !< 0x00000002
  CAN_FS1R_FSC1* = CAN_FS1R_FSC1_Msk
  CAN_FS1R_FSC2_Pos* = (2)
  CAN_FS1R_FSC2_Msk* = (0x00000001 shl CAN_FS1R_FSC2_Pos) ## !< 0x00000004
  CAN_FS1R_FSC2* = CAN_FS1R_FSC2_Msk
  CAN_FS1R_FSC3_Pos* = (3)
  CAN_FS1R_FSC3_Msk* = (0x00000001 shl CAN_FS1R_FSC3_Pos) ## !< 0x00000008
  CAN_FS1R_FSC3* = CAN_FS1R_FSC3_Msk
  CAN_FS1R_FSC4_Pos* = (4)
  CAN_FS1R_FSC4_Msk* = (0x00000001 shl CAN_FS1R_FSC4_Pos) ## !< 0x00000010
  CAN_FS1R_FSC4* = CAN_FS1R_FSC4_Msk
  CAN_FS1R_FSC5_Pos* = (5)
  CAN_FS1R_FSC5_Msk* = (0x00000001 shl CAN_FS1R_FSC5_Pos) ## !< 0x00000020
  CAN_FS1R_FSC5* = CAN_FS1R_FSC5_Msk
  CAN_FS1R_FSC6_Pos* = (6)
  CAN_FS1R_FSC6_Msk* = (0x00000001 shl CAN_FS1R_FSC6_Pos) ## !< 0x00000040
  CAN_FS1R_FSC6* = CAN_FS1R_FSC6_Msk
  CAN_FS1R_FSC7_Pos* = (7)
  CAN_FS1R_FSC7_Msk* = (0x00000001 shl CAN_FS1R_FSC7_Pos) ## !< 0x00000080
  CAN_FS1R_FSC7* = CAN_FS1R_FSC7_Msk
  CAN_FS1R_FSC8_Pos* = (8)
  CAN_FS1R_FSC8_Msk* = (0x00000001 shl CAN_FS1R_FSC8_Pos) ## !< 0x00000100
  CAN_FS1R_FSC8* = CAN_FS1R_FSC8_Msk
  CAN_FS1R_FSC9_Pos* = (9)
  CAN_FS1R_FSC9_Msk* = (0x00000001 shl CAN_FS1R_FSC9_Pos) ## !< 0x00000200
  CAN_FS1R_FSC9* = CAN_FS1R_FSC9_Msk
  CAN_FS1R_FSC10_Pos* = (10)
  CAN_FS1R_FSC10_Msk* = (0x00000001 shl CAN_FS1R_FSC10_Pos) ## !< 0x00000400
  CAN_FS1R_FSC10* = CAN_FS1R_FSC10_Msk
  CAN_FS1R_FSC11_Pos* = (11)
  CAN_FS1R_FSC11_Msk* = (0x00000001 shl CAN_FS1R_FSC11_Pos) ## !< 0x00000800
  CAN_FS1R_FSC11* = CAN_FS1R_FSC11_Msk
  CAN_FS1R_FSC12_Pos* = (12)
  CAN_FS1R_FSC12_Msk* = (0x00000001 shl CAN_FS1R_FSC12_Pos) ## !< 0x00001000
  CAN_FS1R_FSC12* = CAN_FS1R_FSC12_Msk
  CAN_FS1R_FSC13_Pos* = (13)
  CAN_FS1R_FSC13_Msk* = (0x00000001 shl CAN_FS1R_FSC13_Pos) ## !< 0x00002000
  CAN_FS1R_FSC13* = CAN_FS1R_FSC13_Msk
  CAN_FS1R_FSC14_Pos* = (14)
  CAN_FS1R_FSC14_Msk* = (0x00000001 shl CAN_FS1R_FSC14_Pos) ## !< 0x00004000
  CAN_FS1R_FSC14* = CAN_FS1R_FSC14_Msk
  CAN_FS1R_FSC15_Pos* = (15)
  CAN_FS1R_FSC15_Msk* = (0x00000001 shl CAN_FS1R_FSC15_Pos) ## !< 0x00008000
  CAN_FS1R_FSC15* = CAN_FS1R_FSC15_Msk
  CAN_FS1R_FSC16_Pos* = (16)
  CAN_FS1R_FSC16_Msk* = (0x00000001 shl CAN_FS1R_FSC16_Pos) ## !< 0x00010000
  CAN_FS1R_FSC16* = CAN_FS1R_FSC16_Msk
  CAN_FS1R_FSC17_Pos* = (17)
  CAN_FS1R_FSC17_Msk* = (0x00000001 shl CAN_FS1R_FSC17_Pos) ## !< 0x00020000
  CAN_FS1R_FSC17* = CAN_FS1R_FSC17_Msk
  CAN_FS1R_FSC18_Pos* = (18)
  CAN_FS1R_FSC18_Msk* = (0x00000001 shl CAN_FS1R_FSC18_Pos) ## !< 0x00040000
  CAN_FS1R_FSC18* = CAN_FS1R_FSC18_Msk
  CAN_FS1R_FSC19_Pos* = (19)
  CAN_FS1R_FSC19_Msk* = (0x00000001 shl CAN_FS1R_FSC19_Pos) ## !< 0x00080000
  CAN_FS1R_FSC19* = CAN_FS1R_FSC19_Msk
  CAN_FS1R_FSC20_Pos* = (20)
  CAN_FS1R_FSC20_Msk* = (0x00000001 shl CAN_FS1R_FSC20_Pos) ## !< 0x00100000
  CAN_FS1R_FSC20* = CAN_FS1R_FSC20_Msk
  CAN_FS1R_FSC21_Pos* = (21)
  CAN_FS1R_FSC21_Msk* = (0x00000001 shl CAN_FS1R_FSC21_Pos) ## !< 0x00200000
  CAN_FS1R_FSC21* = CAN_FS1R_FSC21_Msk
  CAN_FS1R_FSC22_Pos* = (22)
  CAN_FS1R_FSC22_Msk* = (0x00000001 shl CAN_FS1R_FSC22_Pos) ## !< 0x00400000
  CAN_FS1R_FSC22* = CAN_FS1R_FSC22_Msk
  CAN_FS1R_FSC23_Pos* = (23)
  CAN_FS1R_FSC23_Msk* = (0x00000001 shl CAN_FS1R_FSC23_Pos) ## !< 0x00800000
  CAN_FS1R_FSC23* = CAN_FS1R_FSC23_Msk
  CAN_FS1R_FSC24_Pos* = (24)
  CAN_FS1R_FSC24_Msk* = (0x00000001 shl CAN_FS1R_FSC24_Pos) ## !< 0x01000000
  CAN_FS1R_FSC24* = CAN_FS1R_FSC24_Msk
  CAN_FS1R_FSC25_Pos* = (25)
  CAN_FS1R_FSC25_Msk* = (0x00000001 shl CAN_FS1R_FSC25_Pos) ## !< 0x02000000
  CAN_FS1R_FSC25* = CAN_FS1R_FSC25_Msk
  CAN_FS1R_FSC26_Pos* = (26)
  CAN_FS1R_FSC26_Msk* = (0x00000001 shl CAN_FS1R_FSC26_Pos) ## !< 0x04000000
  CAN_FS1R_FSC26* = CAN_FS1R_FSC26_Msk
  CAN_FS1R_FSC27_Pos* = (27)
  CAN_FS1R_FSC27_Msk* = (0x00000001 shl CAN_FS1R_FSC27_Pos) ## !< 0x08000000
  CAN_FS1R_FSC27* = CAN_FS1R_FSC27_Msk

## *****************  Bit definition for CAN_FFA1R register  ******************

const
  CAN_FFA1R_FFA_Pos* = (0)
  CAN_FFA1R_FFA_Msk* = (0x0FFFFFFF shl CAN_FFA1R_FFA_Pos) ## !< 0x0FFFFFFF
  CAN_FFA1R_FFA* = CAN_FFA1R_FFA_Msk
  CAN_FFA1R_FFA0_Pos* = (0)
  CAN_FFA1R_FFA0_Msk* = (0x00000001 shl CAN_FFA1R_FFA0_Pos) ## !< 0x00000001
  CAN_FFA1R_FFA0* = CAN_FFA1R_FFA0_Msk
  CAN_FFA1R_FFA1_Pos* = (1)
  CAN_FFA1R_FFA1_Msk* = (0x00000001 shl CAN_FFA1R_FFA1_Pos) ## !< 0x00000002
  CAN_FFA1R_FFA1* = CAN_FFA1R_FFA1_Msk
  CAN_FFA1R_FFA2_Pos* = (2)
  CAN_FFA1R_FFA2_Msk* = (0x00000001 shl CAN_FFA1R_FFA2_Pos) ## !< 0x00000004
  CAN_FFA1R_FFA2* = CAN_FFA1R_FFA2_Msk
  CAN_FFA1R_FFA3_Pos* = (3)
  CAN_FFA1R_FFA3_Msk* = (0x00000001 shl CAN_FFA1R_FFA3_Pos) ## !< 0x00000008
  CAN_FFA1R_FFA3* = CAN_FFA1R_FFA3_Msk
  CAN_FFA1R_FFA4_Pos* = (4)
  CAN_FFA1R_FFA4_Msk* = (0x00000001 shl CAN_FFA1R_FFA4_Pos) ## !< 0x00000010
  CAN_FFA1R_FFA4* = CAN_FFA1R_FFA4_Msk
  CAN_FFA1R_FFA5_Pos* = (5)
  CAN_FFA1R_FFA5_Msk* = (0x00000001 shl CAN_FFA1R_FFA5_Pos) ## !< 0x00000020
  CAN_FFA1R_FFA5* = CAN_FFA1R_FFA5_Msk
  CAN_FFA1R_FFA6_Pos* = (6)
  CAN_FFA1R_FFA6_Msk* = (0x00000001 shl CAN_FFA1R_FFA6_Pos) ## !< 0x00000040
  CAN_FFA1R_FFA6* = CAN_FFA1R_FFA6_Msk
  CAN_FFA1R_FFA7_Pos* = (7)
  CAN_FFA1R_FFA7_Msk* = (0x00000001 shl CAN_FFA1R_FFA7_Pos) ## !< 0x00000080
  CAN_FFA1R_FFA7* = CAN_FFA1R_FFA7_Msk
  CAN_FFA1R_FFA8_Pos* = (8)
  CAN_FFA1R_FFA8_Msk* = (0x00000001 shl CAN_FFA1R_FFA8_Pos) ## !< 0x00000100
  CAN_FFA1R_FFA8* = CAN_FFA1R_FFA8_Msk
  CAN_FFA1R_FFA9_Pos* = (9)
  CAN_FFA1R_FFA9_Msk* = (0x00000001 shl CAN_FFA1R_FFA9_Pos) ## !< 0x00000200
  CAN_FFA1R_FFA9* = CAN_FFA1R_FFA9_Msk
  CAN_FFA1R_FFA10_Pos* = (10)
  CAN_FFA1R_FFA10_Msk* = (0x00000001 shl CAN_FFA1R_FFA10_Pos) ## !< 0x00000400
  CAN_FFA1R_FFA10* = CAN_FFA1R_FFA10_Msk
  CAN_FFA1R_FFA11_Pos* = (11)
  CAN_FFA1R_FFA11_Msk* = (0x00000001 shl CAN_FFA1R_FFA11_Pos) ## !< 0x00000800
  CAN_FFA1R_FFA11* = CAN_FFA1R_FFA11_Msk
  CAN_FFA1R_FFA12_Pos* = (12)
  CAN_FFA1R_FFA12_Msk* = (0x00000001 shl CAN_FFA1R_FFA12_Pos) ## !< 0x00001000
  CAN_FFA1R_FFA12* = CAN_FFA1R_FFA12_Msk
  CAN_FFA1R_FFA13_Pos* = (13)
  CAN_FFA1R_FFA13_Msk* = (0x00000001 shl CAN_FFA1R_FFA13_Pos) ## !< 0x00002000
  CAN_FFA1R_FFA13* = CAN_FFA1R_FFA13_Msk
  CAN_FFA1R_FFA14_Pos* = (14)
  CAN_FFA1R_FFA14_Msk* = (0x00000001 shl CAN_FFA1R_FFA14_Pos) ## !< 0x00004000
  CAN_FFA1R_FFA14* = CAN_FFA1R_FFA14_Msk
  CAN_FFA1R_FFA15_Pos* = (15)
  CAN_FFA1R_FFA15_Msk* = (0x00000001 shl CAN_FFA1R_FFA15_Pos) ## !< 0x00008000
  CAN_FFA1R_FFA15* = CAN_FFA1R_FFA15_Msk
  CAN_FFA1R_FFA16_Pos* = (16)
  CAN_FFA1R_FFA16_Msk* = (0x00000001 shl CAN_FFA1R_FFA16_Pos) ## !< 0x00010000
  CAN_FFA1R_FFA16* = CAN_FFA1R_FFA16_Msk
  CAN_FFA1R_FFA17_Pos* = (17)
  CAN_FFA1R_FFA17_Msk* = (0x00000001 shl CAN_FFA1R_FFA17_Pos) ## !< 0x00020000
  CAN_FFA1R_FFA17* = CAN_FFA1R_FFA17_Msk
  CAN_FFA1R_FFA18_Pos* = (18)
  CAN_FFA1R_FFA18_Msk* = (0x00000001 shl CAN_FFA1R_FFA18_Pos) ## !< 0x00040000
  CAN_FFA1R_FFA18* = CAN_FFA1R_FFA18_Msk
  CAN_FFA1R_FFA19_Pos* = (19)
  CAN_FFA1R_FFA19_Msk* = (0x00000001 shl CAN_FFA1R_FFA19_Pos) ## !< 0x00080000
  CAN_FFA1R_FFA19* = CAN_FFA1R_FFA19_Msk
  CAN_FFA1R_FFA20_Pos* = (20)
  CAN_FFA1R_FFA20_Msk* = (0x00000001 shl CAN_FFA1R_FFA20_Pos) ## !< 0x00100000
  CAN_FFA1R_FFA20* = CAN_FFA1R_FFA20_Msk
  CAN_FFA1R_FFA21_Pos* = (21)
  CAN_FFA1R_FFA21_Msk* = (0x00000001 shl CAN_FFA1R_FFA21_Pos) ## !< 0x00200000
  CAN_FFA1R_FFA21* = CAN_FFA1R_FFA21_Msk
  CAN_FFA1R_FFA22_Pos* = (22)
  CAN_FFA1R_FFA22_Msk* = (0x00000001 shl CAN_FFA1R_FFA22_Pos) ## !< 0x00400000
  CAN_FFA1R_FFA22* = CAN_FFA1R_FFA22_Msk
  CAN_FFA1R_FFA23_Pos* = (23)
  CAN_FFA1R_FFA23_Msk* = (0x00000001 shl CAN_FFA1R_FFA23_Pos) ## !< 0x00800000
  CAN_FFA1R_FFA23* = CAN_FFA1R_FFA23_Msk
  CAN_FFA1R_FFA24_Pos* = (24)
  CAN_FFA1R_FFA24_Msk* = (0x00000001 shl CAN_FFA1R_FFA24_Pos) ## !< 0x01000000
  CAN_FFA1R_FFA24* = CAN_FFA1R_FFA24_Msk
  CAN_FFA1R_FFA25_Pos* = (25)
  CAN_FFA1R_FFA25_Msk* = (0x00000001 shl CAN_FFA1R_FFA25_Pos) ## !< 0x02000000
  CAN_FFA1R_FFA25* = CAN_FFA1R_FFA25_Msk
  CAN_FFA1R_FFA26_Pos* = (26)
  CAN_FFA1R_FFA26_Msk* = (0x00000001 shl CAN_FFA1R_FFA26_Pos) ## !< 0x04000000
  CAN_FFA1R_FFA26* = CAN_FFA1R_FFA26_Msk
  CAN_FFA1R_FFA27_Pos* = (27)
  CAN_FFA1R_FFA27_Msk* = (0x00000001 shl CAN_FFA1R_FFA27_Pos) ## !< 0x08000000
  CAN_FFA1R_FFA27* = CAN_FFA1R_FFA27_Msk

## ******************  Bit definition for CAN_FA1R register  ******************

const
  CAN_FA1R_FACT_Pos* = (0)
  CAN_FA1R_FACT_Msk* = (0x0FFFFFFF shl CAN_FA1R_FACT_Pos) ## !< 0x0FFFFFFF
  CAN_FA1R_FACT* = CAN_FA1R_FACT_Msk
  CAN_FA1R_FACT0_Pos* = (0)
  CAN_FA1R_FACT0_Msk* = (0x00000001 shl CAN_FA1R_FACT0_Pos) ## !< 0x00000001
  CAN_FA1R_FACT0* = CAN_FA1R_FACT0_Msk
  CAN_FA1R_FACT1_Pos* = (1)
  CAN_FA1R_FACT1_Msk* = (0x00000001 shl CAN_FA1R_FACT1_Pos) ## !< 0x00000002
  CAN_FA1R_FACT1* = CAN_FA1R_FACT1_Msk
  CAN_FA1R_FACT2_Pos* = (2)
  CAN_FA1R_FACT2_Msk* = (0x00000001 shl CAN_FA1R_FACT2_Pos) ## !< 0x00000004
  CAN_FA1R_FACT2* = CAN_FA1R_FACT2_Msk
  CAN_FA1R_FACT3_Pos* = (3)
  CAN_FA1R_FACT3_Msk* = (0x00000001 shl CAN_FA1R_FACT3_Pos) ## !< 0x00000008
  CAN_FA1R_FACT3* = CAN_FA1R_FACT3_Msk
  CAN_FA1R_FACT4_Pos* = (4)
  CAN_FA1R_FACT4_Msk* = (0x00000001 shl CAN_FA1R_FACT4_Pos) ## !< 0x00000010
  CAN_FA1R_FACT4* = CAN_FA1R_FACT4_Msk
  CAN_FA1R_FACT5_Pos* = (5)
  CAN_FA1R_FACT5_Msk* = (0x00000001 shl CAN_FA1R_FACT5_Pos) ## !< 0x00000020
  CAN_FA1R_FACT5* = CAN_FA1R_FACT5_Msk
  CAN_FA1R_FACT6_Pos* = (6)
  CAN_FA1R_FACT6_Msk* = (0x00000001 shl CAN_FA1R_FACT6_Pos) ## !< 0x00000040
  CAN_FA1R_FACT6* = CAN_FA1R_FACT6_Msk
  CAN_FA1R_FACT7_Pos* = (7)
  CAN_FA1R_FACT7_Msk* = (0x00000001 shl CAN_FA1R_FACT7_Pos) ## !< 0x00000080
  CAN_FA1R_FACT7* = CAN_FA1R_FACT7_Msk
  CAN_FA1R_FACT8_Pos* = (8)
  CAN_FA1R_FACT8_Msk* = (0x00000001 shl CAN_FA1R_FACT8_Pos) ## !< 0x00000100
  CAN_FA1R_FACT8* = CAN_FA1R_FACT8_Msk
  CAN_FA1R_FACT9_Pos* = (9)
  CAN_FA1R_FACT9_Msk* = (0x00000001 shl CAN_FA1R_FACT9_Pos) ## !< 0x00000200
  CAN_FA1R_FACT9* = CAN_FA1R_FACT9_Msk
  CAN_FA1R_FACT10_Pos* = (10)
  CAN_FA1R_FACT10_Msk* = (0x00000001 shl CAN_FA1R_FACT10_Pos) ## !< 0x00000400
  CAN_FA1R_FACT10* = CAN_FA1R_FACT10_Msk
  CAN_FA1R_FACT11_Pos* = (11)
  CAN_FA1R_FACT11_Msk* = (0x00000001 shl CAN_FA1R_FACT11_Pos) ## !< 0x00000800
  CAN_FA1R_FACT11* = CAN_FA1R_FACT11_Msk
  CAN_FA1R_FACT12_Pos* = (12)
  CAN_FA1R_FACT12_Msk* = (0x00000001 shl CAN_FA1R_FACT12_Pos) ## !< 0x00001000
  CAN_FA1R_FACT12* = CAN_FA1R_FACT12_Msk
  CAN_FA1R_FACT13_Pos* = (13)
  CAN_FA1R_FACT13_Msk* = (0x00000001 shl CAN_FA1R_FACT13_Pos) ## !< 0x00002000
  CAN_FA1R_FACT13* = CAN_FA1R_FACT13_Msk
  CAN_FA1R_FACT14_Pos* = (14)
  CAN_FA1R_FACT14_Msk* = (0x00000001 shl CAN_FA1R_FACT14_Pos) ## !< 0x00004000
  CAN_FA1R_FACT14* = CAN_FA1R_FACT14_Msk
  CAN_FA1R_FACT15_Pos* = (15)
  CAN_FA1R_FACT15_Msk* = (0x00000001 shl CAN_FA1R_FACT15_Pos) ## !< 0x00008000
  CAN_FA1R_FACT15* = CAN_FA1R_FACT15_Msk
  CAN_FA1R_FACT16_Pos* = (16)
  CAN_FA1R_FACT16_Msk* = (0x00000001 shl CAN_FA1R_FACT16_Pos) ## !< 0x00010000
  CAN_FA1R_FACT16* = CAN_FA1R_FACT16_Msk
  CAN_FA1R_FACT17_Pos* = (17)
  CAN_FA1R_FACT17_Msk* = (0x00000001 shl CAN_FA1R_FACT17_Pos) ## !< 0x00020000
  CAN_FA1R_FACT17* = CAN_FA1R_FACT17_Msk
  CAN_FA1R_FACT18_Pos* = (18)
  CAN_FA1R_FACT18_Msk* = (0x00000001 shl CAN_FA1R_FACT18_Pos) ## !< 0x00040000
  CAN_FA1R_FACT18* = CAN_FA1R_FACT18_Msk
  CAN_FA1R_FACT19_Pos* = (19)
  CAN_FA1R_FACT19_Msk* = (0x00000001 shl CAN_FA1R_FACT19_Pos) ## !< 0x00080000
  CAN_FA1R_FACT19* = CAN_FA1R_FACT19_Msk
  CAN_FA1R_FACT20_Pos* = (20)
  CAN_FA1R_FACT20_Msk* = (0x00000001 shl CAN_FA1R_FACT20_Pos) ## !< 0x00100000
  CAN_FA1R_FACT20* = CAN_FA1R_FACT20_Msk
  CAN_FA1R_FACT21_Pos* = (21)
  CAN_FA1R_FACT21_Msk* = (0x00000001 shl CAN_FA1R_FACT21_Pos) ## !< 0x00200000
  CAN_FA1R_FACT21* = CAN_FA1R_FACT21_Msk
  CAN_FA1R_FACT22_Pos* = (22)
  CAN_FA1R_FACT22_Msk* = (0x00000001 shl CAN_FA1R_FACT22_Pos) ## !< 0x00400000
  CAN_FA1R_FACT22* = CAN_FA1R_FACT22_Msk
  CAN_FA1R_FACT23_Pos* = (23)
  CAN_FA1R_FACT23_Msk* = (0x00000001 shl CAN_FA1R_FACT23_Pos) ## !< 0x00800000
  CAN_FA1R_FACT23* = CAN_FA1R_FACT23_Msk
  CAN_FA1R_FACT24_Pos* = (24)
  CAN_FA1R_FACT24_Msk* = (0x00000001 shl CAN_FA1R_FACT24_Pos) ## !< 0x01000000
  CAN_FA1R_FACT24* = CAN_FA1R_FACT24_Msk
  CAN_FA1R_FACT25_Pos* = (25)
  CAN_FA1R_FACT25_Msk* = (0x00000001 shl CAN_FA1R_FACT25_Pos) ## !< 0x02000000
  CAN_FA1R_FACT25* = CAN_FA1R_FACT25_Msk
  CAN_FA1R_FACT26_Pos* = (26)
  CAN_FA1R_FACT26_Msk* = (0x00000001 shl CAN_FA1R_FACT26_Pos) ## !< 0x04000000
  CAN_FA1R_FACT26* = CAN_FA1R_FACT26_Msk
  CAN_FA1R_FACT27_Pos* = (27)
  CAN_FA1R_FACT27_Msk* = (0x00000001 shl CAN_FA1R_FACT27_Pos) ## !< 0x08000000
  CAN_FA1R_FACT27* = CAN_FA1R_FACT27_Msk

## ******************  Bit definition for CAN_F0R1 register  ******************

const
  CAN_F0R1_FB0_Pos* = (0)
  CAN_F0R1_FB0_Msk* = (0x00000001 shl CAN_F0R1_FB0_Pos) ## !< 0x00000001
  CAN_F0R1_FB0* = CAN_F0R1_FB0_Msk
  CAN_F0R1_FB1_Pos* = (1)
  CAN_F0R1_FB1_Msk* = (0x00000001 shl CAN_F0R1_FB1_Pos) ## !< 0x00000002
  CAN_F0R1_FB1* = CAN_F0R1_FB1_Msk
  CAN_F0R1_FB2_Pos* = (2)
  CAN_F0R1_FB2_Msk* = (0x00000001 shl CAN_F0R1_FB2_Pos) ## !< 0x00000004
  CAN_F0R1_FB2* = CAN_F0R1_FB2_Msk
  CAN_F0R1_FB3_Pos* = (3)
  CAN_F0R1_FB3_Msk* = (0x00000001 shl CAN_F0R1_FB3_Pos) ## !< 0x00000008
  CAN_F0R1_FB3* = CAN_F0R1_FB3_Msk
  CAN_F0R1_FB4_Pos* = (4)
  CAN_F0R1_FB4_Msk* = (0x00000001 shl CAN_F0R1_FB4_Pos) ## !< 0x00000010
  CAN_F0R1_FB4* = CAN_F0R1_FB4_Msk
  CAN_F0R1_FB5_Pos* = (5)
  CAN_F0R1_FB5_Msk* = (0x00000001 shl CAN_F0R1_FB5_Pos) ## !< 0x00000020
  CAN_F0R1_FB5* = CAN_F0R1_FB5_Msk
  CAN_F0R1_FB6_Pos* = (6)
  CAN_F0R1_FB6_Msk* = (0x00000001 shl CAN_F0R1_FB6_Pos) ## !< 0x00000040
  CAN_F0R1_FB6* = CAN_F0R1_FB6_Msk
  CAN_F0R1_FB7_Pos* = (7)
  CAN_F0R1_FB7_Msk* = (0x00000001 shl CAN_F0R1_FB7_Pos) ## !< 0x00000080
  CAN_F0R1_FB7* = CAN_F0R1_FB7_Msk
  CAN_F0R1_FB8_Pos* = (8)
  CAN_F0R1_FB8_Msk* = (0x00000001 shl CAN_F0R1_FB8_Pos) ## !< 0x00000100
  CAN_F0R1_FB8* = CAN_F0R1_FB8_Msk
  CAN_F0R1_FB9_Pos* = (9)
  CAN_F0R1_FB9_Msk* = (0x00000001 shl CAN_F0R1_FB9_Pos) ## !< 0x00000200
  CAN_F0R1_FB9* = CAN_F0R1_FB9_Msk
  CAN_F0R1_FB10_Pos* = (10)
  CAN_F0R1_FB10_Msk* = (0x00000001 shl CAN_F0R1_FB10_Pos) ## !< 0x00000400
  CAN_F0R1_FB10* = CAN_F0R1_FB10_Msk
  CAN_F0R1_FB11_Pos* = (11)
  CAN_F0R1_FB11_Msk* = (0x00000001 shl CAN_F0R1_FB11_Pos) ## !< 0x00000800
  CAN_F0R1_FB11* = CAN_F0R1_FB11_Msk
  CAN_F0R1_FB12_Pos* = (12)
  CAN_F0R1_FB12_Msk* = (0x00000001 shl CAN_F0R1_FB12_Pos) ## !< 0x00001000
  CAN_F0R1_FB12* = CAN_F0R1_FB12_Msk
  CAN_F0R1_FB13_Pos* = (13)
  CAN_F0R1_FB13_Msk* = (0x00000001 shl CAN_F0R1_FB13_Pos) ## !< 0x00002000
  CAN_F0R1_FB13* = CAN_F0R1_FB13_Msk
  CAN_F0R1_FB14_Pos* = (14)
  CAN_F0R1_FB14_Msk* = (0x00000001 shl CAN_F0R1_FB14_Pos) ## !< 0x00004000
  CAN_F0R1_FB14* = CAN_F0R1_FB14_Msk
  CAN_F0R1_FB15_Pos* = (15)
  CAN_F0R1_FB15_Msk* = (0x00000001 shl CAN_F0R1_FB15_Pos) ## !< 0x00008000
  CAN_F0R1_FB15* = CAN_F0R1_FB15_Msk
  CAN_F0R1_FB16_Pos* = (16)
  CAN_F0R1_FB16_Msk* = (0x00000001 shl CAN_F0R1_FB16_Pos) ## !< 0x00010000
  CAN_F0R1_FB16* = CAN_F0R1_FB16_Msk
  CAN_F0R1_FB17_Pos* = (17)
  CAN_F0R1_FB17_Msk* = (0x00000001 shl CAN_F0R1_FB17_Pos) ## !< 0x00020000
  CAN_F0R1_FB17* = CAN_F0R1_FB17_Msk
  CAN_F0R1_FB18_Pos* = (18)
  CAN_F0R1_FB18_Msk* = (0x00000001 shl CAN_F0R1_FB18_Pos) ## !< 0x00040000
  CAN_F0R1_FB18* = CAN_F0R1_FB18_Msk
  CAN_F0R1_FB19_Pos* = (19)
  CAN_F0R1_FB19_Msk* = (0x00000001 shl CAN_F0R1_FB19_Pos) ## !< 0x00080000
  CAN_F0R1_FB19* = CAN_F0R1_FB19_Msk
  CAN_F0R1_FB20_Pos* = (20)
  CAN_F0R1_FB20_Msk* = (0x00000001 shl CAN_F0R1_FB20_Pos) ## !< 0x00100000
  CAN_F0R1_FB20* = CAN_F0R1_FB20_Msk
  CAN_F0R1_FB21_Pos* = (21)
  CAN_F0R1_FB21_Msk* = (0x00000001 shl CAN_F0R1_FB21_Pos) ## !< 0x00200000
  CAN_F0R1_FB21* = CAN_F0R1_FB21_Msk
  CAN_F0R1_FB22_Pos* = (22)
  CAN_F0R1_FB22_Msk* = (0x00000001 shl CAN_F0R1_FB22_Pos) ## !< 0x00400000
  CAN_F0R1_FB22* = CAN_F0R1_FB22_Msk
  CAN_F0R1_FB23_Pos* = (23)
  CAN_F0R1_FB23_Msk* = (0x00000001 shl CAN_F0R1_FB23_Pos) ## !< 0x00800000
  CAN_F0R1_FB23* = CAN_F0R1_FB23_Msk
  CAN_F0R1_FB24_Pos* = (24)
  CAN_F0R1_FB24_Msk* = (0x00000001 shl CAN_F0R1_FB24_Pos) ## !< 0x01000000
  CAN_F0R1_FB24* = CAN_F0R1_FB24_Msk
  CAN_F0R1_FB25_Pos* = (25)
  CAN_F0R1_FB25_Msk* = (0x00000001 shl CAN_F0R1_FB25_Pos) ## !< 0x02000000
  CAN_F0R1_FB25* = CAN_F0R1_FB25_Msk
  CAN_F0R1_FB26_Pos* = (26)
  CAN_F0R1_FB26_Msk* = (0x00000001 shl CAN_F0R1_FB26_Pos) ## !< 0x04000000
  CAN_F0R1_FB26* = CAN_F0R1_FB26_Msk
  CAN_F0R1_FB27_Pos* = (27)
  CAN_F0R1_FB27_Msk* = (0x00000001 shl CAN_F0R1_FB27_Pos) ## !< 0x08000000
  CAN_F0R1_FB27* = CAN_F0R1_FB27_Msk
  CAN_F0R1_FB28_Pos* = (28)
  CAN_F0R1_FB28_Msk* = (0x00000001 shl CAN_F0R1_FB28_Pos) ## !< 0x10000000
  CAN_F0R1_FB28* = CAN_F0R1_FB28_Msk
  CAN_F0R1_FB29_Pos* = (29)
  CAN_F0R1_FB29_Msk* = (0x00000001 shl CAN_F0R1_FB29_Pos) ## !< 0x20000000
  CAN_F0R1_FB29* = CAN_F0R1_FB29_Msk
  CAN_F0R1_FB30_Pos* = (30)
  CAN_F0R1_FB30_Msk* = (0x00000001 shl CAN_F0R1_FB30_Pos) ## !< 0x40000000
  CAN_F0R1_FB30* = CAN_F0R1_FB30_Msk
  CAN_F0R1_FB31_Pos* = (31)
  CAN_F0R1_FB31_Msk* = (0x00000001 shl CAN_F0R1_FB31_Pos) ## !< 0x80000000
  CAN_F0R1_FB31* = CAN_F0R1_FB31_Msk

## ******************  Bit definition for CAN_F1R1 register  ******************

const
  CAN_F1R1_FB0_Pos* = (0)
  CAN_F1R1_FB0_Msk* = (0x00000001 shl CAN_F1R1_FB0_Pos) ## !< 0x00000001
  CAN_F1R1_FB0* = CAN_F1R1_FB0_Msk
  CAN_F1R1_FB1_Pos* = (1)
  CAN_F1R1_FB1_Msk* = (0x00000001 shl CAN_F1R1_FB1_Pos) ## !< 0x00000002
  CAN_F1R1_FB1* = CAN_F1R1_FB1_Msk
  CAN_F1R1_FB2_Pos* = (2)
  CAN_F1R1_FB2_Msk* = (0x00000001 shl CAN_F1R1_FB2_Pos) ## !< 0x00000004
  CAN_F1R1_FB2* = CAN_F1R1_FB2_Msk
  CAN_F1R1_FB3_Pos* = (3)
  CAN_F1R1_FB3_Msk* = (0x00000001 shl CAN_F1R1_FB3_Pos) ## !< 0x00000008
  CAN_F1R1_FB3* = CAN_F1R1_FB3_Msk
  CAN_F1R1_FB4_Pos* = (4)
  CAN_F1R1_FB4_Msk* = (0x00000001 shl CAN_F1R1_FB4_Pos) ## !< 0x00000010
  CAN_F1R1_FB4* = CAN_F1R1_FB4_Msk
  CAN_F1R1_FB5_Pos* = (5)
  CAN_F1R1_FB5_Msk* = (0x00000001 shl CAN_F1R1_FB5_Pos) ## !< 0x00000020
  CAN_F1R1_FB5* = CAN_F1R1_FB5_Msk
  CAN_F1R1_FB6_Pos* = (6)
  CAN_F1R1_FB6_Msk* = (0x00000001 shl CAN_F1R1_FB6_Pos) ## !< 0x00000040
  CAN_F1R1_FB6* = CAN_F1R1_FB6_Msk
  CAN_F1R1_FB7_Pos* = (7)
  CAN_F1R1_FB7_Msk* = (0x00000001 shl CAN_F1R1_FB7_Pos) ## !< 0x00000080
  CAN_F1R1_FB7* = CAN_F1R1_FB7_Msk
  CAN_F1R1_FB8_Pos* = (8)
  CAN_F1R1_FB8_Msk* = (0x00000001 shl CAN_F1R1_FB8_Pos) ## !< 0x00000100
  CAN_F1R1_FB8* = CAN_F1R1_FB8_Msk
  CAN_F1R1_FB9_Pos* = (9)
  CAN_F1R1_FB9_Msk* = (0x00000001 shl CAN_F1R1_FB9_Pos) ## !< 0x00000200
  CAN_F1R1_FB9* = CAN_F1R1_FB9_Msk
  CAN_F1R1_FB10_Pos* = (10)
  CAN_F1R1_FB10_Msk* = (0x00000001 shl CAN_F1R1_FB10_Pos) ## !< 0x00000400
  CAN_F1R1_FB10* = CAN_F1R1_FB10_Msk
  CAN_F1R1_FB11_Pos* = (11)
  CAN_F1R1_FB11_Msk* = (0x00000001 shl CAN_F1R1_FB11_Pos) ## !< 0x00000800
  CAN_F1R1_FB11* = CAN_F1R1_FB11_Msk
  CAN_F1R1_FB12_Pos* = (12)
  CAN_F1R1_FB12_Msk* = (0x00000001 shl CAN_F1R1_FB12_Pos) ## !< 0x00001000
  CAN_F1R1_FB12* = CAN_F1R1_FB12_Msk
  CAN_F1R1_FB13_Pos* = (13)
  CAN_F1R1_FB13_Msk* = (0x00000001 shl CAN_F1R1_FB13_Pos) ## !< 0x00002000
  CAN_F1R1_FB13* = CAN_F1R1_FB13_Msk
  CAN_F1R1_FB14_Pos* = (14)
  CAN_F1R1_FB14_Msk* = (0x00000001 shl CAN_F1R1_FB14_Pos) ## !< 0x00004000
  CAN_F1R1_FB14* = CAN_F1R1_FB14_Msk
  CAN_F1R1_FB15_Pos* = (15)
  CAN_F1R1_FB15_Msk* = (0x00000001 shl CAN_F1R1_FB15_Pos) ## !< 0x00008000
  CAN_F1R1_FB15* = CAN_F1R1_FB15_Msk
  CAN_F1R1_FB16_Pos* = (16)
  CAN_F1R1_FB16_Msk* = (0x00000001 shl CAN_F1R1_FB16_Pos) ## !< 0x00010000
  CAN_F1R1_FB16* = CAN_F1R1_FB16_Msk
  CAN_F1R1_FB17_Pos* = (17)
  CAN_F1R1_FB17_Msk* = (0x00000001 shl CAN_F1R1_FB17_Pos) ## !< 0x00020000
  CAN_F1R1_FB17* = CAN_F1R1_FB17_Msk
  CAN_F1R1_FB18_Pos* = (18)
  CAN_F1R1_FB18_Msk* = (0x00000001 shl CAN_F1R1_FB18_Pos) ## !< 0x00040000
  CAN_F1R1_FB18* = CAN_F1R1_FB18_Msk
  CAN_F1R1_FB19_Pos* = (19)
  CAN_F1R1_FB19_Msk* = (0x00000001 shl CAN_F1R1_FB19_Pos) ## !< 0x00080000
  CAN_F1R1_FB19* = CAN_F1R1_FB19_Msk
  CAN_F1R1_FB20_Pos* = (20)
  CAN_F1R1_FB20_Msk* = (0x00000001 shl CAN_F1R1_FB20_Pos) ## !< 0x00100000
  CAN_F1R1_FB20* = CAN_F1R1_FB20_Msk
  CAN_F1R1_FB21_Pos* = (21)
  CAN_F1R1_FB21_Msk* = (0x00000001 shl CAN_F1R1_FB21_Pos) ## !< 0x00200000
  CAN_F1R1_FB21* = CAN_F1R1_FB21_Msk
  CAN_F1R1_FB22_Pos* = (22)
  CAN_F1R1_FB22_Msk* = (0x00000001 shl CAN_F1R1_FB22_Pos) ## !< 0x00400000
  CAN_F1R1_FB22* = CAN_F1R1_FB22_Msk
  CAN_F1R1_FB23_Pos* = (23)
  CAN_F1R1_FB23_Msk* = (0x00000001 shl CAN_F1R1_FB23_Pos) ## !< 0x00800000
  CAN_F1R1_FB23* = CAN_F1R1_FB23_Msk
  CAN_F1R1_FB24_Pos* = (24)
  CAN_F1R1_FB24_Msk* = (0x00000001 shl CAN_F1R1_FB24_Pos) ## !< 0x01000000
  CAN_F1R1_FB24* = CAN_F1R1_FB24_Msk
  CAN_F1R1_FB25_Pos* = (25)
  CAN_F1R1_FB25_Msk* = (0x00000001 shl CAN_F1R1_FB25_Pos) ## !< 0x02000000
  CAN_F1R1_FB25* = CAN_F1R1_FB25_Msk
  CAN_F1R1_FB26_Pos* = (26)
  CAN_F1R1_FB26_Msk* = (0x00000001 shl CAN_F1R1_FB26_Pos) ## !< 0x04000000
  CAN_F1R1_FB26* = CAN_F1R1_FB26_Msk
  CAN_F1R1_FB27_Pos* = (27)
  CAN_F1R1_FB27_Msk* = (0x00000001 shl CAN_F1R1_FB27_Pos) ## !< 0x08000000
  CAN_F1R1_FB27* = CAN_F1R1_FB27_Msk
  CAN_F1R1_FB28_Pos* = (28)
  CAN_F1R1_FB28_Msk* = (0x00000001 shl CAN_F1R1_FB28_Pos) ## !< 0x10000000
  CAN_F1R1_FB28* = CAN_F1R1_FB28_Msk
  CAN_F1R1_FB29_Pos* = (29)
  CAN_F1R1_FB29_Msk* = (0x00000001 shl CAN_F1R1_FB29_Pos) ## !< 0x20000000
  CAN_F1R1_FB29* = CAN_F1R1_FB29_Msk
  CAN_F1R1_FB30_Pos* = (30)
  CAN_F1R1_FB30_Msk* = (0x00000001 shl CAN_F1R1_FB30_Pos) ## !< 0x40000000
  CAN_F1R1_FB30* = CAN_F1R1_FB30_Msk
  CAN_F1R1_FB31_Pos* = (31)
  CAN_F1R1_FB31_Msk* = (0x00000001 shl CAN_F1R1_FB31_Pos) ## !< 0x80000000
  CAN_F1R1_FB31* = CAN_F1R1_FB31_Msk

## ******************  Bit definition for CAN_F2R1 register  ******************

const
  CAN_F2R1_FB0_Pos* = (0)
  CAN_F2R1_FB0_Msk* = (0x00000001 shl CAN_F2R1_FB0_Pos) ## !< 0x00000001
  CAN_F2R1_FB0* = CAN_F2R1_FB0_Msk
  CAN_F2R1_FB1_Pos* = (1)
  CAN_F2R1_FB1_Msk* = (0x00000001 shl CAN_F2R1_FB1_Pos) ## !< 0x00000002
  CAN_F2R1_FB1* = CAN_F2R1_FB1_Msk
  CAN_F2R1_FB2_Pos* = (2)
  CAN_F2R1_FB2_Msk* = (0x00000001 shl CAN_F2R1_FB2_Pos) ## !< 0x00000004
  CAN_F2R1_FB2* = CAN_F2R1_FB2_Msk
  CAN_F2R1_FB3_Pos* = (3)
  CAN_F2R1_FB3_Msk* = (0x00000001 shl CAN_F2R1_FB3_Pos) ## !< 0x00000008
  CAN_F2R1_FB3* = CAN_F2R1_FB3_Msk
  CAN_F2R1_FB4_Pos* = (4)
  CAN_F2R1_FB4_Msk* = (0x00000001 shl CAN_F2R1_FB4_Pos) ## !< 0x00000010
  CAN_F2R1_FB4* = CAN_F2R1_FB4_Msk
  CAN_F2R1_FB5_Pos* = (5)
  CAN_F2R1_FB5_Msk* = (0x00000001 shl CAN_F2R1_FB5_Pos) ## !< 0x00000020
  CAN_F2R1_FB5* = CAN_F2R1_FB5_Msk
  CAN_F2R1_FB6_Pos* = (6)
  CAN_F2R1_FB6_Msk* = (0x00000001 shl CAN_F2R1_FB6_Pos) ## !< 0x00000040
  CAN_F2R1_FB6* = CAN_F2R1_FB6_Msk
  CAN_F2R1_FB7_Pos* = (7)
  CAN_F2R1_FB7_Msk* = (0x00000001 shl CAN_F2R1_FB7_Pos) ## !< 0x00000080
  CAN_F2R1_FB7* = CAN_F2R1_FB7_Msk
  CAN_F2R1_FB8_Pos* = (8)
  CAN_F2R1_FB8_Msk* = (0x00000001 shl CAN_F2R1_FB8_Pos) ## !< 0x00000100
  CAN_F2R1_FB8* = CAN_F2R1_FB8_Msk
  CAN_F2R1_FB9_Pos* = (9)
  CAN_F2R1_FB9_Msk* = (0x00000001 shl CAN_F2R1_FB9_Pos) ## !< 0x00000200
  CAN_F2R1_FB9* = CAN_F2R1_FB9_Msk
  CAN_F2R1_FB10_Pos* = (10)
  CAN_F2R1_FB10_Msk* = (0x00000001 shl CAN_F2R1_FB10_Pos) ## !< 0x00000400
  CAN_F2R1_FB10* = CAN_F2R1_FB10_Msk
  CAN_F2R1_FB11_Pos* = (11)
  CAN_F2R1_FB11_Msk* = (0x00000001 shl CAN_F2R1_FB11_Pos) ## !< 0x00000800
  CAN_F2R1_FB11* = CAN_F2R1_FB11_Msk
  CAN_F2R1_FB12_Pos* = (12)
  CAN_F2R1_FB12_Msk* = (0x00000001 shl CAN_F2R1_FB12_Pos) ## !< 0x00001000
  CAN_F2R1_FB12* = CAN_F2R1_FB12_Msk
  CAN_F2R1_FB13_Pos* = (13)
  CAN_F2R1_FB13_Msk* = (0x00000001 shl CAN_F2R1_FB13_Pos) ## !< 0x00002000
  CAN_F2R1_FB13* = CAN_F2R1_FB13_Msk
  CAN_F2R1_FB14_Pos* = (14)
  CAN_F2R1_FB14_Msk* = (0x00000001 shl CAN_F2R1_FB14_Pos) ## !< 0x00004000
  CAN_F2R1_FB14* = CAN_F2R1_FB14_Msk
  CAN_F2R1_FB15_Pos* = (15)
  CAN_F2R1_FB15_Msk* = (0x00000001 shl CAN_F2R1_FB15_Pos) ## !< 0x00008000
  CAN_F2R1_FB15* = CAN_F2R1_FB15_Msk
  CAN_F2R1_FB16_Pos* = (16)
  CAN_F2R1_FB16_Msk* = (0x00000001 shl CAN_F2R1_FB16_Pos) ## !< 0x00010000
  CAN_F2R1_FB16* = CAN_F2R1_FB16_Msk
  CAN_F2R1_FB17_Pos* = (17)
  CAN_F2R1_FB17_Msk* = (0x00000001 shl CAN_F2R1_FB17_Pos) ## !< 0x00020000
  CAN_F2R1_FB17* = CAN_F2R1_FB17_Msk
  CAN_F2R1_FB18_Pos* = (18)
  CAN_F2R1_FB18_Msk* = (0x00000001 shl CAN_F2R1_FB18_Pos) ## !< 0x00040000
  CAN_F2R1_FB18* = CAN_F2R1_FB18_Msk
  CAN_F2R1_FB19_Pos* = (19)
  CAN_F2R1_FB19_Msk* = (0x00000001 shl CAN_F2R1_FB19_Pos) ## !< 0x00080000
  CAN_F2R1_FB19* = CAN_F2R1_FB19_Msk
  CAN_F2R1_FB20_Pos* = (20)
  CAN_F2R1_FB20_Msk* = (0x00000001 shl CAN_F2R1_FB20_Pos) ## !< 0x00100000
  CAN_F2R1_FB20* = CAN_F2R1_FB20_Msk
  CAN_F2R1_FB21_Pos* = (21)
  CAN_F2R1_FB21_Msk* = (0x00000001 shl CAN_F2R1_FB21_Pos) ## !< 0x00200000
  CAN_F2R1_FB21* = CAN_F2R1_FB21_Msk
  CAN_F2R1_FB22_Pos* = (22)
  CAN_F2R1_FB22_Msk* = (0x00000001 shl CAN_F2R1_FB22_Pos) ## !< 0x00400000
  CAN_F2R1_FB22* = CAN_F2R1_FB22_Msk
  CAN_F2R1_FB23_Pos* = (23)
  CAN_F2R1_FB23_Msk* = (0x00000001 shl CAN_F2R1_FB23_Pos) ## !< 0x00800000
  CAN_F2R1_FB23* = CAN_F2R1_FB23_Msk
  CAN_F2R1_FB24_Pos* = (24)
  CAN_F2R1_FB24_Msk* = (0x00000001 shl CAN_F2R1_FB24_Pos) ## !< 0x01000000
  CAN_F2R1_FB24* = CAN_F2R1_FB24_Msk
  CAN_F2R1_FB25_Pos* = (25)
  CAN_F2R1_FB25_Msk* = (0x00000001 shl CAN_F2R1_FB25_Pos) ## !< 0x02000000
  CAN_F2R1_FB25* = CAN_F2R1_FB25_Msk
  CAN_F2R1_FB26_Pos* = (26)
  CAN_F2R1_FB26_Msk* = (0x00000001 shl CAN_F2R1_FB26_Pos) ## !< 0x04000000
  CAN_F2R1_FB26* = CAN_F2R1_FB26_Msk
  CAN_F2R1_FB27_Pos* = (27)
  CAN_F2R1_FB27_Msk* = (0x00000001 shl CAN_F2R1_FB27_Pos) ## !< 0x08000000
  CAN_F2R1_FB27* = CAN_F2R1_FB27_Msk
  CAN_F2R1_FB28_Pos* = (28)
  CAN_F2R1_FB28_Msk* = (0x00000001 shl CAN_F2R1_FB28_Pos) ## !< 0x10000000
  CAN_F2R1_FB28* = CAN_F2R1_FB28_Msk
  CAN_F2R1_FB29_Pos* = (29)
  CAN_F2R1_FB29_Msk* = (0x00000001 shl CAN_F2R1_FB29_Pos) ## !< 0x20000000
  CAN_F2R1_FB29* = CAN_F2R1_FB29_Msk
  CAN_F2R1_FB30_Pos* = (30)
  CAN_F2R1_FB30_Msk* = (0x00000001 shl CAN_F2R1_FB30_Pos) ## !< 0x40000000
  CAN_F2R1_FB30* = CAN_F2R1_FB30_Msk
  CAN_F2R1_FB31_Pos* = (31)
  CAN_F2R1_FB31_Msk* = (0x00000001 shl CAN_F2R1_FB31_Pos) ## !< 0x80000000
  CAN_F2R1_FB31* = CAN_F2R1_FB31_Msk

## ******************  Bit definition for CAN_F3R1 register  ******************

const
  CAN_F3R1_FB0_Pos* = (0)
  CAN_F3R1_FB0_Msk* = (0x00000001 shl CAN_F3R1_FB0_Pos) ## !< 0x00000001
  CAN_F3R1_FB0* = CAN_F3R1_FB0_Msk
  CAN_F3R1_FB1_Pos* = (1)
  CAN_F3R1_FB1_Msk* = (0x00000001 shl CAN_F3R1_FB1_Pos) ## !< 0x00000002
  CAN_F3R1_FB1* = CAN_F3R1_FB1_Msk
  CAN_F3R1_FB2_Pos* = (2)
  CAN_F3R1_FB2_Msk* = (0x00000001 shl CAN_F3R1_FB2_Pos) ## !< 0x00000004
  CAN_F3R1_FB2* = CAN_F3R1_FB2_Msk
  CAN_F3R1_FB3_Pos* = (3)
  CAN_F3R1_FB3_Msk* = (0x00000001 shl CAN_F3R1_FB3_Pos) ## !< 0x00000008
  CAN_F3R1_FB3* = CAN_F3R1_FB3_Msk
  CAN_F3R1_FB4_Pos* = (4)
  CAN_F3R1_FB4_Msk* = (0x00000001 shl CAN_F3R1_FB4_Pos) ## !< 0x00000010
  CAN_F3R1_FB4* = CAN_F3R1_FB4_Msk
  CAN_F3R1_FB5_Pos* = (5)
  CAN_F3R1_FB5_Msk* = (0x00000001 shl CAN_F3R1_FB5_Pos) ## !< 0x00000020
  CAN_F3R1_FB5* = CAN_F3R1_FB5_Msk
  CAN_F3R1_FB6_Pos* = (6)
  CAN_F3R1_FB6_Msk* = (0x00000001 shl CAN_F3R1_FB6_Pos) ## !< 0x00000040
  CAN_F3R1_FB6* = CAN_F3R1_FB6_Msk
  CAN_F3R1_FB7_Pos* = (7)
  CAN_F3R1_FB7_Msk* = (0x00000001 shl CAN_F3R1_FB7_Pos) ## !< 0x00000080
  CAN_F3R1_FB7* = CAN_F3R1_FB7_Msk
  CAN_F3R1_FB8_Pos* = (8)
  CAN_F3R1_FB8_Msk* = (0x00000001 shl CAN_F3R1_FB8_Pos) ## !< 0x00000100
  CAN_F3R1_FB8* = CAN_F3R1_FB8_Msk
  CAN_F3R1_FB9_Pos* = (9)
  CAN_F3R1_FB9_Msk* = (0x00000001 shl CAN_F3R1_FB9_Pos) ## !< 0x00000200
  CAN_F3R1_FB9* = CAN_F3R1_FB9_Msk
  CAN_F3R1_FB10_Pos* = (10)
  CAN_F3R1_FB10_Msk* = (0x00000001 shl CAN_F3R1_FB10_Pos) ## !< 0x00000400
  CAN_F3R1_FB10* = CAN_F3R1_FB10_Msk
  CAN_F3R1_FB11_Pos* = (11)
  CAN_F3R1_FB11_Msk* = (0x00000001 shl CAN_F3R1_FB11_Pos) ## !< 0x00000800
  CAN_F3R1_FB11* = CAN_F3R1_FB11_Msk
  CAN_F3R1_FB12_Pos* = (12)
  CAN_F3R1_FB12_Msk* = (0x00000001 shl CAN_F3R1_FB12_Pos) ## !< 0x00001000
  CAN_F3R1_FB12* = CAN_F3R1_FB12_Msk
  CAN_F3R1_FB13_Pos* = (13)
  CAN_F3R1_FB13_Msk* = (0x00000001 shl CAN_F3R1_FB13_Pos) ## !< 0x00002000
  CAN_F3R1_FB13* = CAN_F3R1_FB13_Msk
  CAN_F3R1_FB14_Pos* = (14)
  CAN_F3R1_FB14_Msk* = (0x00000001 shl CAN_F3R1_FB14_Pos) ## !< 0x00004000
  CAN_F3R1_FB14* = CAN_F3R1_FB14_Msk
  CAN_F3R1_FB15_Pos* = (15)
  CAN_F3R1_FB15_Msk* = (0x00000001 shl CAN_F3R1_FB15_Pos) ## !< 0x00008000
  CAN_F3R1_FB15* = CAN_F3R1_FB15_Msk
  CAN_F3R1_FB16_Pos* = (16)
  CAN_F3R1_FB16_Msk* = (0x00000001 shl CAN_F3R1_FB16_Pos) ## !< 0x00010000
  CAN_F3R1_FB16* = CAN_F3R1_FB16_Msk
  CAN_F3R1_FB17_Pos* = (17)
  CAN_F3R1_FB17_Msk* = (0x00000001 shl CAN_F3R1_FB17_Pos) ## !< 0x00020000
  CAN_F3R1_FB17* = CAN_F3R1_FB17_Msk
  CAN_F3R1_FB18_Pos* = (18)
  CAN_F3R1_FB18_Msk* = (0x00000001 shl CAN_F3R1_FB18_Pos) ## !< 0x00040000
  CAN_F3R1_FB18* = CAN_F3R1_FB18_Msk
  CAN_F3R1_FB19_Pos* = (19)
  CAN_F3R1_FB19_Msk* = (0x00000001 shl CAN_F3R1_FB19_Pos) ## !< 0x00080000
  CAN_F3R1_FB19* = CAN_F3R1_FB19_Msk
  CAN_F3R1_FB20_Pos* = (20)
  CAN_F3R1_FB20_Msk* = (0x00000001 shl CAN_F3R1_FB20_Pos) ## !< 0x00100000
  CAN_F3R1_FB20* = CAN_F3R1_FB20_Msk
  CAN_F3R1_FB21_Pos* = (21)
  CAN_F3R1_FB21_Msk* = (0x00000001 shl CAN_F3R1_FB21_Pos) ## !< 0x00200000
  CAN_F3R1_FB21* = CAN_F3R1_FB21_Msk
  CAN_F3R1_FB22_Pos* = (22)
  CAN_F3R1_FB22_Msk* = (0x00000001 shl CAN_F3R1_FB22_Pos) ## !< 0x00400000
  CAN_F3R1_FB22* = CAN_F3R1_FB22_Msk
  CAN_F3R1_FB23_Pos* = (23)
  CAN_F3R1_FB23_Msk* = (0x00000001 shl CAN_F3R1_FB23_Pos) ## !< 0x00800000
  CAN_F3R1_FB23* = CAN_F3R1_FB23_Msk
  CAN_F3R1_FB24_Pos* = (24)
  CAN_F3R1_FB24_Msk* = (0x00000001 shl CAN_F3R1_FB24_Pos) ## !< 0x01000000
  CAN_F3R1_FB24* = CAN_F3R1_FB24_Msk
  CAN_F3R1_FB25_Pos* = (25)
  CAN_F3R1_FB25_Msk* = (0x00000001 shl CAN_F3R1_FB25_Pos) ## !< 0x02000000
  CAN_F3R1_FB25* = CAN_F3R1_FB25_Msk
  CAN_F3R1_FB26_Pos* = (26)
  CAN_F3R1_FB26_Msk* = (0x00000001 shl CAN_F3R1_FB26_Pos) ## !< 0x04000000
  CAN_F3R1_FB26* = CAN_F3R1_FB26_Msk
  CAN_F3R1_FB27_Pos* = (27)
  CAN_F3R1_FB27_Msk* = (0x00000001 shl CAN_F3R1_FB27_Pos) ## !< 0x08000000
  CAN_F3R1_FB27* = CAN_F3R1_FB27_Msk
  CAN_F3R1_FB28_Pos* = (28)
  CAN_F3R1_FB28_Msk* = (0x00000001 shl CAN_F3R1_FB28_Pos) ## !< 0x10000000
  CAN_F3R1_FB28* = CAN_F3R1_FB28_Msk
  CAN_F3R1_FB29_Pos* = (29)
  CAN_F3R1_FB29_Msk* = (0x00000001 shl CAN_F3R1_FB29_Pos) ## !< 0x20000000
  CAN_F3R1_FB29* = CAN_F3R1_FB29_Msk
  CAN_F3R1_FB30_Pos* = (30)
  CAN_F3R1_FB30_Msk* = (0x00000001 shl CAN_F3R1_FB30_Pos) ## !< 0x40000000
  CAN_F3R1_FB30* = CAN_F3R1_FB30_Msk
  CAN_F3R1_FB31_Pos* = (31)
  CAN_F3R1_FB31_Msk* = (0x00000001 shl CAN_F3R1_FB31_Pos) ## !< 0x80000000
  CAN_F3R1_FB31* = CAN_F3R1_FB31_Msk

## ******************  Bit definition for CAN_F4R1 register  ******************

const
  CAN_F4R1_FB0_Pos* = (0)
  CAN_F4R1_FB0_Msk* = (0x00000001 shl CAN_F4R1_FB0_Pos) ## !< 0x00000001
  CAN_F4R1_FB0* = CAN_F4R1_FB0_Msk
  CAN_F4R1_FB1_Pos* = (1)
  CAN_F4R1_FB1_Msk* = (0x00000001 shl CAN_F4R1_FB1_Pos) ## !< 0x00000002
  CAN_F4R1_FB1* = CAN_F4R1_FB1_Msk
  CAN_F4R1_FB2_Pos* = (2)
  CAN_F4R1_FB2_Msk* = (0x00000001 shl CAN_F4R1_FB2_Pos) ## !< 0x00000004
  CAN_F4R1_FB2* = CAN_F4R1_FB2_Msk
  CAN_F4R1_FB3_Pos* = (3)
  CAN_F4R1_FB3_Msk* = (0x00000001 shl CAN_F4R1_FB3_Pos) ## !< 0x00000008
  CAN_F4R1_FB3* = CAN_F4R1_FB3_Msk
  CAN_F4R1_FB4_Pos* = (4)
  CAN_F4R1_FB4_Msk* = (0x00000001 shl CAN_F4R1_FB4_Pos) ## !< 0x00000010
  CAN_F4R1_FB4* = CAN_F4R1_FB4_Msk
  CAN_F4R1_FB5_Pos* = (5)
  CAN_F4R1_FB5_Msk* = (0x00000001 shl CAN_F4R1_FB5_Pos) ## !< 0x00000020
  CAN_F4R1_FB5* = CAN_F4R1_FB5_Msk
  CAN_F4R1_FB6_Pos* = (6)
  CAN_F4R1_FB6_Msk* = (0x00000001 shl CAN_F4R1_FB6_Pos) ## !< 0x00000040
  CAN_F4R1_FB6* = CAN_F4R1_FB6_Msk
  CAN_F4R1_FB7_Pos* = (7)
  CAN_F4R1_FB7_Msk* = (0x00000001 shl CAN_F4R1_FB7_Pos) ## !< 0x00000080
  CAN_F4R1_FB7* = CAN_F4R1_FB7_Msk
  CAN_F4R1_FB8_Pos* = (8)
  CAN_F4R1_FB8_Msk* = (0x00000001 shl CAN_F4R1_FB8_Pos) ## !< 0x00000100
  CAN_F4R1_FB8* = CAN_F4R1_FB8_Msk
  CAN_F4R1_FB9_Pos* = (9)
  CAN_F4R1_FB9_Msk* = (0x00000001 shl CAN_F4R1_FB9_Pos) ## !< 0x00000200
  CAN_F4R1_FB9* = CAN_F4R1_FB9_Msk
  CAN_F4R1_FB10_Pos* = (10)
  CAN_F4R1_FB10_Msk* = (0x00000001 shl CAN_F4R1_FB10_Pos) ## !< 0x00000400
  CAN_F4R1_FB10* = CAN_F4R1_FB10_Msk
  CAN_F4R1_FB11_Pos* = (11)
  CAN_F4R1_FB11_Msk* = (0x00000001 shl CAN_F4R1_FB11_Pos) ## !< 0x00000800
  CAN_F4R1_FB11* = CAN_F4R1_FB11_Msk
  CAN_F4R1_FB12_Pos* = (12)
  CAN_F4R1_FB12_Msk* = (0x00000001 shl CAN_F4R1_FB12_Pos) ## !< 0x00001000
  CAN_F4R1_FB12* = CAN_F4R1_FB12_Msk
  CAN_F4R1_FB13_Pos* = (13)
  CAN_F4R1_FB13_Msk* = (0x00000001 shl CAN_F4R1_FB13_Pos) ## !< 0x00002000
  CAN_F4R1_FB13* = CAN_F4R1_FB13_Msk
  CAN_F4R1_FB14_Pos* = (14)
  CAN_F4R1_FB14_Msk* = (0x00000001 shl CAN_F4R1_FB14_Pos) ## !< 0x00004000
  CAN_F4R1_FB14* = CAN_F4R1_FB14_Msk
  CAN_F4R1_FB15_Pos* = (15)
  CAN_F4R1_FB15_Msk* = (0x00000001 shl CAN_F4R1_FB15_Pos) ## !< 0x00008000
  CAN_F4R1_FB15* = CAN_F4R1_FB15_Msk
  CAN_F4R1_FB16_Pos* = (16)
  CAN_F4R1_FB16_Msk* = (0x00000001 shl CAN_F4R1_FB16_Pos) ## !< 0x00010000
  CAN_F4R1_FB16* = CAN_F4R1_FB16_Msk
  CAN_F4R1_FB17_Pos* = (17)
  CAN_F4R1_FB17_Msk* = (0x00000001 shl CAN_F4R1_FB17_Pos) ## !< 0x00020000
  CAN_F4R1_FB17* = CAN_F4R1_FB17_Msk
  CAN_F4R1_FB18_Pos* = (18)
  CAN_F4R1_FB18_Msk* = (0x00000001 shl CAN_F4R1_FB18_Pos) ## !< 0x00040000
  CAN_F4R1_FB18* = CAN_F4R1_FB18_Msk
  CAN_F4R1_FB19_Pos* = (19)
  CAN_F4R1_FB19_Msk* = (0x00000001 shl CAN_F4R1_FB19_Pos) ## !< 0x00080000
  CAN_F4R1_FB19* = CAN_F4R1_FB19_Msk
  CAN_F4R1_FB20_Pos* = (20)
  CAN_F4R1_FB20_Msk* = (0x00000001 shl CAN_F4R1_FB20_Pos) ## !< 0x00100000
  CAN_F4R1_FB20* = CAN_F4R1_FB20_Msk
  CAN_F4R1_FB21_Pos* = (21)
  CAN_F4R1_FB21_Msk* = (0x00000001 shl CAN_F4R1_FB21_Pos) ## !< 0x00200000
  CAN_F4R1_FB21* = CAN_F4R1_FB21_Msk
  CAN_F4R1_FB22_Pos* = (22)
  CAN_F4R1_FB22_Msk* = (0x00000001 shl CAN_F4R1_FB22_Pos) ## !< 0x00400000
  CAN_F4R1_FB22* = CAN_F4R1_FB22_Msk
  CAN_F4R1_FB23_Pos* = (23)
  CAN_F4R1_FB23_Msk* = (0x00000001 shl CAN_F4R1_FB23_Pos) ## !< 0x00800000
  CAN_F4R1_FB23* = CAN_F4R1_FB23_Msk
  CAN_F4R1_FB24_Pos* = (24)
  CAN_F4R1_FB24_Msk* = (0x00000001 shl CAN_F4R1_FB24_Pos) ## !< 0x01000000
  CAN_F4R1_FB24* = CAN_F4R1_FB24_Msk
  CAN_F4R1_FB25_Pos* = (25)
  CAN_F4R1_FB25_Msk* = (0x00000001 shl CAN_F4R1_FB25_Pos) ## !< 0x02000000
  CAN_F4R1_FB25* = CAN_F4R1_FB25_Msk
  CAN_F4R1_FB26_Pos* = (26)
  CAN_F4R1_FB26_Msk* = (0x00000001 shl CAN_F4R1_FB26_Pos) ## !< 0x04000000
  CAN_F4R1_FB26* = CAN_F4R1_FB26_Msk
  CAN_F4R1_FB27_Pos* = (27)
  CAN_F4R1_FB27_Msk* = (0x00000001 shl CAN_F4R1_FB27_Pos) ## !< 0x08000000
  CAN_F4R1_FB27* = CAN_F4R1_FB27_Msk
  CAN_F4R1_FB28_Pos* = (28)
  CAN_F4R1_FB28_Msk* = (0x00000001 shl CAN_F4R1_FB28_Pos) ## !< 0x10000000
  CAN_F4R1_FB28* = CAN_F4R1_FB28_Msk
  CAN_F4R1_FB29_Pos* = (29)
  CAN_F4R1_FB29_Msk* = (0x00000001 shl CAN_F4R1_FB29_Pos) ## !< 0x20000000
  CAN_F4R1_FB29* = CAN_F4R1_FB29_Msk
  CAN_F4R1_FB30_Pos* = (30)
  CAN_F4R1_FB30_Msk* = (0x00000001 shl CAN_F4R1_FB30_Pos) ## !< 0x40000000
  CAN_F4R1_FB30* = CAN_F4R1_FB30_Msk
  CAN_F4R1_FB31_Pos* = (31)
  CAN_F4R1_FB31_Msk* = (0x00000001 shl CAN_F4R1_FB31_Pos) ## !< 0x80000000
  CAN_F4R1_FB31* = CAN_F4R1_FB31_Msk

## ******************  Bit definition for CAN_F5R1 register  ******************

const
  CAN_F5R1_FB0_Pos* = (0)
  CAN_F5R1_FB0_Msk* = (0x00000001 shl CAN_F5R1_FB0_Pos) ## !< 0x00000001
  CAN_F5R1_FB0* = CAN_F5R1_FB0_Msk
  CAN_F5R1_FB1_Pos* = (1)
  CAN_F5R1_FB1_Msk* = (0x00000001 shl CAN_F5R1_FB1_Pos) ## !< 0x00000002
  CAN_F5R1_FB1* = CAN_F5R1_FB1_Msk
  CAN_F5R1_FB2_Pos* = (2)
  CAN_F5R1_FB2_Msk* = (0x00000001 shl CAN_F5R1_FB2_Pos) ## !< 0x00000004
  CAN_F5R1_FB2* = CAN_F5R1_FB2_Msk
  CAN_F5R1_FB3_Pos* = (3)
  CAN_F5R1_FB3_Msk* = (0x00000001 shl CAN_F5R1_FB3_Pos) ## !< 0x00000008
  CAN_F5R1_FB3* = CAN_F5R1_FB3_Msk
  CAN_F5R1_FB4_Pos* = (4)
  CAN_F5R1_FB4_Msk* = (0x00000001 shl CAN_F5R1_FB4_Pos) ## !< 0x00000010
  CAN_F5R1_FB4* = CAN_F5R1_FB4_Msk
  CAN_F5R1_FB5_Pos* = (5)
  CAN_F5R1_FB5_Msk* = (0x00000001 shl CAN_F5R1_FB5_Pos) ## !< 0x00000020
  CAN_F5R1_FB5* = CAN_F5R1_FB5_Msk
  CAN_F5R1_FB6_Pos* = (6)
  CAN_F5R1_FB6_Msk* = (0x00000001 shl CAN_F5R1_FB6_Pos) ## !< 0x00000040
  CAN_F5R1_FB6* = CAN_F5R1_FB6_Msk
  CAN_F5R1_FB7_Pos* = (7)
  CAN_F5R1_FB7_Msk* = (0x00000001 shl CAN_F5R1_FB7_Pos) ## !< 0x00000080
  CAN_F5R1_FB7* = CAN_F5R1_FB7_Msk
  CAN_F5R1_FB8_Pos* = (8)
  CAN_F5R1_FB8_Msk* = (0x00000001 shl CAN_F5R1_FB8_Pos) ## !< 0x00000100
  CAN_F5R1_FB8* = CAN_F5R1_FB8_Msk
  CAN_F5R1_FB9_Pos* = (9)
  CAN_F5R1_FB9_Msk* = (0x00000001 shl CAN_F5R1_FB9_Pos) ## !< 0x00000200
  CAN_F5R1_FB9* = CAN_F5R1_FB9_Msk
  CAN_F5R1_FB10_Pos* = (10)
  CAN_F5R1_FB10_Msk* = (0x00000001 shl CAN_F5R1_FB10_Pos) ## !< 0x00000400
  CAN_F5R1_FB10* = CAN_F5R1_FB10_Msk
  CAN_F5R1_FB11_Pos* = (11)
  CAN_F5R1_FB11_Msk* = (0x00000001 shl CAN_F5R1_FB11_Pos) ## !< 0x00000800
  CAN_F5R1_FB11* = CAN_F5R1_FB11_Msk
  CAN_F5R1_FB12_Pos* = (12)
  CAN_F5R1_FB12_Msk* = (0x00000001 shl CAN_F5R1_FB12_Pos) ## !< 0x00001000
  CAN_F5R1_FB12* = CAN_F5R1_FB12_Msk
  CAN_F5R1_FB13_Pos* = (13)
  CAN_F5R1_FB13_Msk* = (0x00000001 shl CAN_F5R1_FB13_Pos) ## !< 0x00002000
  CAN_F5R1_FB13* = CAN_F5R1_FB13_Msk
  CAN_F5R1_FB14_Pos* = (14)
  CAN_F5R1_FB14_Msk* = (0x00000001 shl CAN_F5R1_FB14_Pos) ## !< 0x00004000
  CAN_F5R1_FB14* = CAN_F5R1_FB14_Msk
  CAN_F5R1_FB15_Pos* = (15)
  CAN_F5R1_FB15_Msk* = (0x00000001 shl CAN_F5R1_FB15_Pos) ## !< 0x00008000
  CAN_F5R1_FB15* = CAN_F5R1_FB15_Msk
  CAN_F5R1_FB16_Pos* = (16)
  CAN_F5R1_FB16_Msk* = (0x00000001 shl CAN_F5R1_FB16_Pos) ## !< 0x00010000
  CAN_F5R1_FB16* = CAN_F5R1_FB16_Msk
  CAN_F5R1_FB17_Pos* = (17)
  CAN_F5R1_FB17_Msk* = (0x00000001 shl CAN_F5R1_FB17_Pos) ## !< 0x00020000
  CAN_F5R1_FB17* = CAN_F5R1_FB17_Msk
  CAN_F5R1_FB18_Pos* = (18)
  CAN_F5R1_FB18_Msk* = (0x00000001 shl CAN_F5R1_FB18_Pos) ## !< 0x00040000
  CAN_F5R1_FB18* = CAN_F5R1_FB18_Msk
  CAN_F5R1_FB19_Pos* = (19)
  CAN_F5R1_FB19_Msk* = (0x00000001 shl CAN_F5R1_FB19_Pos) ## !< 0x00080000
  CAN_F5R1_FB19* = CAN_F5R1_FB19_Msk
  CAN_F5R1_FB20_Pos* = (20)
  CAN_F5R1_FB20_Msk* = (0x00000001 shl CAN_F5R1_FB20_Pos) ## !< 0x00100000
  CAN_F5R1_FB20* = CAN_F5R1_FB20_Msk
  CAN_F5R1_FB21_Pos* = (21)
  CAN_F5R1_FB21_Msk* = (0x00000001 shl CAN_F5R1_FB21_Pos) ## !< 0x00200000
  CAN_F5R1_FB21* = CAN_F5R1_FB21_Msk
  CAN_F5R1_FB22_Pos* = (22)
  CAN_F5R1_FB22_Msk* = (0x00000001 shl CAN_F5R1_FB22_Pos) ## !< 0x00400000
  CAN_F5R1_FB22* = CAN_F5R1_FB22_Msk
  CAN_F5R1_FB23_Pos* = (23)
  CAN_F5R1_FB23_Msk* = (0x00000001 shl CAN_F5R1_FB23_Pos) ## !< 0x00800000
  CAN_F5R1_FB23* = CAN_F5R1_FB23_Msk
  CAN_F5R1_FB24_Pos* = (24)
  CAN_F5R1_FB24_Msk* = (0x00000001 shl CAN_F5R1_FB24_Pos) ## !< 0x01000000
  CAN_F5R1_FB24* = CAN_F5R1_FB24_Msk
  CAN_F5R1_FB25_Pos* = (25)
  CAN_F5R1_FB25_Msk* = (0x00000001 shl CAN_F5R1_FB25_Pos) ## !< 0x02000000
  CAN_F5R1_FB25* = CAN_F5R1_FB25_Msk
  CAN_F5R1_FB26_Pos* = (26)
  CAN_F5R1_FB26_Msk* = (0x00000001 shl CAN_F5R1_FB26_Pos) ## !< 0x04000000
  CAN_F5R1_FB26* = CAN_F5R1_FB26_Msk
  CAN_F5R1_FB27_Pos* = (27)
  CAN_F5R1_FB27_Msk* = (0x00000001 shl CAN_F5R1_FB27_Pos) ## !< 0x08000000
  CAN_F5R1_FB27* = CAN_F5R1_FB27_Msk
  CAN_F5R1_FB28_Pos* = (28)
  CAN_F5R1_FB28_Msk* = (0x00000001 shl CAN_F5R1_FB28_Pos) ## !< 0x10000000
  CAN_F5R1_FB28* = CAN_F5R1_FB28_Msk
  CAN_F5R1_FB29_Pos* = (29)
  CAN_F5R1_FB29_Msk* = (0x00000001 shl CAN_F5R1_FB29_Pos) ## !< 0x20000000
  CAN_F5R1_FB29* = CAN_F5R1_FB29_Msk
  CAN_F5R1_FB30_Pos* = (30)
  CAN_F5R1_FB30_Msk* = (0x00000001 shl CAN_F5R1_FB30_Pos) ## !< 0x40000000
  CAN_F5R1_FB30* = CAN_F5R1_FB30_Msk
  CAN_F5R1_FB31_Pos* = (31)
  CAN_F5R1_FB31_Msk* = (0x00000001 shl CAN_F5R1_FB31_Pos) ## !< 0x80000000
  CAN_F5R1_FB31* = CAN_F5R1_FB31_Msk

## ******************  Bit definition for CAN_F6R1 register  ******************

const
  CAN_F6R1_FB0_Pos* = (0)
  CAN_F6R1_FB0_Msk* = (0x00000001 shl CAN_F6R1_FB0_Pos) ## !< 0x00000001
  CAN_F6R1_FB0* = CAN_F6R1_FB0_Msk
  CAN_F6R1_FB1_Pos* = (1)
  CAN_F6R1_FB1_Msk* = (0x00000001 shl CAN_F6R1_FB1_Pos) ## !< 0x00000002
  CAN_F6R1_FB1* = CAN_F6R1_FB1_Msk
  CAN_F6R1_FB2_Pos* = (2)
  CAN_F6R1_FB2_Msk* = (0x00000001 shl CAN_F6R1_FB2_Pos) ## !< 0x00000004
  CAN_F6R1_FB2* = CAN_F6R1_FB2_Msk
  CAN_F6R1_FB3_Pos* = (3)
  CAN_F6R1_FB3_Msk* = (0x00000001 shl CAN_F6R1_FB3_Pos) ## !< 0x00000008
  CAN_F6R1_FB3* = CAN_F6R1_FB3_Msk
  CAN_F6R1_FB4_Pos* = (4)
  CAN_F6R1_FB4_Msk* = (0x00000001 shl CAN_F6R1_FB4_Pos) ## !< 0x00000010
  CAN_F6R1_FB4* = CAN_F6R1_FB4_Msk
  CAN_F6R1_FB5_Pos* = (5)
  CAN_F6R1_FB5_Msk* = (0x00000001 shl CAN_F6R1_FB5_Pos) ## !< 0x00000020
  CAN_F6R1_FB5* = CAN_F6R1_FB5_Msk
  CAN_F6R1_FB6_Pos* = (6)
  CAN_F6R1_FB6_Msk* = (0x00000001 shl CAN_F6R1_FB6_Pos) ## !< 0x00000040
  CAN_F6R1_FB6* = CAN_F6R1_FB6_Msk
  CAN_F6R1_FB7_Pos* = (7)
  CAN_F6R1_FB7_Msk* = (0x00000001 shl CAN_F6R1_FB7_Pos) ## !< 0x00000080
  CAN_F6R1_FB7* = CAN_F6R1_FB7_Msk
  CAN_F6R1_FB8_Pos* = (8)
  CAN_F6R1_FB8_Msk* = (0x00000001 shl CAN_F6R1_FB8_Pos) ## !< 0x00000100
  CAN_F6R1_FB8* = CAN_F6R1_FB8_Msk
  CAN_F6R1_FB9_Pos* = (9)
  CAN_F6R1_FB9_Msk* = (0x00000001 shl CAN_F6R1_FB9_Pos) ## !< 0x00000200
  CAN_F6R1_FB9* = CAN_F6R1_FB9_Msk
  CAN_F6R1_FB10_Pos* = (10)
  CAN_F6R1_FB10_Msk* = (0x00000001 shl CAN_F6R1_FB10_Pos) ## !< 0x00000400
  CAN_F6R1_FB10* = CAN_F6R1_FB10_Msk
  CAN_F6R1_FB11_Pos* = (11)
  CAN_F6R1_FB11_Msk* = (0x00000001 shl CAN_F6R1_FB11_Pos) ## !< 0x00000800
  CAN_F6R1_FB11* = CAN_F6R1_FB11_Msk
  CAN_F6R1_FB12_Pos* = (12)
  CAN_F6R1_FB12_Msk* = (0x00000001 shl CAN_F6R1_FB12_Pos) ## !< 0x00001000
  CAN_F6R1_FB12* = CAN_F6R1_FB12_Msk
  CAN_F6R1_FB13_Pos* = (13)
  CAN_F6R1_FB13_Msk* = (0x00000001 shl CAN_F6R1_FB13_Pos) ## !< 0x00002000
  CAN_F6R1_FB13* = CAN_F6R1_FB13_Msk
  CAN_F6R1_FB14_Pos* = (14)
  CAN_F6R1_FB14_Msk* = (0x00000001 shl CAN_F6R1_FB14_Pos) ## !< 0x00004000
  CAN_F6R1_FB14* = CAN_F6R1_FB14_Msk
  CAN_F6R1_FB15_Pos* = (15)
  CAN_F6R1_FB15_Msk* = (0x00000001 shl CAN_F6R1_FB15_Pos) ## !< 0x00008000
  CAN_F6R1_FB15* = CAN_F6R1_FB15_Msk
  CAN_F6R1_FB16_Pos* = (16)
  CAN_F6R1_FB16_Msk* = (0x00000001 shl CAN_F6R1_FB16_Pos) ## !< 0x00010000
  CAN_F6R1_FB16* = CAN_F6R1_FB16_Msk
  CAN_F6R1_FB17_Pos* = (17)
  CAN_F6R1_FB17_Msk* = (0x00000001 shl CAN_F6R1_FB17_Pos) ## !< 0x00020000
  CAN_F6R1_FB17* = CAN_F6R1_FB17_Msk
  CAN_F6R1_FB18_Pos* = (18)
  CAN_F6R1_FB18_Msk* = (0x00000001 shl CAN_F6R1_FB18_Pos) ## !< 0x00040000
  CAN_F6R1_FB18* = CAN_F6R1_FB18_Msk
  CAN_F6R1_FB19_Pos* = (19)
  CAN_F6R1_FB19_Msk* = (0x00000001 shl CAN_F6R1_FB19_Pos) ## !< 0x00080000
  CAN_F6R1_FB19* = CAN_F6R1_FB19_Msk
  CAN_F6R1_FB20_Pos* = (20)
  CAN_F6R1_FB20_Msk* = (0x00000001 shl CAN_F6R1_FB20_Pos) ## !< 0x00100000
  CAN_F6R1_FB20* = CAN_F6R1_FB20_Msk
  CAN_F6R1_FB21_Pos* = (21)
  CAN_F6R1_FB21_Msk* = (0x00000001 shl CAN_F6R1_FB21_Pos) ## !< 0x00200000
  CAN_F6R1_FB21* = CAN_F6R1_FB21_Msk
  CAN_F6R1_FB22_Pos* = (22)
  CAN_F6R1_FB22_Msk* = (0x00000001 shl CAN_F6R1_FB22_Pos) ## !< 0x00400000
  CAN_F6R1_FB22* = CAN_F6R1_FB22_Msk
  CAN_F6R1_FB23_Pos* = (23)
  CAN_F6R1_FB23_Msk* = (0x00000001 shl CAN_F6R1_FB23_Pos) ## !< 0x00800000
  CAN_F6R1_FB23* = CAN_F6R1_FB23_Msk
  CAN_F6R1_FB24_Pos* = (24)
  CAN_F6R1_FB24_Msk* = (0x00000001 shl CAN_F6R1_FB24_Pos) ## !< 0x01000000
  CAN_F6R1_FB24* = CAN_F6R1_FB24_Msk
  CAN_F6R1_FB25_Pos* = (25)
  CAN_F6R1_FB25_Msk* = (0x00000001 shl CAN_F6R1_FB25_Pos) ## !< 0x02000000
  CAN_F6R1_FB25* = CAN_F6R1_FB25_Msk
  CAN_F6R1_FB26_Pos* = (26)
  CAN_F6R1_FB26_Msk* = (0x00000001 shl CAN_F6R1_FB26_Pos) ## !< 0x04000000
  CAN_F6R1_FB26* = CAN_F6R1_FB26_Msk
  CAN_F6R1_FB27_Pos* = (27)
  CAN_F6R1_FB27_Msk* = (0x00000001 shl CAN_F6R1_FB27_Pos) ## !< 0x08000000
  CAN_F6R1_FB27* = CAN_F6R1_FB27_Msk
  CAN_F6R1_FB28_Pos* = (28)
  CAN_F6R1_FB28_Msk* = (0x00000001 shl CAN_F6R1_FB28_Pos) ## !< 0x10000000
  CAN_F6R1_FB28* = CAN_F6R1_FB28_Msk
  CAN_F6R1_FB29_Pos* = (29)
  CAN_F6R1_FB29_Msk* = (0x00000001 shl CAN_F6R1_FB29_Pos) ## !< 0x20000000
  CAN_F6R1_FB29* = CAN_F6R1_FB29_Msk
  CAN_F6R1_FB30_Pos* = (30)
  CAN_F6R1_FB30_Msk* = (0x00000001 shl CAN_F6R1_FB30_Pos) ## !< 0x40000000
  CAN_F6R1_FB30* = CAN_F6R1_FB30_Msk
  CAN_F6R1_FB31_Pos* = (31)
  CAN_F6R1_FB31_Msk* = (0x00000001 shl CAN_F6R1_FB31_Pos) ## !< 0x80000000
  CAN_F6R1_FB31* = CAN_F6R1_FB31_Msk

## ******************  Bit definition for CAN_F7R1 register  ******************

const
  CAN_F7R1_FB0_Pos* = (0)
  CAN_F7R1_FB0_Msk* = (0x00000001 shl CAN_F7R1_FB0_Pos) ## !< 0x00000001
  CAN_F7R1_FB0* = CAN_F7R1_FB0_Msk
  CAN_F7R1_FB1_Pos* = (1)
  CAN_F7R1_FB1_Msk* = (0x00000001 shl CAN_F7R1_FB1_Pos) ## !< 0x00000002
  CAN_F7R1_FB1* = CAN_F7R1_FB1_Msk
  CAN_F7R1_FB2_Pos* = (2)
  CAN_F7R1_FB2_Msk* = (0x00000001 shl CAN_F7R1_FB2_Pos) ## !< 0x00000004
  CAN_F7R1_FB2* = CAN_F7R1_FB2_Msk
  CAN_F7R1_FB3_Pos* = (3)
  CAN_F7R1_FB3_Msk* = (0x00000001 shl CAN_F7R1_FB3_Pos) ## !< 0x00000008
  CAN_F7R1_FB3* = CAN_F7R1_FB3_Msk
  CAN_F7R1_FB4_Pos* = (4)
  CAN_F7R1_FB4_Msk* = (0x00000001 shl CAN_F7R1_FB4_Pos) ## !< 0x00000010
  CAN_F7R1_FB4* = CAN_F7R1_FB4_Msk
  CAN_F7R1_FB5_Pos* = (5)
  CAN_F7R1_FB5_Msk* = (0x00000001 shl CAN_F7R1_FB5_Pos) ## !< 0x00000020
  CAN_F7R1_FB5* = CAN_F7R1_FB5_Msk
  CAN_F7R1_FB6_Pos* = (6)
  CAN_F7R1_FB6_Msk* = (0x00000001 shl CAN_F7R1_FB6_Pos) ## !< 0x00000040
  CAN_F7R1_FB6* = CAN_F7R1_FB6_Msk
  CAN_F7R1_FB7_Pos* = (7)
  CAN_F7R1_FB7_Msk* = (0x00000001 shl CAN_F7R1_FB7_Pos) ## !< 0x00000080
  CAN_F7R1_FB7* = CAN_F7R1_FB7_Msk
  CAN_F7R1_FB8_Pos* = (8)
  CAN_F7R1_FB8_Msk* = (0x00000001 shl CAN_F7R1_FB8_Pos) ## !< 0x00000100
  CAN_F7R1_FB8* = CAN_F7R1_FB8_Msk
  CAN_F7R1_FB9_Pos* = (9)
  CAN_F7R1_FB9_Msk* = (0x00000001 shl CAN_F7R1_FB9_Pos) ## !< 0x00000200
  CAN_F7R1_FB9* = CAN_F7R1_FB9_Msk
  CAN_F7R1_FB10_Pos* = (10)
  CAN_F7R1_FB10_Msk* = (0x00000001 shl CAN_F7R1_FB10_Pos) ## !< 0x00000400
  CAN_F7R1_FB10* = CAN_F7R1_FB10_Msk
  CAN_F7R1_FB11_Pos* = (11)
  CAN_F7R1_FB11_Msk* = (0x00000001 shl CAN_F7R1_FB11_Pos) ## !< 0x00000800
  CAN_F7R1_FB11* = CAN_F7R1_FB11_Msk
  CAN_F7R1_FB12_Pos* = (12)
  CAN_F7R1_FB12_Msk* = (0x00000001 shl CAN_F7R1_FB12_Pos) ## !< 0x00001000
  CAN_F7R1_FB12* = CAN_F7R1_FB12_Msk
  CAN_F7R1_FB13_Pos* = (13)
  CAN_F7R1_FB13_Msk* = (0x00000001 shl CAN_F7R1_FB13_Pos) ## !< 0x00002000
  CAN_F7R1_FB13* = CAN_F7R1_FB13_Msk
  CAN_F7R1_FB14_Pos* = (14)
  CAN_F7R1_FB14_Msk* = (0x00000001 shl CAN_F7R1_FB14_Pos) ## !< 0x00004000
  CAN_F7R1_FB14* = CAN_F7R1_FB14_Msk
  CAN_F7R1_FB15_Pos* = (15)
  CAN_F7R1_FB15_Msk* = (0x00000001 shl CAN_F7R1_FB15_Pos) ## !< 0x00008000
  CAN_F7R1_FB15* = CAN_F7R1_FB15_Msk
  CAN_F7R1_FB16_Pos* = (16)
  CAN_F7R1_FB16_Msk* = (0x00000001 shl CAN_F7R1_FB16_Pos) ## !< 0x00010000
  CAN_F7R1_FB16* = CAN_F7R1_FB16_Msk
  CAN_F7R1_FB17_Pos* = (17)
  CAN_F7R1_FB17_Msk* = (0x00000001 shl CAN_F7R1_FB17_Pos) ## !< 0x00020000
  CAN_F7R1_FB17* = CAN_F7R1_FB17_Msk
  CAN_F7R1_FB18_Pos* = (18)
  CAN_F7R1_FB18_Msk* = (0x00000001 shl CAN_F7R1_FB18_Pos) ## !< 0x00040000
  CAN_F7R1_FB18* = CAN_F7R1_FB18_Msk
  CAN_F7R1_FB19_Pos* = (19)
  CAN_F7R1_FB19_Msk* = (0x00000001 shl CAN_F7R1_FB19_Pos) ## !< 0x00080000
  CAN_F7R1_FB19* = CAN_F7R1_FB19_Msk
  CAN_F7R1_FB20_Pos* = (20)
  CAN_F7R1_FB20_Msk* = (0x00000001 shl CAN_F7R1_FB20_Pos) ## !< 0x00100000
  CAN_F7R1_FB20* = CAN_F7R1_FB20_Msk
  CAN_F7R1_FB21_Pos* = (21)
  CAN_F7R1_FB21_Msk* = (0x00000001 shl CAN_F7R1_FB21_Pos) ## !< 0x00200000
  CAN_F7R1_FB21* = CAN_F7R1_FB21_Msk
  CAN_F7R1_FB22_Pos* = (22)
  CAN_F7R1_FB22_Msk* = (0x00000001 shl CAN_F7R1_FB22_Pos) ## !< 0x00400000
  CAN_F7R1_FB22* = CAN_F7R1_FB22_Msk
  CAN_F7R1_FB23_Pos* = (23)
  CAN_F7R1_FB23_Msk* = (0x00000001 shl CAN_F7R1_FB23_Pos) ## !< 0x00800000
  CAN_F7R1_FB23* = CAN_F7R1_FB23_Msk
  CAN_F7R1_FB24_Pos* = (24)
  CAN_F7R1_FB24_Msk* = (0x00000001 shl CAN_F7R1_FB24_Pos) ## !< 0x01000000
  CAN_F7R1_FB24* = CAN_F7R1_FB24_Msk
  CAN_F7R1_FB25_Pos* = (25)
  CAN_F7R1_FB25_Msk* = (0x00000001 shl CAN_F7R1_FB25_Pos) ## !< 0x02000000
  CAN_F7R1_FB25* = CAN_F7R1_FB25_Msk
  CAN_F7R1_FB26_Pos* = (26)
  CAN_F7R1_FB26_Msk* = (0x00000001 shl CAN_F7R1_FB26_Pos) ## !< 0x04000000
  CAN_F7R1_FB26* = CAN_F7R1_FB26_Msk
  CAN_F7R1_FB27_Pos* = (27)
  CAN_F7R1_FB27_Msk* = (0x00000001 shl CAN_F7R1_FB27_Pos) ## !< 0x08000000
  CAN_F7R1_FB27* = CAN_F7R1_FB27_Msk
  CAN_F7R1_FB28_Pos* = (28)
  CAN_F7R1_FB28_Msk* = (0x00000001 shl CAN_F7R1_FB28_Pos) ## !< 0x10000000
  CAN_F7R1_FB28* = CAN_F7R1_FB28_Msk
  CAN_F7R1_FB29_Pos* = (29)
  CAN_F7R1_FB29_Msk* = (0x00000001 shl CAN_F7R1_FB29_Pos) ## !< 0x20000000
  CAN_F7R1_FB29* = CAN_F7R1_FB29_Msk
  CAN_F7R1_FB30_Pos* = (30)
  CAN_F7R1_FB30_Msk* = (0x00000001 shl CAN_F7R1_FB30_Pos) ## !< 0x40000000
  CAN_F7R1_FB30* = CAN_F7R1_FB30_Msk
  CAN_F7R1_FB31_Pos* = (31)
  CAN_F7R1_FB31_Msk* = (0x00000001 shl CAN_F7R1_FB31_Pos) ## !< 0x80000000
  CAN_F7R1_FB31* = CAN_F7R1_FB31_Msk

## ******************  Bit definition for CAN_F8R1 register  ******************

const
  CAN_F8R1_FB0_Pos* = (0)
  CAN_F8R1_FB0_Msk* = (0x00000001 shl CAN_F8R1_FB0_Pos) ## !< 0x00000001
  CAN_F8R1_FB0* = CAN_F8R1_FB0_Msk
  CAN_F8R1_FB1_Pos* = (1)
  CAN_F8R1_FB1_Msk* = (0x00000001 shl CAN_F8R1_FB1_Pos) ## !< 0x00000002
  CAN_F8R1_FB1* = CAN_F8R1_FB1_Msk
  CAN_F8R1_FB2_Pos* = (2)
  CAN_F8R1_FB2_Msk* = (0x00000001 shl CAN_F8R1_FB2_Pos) ## !< 0x00000004
  CAN_F8R1_FB2* = CAN_F8R1_FB2_Msk
  CAN_F8R1_FB3_Pos* = (3)
  CAN_F8R1_FB3_Msk* = (0x00000001 shl CAN_F8R1_FB3_Pos) ## !< 0x00000008
  CAN_F8R1_FB3* = CAN_F8R1_FB3_Msk
  CAN_F8R1_FB4_Pos* = (4)
  CAN_F8R1_FB4_Msk* = (0x00000001 shl CAN_F8R1_FB4_Pos) ## !< 0x00000010
  CAN_F8R1_FB4* = CAN_F8R1_FB4_Msk
  CAN_F8R1_FB5_Pos* = (5)
  CAN_F8R1_FB5_Msk* = (0x00000001 shl CAN_F8R1_FB5_Pos) ## !< 0x00000020
  CAN_F8R1_FB5* = CAN_F8R1_FB5_Msk
  CAN_F8R1_FB6_Pos* = (6)
  CAN_F8R1_FB6_Msk* = (0x00000001 shl CAN_F8R1_FB6_Pos) ## !< 0x00000040
  CAN_F8R1_FB6* = CAN_F8R1_FB6_Msk
  CAN_F8R1_FB7_Pos* = (7)
  CAN_F8R1_FB7_Msk* = (0x00000001 shl CAN_F8R1_FB7_Pos) ## !< 0x00000080
  CAN_F8R1_FB7* = CAN_F8R1_FB7_Msk
  CAN_F8R1_FB8_Pos* = (8)
  CAN_F8R1_FB8_Msk* = (0x00000001 shl CAN_F8R1_FB8_Pos) ## !< 0x00000100
  CAN_F8R1_FB8* = CAN_F8R1_FB8_Msk
  CAN_F8R1_FB9_Pos* = (9)
  CAN_F8R1_FB9_Msk* = (0x00000001 shl CAN_F8R1_FB9_Pos) ## !< 0x00000200
  CAN_F8R1_FB9* = CAN_F8R1_FB9_Msk
  CAN_F8R1_FB10_Pos* = (10)
  CAN_F8R1_FB10_Msk* = (0x00000001 shl CAN_F8R1_FB10_Pos) ## !< 0x00000400
  CAN_F8R1_FB10* = CAN_F8R1_FB10_Msk
  CAN_F8R1_FB11_Pos* = (11)
  CAN_F8R1_FB11_Msk* = (0x00000001 shl CAN_F8R1_FB11_Pos) ## !< 0x00000800
  CAN_F8R1_FB11* = CAN_F8R1_FB11_Msk
  CAN_F8R1_FB12_Pos* = (12)
  CAN_F8R1_FB12_Msk* = (0x00000001 shl CAN_F8R1_FB12_Pos) ## !< 0x00001000
  CAN_F8R1_FB12* = CAN_F8R1_FB12_Msk
  CAN_F8R1_FB13_Pos* = (13)
  CAN_F8R1_FB13_Msk* = (0x00000001 shl CAN_F8R1_FB13_Pos) ## !< 0x00002000
  CAN_F8R1_FB13* = CAN_F8R1_FB13_Msk
  CAN_F8R1_FB14_Pos* = (14)
  CAN_F8R1_FB14_Msk* = (0x00000001 shl CAN_F8R1_FB14_Pos) ## !< 0x00004000
  CAN_F8R1_FB14* = CAN_F8R1_FB14_Msk
  CAN_F8R1_FB15_Pos* = (15)
  CAN_F8R1_FB15_Msk* = (0x00000001 shl CAN_F8R1_FB15_Pos) ## !< 0x00008000
  CAN_F8R1_FB15* = CAN_F8R1_FB15_Msk
  CAN_F8R1_FB16_Pos* = (16)
  CAN_F8R1_FB16_Msk* = (0x00000001 shl CAN_F8R1_FB16_Pos) ## !< 0x00010000
  CAN_F8R1_FB16* = CAN_F8R1_FB16_Msk
  CAN_F8R1_FB17_Pos* = (17)
  CAN_F8R1_FB17_Msk* = (0x00000001 shl CAN_F8R1_FB17_Pos) ## !< 0x00020000
  CAN_F8R1_FB17* = CAN_F8R1_FB17_Msk
  CAN_F8R1_FB18_Pos* = (18)
  CAN_F8R1_FB18_Msk* = (0x00000001 shl CAN_F8R1_FB18_Pos) ## !< 0x00040000
  CAN_F8R1_FB18* = CAN_F8R1_FB18_Msk
  CAN_F8R1_FB19_Pos* = (19)
  CAN_F8R1_FB19_Msk* = (0x00000001 shl CAN_F8R1_FB19_Pos) ## !< 0x00080000
  CAN_F8R1_FB19* = CAN_F8R1_FB19_Msk
  CAN_F8R1_FB20_Pos* = (20)
  CAN_F8R1_FB20_Msk* = (0x00000001 shl CAN_F8R1_FB20_Pos) ## !< 0x00100000
  CAN_F8R1_FB20* = CAN_F8R1_FB20_Msk
  CAN_F8R1_FB21_Pos* = (21)
  CAN_F8R1_FB21_Msk* = (0x00000001 shl CAN_F8R1_FB21_Pos) ## !< 0x00200000
  CAN_F8R1_FB21* = CAN_F8R1_FB21_Msk
  CAN_F8R1_FB22_Pos* = (22)
  CAN_F8R1_FB22_Msk* = (0x00000001 shl CAN_F8R1_FB22_Pos) ## !< 0x00400000
  CAN_F8R1_FB22* = CAN_F8R1_FB22_Msk
  CAN_F8R1_FB23_Pos* = (23)
  CAN_F8R1_FB23_Msk* = (0x00000001 shl CAN_F8R1_FB23_Pos) ## !< 0x00800000
  CAN_F8R1_FB23* = CAN_F8R1_FB23_Msk
  CAN_F8R1_FB24_Pos* = (24)
  CAN_F8R1_FB24_Msk* = (0x00000001 shl CAN_F8R1_FB24_Pos) ## !< 0x01000000
  CAN_F8R1_FB24* = CAN_F8R1_FB24_Msk
  CAN_F8R1_FB25_Pos* = (25)
  CAN_F8R1_FB25_Msk* = (0x00000001 shl CAN_F8R1_FB25_Pos) ## !< 0x02000000
  CAN_F8R1_FB25* = CAN_F8R1_FB25_Msk
  CAN_F8R1_FB26_Pos* = (26)
  CAN_F8R1_FB26_Msk* = (0x00000001 shl CAN_F8R1_FB26_Pos) ## !< 0x04000000
  CAN_F8R1_FB26* = CAN_F8R1_FB26_Msk
  CAN_F8R1_FB27_Pos* = (27)
  CAN_F8R1_FB27_Msk* = (0x00000001 shl CAN_F8R1_FB27_Pos) ## !< 0x08000000
  CAN_F8R1_FB27* = CAN_F8R1_FB27_Msk
  CAN_F8R1_FB28_Pos* = (28)
  CAN_F8R1_FB28_Msk* = (0x00000001 shl CAN_F8R1_FB28_Pos) ## !< 0x10000000
  CAN_F8R1_FB28* = CAN_F8R1_FB28_Msk
  CAN_F8R1_FB29_Pos* = (29)
  CAN_F8R1_FB29_Msk* = (0x00000001 shl CAN_F8R1_FB29_Pos) ## !< 0x20000000
  CAN_F8R1_FB29* = CAN_F8R1_FB29_Msk
  CAN_F8R1_FB30_Pos* = (30)
  CAN_F8R1_FB30_Msk* = (0x00000001 shl CAN_F8R1_FB30_Pos) ## !< 0x40000000
  CAN_F8R1_FB30* = CAN_F8R1_FB30_Msk
  CAN_F8R1_FB31_Pos* = (31)
  CAN_F8R1_FB31_Msk* = (0x00000001 shl CAN_F8R1_FB31_Pos) ## !< 0x80000000
  CAN_F8R1_FB31* = CAN_F8R1_FB31_Msk

## ******************  Bit definition for CAN_F9R1 register  ******************

const
  CAN_F9R1_FB0_Pos* = (0)
  CAN_F9R1_FB0_Msk* = (0x00000001 shl CAN_F9R1_FB0_Pos) ## !< 0x00000001
  CAN_F9R1_FB0* = CAN_F9R1_FB0_Msk
  CAN_F9R1_FB1_Pos* = (1)
  CAN_F9R1_FB1_Msk* = (0x00000001 shl CAN_F9R1_FB1_Pos) ## !< 0x00000002
  CAN_F9R1_FB1* = CAN_F9R1_FB1_Msk
  CAN_F9R1_FB2_Pos* = (2)
  CAN_F9R1_FB2_Msk* = (0x00000001 shl CAN_F9R1_FB2_Pos) ## !< 0x00000004
  CAN_F9R1_FB2* = CAN_F9R1_FB2_Msk
  CAN_F9R1_FB3_Pos* = (3)
  CAN_F9R1_FB3_Msk* = (0x00000001 shl CAN_F9R1_FB3_Pos) ## !< 0x00000008
  CAN_F9R1_FB3* = CAN_F9R1_FB3_Msk
  CAN_F9R1_FB4_Pos* = (4)
  CAN_F9R1_FB4_Msk* = (0x00000001 shl CAN_F9R1_FB4_Pos) ## !< 0x00000010
  CAN_F9R1_FB4* = CAN_F9R1_FB4_Msk
  CAN_F9R1_FB5_Pos* = (5)
  CAN_F9R1_FB5_Msk* = (0x00000001 shl CAN_F9R1_FB5_Pos) ## !< 0x00000020
  CAN_F9R1_FB5* = CAN_F9R1_FB5_Msk
  CAN_F9R1_FB6_Pos* = (6)
  CAN_F9R1_FB6_Msk* = (0x00000001 shl CAN_F9R1_FB6_Pos) ## !< 0x00000040
  CAN_F9R1_FB6* = CAN_F9R1_FB6_Msk
  CAN_F9R1_FB7_Pos* = (7)
  CAN_F9R1_FB7_Msk* = (0x00000001 shl CAN_F9R1_FB7_Pos) ## !< 0x00000080
  CAN_F9R1_FB7* = CAN_F9R1_FB7_Msk
  CAN_F9R1_FB8_Pos* = (8)
  CAN_F9R1_FB8_Msk* = (0x00000001 shl CAN_F9R1_FB8_Pos) ## !< 0x00000100
  CAN_F9R1_FB8* = CAN_F9R1_FB8_Msk
  CAN_F9R1_FB9_Pos* = (9)
  CAN_F9R1_FB9_Msk* = (0x00000001 shl CAN_F9R1_FB9_Pos) ## !< 0x00000200
  CAN_F9R1_FB9* = CAN_F9R1_FB9_Msk
  CAN_F9R1_FB10_Pos* = (10)
  CAN_F9R1_FB10_Msk* = (0x00000001 shl CAN_F9R1_FB10_Pos) ## !< 0x00000400
  CAN_F9R1_FB10* = CAN_F9R1_FB10_Msk
  CAN_F9R1_FB11_Pos* = (11)
  CAN_F9R1_FB11_Msk* = (0x00000001 shl CAN_F9R1_FB11_Pos) ## !< 0x00000800
  CAN_F9R1_FB11* = CAN_F9R1_FB11_Msk
  CAN_F9R1_FB12_Pos* = (12)
  CAN_F9R1_FB12_Msk* = (0x00000001 shl CAN_F9R1_FB12_Pos) ## !< 0x00001000
  CAN_F9R1_FB12* = CAN_F9R1_FB12_Msk
  CAN_F9R1_FB13_Pos* = (13)
  CAN_F9R1_FB13_Msk* = (0x00000001 shl CAN_F9R1_FB13_Pos) ## !< 0x00002000
  CAN_F9R1_FB13* = CAN_F9R1_FB13_Msk
  CAN_F9R1_FB14_Pos* = (14)
  CAN_F9R1_FB14_Msk* = (0x00000001 shl CAN_F9R1_FB14_Pos) ## !< 0x00004000
  CAN_F9R1_FB14* = CAN_F9R1_FB14_Msk
  CAN_F9R1_FB15_Pos* = (15)
  CAN_F9R1_FB15_Msk* = (0x00000001 shl CAN_F9R1_FB15_Pos) ## !< 0x00008000
  CAN_F9R1_FB15* = CAN_F9R1_FB15_Msk
  CAN_F9R1_FB16_Pos* = (16)
  CAN_F9R1_FB16_Msk* = (0x00000001 shl CAN_F9R1_FB16_Pos) ## !< 0x00010000
  CAN_F9R1_FB16* = CAN_F9R1_FB16_Msk
  CAN_F9R1_FB17_Pos* = (17)
  CAN_F9R1_FB17_Msk* = (0x00000001 shl CAN_F9R1_FB17_Pos) ## !< 0x00020000
  CAN_F9R1_FB17* = CAN_F9R1_FB17_Msk
  CAN_F9R1_FB18_Pos* = (18)
  CAN_F9R1_FB18_Msk* = (0x00000001 shl CAN_F9R1_FB18_Pos) ## !< 0x00040000
  CAN_F9R1_FB18* = CAN_F9R1_FB18_Msk
  CAN_F9R1_FB19_Pos* = (19)
  CAN_F9R1_FB19_Msk* = (0x00000001 shl CAN_F9R1_FB19_Pos) ## !< 0x00080000
  CAN_F9R1_FB19* = CAN_F9R1_FB19_Msk
  CAN_F9R1_FB20_Pos* = (20)
  CAN_F9R1_FB20_Msk* = (0x00000001 shl CAN_F9R1_FB20_Pos) ## !< 0x00100000
  CAN_F9R1_FB20* = CAN_F9R1_FB20_Msk
  CAN_F9R1_FB21_Pos* = (21)
  CAN_F9R1_FB21_Msk* = (0x00000001 shl CAN_F9R1_FB21_Pos) ## !< 0x00200000
  CAN_F9R1_FB21* = CAN_F9R1_FB21_Msk
  CAN_F9R1_FB22_Pos* = (22)
  CAN_F9R1_FB22_Msk* = (0x00000001 shl CAN_F9R1_FB22_Pos) ## !< 0x00400000
  CAN_F9R1_FB22* = CAN_F9R1_FB22_Msk
  CAN_F9R1_FB23_Pos* = (23)
  CAN_F9R1_FB23_Msk* = (0x00000001 shl CAN_F9R1_FB23_Pos) ## !< 0x00800000
  CAN_F9R1_FB23* = CAN_F9R1_FB23_Msk
  CAN_F9R1_FB24_Pos* = (24)
  CAN_F9R1_FB24_Msk* = (0x00000001 shl CAN_F9R1_FB24_Pos) ## !< 0x01000000
  CAN_F9R1_FB24* = CAN_F9R1_FB24_Msk
  CAN_F9R1_FB25_Pos* = (25)
  CAN_F9R1_FB25_Msk* = (0x00000001 shl CAN_F9R1_FB25_Pos) ## !< 0x02000000
  CAN_F9R1_FB25* = CAN_F9R1_FB25_Msk
  CAN_F9R1_FB26_Pos* = (26)
  CAN_F9R1_FB26_Msk* = (0x00000001 shl CAN_F9R1_FB26_Pos) ## !< 0x04000000
  CAN_F9R1_FB26* = CAN_F9R1_FB26_Msk
  CAN_F9R1_FB27_Pos* = (27)
  CAN_F9R1_FB27_Msk* = (0x00000001 shl CAN_F9R1_FB27_Pos) ## !< 0x08000000
  CAN_F9R1_FB27* = CAN_F9R1_FB27_Msk
  CAN_F9R1_FB28_Pos* = (28)
  CAN_F9R1_FB28_Msk* = (0x00000001 shl CAN_F9R1_FB28_Pos) ## !< 0x10000000
  CAN_F9R1_FB28* = CAN_F9R1_FB28_Msk
  CAN_F9R1_FB29_Pos* = (29)
  CAN_F9R1_FB29_Msk* = (0x00000001 shl CAN_F9R1_FB29_Pos) ## !< 0x20000000
  CAN_F9R1_FB29* = CAN_F9R1_FB29_Msk
  CAN_F9R1_FB30_Pos* = (30)
  CAN_F9R1_FB30_Msk* = (0x00000001 shl CAN_F9R1_FB30_Pos) ## !< 0x40000000
  CAN_F9R1_FB30* = CAN_F9R1_FB30_Msk
  CAN_F9R1_FB31_Pos* = (31)
  CAN_F9R1_FB31_Msk* = (0x00000001 shl CAN_F9R1_FB31_Pos) ## !< 0x80000000
  CAN_F9R1_FB31* = CAN_F9R1_FB31_Msk

## ******************  Bit definition for CAN_F10R1 register  *****************

const
  CAN_F10R1_FB0_Pos* = (0)
  CAN_F10R1_FB0_Msk* = (0x00000001 shl CAN_F10R1_FB0_Pos) ## !< 0x00000001
  CAN_F10R1_FB0* = CAN_F10R1_FB0_Msk
  CAN_F10R1_FB1_Pos* = (1)
  CAN_F10R1_FB1_Msk* = (0x00000001 shl CAN_F10R1_FB1_Pos) ## !< 0x00000002
  CAN_F10R1_FB1* = CAN_F10R1_FB1_Msk
  CAN_F10R1_FB2_Pos* = (2)
  CAN_F10R1_FB2_Msk* = (0x00000001 shl CAN_F10R1_FB2_Pos) ## !< 0x00000004
  CAN_F10R1_FB2* = CAN_F10R1_FB2_Msk
  CAN_F10R1_FB3_Pos* = (3)
  CAN_F10R1_FB3_Msk* = (0x00000001 shl CAN_F10R1_FB3_Pos) ## !< 0x00000008
  CAN_F10R1_FB3* = CAN_F10R1_FB3_Msk
  CAN_F10R1_FB4_Pos* = (4)
  CAN_F10R1_FB4_Msk* = (0x00000001 shl CAN_F10R1_FB4_Pos) ## !< 0x00000010
  CAN_F10R1_FB4* = CAN_F10R1_FB4_Msk
  CAN_F10R1_FB5_Pos* = (5)
  CAN_F10R1_FB5_Msk* = (0x00000001 shl CAN_F10R1_FB5_Pos) ## !< 0x00000020
  CAN_F10R1_FB5* = CAN_F10R1_FB5_Msk
  CAN_F10R1_FB6_Pos* = (6)
  CAN_F10R1_FB6_Msk* = (0x00000001 shl CAN_F10R1_FB6_Pos) ## !< 0x00000040
  CAN_F10R1_FB6* = CAN_F10R1_FB6_Msk
  CAN_F10R1_FB7_Pos* = (7)
  CAN_F10R1_FB7_Msk* = (0x00000001 shl CAN_F10R1_FB7_Pos) ## !< 0x00000080
  CAN_F10R1_FB7* = CAN_F10R1_FB7_Msk
  CAN_F10R1_FB8_Pos* = (8)
  CAN_F10R1_FB8_Msk* = (0x00000001 shl CAN_F10R1_FB8_Pos) ## !< 0x00000100
  CAN_F10R1_FB8* = CAN_F10R1_FB8_Msk
  CAN_F10R1_FB9_Pos* = (9)
  CAN_F10R1_FB9_Msk* = (0x00000001 shl CAN_F10R1_FB9_Pos) ## !< 0x00000200
  CAN_F10R1_FB9* = CAN_F10R1_FB9_Msk
  CAN_F10R1_FB10_Pos* = (10)
  CAN_F10R1_FB10_Msk* = (0x00000001 shl CAN_F10R1_FB10_Pos) ## !< 0x00000400
  CAN_F10R1_FB10* = CAN_F10R1_FB10_Msk
  CAN_F10R1_FB11_Pos* = (11)
  CAN_F10R1_FB11_Msk* = (0x00000001 shl CAN_F10R1_FB11_Pos) ## !< 0x00000800
  CAN_F10R1_FB11* = CAN_F10R1_FB11_Msk
  CAN_F10R1_FB12_Pos* = (12)
  CAN_F10R1_FB12_Msk* = (0x00000001 shl CAN_F10R1_FB12_Pos) ## !< 0x00001000
  CAN_F10R1_FB12* = CAN_F10R1_FB12_Msk
  CAN_F10R1_FB13_Pos* = (13)
  CAN_F10R1_FB13_Msk* = (0x00000001 shl CAN_F10R1_FB13_Pos) ## !< 0x00002000
  CAN_F10R1_FB13* = CAN_F10R1_FB13_Msk
  CAN_F10R1_FB14_Pos* = (14)
  CAN_F10R1_FB14_Msk* = (0x00000001 shl CAN_F10R1_FB14_Pos) ## !< 0x00004000
  CAN_F10R1_FB14* = CAN_F10R1_FB14_Msk
  CAN_F10R1_FB15_Pos* = (15)
  CAN_F10R1_FB15_Msk* = (0x00000001 shl CAN_F10R1_FB15_Pos) ## !< 0x00008000
  CAN_F10R1_FB15* = CAN_F10R1_FB15_Msk
  CAN_F10R1_FB16_Pos* = (16)
  CAN_F10R1_FB16_Msk* = (0x00000001 shl CAN_F10R1_FB16_Pos) ## !< 0x00010000
  CAN_F10R1_FB16* = CAN_F10R1_FB16_Msk
  CAN_F10R1_FB17_Pos* = (17)
  CAN_F10R1_FB17_Msk* = (0x00000001 shl CAN_F10R1_FB17_Pos) ## !< 0x00020000
  CAN_F10R1_FB17* = CAN_F10R1_FB17_Msk
  CAN_F10R1_FB18_Pos* = (18)
  CAN_F10R1_FB18_Msk* = (0x00000001 shl CAN_F10R1_FB18_Pos) ## !< 0x00040000
  CAN_F10R1_FB18* = CAN_F10R1_FB18_Msk
  CAN_F10R1_FB19_Pos* = (19)
  CAN_F10R1_FB19_Msk* = (0x00000001 shl CAN_F10R1_FB19_Pos) ## !< 0x00080000
  CAN_F10R1_FB19* = CAN_F10R1_FB19_Msk
  CAN_F10R1_FB20_Pos* = (20)
  CAN_F10R1_FB20_Msk* = (0x00000001 shl CAN_F10R1_FB20_Pos) ## !< 0x00100000
  CAN_F10R1_FB20* = CAN_F10R1_FB20_Msk
  CAN_F10R1_FB21_Pos* = (21)
  CAN_F10R1_FB21_Msk* = (0x00000001 shl CAN_F10R1_FB21_Pos) ## !< 0x00200000
  CAN_F10R1_FB21* = CAN_F10R1_FB21_Msk
  CAN_F10R1_FB22_Pos* = (22)
  CAN_F10R1_FB22_Msk* = (0x00000001 shl CAN_F10R1_FB22_Pos) ## !< 0x00400000
  CAN_F10R1_FB22* = CAN_F10R1_FB22_Msk
  CAN_F10R1_FB23_Pos* = (23)
  CAN_F10R1_FB23_Msk* = (0x00000001 shl CAN_F10R1_FB23_Pos) ## !< 0x00800000
  CAN_F10R1_FB23* = CAN_F10R1_FB23_Msk
  CAN_F10R1_FB24_Pos* = (24)
  CAN_F10R1_FB24_Msk* = (0x00000001 shl CAN_F10R1_FB24_Pos) ## !< 0x01000000
  CAN_F10R1_FB24* = CAN_F10R1_FB24_Msk
  CAN_F10R1_FB25_Pos* = (25)
  CAN_F10R1_FB25_Msk* = (0x00000001 shl CAN_F10R1_FB25_Pos) ## !< 0x02000000
  CAN_F10R1_FB25* = CAN_F10R1_FB25_Msk
  CAN_F10R1_FB26_Pos* = (26)
  CAN_F10R1_FB26_Msk* = (0x00000001 shl CAN_F10R1_FB26_Pos) ## !< 0x04000000
  CAN_F10R1_FB26* = CAN_F10R1_FB26_Msk
  CAN_F10R1_FB27_Pos* = (27)
  CAN_F10R1_FB27_Msk* = (0x00000001 shl CAN_F10R1_FB27_Pos) ## !< 0x08000000
  CAN_F10R1_FB27* = CAN_F10R1_FB27_Msk
  CAN_F10R1_FB28_Pos* = (28)
  CAN_F10R1_FB28_Msk* = (0x00000001 shl CAN_F10R1_FB28_Pos) ## !< 0x10000000
  CAN_F10R1_FB28* = CAN_F10R1_FB28_Msk
  CAN_F10R1_FB29_Pos* = (29)
  CAN_F10R1_FB29_Msk* = (0x00000001 shl CAN_F10R1_FB29_Pos) ## !< 0x20000000
  CAN_F10R1_FB29* = CAN_F10R1_FB29_Msk
  CAN_F10R1_FB30_Pos* = (30)
  CAN_F10R1_FB30_Msk* = (0x00000001 shl CAN_F10R1_FB30_Pos) ## !< 0x40000000
  CAN_F10R1_FB30* = CAN_F10R1_FB30_Msk
  CAN_F10R1_FB31_Pos* = (31)
  CAN_F10R1_FB31_Msk* = (0x00000001 shl CAN_F10R1_FB31_Pos) ## !< 0x80000000
  CAN_F10R1_FB31* = CAN_F10R1_FB31_Msk

## ******************  Bit definition for CAN_F11R1 register  *****************

const
  CAN_F11R1_FB0_Pos* = (0)
  CAN_F11R1_FB0_Msk* = (0x00000001 shl CAN_F11R1_FB0_Pos) ## !< 0x00000001
  CAN_F11R1_FB0* = CAN_F11R1_FB0_Msk
  CAN_F11R1_FB1_Pos* = (1)
  CAN_F11R1_FB1_Msk* = (0x00000001 shl CAN_F11R1_FB1_Pos) ## !< 0x00000002
  CAN_F11R1_FB1* = CAN_F11R1_FB1_Msk
  CAN_F11R1_FB2_Pos* = (2)
  CAN_F11R1_FB2_Msk* = (0x00000001 shl CAN_F11R1_FB2_Pos) ## !< 0x00000004
  CAN_F11R1_FB2* = CAN_F11R1_FB2_Msk
  CAN_F11R1_FB3_Pos* = (3)
  CAN_F11R1_FB3_Msk* = (0x00000001 shl CAN_F11R1_FB3_Pos) ## !< 0x00000008
  CAN_F11R1_FB3* = CAN_F11R1_FB3_Msk
  CAN_F11R1_FB4_Pos* = (4)
  CAN_F11R1_FB4_Msk* = (0x00000001 shl CAN_F11R1_FB4_Pos) ## !< 0x00000010
  CAN_F11R1_FB4* = CAN_F11R1_FB4_Msk
  CAN_F11R1_FB5_Pos* = (5)
  CAN_F11R1_FB5_Msk* = (0x00000001 shl CAN_F11R1_FB5_Pos) ## !< 0x00000020
  CAN_F11R1_FB5* = CAN_F11R1_FB5_Msk
  CAN_F11R1_FB6_Pos* = (6)
  CAN_F11R1_FB6_Msk* = (0x00000001 shl CAN_F11R1_FB6_Pos) ## !< 0x00000040
  CAN_F11R1_FB6* = CAN_F11R1_FB6_Msk
  CAN_F11R1_FB7_Pos* = (7)
  CAN_F11R1_FB7_Msk* = (0x00000001 shl CAN_F11R1_FB7_Pos) ## !< 0x00000080
  CAN_F11R1_FB7* = CAN_F11R1_FB7_Msk
  CAN_F11R1_FB8_Pos* = (8)
  CAN_F11R1_FB8_Msk* = (0x00000001 shl CAN_F11R1_FB8_Pos) ## !< 0x00000100
  CAN_F11R1_FB8* = CAN_F11R1_FB8_Msk
  CAN_F11R1_FB9_Pos* = (9)
  CAN_F11R1_FB9_Msk* = (0x00000001 shl CAN_F11R1_FB9_Pos) ## !< 0x00000200
  CAN_F11R1_FB9* = CAN_F11R1_FB9_Msk
  CAN_F11R1_FB10_Pos* = (10)
  CAN_F11R1_FB10_Msk* = (0x00000001 shl CAN_F11R1_FB10_Pos) ## !< 0x00000400
  CAN_F11R1_FB10* = CAN_F11R1_FB10_Msk
  CAN_F11R1_FB11_Pos* = (11)
  CAN_F11R1_FB11_Msk* = (0x00000001 shl CAN_F11R1_FB11_Pos) ## !< 0x00000800
  CAN_F11R1_FB11* = CAN_F11R1_FB11_Msk
  CAN_F11R1_FB12_Pos* = (12)
  CAN_F11R1_FB12_Msk* = (0x00000001 shl CAN_F11R1_FB12_Pos) ## !< 0x00001000
  CAN_F11R1_FB12* = CAN_F11R1_FB12_Msk
  CAN_F11R1_FB13_Pos* = (13)
  CAN_F11R1_FB13_Msk* = (0x00000001 shl CAN_F11R1_FB13_Pos) ## !< 0x00002000
  CAN_F11R1_FB13* = CAN_F11R1_FB13_Msk
  CAN_F11R1_FB14_Pos* = (14)
  CAN_F11R1_FB14_Msk* = (0x00000001 shl CAN_F11R1_FB14_Pos) ## !< 0x00004000
  CAN_F11R1_FB14* = CAN_F11R1_FB14_Msk
  CAN_F11R1_FB15_Pos* = (15)
  CAN_F11R1_FB15_Msk* = (0x00000001 shl CAN_F11R1_FB15_Pos) ## !< 0x00008000
  CAN_F11R1_FB15* = CAN_F11R1_FB15_Msk
  CAN_F11R1_FB16_Pos* = (16)
  CAN_F11R1_FB16_Msk* = (0x00000001 shl CAN_F11R1_FB16_Pos) ## !< 0x00010000
  CAN_F11R1_FB16* = CAN_F11R1_FB16_Msk
  CAN_F11R1_FB17_Pos* = (17)
  CAN_F11R1_FB17_Msk* = (0x00000001 shl CAN_F11R1_FB17_Pos) ## !< 0x00020000
  CAN_F11R1_FB17* = CAN_F11R1_FB17_Msk
  CAN_F11R1_FB18_Pos* = (18)
  CAN_F11R1_FB18_Msk* = (0x00000001 shl CAN_F11R1_FB18_Pos) ## !< 0x00040000
  CAN_F11R1_FB18* = CAN_F11R1_FB18_Msk
  CAN_F11R1_FB19_Pos* = (19)
  CAN_F11R1_FB19_Msk* = (0x00000001 shl CAN_F11R1_FB19_Pos) ## !< 0x00080000
  CAN_F11R1_FB19* = CAN_F11R1_FB19_Msk
  CAN_F11R1_FB20_Pos* = (20)
  CAN_F11R1_FB20_Msk* = (0x00000001 shl CAN_F11R1_FB20_Pos) ## !< 0x00100000
  CAN_F11R1_FB20* = CAN_F11R1_FB20_Msk
  CAN_F11R1_FB21_Pos* = (21)
  CAN_F11R1_FB21_Msk* = (0x00000001 shl CAN_F11R1_FB21_Pos) ## !< 0x00200000
  CAN_F11R1_FB21* = CAN_F11R1_FB21_Msk
  CAN_F11R1_FB22_Pos* = (22)
  CAN_F11R1_FB22_Msk* = (0x00000001 shl CAN_F11R1_FB22_Pos) ## !< 0x00400000
  CAN_F11R1_FB22* = CAN_F11R1_FB22_Msk
  CAN_F11R1_FB23_Pos* = (23)
  CAN_F11R1_FB23_Msk* = (0x00000001 shl CAN_F11R1_FB23_Pos) ## !< 0x00800000
  CAN_F11R1_FB23* = CAN_F11R1_FB23_Msk
  CAN_F11R1_FB24_Pos* = (24)
  CAN_F11R1_FB24_Msk* = (0x00000001 shl CAN_F11R1_FB24_Pos) ## !< 0x01000000
  CAN_F11R1_FB24* = CAN_F11R1_FB24_Msk
  CAN_F11R1_FB25_Pos* = (25)
  CAN_F11R1_FB25_Msk* = (0x00000001 shl CAN_F11R1_FB25_Pos) ## !< 0x02000000
  CAN_F11R1_FB25* = CAN_F11R1_FB25_Msk
  CAN_F11R1_FB26_Pos* = (26)
  CAN_F11R1_FB26_Msk* = (0x00000001 shl CAN_F11R1_FB26_Pos) ## !< 0x04000000
  CAN_F11R1_FB26* = CAN_F11R1_FB26_Msk
  CAN_F11R1_FB27_Pos* = (27)
  CAN_F11R1_FB27_Msk* = (0x00000001 shl CAN_F11R1_FB27_Pos) ## !< 0x08000000
  CAN_F11R1_FB27* = CAN_F11R1_FB27_Msk
  CAN_F11R1_FB28_Pos* = (28)
  CAN_F11R1_FB28_Msk* = (0x00000001 shl CAN_F11R1_FB28_Pos) ## !< 0x10000000
  CAN_F11R1_FB28* = CAN_F11R1_FB28_Msk
  CAN_F11R1_FB29_Pos* = (29)
  CAN_F11R1_FB29_Msk* = (0x00000001 shl CAN_F11R1_FB29_Pos) ## !< 0x20000000
  CAN_F11R1_FB29* = CAN_F11R1_FB29_Msk
  CAN_F11R1_FB30_Pos* = (30)
  CAN_F11R1_FB30_Msk* = (0x00000001 shl CAN_F11R1_FB30_Pos) ## !< 0x40000000
  CAN_F11R1_FB30* = CAN_F11R1_FB30_Msk
  CAN_F11R1_FB31_Pos* = (31)
  CAN_F11R1_FB31_Msk* = (0x00000001 shl CAN_F11R1_FB31_Pos) ## !< 0x80000000
  CAN_F11R1_FB31* = CAN_F11R1_FB31_Msk

## ******************  Bit definition for CAN_F12R1 register  *****************

const
  CAN_F12R1_FB0_Pos* = (0)
  CAN_F12R1_FB0_Msk* = (0x00000001 shl CAN_F12R1_FB0_Pos) ## !< 0x00000001
  CAN_F12R1_FB0* = CAN_F12R1_FB0_Msk
  CAN_F12R1_FB1_Pos* = (1)
  CAN_F12R1_FB1_Msk* = (0x00000001 shl CAN_F12R1_FB1_Pos) ## !< 0x00000002
  CAN_F12R1_FB1* = CAN_F12R1_FB1_Msk
  CAN_F12R1_FB2_Pos* = (2)
  CAN_F12R1_FB2_Msk* = (0x00000001 shl CAN_F12R1_FB2_Pos) ## !< 0x00000004
  CAN_F12R1_FB2* = CAN_F12R1_FB2_Msk
  CAN_F12R1_FB3_Pos* = (3)
  CAN_F12R1_FB3_Msk* = (0x00000001 shl CAN_F12R1_FB3_Pos) ## !< 0x00000008
  CAN_F12R1_FB3* = CAN_F12R1_FB3_Msk
  CAN_F12R1_FB4_Pos* = (4)
  CAN_F12R1_FB4_Msk* = (0x00000001 shl CAN_F12R1_FB4_Pos) ## !< 0x00000010
  CAN_F12R1_FB4* = CAN_F12R1_FB4_Msk
  CAN_F12R1_FB5_Pos* = (5)
  CAN_F12R1_FB5_Msk* = (0x00000001 shl CAN_F12R1_FB5_Pos) ## !< 0x00000020
  CAN_F12R1_FB5* = CAN_F12R1_FB5_Msk
  CAN_F12R1_FB6_Pos* = (6)
  CAN_F12R1_FB6_Msk* = (0x00000001 shl CAN_F12R1_FB6_Pos) ## !< 0x00000040
  CAN_F12R1_FB6* = CAN_F12R1_FB6_Msk
  CAN_F12R1_FB7_Pos* = (7)
  CAN_F12R1_FB7_Msk* = (0x00000001 shl CAN_F12R1_FB7_Pos) ## !< 0x00000080
  CAN_F12R1_FB7* = CAN_F12R1_FB7_Msk
  CAN_F12R1_FB8_Pos* = (8)
  CAN_F12R1_FB8_Msk* = (0x00000001 shl CAN_F12R1_FB8_Pos) ## !< 0x00000100
  CAN_F12R1_FB8* = CAN_F12R1_FB8_Msk
  CAN_F12R1_FB9_Pos* = (9)
  CAN_F12R1_FB9_Msk* = (0x00000001 shl CAN_F12R1_FB9_Pos) ## !< 0x00000200
  CAN_F12R1_FB9* = CAN_F12R1_FB9_Msk
  CAN_F12R1_FB10_Pos* = (10)
  CAN_F12R1_FB10_Msk* = (0x00000001 shl CAN_F12R1_FB10_Pos) ## !< 0x00000400
  CAN_F12R1_FB10* = CAN_F12R1_FB10_Msk
  CAN_F12R1_FB11_Pos* = (11)
  CAN_F12R1_FB11_Msk* = (0x00000001 shl CAN_F12R1_FB11_Pos) ## !< 0x00000800
  CAN_F12R1_FB11* = CAN_F12R1_FB11_Msk
  CAN_F12R1_FB12_Pos* = (12)
  CAN_F12R1_FB12_Msk* = (0x00000001 shl CAN_F12R1_FB12_Pos) ## !< 0x00001000
  CAN_F12R1_FB12* = CAN_F12R1_FB12_Msk
  CAN_F12R1_FB13_Pos* = (13)
  CAN_F12R1_FB13_Msk* = (0x00000001 shl CAN_F12R1_FB13_Pos) ## !< 0x00002000
  CAN_F12R1_FB13* = CAN_F12R1_FB13_Msk
  CAN_F12R1_FB14_Pos* = (14)
  CAN_F12R1_FB14_Msk* = (0x00000001 shl CAN_F12R1_FB14_Pos) ## !< 0x00004000
  CAN_F12R1_FB14* = CAN_F12R1_FB14_Msk
  CAN_F12R1_FB15_Pos* = (15)
  CAN_F12R1_FB15_Msk* = (0x00000001 shl CAN_F12R1_FB15_Pos) ## !< 0x00008000
  CAN_F12R1_FB15* = CAN_F12R1_FB15_Msk
  CAN_F12R1_FB16_Pos* = (16)
  CAN_F12R1_FB16_Msk* = (0x00000001 shl CAN_F12R1_FB16_Pos) ## !< 0x00010000
  CAN_F12R1_FB16* = CAN_F12R1_FB16_Msk
  CAN_F12R1_FB17_Pos* = (17)
  CAN_F12R1_FB17_Msk* = (0x00000001 shl CAN_F12R1_FB17_Pos) ## !< 0x00020000
  CAN_F12R1_FB17* = CAN_F12R1_FB17_Msk
  CAN_F12R1_FB18_Pos* = (18)
  CAN_F12R1_FB18_Msk* = (0x00000001 shl CAN_F12R1_FB18_Pos) ## !< 0x00040000
  CAN_F12R1_FB18* = CAN_F12R1_FB18_Msk
  CAN_F12R1_FB19_Pos* = (19)
  CAN_F12R1_FB19_Msk* = (0x00000001 shl CAN_F12R1_FB19_Pos) ## !< 0x00080000
  CAN_F12R1_FB19* = CAN_F12R1_FB19_Msk
  CAN_F12R1_FB20_Pos* = (20)
  CAN_F12R1_FB20_Msk* = (0x00000001 shl CAN_F12R1_FB20_Pos) ## !< 0x00100000
  CAN_F12R1_FB20* = CAN_F12R1_FB20_Msk
  CAN_F12R1_FB21_Pos* = (21)
  CAN_F12R1_FB21_Msk* = (0x00000001 shl CAN_F12R1_FB21_Pos) ## !< 0x00200000
  CAN_F12R1_FB21* = CAN_F12R1_FB21_Msk
  CAN_F12R1_FB22_Pos* = (22)
  CAN_F12R1_FB22_Msk* = (0x00000001 shl CAN_F12R1_FB22_Pos) ## !< 0x00400000
  CAN_F12R1_FB22* = CAN_F12R1_FB22_Msk
  CAN_F12R1_FB23_Pos* = (23)
  CAN_F12R1_FB23_Msk* = (0x00000001 shl CAN_F12R1_FB23_Pos) ## !< 0x00800000
  CAN_F12R1_FB23* = CAN_F12R1_FB23_Msk
  CAN_F12R1_FB24_Pos* = (24)
  CAN_F12R1_FB24_Msk* = (0x00000001 shl CAN_F12R1_FB24_Pos) ## !< 0x01000000
  CAN_F12R1_FB24* = CAN_F12R1_FB24_Msk
  CAN_F12R1_FB25_Pos* = (25)
  CAN_F12R1_FB25_Msk* = (0x00000001 shl CAN_F12R1_FB25_Pos) ## !< 0x02000000
  CAN_F12R1_FB25* = CAN_F12R1_FB25_Msk
  CAN_F12R1_FB26_Pos* = (26)
  CAN_F12R1_FB26_Msk* = (0x00000001 shl CAN_F12R1_FB26_Pos) ## !< 0x04000000
  CAN_F12R1_FB26* = CAN_F12R1_FB26_Msk
  CAN_F12R1_FB27_Pos* = (27)
  CAN_F12R1_FB27_Msk* = (0x00000001 shl CAN_F12R1_FB27_Pos) ## !< 0x08000000
  CAN_F12R1_FB27* = CAN_F12R1_FB27_Msk
  CAN_F12R1_FB28_Pos* = (28)
  CAN_F12R1_FB28_Msk* = (0x00000001 shl CAN_F12R1_FB28_Pos) ## !< 0x10000000
  CAN_F12R1_FB28* = CAN_F12R1_FB28_Msk
  CAN_F12R1_FB29_Pos* = (29)
  CAN_F12R1_FB29_Msk* = (0x00000001 shl CAN_F12R1_FB29_Pos) ## !< 0x20000000
  CAN_F12R1_FB29* = CAN_F12R1_FB29_Msk
  CAN_F12R1_FB30_Pos* = (30)
  CAN_F12R1_FB30_Msk* = (0x00000001 shl CAN_F12R1_FB30_Pos) ## !< 0x40000000
  CAN_F12R1_FB30* = CAN_F12R1_FB30_Msk
  CAN_F12R1_FB31_Pos* = (31)
  CAN_F12R1_FB31_Msk* = (0x00000001 shl CAN_F12R1_FB31_Pos) ## !< 0x80000000
  CAN_F12R1_FB31* = CAN_F12R1_FB31_Msk

## ******************  Bit definition for CAN_F13R1 register  *****************

const
  CAN_F13R1_FB0_Pos* = (0)
  CAN_F13R1_FB0_Msk* = (0x00000001 shl CAN_F13R1_FB0_Pos) ## !< 0x00000001
  CAN_F13R1_FB0* = CAN_F13R1_FB0_Msk
  CAN_F13R1_FB1_Pos* = (1)
  CAN_F13R1_FB1_Msk* = (0x00000001 shl CAN_F13R1_FB1_Pos) ## !< 0x00000002
  CAN_F13R1_FB1* = CAN_F13R1_FB1_Msk
  CAN_F13R1_FB2_Pos* = (2)
  CAN_F13R1_FB2_Msk* = (0x00000001 shl CAN_F13R1_FB2_Pos) ## !< 0x00000004
  CAN_F13R1_FB2* = CAN_F13R1_FB2_Msk
  CAN_F13R1_FB3_Pos* = (3)
  CAN_F13R1_FB3_Msk* = (0x00000001 shl CAN_F13R1_FB3_Pos) ## !< 0x00000008
  CAN_F13R1_FB3* = CAN_F13R1_FB3_Msk
  CAN_F13R1_FB4_Pos* = (4)
  CAN_F13R1_FB4_Msk* = (0x00000001 shl CAN_F13R1_FB4_Pos) ## !< 0x00000010
  CAN_F13R1_FB4* = CAN_F13R1_FB4_Msk
  CAN_F13R1_FB5_Pos* = (5)
  CAN_F13R1_FB5_Msk* = (0x00000001 shl CAN_F13R1_FB5_Pos) ## !< 0x00000020
  CAN_F13R1_FB5* = CAN_F13R1_FB5_Msk
  CAN_F13R1_FB6_Pos* = (6)
  CAN_F13R1_FB6_Msk* = (0x00000001 shl CAN_F13R1_FB6_Pos) ## !< 0x00000040
  CAN_F13R1_FB6* = CAN_F13R1_FB6_Msk
  CAN_F13R1_FB7_Pos* = (7)
  CAN_F13R1_FB7_Msk* = (0x00000001 shl CAN_F13R1_FB7_Pos) ## !< 0x00000080
  CAN_F13R1_FB7* = CAN_F13R1_FB7_Msk
  CAN_F13R1_FB8_Pos* = (8)
  CAN_F13R1_FB8_Msk* = (0x00000001 shl CAN_F13R1_FB8_Pos) ## !< 0x00000100
  CAN_F13R1_FB8* = CAN_F13R1_FB8_Msk
  CAN_F13R1_FB9_Pos* = (9)
  CAN_F13R1_FB9_Msk* = (0x00000001 shl CAN_F13R1_FB9_Pos) ## !< 0x00000200
  CAN_F13R1_FB9* = CAN_F13R1_FB9_Msk
  CAN_F13R1_FB10_Pos* = (10)
  CAN_F13R1_FB10_Msk* = (0x00000001 shl CAN_F13R1_FB10_Pos) ## !< 0x00000400
  CAN_F13R1_FB10* = CAN_F13R1_FB10_Msk
  CAN_F13R1_FB11_Pos* = (11)
  CAN_F13R1_FB11_Msk* = (0x00000001 shl CAN_F13R1_FB11_Pos) ## !< 0x00000800
  CAN_F13R1_FB11* = CAN_F13R1_FB11_Msk
  CAN_F13R1_FB12_Pos* = (12)
  CAN_F13R1_FB12_Msk* = (0x00000001 shl CAN_F13R1_FB12_Pos) ## !< 0x00001000
  CAN_F13R1_FB12* = CAN_F13R1_FB12_Msk
  CAN_F13R1_FB13_Pos* = (13)
  CAN_F13R1_FB13_Msk* = (0x00000001 shl CAN_F13R1_FB13_Pos) ## !< 0x00002000
  CAN_F13R1_FB13* = CAN_F13R1_FB13_Msk
  CAN_F13R1_FB14_Pos* = (14)
  CAN_F13R1_FB14_Msk* = (0x00000001 shl CAN_F13R1_FB14_Pos) ## !< 0x00004000
  CAN_F13R1_FB14* = CAN_F13R1_FB14_Msk
  CAN_F13R1_FB15_Pos* = (15)
  CAN_F13R1_FB15_Msk* = (0x00000001 shl CAN_F13R1_FB15_Pos) ## !< 0x00008000
  CAN_F13R1_FB15* = CAN_F13R1_FB15_Msk
  CAN_F13R1_FB16_Pos* = (16)
  CAN_F13R1_FB16_Msk* = (0x00000001 shl CAN_F13R1_FB16_Pos) ## !< 0x00010000
  CAN_F13R1_FB16* = CAN_F13R1_FB16_Msk
  CAN_F13R1_FB17_Pos* = (17)
  CAN_F13R1_FB17_Msk* = (0x00000001 shl CAN_F13R1_FB17_Pos) ## !< 0x00020000
  CAN_F13R1_FB17* = CAN_F13R1_FB17_Msk
  CAN_F13R1_FB18_Pos* = (18)
  CAN_F13R1_FB18_Msk* = (0x00000001 shl CAN_F13R1_FB18_Pos) ## !< 0x00040000
  CAN_F13R1_FB18* = CAN_F13R1_FB18_Msk
  CAN_F13R1_FB19_Pos* = (19)
  CAN_F13R1_FB19_Msk* = (0x00000001 shl CAN_F13R1_FB19_Pos) ## !< 0x00080000
  CAN_F13R1_FB19* = CAN_F13R1_FB19_Msk
  CAN_F13R1_FB20_Pos* = (20)
  CAN_F13R1_FB20_Msk* = (0x00000001 shl CAN_F13R1_FB20_Pos) ## !< 0x00100000
  CAN_F13R1_FB20* = CAN_F13R1_FB20_Msk
  CAN_F13R1_FB21_Pos* = (21)
  CAN_F13R1_FB21_Msk* = (0x00000001 shl CAN_F13R1_FB21_Pos) ## !< 0x00200000
  CAN_F13R1_FB21* = CAN_F13R1_FB21_Msk
  CAN_F13R1_FB22_Pos* = (22)
  CAN_F13R1_FB22_Msk* = (0x00000001 shl CAN_F13R1_FB22_Pos) ## !< 0x00400000
  CAN_F13R1_FB22* = CAN_F13R1_FB22_Msk
  CAN_F13R1_FB23_Pos* = (23)
  CAN_F13R1_FB23_Msk* = (0x00000001 shl CAN_F13R1_FB23_Pos) ## !< 0x00800000
  CAN_F13R1_FB23* = CAN_F13R1_FB23_Msk
  CAN_F13R1_FB24_Pos* = (24)
  CAN_F13R1_FB24_Msk* = (0x00000001 shl CAN_F13R1_FB24_Pos) ## !< 0x01000000
  CAN_F13R1_FB24* = CAN_F13R1_FB24_Msk
  CAN_F13R1_FB25_Pos* = (25)
  CAN_F13R1_FB25_Msk* = (0x00000001 shl CAN_F13R1_FB25_Pos) ## !< 0x02000000
  CAN_F13R1_FB25* = CAN_F13R1_FB25_Msk
  CAN_F13R1_FB26_Pos* = (26)
  CAN_F13R1_FB26_Msk* = (0x00000001 shl CAN_F13R1_FB26_Pos) ## !< 0x04000000
  CAN_F13R1_FB26* = CAN_F13R1_FB26_Msk
  CAN_F13R1_FB27_Pos* = (27)
  CAN_F13R1_FB27_Msk* = (0x00000001 shl CAN_F13R1_FB27_Pos) ## !< 0x08000000
  CAN_F13R1_FB27* = CAN_F13R1_FB27_Msk
  CAN_F13R1_FB28_Pos* = (28)
  CAN_F13R1_FB28_Msk* = (0x00000001 shl CAN_F13R1_FB28_Pos) ## !< 0x10000000
  CAN_F13R1_FB28* = CAN_F13R1_FB28_Msk
  CAN_F13R1_FB29_Pos* = (29)
  CAN_F13R1_FB29_Msk* = (0x00000001 shl CAN_F13R1_FB29_Pos) ## !< 0x20000000
  CAN_F13R1_FB29* = CAN_F13R1_FB29_Msk
  CAN_F13R1_FB30_Pos* = (30)
  CAN_F13R1_FB30_Msk* = (0x00000001 shl CAN_F13R1_FB30_Pos) ## !< 0x40000000
  CAN_F13R1_FB30* = CAN_F13R1_FB30_Msk
  CAN_F13R1_FB31_Pos* = (31)
  CAN_F13R1_FB31_Msk* = (0x00000001 shl CAN_F13R1_FB31_Pos) ## !< 0x80000000
  CAN_F13R1_FB31* = CAN_F13R1_FB31_Msk

## ******************  Bit definition for CAN_F0R2 register  ******************

const
  CAN_F0R2_FB0_Pos* = (0)
  CAN_F0R2_FB0_Msk* = (0x00000001 shl CAN_F0R2_FB0_Pos) ## !< 0x00000001
  CAN_F0R2_FB0* = CAN_F0R2_FB0_Msk
  CAN_F0R2_FB1_Pos* = (1)
  CAN_F0R2_FB1_Msk* = (0x00000001 shl CAN_F0R2_FB1_Pos) ## !< 0x00000002
  CAN_F0R2_FB1* = CAN_F0R2_FB1_Msk
  CAN_F0R2_FB2_Pos* = (2)
  CAN_F0R2_FB2_Msk* = (0x00000001 shl CAN_F0R2_FB2_Pos) ## !< 0x00000004
  CAN_F0R2_FB2* = CAN_F0R2_FB2_Msk
  CAN_F0R2_FB3_Pos* = (3)
  CAN_F0R2_FB3_Msk* = (0x00000001 shl CAN_F0R2_FB3_Pos) ## !< 0x00000008
  CAN_F0R2_FB3* = CAN_F0R2_FB3_Msk
  CAN_F0R2_FB4_Pos* = (4)
  CAN_F0R2_FB4_Msk* = (0x00000001 shl CAN_F0R2_FB4_Pos) ## !< 0x00000010
  CAN_F0R2_FB4* = CAN_F0R2_FB4_Msk
  CAN_F0R2_FB5_Pos* = (5)
  CAN_F0R2_FB5_Msk* = (0x00000001 shl CAN_F0R2_FB5_Pos) ## !< 0x00000020
  CAN_F0R2_FB5* = CAN_F0R2_FB5_Msk
  CAN_F0R2_FB6_Pos* = (6)
  CAN_F0R2_FB6_Msk* = (0x00000001 shl CAN_F0R2_FB6_Pos) ## !< 0x00000040
  CAN_F0R2_FB6* = CAN_F0R2_FB6_Msk
  CAN_F0R2_FB7_Pos* = (7)
  CAN_F0R2_FB7_Msk* = (0x00000001 shl CAN_F0R2_FB7_Pos) ## !< 0x00000080
  CAN_F0R2_FB7* = CAN_F0R2_FB7_Msk
  CAN_F0R2_FB8_Pos* = (8)
  CAN_F0R2_FB8_Msk* = (0x00000001 shl CAN_F0R2_FB8_Pos) ## !< 0x00000100
  CAN_F0R2_FB8* = CAN_F0R2_FB8_Msk
  CAN_F0R2_FB9_Pos* = (9)
  CAN_F0R2_FB9_Msk* = (0x00000001 shl CAN_F0R2_FB9_Pos) ## !< 0x00000200
  CAN_F0R2_FB9* = CAN_F0R2_FB9_Msk
  CAN_F0R2_FB10_Pos* = (10)
  CAN_F0R2_FB10_Msk* = (0x00000001 shl CAN_F0R2_FB10_Pos) ## !< 0x00000400
  CAN_F0R2_FB10* = CAN_F0R2_FB10_Msk
  CAN_F0R2_FB11_Pos* = (11)
  CAN_F0R2_FB11_Msk* = (0x00000001 shl CAN_F0R2_FB11_Pos) ## !< 0x00000800
  CAN_F0R2_FB11* = CAN_F0R2_FB11_Msk
  CAN_F0R2_FB12_Pos* = (12)
  CAN_F0R2_FB12_Msk* = (0x00000001 shl CAN_F0R2_FB12_Pos) ## !< 0x00001000
  CAN_F0R2_FB12* = CAN_F0R2_FB12_Msk
  CAN_F0R2_FB13_Pos* = (13)
  CAN_F0R2_FB13_Msk* = (0x00000001 shl CAN_F0R2_FB13_Pos) ## !< 0x00002000
  CAN_F0R2_FB13* = CAN_F0R2_FB13_Msk
  CAN_F0R2_FB14_Pos* = (14)
  CAN_F0R2_FB14_Msk* = (0x00000001 shl CAN_F0R2_FB14_Pos) ## !< 0x00004000
  CAN_F0R2_FB14* = CAN_F0R2_FB14_Msk
  CAN_F0R2_FB15_Pos* = (15)
  CAN_F0R2_FB15_Msk* = (0x00000001 shl CAN_F0R2_FB15_Pos) ## !< 0x00008000
  CAN_F0R2_FB15* = CAN_F0R2_FB15_Msk
  CAN_F0R2_FB16_Pos* = (16)
  CAN_F0R2_FB16_Msk* = (0x00000001 shl CAN_F0R2_FB16_Pos) ## !< 0x00010000
  CAN_F0R2_FB16* = CAN_F0R2_FB16_Msk
  CAN_F0R2_FB17_Pos* = (17)
  CAN_F0R2_FB17_Msk* = (0x00000001 shl CAN_F0R2_FB17_Pos) ## !< 0x00020000
  CAN_F0R2_FB17* = CAN_F0R2_FB17_Msk
  CAN_F0R2_FB18_Pos* = (18)
  CAN_F0R2_FB18_Msk* = (0x00000001 shl CAN_F0R2_FB18_Pos) ## !< 0x00040000
  CAN_F0R2_FB18* = CAN_F0R2_FB18_Msk
  CAN_F0R2_FB19_Pos* = (19)
  CAN_F0R2_FB19_Msk* = (0x00000001 shl CAN_F0R2_FB19_Pos) ## !< 0x00080000
  CAN_F0R2_FB19* = CAN_F0R2_FB19_Msk
  CAN_F0R2_FB20_Pos* = (20)
  CAN_F0R2_FB20_Msk* = (0x00000001 shl CAN_F0R2_FB20_Pos) ## !< 0x00100000
  CAN_F0R2_FB20* = CAN_F0R2_FB20_Msk
  CAN_F0R2_FB21_Pos* = (21)
  CAN_F0R2_FB21_Msk* = (0x00000001 shl CAN_F0R2_FB21_Pos) ## !< 0x00200000
  CAN_F0R2_FB21* = CAN_F0R2_FB21_Msk
  CAN_F0R2_FB22_Pos* = (22)
  CAN_F0R2_FB22_Msk* = (0x00000001 shl CAN_F0R2_FB22_Pos) ## !< 0x00400000
  CAN_F0R2_FB22* = CAN_F0R2_FB22_Msk
  CAN_F0R2_FB23_Pos* = (23)
  CAN_F0R2_FB23_Msk* = (0x00000001 shl CAN_F0R2_FB23_Pos) ## !< 0x00800000
  CAN_F0R2_FB23* = CAN_F0R2_FB23_Msk
  CAN_F0R2_FB24_Pos* = (24)
  CAN_F0R2_FB24_Msk* = (0x00000001 shl CAN_F0R2_FB24_Pos) ## !< 0x01000000
  CAN_F0R2_FB24* = CAN_F0R2_FB24_Msk
  CAN_F0R2_FB25_Pos* = (25)
  CAN_F0R2_FB25_Msk* = (0x00000001 shl CAN_F0R2_FB25_Pos) ## !< 0x02000000
  CAN_F0R2_FB25* = CAN_F0R2_FB25_Msk
  CAN_F0R2_FB26_Pos* = (26)
  CAN_F0R2_FB26_Msk* = (0x00000001 shl CAN_F0R2_FB26_Pos) ## !< 0x04000000
  CAN_F0R2_FB26* = CAN_F0R2_FB26_Msk
  CAN_F0R2_FB27_Pos* = (27)
  CAN_F0R2_FB27_Msk* = (0x00000001 shl CAN_F0R2_FB27_Pos) ## !< 0x08000000
  CAN_F0R2_FB27* = CAN_F0R2_FB27_Msk
  CAN_F0R2_FB28_Pos* = (28)
  CAN_F0R2_FB28_Msk* = (0x00000001 shl CAN_F0R2_FB28_Pos) ## !< 0x10000000
  CAN_F0R2_FB28* = CAN_F0R2_FB28_Msk
  CAN_F0R2_FB29_Pos* = (29)
  CAN_F0R2_FB29_Msk* = (0x00000001 shl CAN_F0R2_FB29_Pos) ## !< 0x20000000
  CAN_F0R2_FB29* = CAN_F0R2_FB29_Msk
  CAN_F0R2_FB30_Pos* = (30)
  CAN_F0R2_FB30_Msk* = (0x00000001 shl CAN_F0R2_FB30_Pos) ## !< 0x40000000
  CAN_F0R2_FB30* = CAN_F0R2_FB30_Msk
  CAN_F0R2_FB31_Pos* = (31)
  CAN_F0R2_FB31_Msk* = (0x00000001 shl CAN_F0R2_FB31_Pos) ## !< 0x80000000
  CAN_F0R2_FB31* = CAN_F0R2_FB31_Msk

## ******************  Bit definition for CAN_F1R2 register  ******************

const
  CAN_F1R2_FB0_Pos* = (0)
  CAN_F1R2_FB0_Msk* = (0x00000001 shl CAN_F1R2_FB0_Pos) ## !< 0x00000001
  CAN_F1R2_FB0* = CAN_F1R2_FB0_Msk
  CAN_F1R2_FB1_Pos* = (1)
  CAN_F1R2_FB1_Msk* = (0x00000001 shl CAN_F1R2_FB1_Pos) ## !< 0x00000002
  CAN_F1R2_FB1* = CAN_F1R2_FB1_Msk
  CAN_F1R2_FB2_Pos* = (2)
  CAN_F1R2_FB2_Msk* = (0x00000001 shl CAN_F1R2_FB2_Pos) ## !< 0x00000004
  CAN_F1R2_FB2* = CAN_F1R2_FB2_Msk
  CAN_F1R2_FB3_Pos* = (3)
  CAN_F1R2_FB3_Msk* = (0x00000001 shl CAN_F1R2_FB3_Pos) ## !< 0x00000008
  CAN_F1R2_FB3* = CAN_F1R2_FB3_Msk
  CAN_F1R2_FB4_Pos* = (4)
  CAN_F1R2_FB4_Msk* = (0x00000001 shl CAN_F1R2_FB4_Pos) ## !< 0x00000010
  CAN_F1R2_FB4* = CAN_F1R2_FB4_Msk
  CAN_F1R2_FB5_Pos* = (5)
  CAN_F1R2_FB5_Msk* = (0x00000001 shl CAN_F1R2_FB5_Pos) ## !< 0x00000020
  CAN_F1R2_FB5* = CAN_F1R2_FB5_Msk
  CAN_F1R2_FB6_Pos* = (6)
  CAN_F1R2_FB6_Msk* = (0x00000001 shl CAN_F1R2_FB6_Pos) ## !< 0x00000040
  CAN_F1R2_FB6* = CAN_F1R2_FB6_Msk
  CAN_F1R2_FB7_Pos* = (7)
  CAN_F1R2_FB7_Msk* = (0x00000001 shl CAN_F1R2_FB7_Pos) ## !< 0x00000080
  CAN_F1R2_FB7* = CAN_F1R2_FB7_Msk
  CAN_F1R2_FB8_Pos* = (8)
  CAN_F1R2_FB8_Msk* = (0x00000001 shl CAN_F1R2_FB8_Pos) ## !< 0x00000100
  CAN_F1R2_FB8* = CAN_F1R2_FB8_Msk
  CAN_F1R2_FB9_Pos* = (9)
  CAN_F1R2_FB9_Msk* = (0x00000001 shl CAN_F1R2_FB9_Pos) ## !< 0x00000200
  CAN_F1R2_FB9* = CAN_F1R2_FB9_Msk
  CAN_F1R2_FB10_Pos* = (10)
  CAN_F1R2_FB10_Msk* = (0x00000001 shl CAN_F1R2_FB10_Pos) ## !< 0x00000400
  CAN_F1R2_FB10* = CAN_F1R2_FB10_Msk
  CAN_F1R2_FB11_Pos* = (11)
  CAN_F1R2_FB11_Msk* = (0x00000001 shl CAN_F1R2_FB11_Pos) ## !< 0x00000800
  CAN_F1R2_FB11* = CAN_F1R2_FB11_Msk
  CAN_F1R2_FB12_Pos* = (12)
  CAN_F1R2_FB12_Msk* = (0x00000001 shl CAN_F1R2_FB12_Pos) ## !< 0x00001000
  CAN_F1R2_FB12* = CAN_F1R2_FB12_Msk
  CAN_F1R2_FB13_Pos* = (13)
  CAN_F1R2_FB13_Msk* = (0x00000001 shl CAN_F1R2_FB13_Pos) ## !< 0x00002000
  CAN_F1R2_FB13* = CAN_F1R2_FB13_Msk
  CAN_F1R2_FB14_Pos* = (14)
  CAN_F1R2_FB14_Msk* = (0x00000001 shl CAN_F1R2_FB14_Pos) ## !< 0x00004000
  CAN_F1R2_FB14* = CAN_F1R2_FB14_Msk
  CAN_F1R2_FB15_Pos* = (15)
  CAN_F1R2_FB15_Msk* = (0x00000001 shl CAN_F1R2_FB15_Pos) ## !< 0x00008000
  CAN_F1R2_FB15* = CAN_F1R2_FB15_Msk
  CAN_F1R2_FB16_Pos* = (16)
  CAN_F1R2_FB16_Msk* = (0x00000001 shl CAN_F1R2_FB16_Pos) ## !< 0x00010000
  CAN_F1R2_FB16* = CAN_F1R2_FB16_Msk
  CAN_F1R2_FB17_Pos* = (17)
  CAN_F1R2_FB17_Msk* = (0x00000001 shl CAN_F1R2_FB17_Pos) ## !< 0x00020000
  CAN_F1R2_FB17* = CAN_F1R2_FB17_Msk
  CAN_F1R2_FB18_Pos* = (18)
  CAN_F1R2_FB18_Msk* = (0x00000001 shl CAN_F1R2_FB18_Pos) ## !< 0x00040000
  CAN_F1R2_FB18* = CAN_F1R2_FB18_Msk
  CAN_F1R2_FB19_Pos* = (19)
  CAN_F1R2_FB19_Msk* = (0x00000001 shl CAN_F1R2_FB19_Pos) ## !< 0x00080000
  CAN_F1R2_FB19* = CAN_F1R2_FB19_Msk
  CAN_F1R2_FB20_Pos* = (20)
  CAN_F1R2_FB20_Msk* = (0x00000001 shl CAN_F1R2_FB20_Pos) ## !< 0x00100000
  CAN_F1R2_FB20* = CAN_F1R2_FB20_Msk
  CAN_F1R2_FB21_Pos* = (21)
  CAN_F1R2_FB21_Msk* = (0x00000001 shl CAN_F1R2_FB21_Pos) ## !< 0x00200000
  CAN_F1R2_FB21* = CAN_F1R2_FB21_Msk
  CAN_F1R2_FB22_Pos* = (22)
  CAN_F1R2_FB22_Msk* = (0x00000001 shl CAN_F1R2_FB22_Pos) ## !< 0x00400000
  CAN_F1R2_FB22* = CAN_F1R2_FB22_Msk
  CAN_F1R2_FB23_Pos* = (23)
  CAN_F1R2_FB23_Msk* = (0x00000001 shl CAN_F1R2_FB23_Pos) ## !< 0x00800000
  CAN_F1R2_FB23* = CAN_F1R2_FB23_Msk
  CAN_F1R2_FB24_Pos* = (24)
  CAN_F1R2_FB24_Msk* = (0x00000001 shl CAN_F1R2_FB24_Pos) ## !< 0x01000000
  CAN_F1R2_FB24* = CAN_F1R2_FB24_Msk
  CAN_F1R2_FB25_Pos* = (25)
  CAN_F1R2_FB25_Msk* = (0x00000001 shl CAN_F1R2_FB25_Pos) ## !< 0x02000000
  CAN_F1R2_FB25* = CAN_F1R2_FB25_Msk
  CAN_F1R2_FB26_Pos* = (26)
  CAN_F1R2_FB26_Msk* = (0x00000001 shl CAN_F1R2_FB26_Pos) ## !< 0x04000000
  CAN_F1R2_FB26* = CAN_F1R2_FB26_Msk
  CAN_F1R2_FB27_Pos* = (27)
  CAN_F1R2_FB27_Msk* = (0x00000001 shl CAN_F1R2_FB27_Pos) ## !< 0x08000000
  CAN_F1R2_FB27* = CAN_F1R2_FB27_Msk
  CAN_F1R2_FB28_Pos* = (28)
  CAN_F1R2_FB28_Msk* = (0x00000001 shl CAN_F1R2_FB28_Pos) ## !< 0x10000000
  CAN_F1R2_FB28* = CAN_F1R2_FB28_Msk
  CAN_F1R2_FB29_Pos* = (29)
  CAN_F1R2_FB29_Msk* = (0x00000001 shl CAN_F1R2_FB29_Pos) ## !< 0x20000000
  CAN_F1R2_FB29* = CAN_F1R2_FB29_Msk
  CAN_F1R2_FB30_Pos* = (30)
  CAN_F1R2_FB30_Msk* = (0x00000001 shl CAN_F1R2_FB30_Pos) ## !< 0x40000000
  CAN_F1R2_FB30* = CAN_F1R2_FB30_Msk
  CAN_F1R2_FB31_Pos* = (31)
  CAN_F1R2_FB31_Msk* = (0x00000001 shl CAN_F1R2_FB31_Pos) ## !< 0x80000000
  CAN_F1R2_FB31* = CAN_F1R2_FB31_Msk

## ******************  Bit definition for CAN_F2R2 register  ******************

const
  CAN_F2R2_FB0_Pos* = (0)
  CAN_F2R2_FB0_Msk* = (0x00000001 shl CAN_F2R2_FB0_Pos) ## !< 0x00000001
  CAN_F2R2_FB0* = CAN_F2R2_FB0_Msk
  CAN_F2R2_FB1_Pos* = (1)
  CAN_F2R2_FB1_Msk* = (0x00000001 shl CAN_F2R2_FB1_Pos) ## !< 0x00000002
  CAN_F2R2_FB1* = CAN_F2R2_FB1_Msk
  CAN_F2R2_FB2_Pos* = (2)
  CAN_F2R2_FB2_Msk* = (0x00000001 shl CAN_F2R2_FB2_Pos) ## !< 0x00000004
  CAN_F2R2_FB2* = CAN_F2R2_FB2_Msk
  CAN_F2R2_FB3_Pos* = (3)
  CAN_F2R2_FB3_Msk* = (0x00000001 shl CAN_F2R2_FB3_Pos) ## !< 0x00000008
  CAN_F2R2_FB3* = CAN_F2R2_FB3_Msk
  CAN_F2R2_FB4_Pos* = (4)
  CAN_F2R2_FB4_Msk* = (0x00000001 shl CAN_F2R2_FB4_Pos) ## !< 0x00000010
  CAN_F2R2_FB4* = CAN_F2R2_FB4_Msk
  CAN_F2R2_FB5_Pos* = (5)
  CAN_F2R2_FB5_Msk* = (0x00000001 shl CAN_F2R2_FB5_Pos) ## !< 0x00000020
  CAN_F2R2_FB5* = CAN_F2R2_FB5_Msk
  CAN_F2R2_FB6_Pos* = (6)
  CAN_F2R2_FB6_Msk* = (0x00000001 shl CAN_F2R2_FB6_Pos) ## !< 0x00000040
  CAN_F2R2_FB6* = CAN_F2R2_FB6_Msk
  CAN_F2R2_FB7_Pos* = (7)
  CAN_F2R2_FB7_Msk* = (0x00000001 shl CAN_F2R2_FB7_Pos) ## !< 0x00000080
  CAN_F2R2_FB7* = CAN_F2R2_FB7_Msk
  CAN_F2R2_FB8_Pos* = (8)
  CAN_F2R2_FB8_Msk* = (0x00000001 shl CAN_F2R2_FB8_Pos) ## !< 0x00000100
  CAN_F2R2_FB8* = CAN_F2R2_FB8_Msk
  CAN_F2R2_FB9_Pos* = (9)
  CAN_F2R2_FB9_Msk* = (0x00000001 shl CAN_F2R2_FB9_Pos) ## !< 0x00000200
  CAN_F2R2_FB9* = CAN_F2R2_FB9_Msk
  CAN_F2R2_FB10_Pos* = (10)
  CAN_F2R2_FB10_Msk* = (0x00000001 shl CAN_F2R2_FB10_Pos) ## !< 0x00000400
  CAN_F2R2_FB10* = CAN_F2R2_FB10_Msk
  CAN_F2R2_FB11_Pos* = (11)
  CAN_F2R2_FB11_Msk* = (0x00000001 shl CAN_F2R2_FB11_Pos) ## !< 0x00000800
  CAN_F2R2_FB11* = CAN_F2R2_FB11_Msk
  CAN_F2R2_FB12_Pos* = (12)
  CAN_F2R2_FB12_Msk* = (0x00000001 shl CAN_F2R2_FB12_Pos) ## !< 0x00001000
  CAN_F2R2_FB12* = CAN_F2R2_FB12_Msk
  CAN_F2R2_FB13_Pos* = (13)
  CAN_F2R2_FB13_Msk* = (0x00000001 shl CAN_F2R2_FB13_Pos) ## !< 0x00002000
  CAN_F2R2_FB13* = CAN_F2R2_FB13_Msk
  CAN_F2R2_FB14_Pos* = (14)
  CAN_F2R2_FB14_Msk* = (0x00000001 shl CAN_F2R2_FB14_Pos) ## !< 0x00004000
  CAN_F2R2_FB14* = CAN_F2R2_FB14_Msk
  CAN_F2R2_FB15_Pos* = (15)
  CAN_F2R2_FB15_Msk* = (0x00000001 shl CAN_F2R2_FB15_Pos) ## !< 0x00008000
  CAN_F2R2_FB15* = CAN_F2R2_FB15_Msk
  CAN_F2R2_FB16_Pos* = (16)
  CAN_F2R2_FB16_Msk* = (0x00000001 shl CAN_F2R2_FB16_Pos) ## !< 0x00010000
  CAN_F2R2_FB16* = CAN_F2R2_FB16_Msk
  CAN_F2R2_FB17_Pos* = (17)
  CAN_F2R2_FB17_Msk* = (0x00000001 shl CAN_F2R2_FB17_Pos) ## !< 0x00020000
  CAN_F2R2_FB17* = CAN_F2R2_FB17_Msk
  CAN_F2R2_FB18_Pos* = (18)
  CAN_F2R2_FB18_Msk* = (0x00000001 shl CAN_F2R2_FB18_Pos) ## !< 0x00040000
  CAN_F2R2_FB18* = CAN_F2R2_FB18_Msk
  CAN_F2R2_FB19_Pos* = (19)
  CAN_F2R2_FB19_Msk* = (0x00000001 shl CAN_F2R2_FB19_Pos) ## !< 0x00080000
  CAN_F2R2_FB19* = CAN_F2R2_FB19_Msk
  CAN_F2R2_FB20_Pos* = (20)
  CAN_F2R2_FB20_Msk* = (0x00000001 shl CAN_F2R2_FB20_Pos) ## !< 0x00100000
  CAN_F2R2_FB20* = CAN_F2R2_FB20_Msk
  CAN_F2R2_FB21_Pos* = (21)
  CAN_F2R2_FB21_Msk* = (0x00000001 shl CAN_F2R2_FB21_Pos) ## !< 0x00200000
  CAN_F2R2_FB21* = CAN_F2R2_FB21_Msk
  CAN_F2R2_FB22_Pos* = (22)
  CAN_F2R2_FB22_Msk* = (0x00000001 shl CAN_F2R2_FB22_Pos) ## !< 0x00400000
  CAN_F2R2_FB22* = CAN_F2R2_FB22_Msk
  CAN_F2R2_FB23_Pos* = (23)
  CAN_F2R2_FB23_Msk* = (0x00000001 shl CAN_F2R2_FB23_Pos) ## !< 0x00800000
  CAN_F2R2_FB23* = CAN_F2R2_FB23_Msk
  CAN_F2R2_FB24_Pos* = (24)
  CAN_F2R2_FB24_Msk* = (0x00000001 shl CAN_F2R2_FB24_Pos) ## !< 0x01000000
  CAN_F2R2_FB24* = CAN_F2R2_FB24_Msk
  CAN_F2R2_FB25_Pos* = (25)
  CAN_F2R2_FB25_Msk* = (0x00000001 shl CAN_F2R2_FB25_Pos) ## !< 0x02000000
  CAN_F2R2_FB25* = CAN_F2R2_FB25_Msk
  CAN_F2R2_FB26_Pos* = (26)
  CAN_F2R2_FB26_Msk* = (0x00000001 shl CAN_F2R2_FB26_Pos) ## !< 0x04000000
  CAN_F2R2_FB26* = CAN_F2R2_FB26_Msk
  CAN_F2R2_FB27_Pos* = (27)
  CAN_F2R2_FB27_Msk* = (0x00000001 shl CAN_F2R2_FB27_Pos) ## !< 0x08000000
  CAN_F2R2_FB27* = CAN_F2R2_FB27_Msk
  CAN_F2R2_FB28_Pos* = (28)
  CAN_F2R2_FB28_Msk* = (0x00000001 shl CAN_F2R2_FB28_Pos) ## !< 0x10000000
  CAN_F2R2_FB28* = CAN_F2R2_FB28_Msk
  CAN_F2R2_FB29_Pos* = (29)
  CAN_F2R2_FB29_Msk* = (0x00000001 shl CAN_F2R2_FB29_Pos) ## !< 0x20000000
  CAN_F2R2_FB29* = CAN_F2R2_FB29_Msk
  CAN_F2R2_FB30_Pos* = (30)
  CAN_F2R2_FB30_Msk* = (0x00000001 shl CAN_F2R2_FB30_Pos) ## !< 0x40000000
  CAN_F2R2_FB30* = CAN_F2R2_FB30_Msk
  CAN_F2R2_FB31_Pos* = (31)
  CAN_F2R2_FB31_Msk* = (0x00000001 shl CAN_F2R2_FB31_Pos) ## !< 0x80000000
  CAN_F2R2_FB31* = CAN_F2R2_FB31_Msk

## ******************  Bit definition for CAN_F3R2 register  ******************

const
  CAN_F3R2_FB0_Pos* = (0)
  CAN_F3R2_FB0_Msk* = (0x00000001 shl CAN_F3R2_FB0_Pos) ## !< 0x00000001
  CAN_F3R2_FB0* = CAN_F3R2_FB0_Msk
  CAN_F3R2_FB1_Pos* = (1)
  CAN_F3R2_FB1_Msk* = (0x00000001 shl CAN_F3R2_FB1_Pos) ## !< 0x00000002
  CAN_F3R2_FB1* = CAN_F3R2_FB1_Msk
  CAN_F3R2_FB2_Pos* = (2)
  CAN_F3R2_FB2_Msk* = (0x00000001 shl CAN_F3R2_FB2_Pos) ## !< 0x00000004
  CAN_F3R2_FB2* = CAN_F3R2_FB2_Msk
  CAN_F3R2_FB3_Pos* = (3)
  CAN_F3R2_FB3_Msk* = (0x00000001 shl CAN_F3R2_FB3_Pos) ## !< 0x00000008
  CAN_F3R2_FB3* = CAN_F3R2_FB3_Msk
  CAN_F3R2_FB4_Pos* = (4)
  CAN_F3R2_FB4_Msk* = (0x00000001 shl CAN_F3R2_FB4_Pos) ## !< 0x00000010
  CAN_F3R2_FB4* = CAN_F3R2_FB4_Msk
  CAN_F3R2_FB5_Pos* = (5)
  CAN_F3R2_FB5_Msk* = (0x00000001 shl CAN_F3R2_FB5_Pos) ## !< 0x00000020
  CAN_F3R2_FB5* = CAN_F3R2_FB5_Msk
  CAN_F3R2_FB6_Pos* = (6)
  CAN_F3R2_FB6_Msk* = (0x00000001 shl CAN_F3R2_FB6_Pos) ## !< 0x00000040
  CAN_F3R2_FB6* = CAN_F3R2_FB6_Msk
  CAN_F3R2_FB7_Pos* = (7)
  CAN_F3R2_FB7_Msk* = (0x00000001 shl CAN_F3R2_FB7_Pos) ## !< 0x00000080
  CAN_F3R2_FB7* = CAN_F3R2_FB7_Msk
  CAN_F3R2_FB8_Pos* = (8)
  CAN_F3R2_FB8_Msk* = (0x00000001 shl CAN_F3R2_FB8_Pos) ## !< 0x00000100
  CAN_F3R2_FB8* = CAN_F3R2_FB8_Msk
  CAN_F3R2_FB9_Pos* = (9)
  CAN_F3R2_FB9_Msk* = (0x00000001 shl CAN_F3R2_FB9_Pos) ## !< 0x00000200
  CAN_F3R2_FB9* = CAN_F3R2_FB9_Msk
  CAN_F3R2_FB10_Pos* = (10)
  CAN_F3R2_FB10_Msk* = (0x00000001 shl CAN_F3R2_FB10_Pos) ## !< 0x00000400
  CAN_F3R2_FB10* = CAN_F3R2_FB10_Msk
  CAN_F3R2_FB11_Pos* = (11)
  CAN_F3R2_FB11_Msk* = (0x00000001 shl CAN_F3R2_FB11_Pos) ## !< 0x00000800
  CAN_F3R2_FB11* = CAN_F3R2_FB11_Msk
  CAN_F3R2_FB12_Pos* = (12)
  CAN_F3R2_FB12_Msk* = (0x00000001 shl CAN_F3R2_FB12_Pos) ## !< 0x00001000
  CAN_F3R2_FB12* = CAN_F3R2_FB12_Msk
  CAN_F3R2_FB13_Pos* = (13)
  CAN_F3R2_FB13_Msk* = (0x00000001 shl CAN_F3R2_FB13_Pos) ## !< 0x00002000
  CAN_F3R2_FB13* = CAN_F3R2_FB13_Msk
  CAN_F3R2_FB14_Pos* = (14)
  CAN_F3R2_FB14_Msk* = (0x00000001 shl CAN_F3R2_FB14_Pos) ## !< 0x00004000
  CAN_F3R2_FB14* = CAN_F3R2_FB14_Msk
  CAN_F3R2_FB15_Pos* = (15)
  CAN_F3R2_FB15_Msk* = (0x00000001 shl CAN_F3R2_FB15_Pos) ## !< 0x00008000
  CAN_F3R2_FB15* = CAN_F3R2_FB15_Msk
  CAN_F3R2_FB16_Pos* = (16)
  CAN_F3R2_FB16_Msk* = (0x00000001 shl CAN_F3R2_FB16_Pos) ## !< 0x00010000
  CAN_F3R2_FB16* = CAN_F3R2_FB16_Msk
  CAN_F3R2_FB17_Pos* = (17)
  CAN_F3R2_FB17_Msk* = (0x00000001 shl CAN_F3R2_FB17_Pos) ## !< 0x00020000
  CAN_F3R2_FB17* = CAN_F3R2_FB17_Msk
  CAN_F3R2_FB18_Pos* = (18)
  CAN_F3R2_FB18_Msk* = (0x00000001 shl CAN_F3R2_FB18_Pos) ## !< 0x00040000
  CAN_F3R2_FB18* = CAN_F3R2_FB18_Msk
  CAN_F3R2_FB19_Pos* = (19)
  CAN_F3R2_FB19_Msk* = (0x00000001 shl CAN_F3R2_FB19_Pos) ## !< 0x00080000
  CAN_F3R2_FB19* = CAN_F3R2_FB19_Msk
  CAN_F3R2_FB20_Pos* = (20)
  CAN_F3R2_FB20_Msk* = (0x00000001 shl CAN_F3R2_FB20_Pos) ## !< 0x00100000
  CAN_F3R2_FB20* = CAN_F3R2_FB20_Msk
  CAN_F3R2_FB21_Pos* = (21)
  CAN_F3R2_FB21_Msk* = (0x00000001 shl CAN_F3R2_FB21_Pos) ## !< 0x00200000
  CAN_F3R2_FB21* = CAN_F3R2_FB21_Msk
  CAN_F3R2_FB22_Pos* = (22)
  CAN_F3R2_FB22_Msk* = (0x00000001 shl CAN_F3R2_FB22_Pos) ## !< 0x00400000
  CAN_F3R2_FB22* = CAN_F3R2_FB22_Msk
  CAN_F3R2_FB23_Pos* = (23)
  CAN_F3R2_FB23_Msk* = (0x00000001 shl CAN_F3R2_FB23_Pos) ## !< 0x00800000
  CAN_F3R2_FB23* = CAN_F3R2_FB23_Msk
  CAN_F3R2_FB24_Pos* = (24)
  CAN_F3R2_FB24_Msk* = (0x00000001 shl CAN_F3R2_FB24_Pos) ## !< 0x01000000
  CAN_F3R2_FB24* = CAN_F3R2_FB24_Msk
  CAN_F3R2_FB25_Pos* = (25)
  CAN_F3R2_FB25_Msk* = (0x00000001 shl CAN_F3R2_FB25_Pos) ## !< 0x02000000
  CAN_F3R2_FB25* = CAN_F3R2_FB25_Msk
  CAN_F3R2_FB26_Pos* = (26)
  CAN_F3R2_FB26_Msk* = (0x00000001 shl CAN_F3R2_FB26_Pos) ## !< 0x04000000
  CAN_F3R2_FB26* = CAN_F3R2_FB26_Msk
  CAN_F3R2_FB27_Pos* = (27)
  CAN_F3R2_FB27_Msk* = (0x00000001 shl CAN_F3R2_FB27_Pos) ## !< 0x08000000
  CAN_F3R2_FB27* = CAN_F3R2_FB27_Msk
  CAN_F3R2_FB28_Pos* = (28)
  CAN_F3R2_FB28_Msk* = (0x00000001 shl CAN_F3R2_FB28_Pos) ## !< 0x10000000
  CAN_F3R2_FB28* = CAN_F3R2_FB28_Msk
  CAN_F3R2_FB29_Pos* = (29)
  CAN_F3R2_FB29_Msk* = (0x00000001 shl CAN_F3R2_FB29_Pos) ## !< 0x20000000
  CAN_F3R2_FB29* = CAN_F3R2_FB29_Msk
  CAN_F3R2_FB30_Pos* = (30)
  CAN_F3R2_FB30_Msk* = (0x00000001 shl CAN_F3R2_FB30_Pos) ## !< 0x40000000
  CAN_F3R2_FB30* = CAN_F3R2_FB30_Msk
  CAN_F3R2_FB31_Pos* = (31)
  CAN_F3R2_FB31_Msk* = (0x00000001 shl CAN_F3R2_FB31_Pos) ## !< 0x80000000
  CAN_F3R2_FB31* = CAN_F3R2_FB31_Msk

## ******************  Bit definition for CAN_F4R2 register  ******************

const
  CAN_F4R2_FB0_Pos* = (0)
  CAN_F4R2_FB0_Msk* = (0x00000001 shl CAN_F4R2_FB0_Pos) ## !< 0x00000001
  CAN_F4R2_FB0* = CAN_F4R2_FB0_Msk
  CAN_F4R2_FB1_Pos* = (1)
  CAN_F4R2_FB1_Msk* = (0x00000001 shl CAN_F4R2_FB1_Pos) ## !< 0x00000002
  CAN_F4R2_FB1* = CAN_F4R2_FB1_Msk
  CAN_F4R2_FB2_Pos* = (2)
  CAN_F4R2_FB2_Msk* = (0x00000001 shl CAN_F4R2_FB2_Pos) ## !< 0x00000004
  CAN_F4R2_FB2* = CAN_F4R2_FB2_Msk
  CAN_F4R2_FB3_Pos* = (3)
  CAN_F4R2_FB3_Msk* = (0x00000001 shl CAN_F4R2_FB3_Pos) ## !< 0x00000008
  CAN_F4R2_FB3* = CAN_F4R2_FB3_Msk
  CAN_F4R2_FB4_Pos* = (4)
  CAN_F4R2_FB4_Msk* = (0x00000001 shl CAN_F4R2_FB4_Pos) ## !< 0x00000010
  CAN_F4R2_FB4* = CAN_F4R2_FB4_Msk
  CAN_F4R2_FB5_Pos* = (5)
  CAN_F4R2_FB5_Msk* = (0x00000001 shl CAN_F4R2_FB5_Pos) ## !< 0x00000020
  CAN_F4R2_FB5* = CAN_F4R2_FB5_Msk
  CAN_F4R2_FB6_Pos* = (6)
  CAN_F4R2_FB6_Msk* = (0x00000001 shl CAN_F4R2_FB6_Pos) ## !< 0x00000040
  CAN_F4R2_FB6* = CAN_F4R2_FB6_Msk
  CAN_F4R2_FB7_Pos* = (7)
  CAN_F4R2_FB7_Msk* = (0x00000001 shl CAN_F4R2_FB7_Pos) ## !< 0x00000080
  CAN_F4R2_FB7* = CAN_F4R2_FB7_Msk
  CAN_F4R2_FB8_Pos* = (8)
  CAN_F4R2_FB8_Msk* = (0x00000001 shl CAN_F4R2_FB8_Pos) ## !< 0x00000100
  CAN_F4R2_FB8* = CAN_F4R2_FB8_Msk
  CAN_F4R2_FB9_Pos* = (9)
  CAN_F4R2_FB9_Msk* = (0x00000001 shl CAN_F4R2_FB9_Pos) ## !< 0x00000200
  CAN_F4R2_FB9* = CAN_F4R2_FB9_Msk
  CAN_F4R2_FB10_Pos* = (10)
  CAN_F4R2_FB10_Msk* = (0x00000001 shl CAN_F4R2_FB10_Pos) ## !< 0x00000400
  CAN_F4R2_FB10* = CAN_F4R2_FB10_Msk
  CAN_F4R2_FB11_Pos* = (11)
  CAN_F4R2_FB11_Msk* = (0x00000001 shl CAN_F4R2_FB11_Pos) ## !< 0x00000800
  CAN_F4R2_FB11* = CAN_F4R2_FB11_Msk
  CAN_F4R2_FB12_Pos* = (12)
  CAN_F4R2_FB12_Msk* = (0x00000001 shl CAN_F4R2_FB12_Pos) ## !< 0x00001000
  CAN_F4R2_FB12* = CAN_F4R2_FB12_Msk
  CAN_F4R2_FB13_Pos* = (13)
  CAN_F4R2_FB13_Msk* = (0x00000001 shl CAN_F4R2_FB13_Pos) ## !< 0x00002000
  CAN_F4R2_FB13* = CAN_F4R2_FB13_Msk
  CAN_F4R2_FB14_Pos* = (14)
  CAN_F4R2_FB14_Msk* = (0x00000001 shl CAN_F4R2_FB14_Pos) ## !< 0x00004000
  CAN_F4R2_FB14* = CAN_F4R2_FB14_Msk
  CAN_F4R2_FB15_Pos* = (15)
  CAN_F4R2_FB15_Msk* = (0x00000001 shl CAN_F4R2_FB15_Pos) ## !< 0x00008000
  CAN_F4R2_FB15* = CAN_F4R2_FB15_Msk
  CAN_F4R2_FB16_Pos* = (16)
  CAN_F4R2_FB16_Msk* = (0x00000001 shl CAN_F4R2_FB16_Pos) ## !< 0x00010000
  CAN_F4R2_FB16* = CAN_F4R2_FB16_Msk
  CAN_F4R2_FB17_Pos* = (17)
  CAN_F4R2_FB17_Msk* = (0x00000001 shl CAN_F4R2_FB17_Pos) ## !< 0x00020000
  CAN_F4R2_FB17* = CAN_F4R2_FB17_Msk
  CAN_F4R2_FB18_Pos* = (18)
  CAN_F4R2_FB18_Msk* = (0x00000001 shl CAN_F4R2_FB18_Pos) ## !< 0x00040000
  CAN_F4R2_FB18* = CAN_F4R2_FB18_Msk
  CAN_F4R2_FB19_Pos* = (19)
  CAN_F4R2_FB19_Msk* = (0x00000001 shl CAN_F4R2_FB19_Pos) ## !< 0x00080000
  CAN_F4R2_FB19* = CAN_F4R2_FB19_Msk
  CAN_F4R2_FB20_Pos* = (20)
  CAN_F4R2_FB20_Msk* = (0x00000001 shl CAN_F4R2_FB20_Pos) ## !< 0x00100000
  CAN_F4R2_FB20* = CAN_F4R2_FB20_Msk
  CAN_F4R2_FB21_Pos* = (21)
  CAN_F4R2_FB21_Msk* = (0x00000001 shl CAN_F4R2_FB21_Pos) ## !< 0x00200000
  CAN_F4R2_FB21* = CAN_F4R2_FB21_Msk
  CAN_F4R2_FB22_Pos* = (22)
  CAN_F4R2_FB22_Msk* = (0x00000001 shl CAN_F4R2_FB22_Pos) ## !< 0x00400000
  CAN_F4R2_FB22* = CAN_F4R2_FB22_Msk
  CAN_F4R2_FB23_Pos* = (23)
  CAN_F4R2_FB23_Msk* = (0x00000001 shl CAN_F4R2_FB23_Pos) ## !< 0x00800000
  CAN_F4R2_FB23* = CAN_F4R2_FB23_Msk
  CAN_F4R2_FB24_Pos* = (24)
  CAN_F4R2_FB24_Msk* = (0x00000001 shl CAN_F4R2_FB24_Pos) ## !< 0x01000000
  CAN_F4R2_FB24* = CAN_F4R2_FB24_Msk
  CAN_F4R2_FB25_Pos* = (25)
  CAN_F4R2_FB25_Msk* = (0x00000001 shl CAN_F4R2_FB25_Pos) ## !< 0x02000000
  CAN_F4R2_FB25* = CAN_F4R2_FB25_Msk
  CAN_F4R2_FB26_Pos* = (26)
  CAN_F4R2_FB26_Msk* = (0x00000001 shl CAN_F4R2_FB26_Pos) ## !< 0x04000000
  CAN_F4R2_FB26* = CAN_F4R2_FB26_Msk
  CAN_F4R2_FB27_Pos* = (27)
  CAN_F4R2_FB27_Msk* = (0x00000001 shl CAN_F4R2_FB27_Pos) ## !< 0x08000000
  CAN_F4R2_FB27* = CAN_F4R2_FB27_Msk
  CAN_F4R2_FB28_Pos* = (28)
  CAN_F4R2_FB28_Msk* = (0x00000001 shl CAN_F4R2_FB28_Pos) ## !< 0x10000000
  CAN_F4R2_FB28* = CAN_F4R2_FB28_Msk
  CAN_F4R2_FB29_Pos* = (29)
  CAN_F4R2_FB29_Msk* = (0x00000001 shl CAN_F4R2_FB29_Pos) ## !< 0x20000000
  CAN_F4R2_FB29* = CAN_F4R2_FB29_Msk
  CAN_F4R2_FB30_Pos* = (30)
  CAN_F4R2_FB30_Msk* = (0x00000001 shl CAN_F4R2_FB30_Pos) ## !< 0x40000000
  CAN_F4R2_FB30* = CAN_F4R2_FB30_Msk
  CAN_F4R2_FB31_Pos* = (31)
  CAN_F4R2_FB31_Msk* = (0x00000001 shl CAN_F4R2_FB31_Pos) ## !< 0x80000000
  CAN_F4R2_FB31* = CAN_F4R2_FB31_Msk

## ******************  Bit definition for CAN_F5R2 register  ******************

const
  CAN_F5R2_FB0_Pos* = (0)
  CAN_F5R2_FB0_Msk* = (0x00000001 shl CAN_F5R2_FB0_Pos) ## !< 0x00000001
  CAN_F5R2_FB0* = CAN_F5R2_FB0_Msk
  CAN_F5R2_FB1_Pos* = (1)
  CAN_F5R2_FB1_Msk* = (0x00000001 shl CAN_F5R2_FB1_Pos) ## !< 0x00000002
  CAN_F5R2_FB1* = CAN_F5R2_FB1_Msk
  CAN_F5R2_FB2_Pos* = (2)
  CAN_F5R2_FB2_Msk* = (0x00000001 shl CAN_F5R2_FB2_Pos) ## !< 0x00000004
  CAN_F5R2_FB2* = CAN_F5R2_FB2_Msk
  CAN_F5R2_FB3_Pos* = (3)
  CAN_F5R2_FB3_Msk* = (0x00000001 shl CAN_F5R2_FB3_Pos) ## !< 0x00000008
  CAN_F5R2_FB3* = CAN_F5R2_FB3_Msk
  CAN_F5R2_FB4_Pos* = (4)
  CAN_F5R2_FB4_Msk* = (0x00000001 shl CAN_F5R2_FB4_Pos) ## !< 0x00000010
  CAN_F5R2_FB4* = CAN_F5R2_FB4_Msk
  CAN_F5R2_FB5_Pos* = (5)
  CAN_F5R2_FB5_Msk* = (0x00000001 shl CAN_F5R2_FB5_Pos) ## !< 0x00000020
  CAN_F5R2_FB5* = CAN_F5R2_FB5_Msk
  CAN_F5R2_FB6_Pos* = (6)
  CAN_F5R2_FB6_Msk* = (0x00000001 shl CAN_F5R2_FB6_Pos) ## !< 0x00000040
  CAN_F5R2_FB6* = CAN_F5R2_FB6_Msk
  CAN_F5R2_FB7_Pos* = (7)
  CAN_F5R2_FB7_Msk* = (0x00000001 shl CAN_F5R2_FB7_Pos) ## !< 0x00000080
  CAN_F5R2_FB7* = CAN_F5R2_FB7_Msk
  CAN_F5R2_FB8_Pos* = (8)
  CAN_F5R2_FB8_Msk* = (0x00000001 shl CAN_F5R2_FB8_Pos) ## !< 0x00000100
  CAN_F5R2_FB8* = CAN_F5R2_FB8_Msk
  CAN_F5R2_FB9_Pos* = (9)
  CAN_F5R2_FB9_Msk* = (0x00000001 shl CAN_F5R2_FB9_Pos) ## !< 0x00000200
  CAN_F5R2_FB9* = CAN_F5R2_FB9_Msk
  CAN_F5R2_FB10_Pos* = (10)
  CAN_F5R2_FB10_Msk* = (0x00000001 shl CAN_F5R2_FB10_Pos) ## !< 0x00000400
  CAN_F5R2_FB10* = CAN_F5R2_FB10_Msk
  CAN_F5R2_FB11_Pos* = (11)
  CAN_F5R2_FB11_Msk* = (0x00000001 shl CAN_F5R2_FB11_Pos) ## !< 0x00000800
  CAN_F5R2_FB11* = CAN_F5R2_FB11_Msk
  CAN_F5R2_FB12_Pos* = (12)
  CAN_F5R2_FB12_Msk* = (0x00000001 shl CAN_F5R2_FB12_Pos) ## !< 0x00001000
  CAN_F5R2_FB12* = CAN_F5R2_FB12_Msk
  CAN_F5R2_FB13_Pos* = (13)
  CAN_F5R2_FB13_Msk* = (0x00000001 shl CAN_F5R2_FB13_Pos) ## !< 0x00002000
  CAN_F5R2_FB13* = CAN_F5R2_FB13_Msk
  CAN_F5R2_FB14_Pos* = (14)
  CAN_F5R2_FB14_Msk* = (0x00000001 shl CAN_F5R2_FB14_Pos) ## !< 0x00004000
  CAN_F5R2_FB14* = CAN_F5R2_FB14_Msk
  CAN_F5R2_FB15_Pos* = (15)
  CAN_F5R2_FB15_Msk* = (0x00000001 shl CAN_F5R2_FB15_Pos) ## !< 0x00008000
  CAN_F5R2_FB15* = CAN_F5R2_FB15_Msk
  CAN_F5R2_FB16_Pos* = (16)
  CAN_F5R2_FB16_Msk* = (0x00000001 shl CAN_F5R2_FB16_Pos) ## !< 0x00010000
  CAN_F5R2_FB16* = CAN_F5R2_FB16_Msk
  CAN_F5R2_FB17_Pos* = (17)
  CAN_F5R2_FB17_Msk* = (0x00000001 shl CAN_F5R2_FB17_Pos) ## !< 0x00020000
  CAN_F5R2_FB17* = CAN_F5R2_FB17_Msk
  CAN_F5R2_FB18_Pos* = (18)
  CAN_F5R2_FB18_Msk* = (0x00000001 shl CAN_F5R2_FB18_Pos) ## !< 0x00040000
  CAN_F5R2_FB18* = CAN_F5R2_FB18_Msk
  CAN_F5R2_FB19_Pos* = (19)
  CAN_F5R2_FB19_Msk* = (0x00000001 shl CAN_F5R2_FB19_Pos) ## !< 0x00080000
  CAN_F5R2_FB19* = CAN_F5R2_FB19_Msk
  CAN_F5R2_FB20_Pos* = (20)
  CAN_F5R2_FB20_Msk* = (0x00000001 shl CAN_F5R2_FB20_Pos) ## !< 0x00100000
  CAN_F5R2_FB20* = CAN_F5R2_FB20_Msk
  CAN_F5R2_FB21_Pos* = (21)
  CAN_F5R2_FB21_Msk* = (0x00000001 shl CAN_F5R2_FB21_Pos) ## !< 0x00200000
  CAN_F5R2_FB21* = CAN_F5R2_FB21_Msk
  CAN_F5R2_FB22_Pos* = (22)
  CAN_F5R2_FB22_Msk* = (0x00000001 shl CAN_F5R2_FB22_Pos) ## !< 0x00400000
  CAN_F5R2_FB22* = CAN_F5R2_FB22_Msk
  CAN_F5R2_FB23_Pos* = (23)
  CAN_F5R2_FB23_Msk* = (0x00000001 shl CAN_F5R2_FB23_Pos) ## !< 0x00800000
  CAN_F5R2_FB23* = CAN_F5R2_FB23_Msk
  CAN_F5R2_FB24_Pos* = (24)
  CAN_F5R2_FB24_Msk* = (0x00000001 shl CAN_F5R2_FB24_Pos) ## !< 0x01000000
  CAN_F5R2_FB24* = CAN_F5R2_FB24_Msk
  CAN_F5R2_FB25_Pos* = (25)
  CAN_F5R2_FB25_Msk* = (0x00000001 shl CAN_F5R2_FB25_Pos) ## !< 0x02000000
  CAN_F5R2_FB25* = CAN_F5R2_FB25_Msk
  CAN_F5R2_FB26_Pos* = (26)
  CAN_F5R2_FB26_Msk* = (0x00000001 shl CAN_F5R2_FB26_Pos) ## !< 0x04000000
  CAN_F5R2_FB26* = CAN_F5R2_FB26_Msk
  CAN_F5R2_FB27_Pos* = (27)
  CAN_F5R2_FB27_Msk* = (0x00000001 shl CAN_F5R2_FB27_Pos) ## !< 0x08000000
  CAN_F5R2_FB27* = CAN_F5R2_FB27_Msk
  CAN_F5R2_FB28_Pos* = (28)
  CAN_F5R2_FB28_Msk* = (0x00000001 shl CAN_F5R2_FB28_Pos) ## !< 0x10000000
  CAN_F5R2_FB28* = CAN_F5R2_FB28_Msk
  CAN_F5R2_FB29_Pos* = (29)
  CAN_F5R2_FB29_Msk* = (0x00000001 shl CAN_F5R2_FB29_Pos) ## !< 0x20000000
  CAN_F5R2_FB29* = CAN_F5R2_FB29_Msk
  CAN_F5R2_FB30_Pos* = (30)
  CAN_F5R2_FB30_Msk* = (0x00000001 shl CAN_F5R2_FB30_Pos) ## !< 0x40000000
  CAN_F5R2_FB30* = CAN_F5R2_FB30_Msk
  CAN_F5R2_FB31_Pos* = (31)
  CAN_F5R2_FB31_Msk* = (0x00000001 shl CAN_F5R2_FB31_Pos) ## !< 0x80000000
  CAN_F5R2_FB31* = CAN_F5R2_FB31_Msk

## ******************  Bit definition for CAN_F6R2 register  ******************

const
  CAN_F6R2_FB0_Pos* = (0)
  CAN_F6R2_FB0_Msk* = (0x00000001 shl CAN_F6R2_FB0_Pos) ## !< 0x00000001
  CAN_F6R2_FB0* = CAN_F6R2_FB0_Msk
  CAN_F6R2_FB1_Pos* = (1)
  CAN_F6R2_FB1_Msk* = (0x00000001 shl CAN_F6R2_FB1_Pos) ## !< 0x00000002
  CAN_F6R2_FB1* = CAN_F6R2_FB1_Msk
  CAN_F6R2_FB2_Pos* = (2)
  CAN_F6R2_FB2_Msk* = (0x00000001 shl CAN_F6R2_FB2_Pos) ## !< 0x00000004
  CAN_F6R2_FB2* = CAN_F6R2_FB2_Msk
  CAN_F6R2_FB3_Pos* = (3)
  CAN_F6R2_FB3_Msk* = (0x00000001 shl CAN_F6R2_FB3_Pos) ## !< 0x00000008
  CAN_F6R2_FB3* = CAN_F6R2_FB3_Msk
  CAN_F6R2_FB4_Pos* = (4)
  CAN_F6R2_FB4_Msk* = (0x00000001 shl CAN_F6R2_FB4_Pos) ## !< 0x00000010
  CAN_F6R2_FB4* = CAN_F6R2_FB4_Msk
  CAN_F6R2_FB5_Pos* = (5)
  CAN_F6R2_FB5_Msk* = (0x00000001 shl CAN_F6R2_FB5_Pos) ## !< 0x00000020
  CAN_F6R2_FB5* = CAN_F6R2_FB5_Msk
  CAN_F6R2_FB6_Pos* = (6)
  CAN_F6R2_FB6_Msk* = (0x00000001 shl CAN_F6R2_FB6_Pos) ## !< 0x00000040
  CAN_F6R2_FB6* = CAN_F6R2_FB6_Msk
  CAN_F6R2_FB7_Pos* = (7)
  CAN_F6R2_FB7_Msk* = (0x00000001 shl CAN_F6R2_FB7_Pos) ## !< 0x00000080
  CAN_F6R2_FB7* = CAN_F6R2_FB7_Msk
  CAN_F6R2_FB8_Pos* = (8)
  CAN_F6R2_FB8_Msk* = (0x00000001 shl CAN_F6R2_FB8_Pos) ## !< 0x00000100
  CAN_F6R2_FB8* = CAN_F6R2_FB8_Msk
  CAN_F6R2_FB9_Pos* = (9)
  CAN_F6R2_FB9_Msk* = (0x00000001 shl CAN_F6R2_FB9_Pos) ## !< 0x00000200
  CAN_F6R2_FB9* = CAN_F6R2_FB9_Msk
  CAN_F6R2_FB10_Pos* = (10)
  CAN_F6R2_FB10_Msk* = (0x00000001 shl CAN_F6R2_FB10_Pos) ## !< 0x00000400
  CAN_F6R2_FB10* = CAN_F6R2_FB10_Msk
  CAN_F6R2_FB11_Pos* = (11)
  CAN_F6R2_FB11_Msk* = (0x00000001 shl CAN_F6R2_FB11_Pos) ## !< 0x00000800
  CAN_F6R2_FB11* = CAN_F6R2_FB11_Msk
  CAN_F6R2_FB12_Pos* = (12)
  CAN_F6R2_FB12_Msk* = (0x00000001 shl CAN_F6R2_FB12_Pos) ## !< 0x00001000
  CAN_F6R2_FB12* = CAN_F6R2_FB12_Msk
  CAN_F6R2_FB13_Pos* = (13)
  CAN_F6R2_FB13_Msk* = (0x00000001 shl CAN_F6R2_FB13_Pos) ## !< 0x00002000
  CAN_F6R2_FB13* = CAN_F6R2_FB13_Msk
  CAN_F6R2_FB14_Pos* = (14)
  CAN_F6R2_FB14_Msk* = (0x00000001 shl CAN_F6R2_FB14_Pos) ## !< 0x00004000
  CAN_F6R2_FB14* = CAN_F6R2_FB14_Msk
  CAN_F6R2_FB15_Pos* = (15)
  CAN_F6R2_FB15_Msk* = (0x00000001 shl CAN_F6R2_FB15_Pos) ## !< 0x00008000
  CAN_F6R2_FB15* = CAN_F6R2_FB15_Msk
  CAN_F6R2_FB16_Pos* = (16)
  CAN_F6R2_FB16_Msk* = (0x00000001 shl CAN_F6R2_FB16_Pos) ## !< 0x00010000
  CAN_F6R2_FB16* = CAN_F6R2_FB16_Msk
  CAN_F6R2_FB17_Pos* = (17)
  CAN_F6R2_FB17_Msk* = (0x00000001 shl CAN_F6R2_FB17_Pos) ## !< 0x00020000
  CAN_F6R2_FB17* = CAN_F6R2_FB17_Msk
  CAN_F6R2_FB18_Pos* = (18)
  CAN_F6R2_FB18_Msk* = (0x00000001 shl CAN_F6R2_FB18_Pos) ## !< 0x00040000
  CAN_F6R2_FB18* = CAN_F6R2_FB18_Msk
  CAN_F6R2_FB19_Pos* = (19)
  CAN_F6R2_FB19_Msk* = (0x00000001 shl CAN_F6R2_FB19_Pos) ## !< 0x00080000
  CAN_F6R2_FB19* = CAN_F6R2_FB19_Msk
  CAN_F6R2_FB20_Pos* = (20)
  CAN_F6R2_FB20_Msk* = (0x00000001 shl CAN_F6R2_FB20_Pos) ## !< 0x00100000
  CAN_F6R2_FB20* = CAN_F6R2_FB20_Msk
  CAN_F6R2_FB21_Pos* = (21)
  CAN_F6R2_FB21_Msk* = (0x00000001 shl CAN_F6R2_FB21_Pos) ## !< 0x00200000
  CAN_F6R2_FB21* = CAN_F6R2_FB21_Msk
  CAN_F6R2_FB22_Pos* = (22)
  CAN_F6R2_FB22_Msk* = (0x00000001 shl CAN_F6R2_FB22_Pos) ## !< 0x00400000
  CAN_F6R2_FB22* = CAN_F6R2_FB22_Msk
  CAN_F6R2_FB23_Pos* = (23)
  CAN_F6R2_FB23_Msk* = (0x00000001 shl CAN_F6R2_FB23_Pos) ## !< 0x00800000
  CAN_F6R2_FB23* = CAN_F6R2_FB23_Msk
  CAN_F6R2_FB24_Pos* = (24)
  CAN_F6R2_FB24_Msk* = (0x00000001 shl CAN_F6R2_FB24_Pos) ## !< 0x01000000
  CAN_F6R2_FB24* = CAN_F6R2_FB24_Msk
  CAN_F6R2_FB25_Pos* = (25)
  CAN_F6R2_FB25_Msk* = (0x00000001 shl CAN_F6R2_FB25_Pos) ## !< 0x02000000
  CAN_F6R2_FB25* = CAN_F6R2_FB25_Msk
  CAN_F6R2_FB26_Pos* = (26)
  CAN_F6R2_FB26_Msk* = (0x00000001 shl CAN_F6R2_FB26_Pos) ## !< 0x04000000
  CAN_F6R2_FB26* = CAN_F6R2_FB26_Msk
  CAN_F6R2_FB27_Pos* = (27)
  CAN_F6R2_FB27_Msk* = (0x00000001 shl CAN_F6R2_FB27_Pos) ## !< 0x08000000
  CAN_F6R2_FB27* = CAN_F6R2_FB27_Msk
  CAN_F6R2_FB28_Pos* = (28)
  CAN_F6R2_FB28_Msk* = (0x00000001 shl CAN_F6R2_FB28_Pos) ## !< 0x10000000
  CAN_F6R2_FB28* = CAN_F6R2_FB28_Msk
  CAN_F6R2_FB29_Pos* = (29)
  CAN_F6R2_FB29_Msk* = (0x00000001 shl CAN_F6R2_FB29_Pos) ## !< 0x20000000
  CAN_F6R2_FB29* = CAN_F6R2_FB29_Msk
  CAN_F6R2_FB30_Pos* = (30)
  CAN_F6R2_FB30_Msk* = (0x00000001 shl CAN_F6R2_FB30_Pos) ## !< 0x40000000
  CAN_F6R2_FB30* = CAN_F6R2_FB30_Msk
  CAN_F6R2_FB31_Pos* = (31)
  CAN_F6R2_FB31_Msk* = (0x00000001 shl CAN_F6R2_FB31_Pos) ## !< 0x80000000
  CAN_F6R2_FB31* = CAN_F6R2_FB31_Msk

## ******************  Bit definition for CAN_F7R2 register  ******************

const
  CAN_F7R2_FB0_Pos* = (0)
  CAN_F7R2_FB0_Msk* = (0x00000001 shl CAN_F7R2_FB0_Pos) ## !< 0x00000001
  CAN_F7R2_FB0* = CAN_F7R2_FB0_Msk
  CAN_F7R2_FB1_Pos* = (1)
  CAN_F7R2_FB1_Msk* = (0x00000001 shl CAN_F7R2_FB1_Pos) ## !< 0x00000002
  CAN_F7R2_FB1* = CAN_F7R2_FB1_Msk
  CAN_F7R2_FB2_Pos* = (2)
  CAN_F7R2_FB2_Msk* = (0x00000001 shl CAN_F7R2_FB2_Pos) ## !< 0x00000004
  CAN_F7R2_FB2* = CAN_F7R2_FB2_Msk
  CAN_F7R2_FB3_Pos* = (3)
  CAN_F7R2_FB3_Msk* = (0x00000001 shl CAN_F7R2_FB3_Pos) ## !< 0x00000008
  CAN_F7R2_FB3* = CAN_F7R2_FB3_Msk
  CAN_F7R2_FB4_Pos* = (4)
  CAN_F7R2_FB4_Msk* = (0x00000001 shl CAN_F7R2_FB4_Pos) ## !< 0x00000010
  CAN_F7R2_FB4* = CAN_F7R2_FB4_Msk
  CAN_F7R2_FB5_Pos* = (5)
  CAN_F7R2_FB5_Msk* = (0x00000001 shl CAN_F7R2_FB5_Pos) ## !< 0x00000020
  CAN_F7R2_FB5* = CAN_F7R2_FB5_Msk
  CAN_F7R2_FB6_Pos* = (6)
  CAN_F7R2_FB6_Msk* = (0x00000001 shl CAN_F7R2_FB6_Pos) ## !< 0x00000040
  CAN_F7R2_FB6* = CAN_F7R2_FB6_Msk
  CAN_F7R2_FB7_Pos* = (7)
  CAN_F7R2_FB7_Msk* = (0x00000001 shl CAN_F7R2_FB7_Pos) ## !< 0x00000080
  CAN_F7R2_FB7* = CAN_F7R2_FB7_Msk
  CAN_F7R2_FB8_Pos* = (8)
  CAN_F7R2_FB8_Msk* = (0x00000001 shl CAN_F7R2_FB8_Pos) ## !< 0x00000100
  CAN_F7R2_FB8* = CAN_F7R2_FB8_Msk
  CAN_F7R2_FB9_Pos* = (9)
  CAN_F7R2_FB9_Msk* = (0x00000001 shl CAN_F7R2_FB9_Pos) ## !< 0x00000200
  CAN_F7R2_FB9* = CAN_F7R2_FB9_Msk
  CAN_F7R2_FB10_Pos* = (10)
  CAN_F7R2_FB10_Msk* = (0x00000001 shl CAN_F7R2_FB10_Pos) ## !< 0x00000400
  CAN_F7R2_FB10* = CAN_F7R2_FB10_Msk
  CAN_F7R2_FB11_Pos* = (11)
  CAN_F7R2_FB11_Msk* = (0x00000001 shl CAN_F7R2_FB11_Pos) ## !< 0x00000800
  CAN_F7R2_FB11* = CAN_F7R2_FB11_Msk
  CAN_F7R2_FB12_Pos* = (12)
  CAN_F7R2_FB12_Msk* = (0x00000001 shl CAN_F7R2_FB12_Pos) ## !< 0x00001000
  CAN_F7R2_FB12* = CAN_F7R2_FB12_Msk
  CAN_F7R2_FB13_Pos* = (13)
  CAN_F7R2_FB13_Msk* = (0x00000001 shl CAN_F7R2_FB13_Pos) ## !< 0x00002000
  CAN_F7R2_FB13* = CAN_F7R2_FB13_Msk
  CAN_F7R2_FB14_Pos* = (14)
  CAN_F7R2_FB14_Msk* = (0x00000001 shl CAN_F7R2_FB14_Pos) ## !< 0x00004000
  CAN_F7R2_FB14* = CAN_F7R2_FB14_Msk
  CAN_F7R2_FB15_Pos* = (15)
  CAN_F7R2_FB15_Msk* = (0x00000001 shl CAN_F7R2_FB15_Pos) ## !< 0x00008000
  CAN_F7R2_FB15* = CAN_F7R2_FB15_Msk
  CAN_F7R2_FB16_Pos* = (16)
  CAN_F7R2_FB16_Msk* = (0x00000001 shl CAN_F7R2_FB16_Pos) ## !< 0x00010000
  CAN_F7R2_FB16* = CAN_F7R2_FB16_Msk
  CAN_F7R2_FB17_Pos* = (17)
  CAN_F7R2_FB17_Msk* = (0x00000001 shl CAN_F7R2_FB17_Pos) ## !< 0x00020000
  CAN_F7R2_FB17* = CAN_F7R2_FB17_Msk
  CAN_F7R2_FB18_Pos* = (18)
  CAN_F7R2_FB18_Msk* = (0x00000001 shl CAN_F7R2_FB18_Pos) ## !< 0x00040000
  CAN_F7R2_FB18* = CAN_F7R2_FB18_Msk
  CAN_F7R2_FB19_Pos* = (19)
  CAN_F7R2_FB19_Msk* = (0x00000001 shl CAN_F7R2_FB19_Pos) ## !< 0x00080000
  CAN_F7R2_FB19* = CAN_F7R2_FB19_Msk
  CAN_F7R2_FB20_Pos* = (20)
  CAN_F7R2_FB20_Msk* = (0x00000001 shl CAN_F7R2_FB20_Pos) ## !< 0x00100000
  CAN_F7R2_FB20* = CAN_F7R2_FB20_Msk
  CAN_F7R2_FB21_Pos* = (21)
  CAN_F7R2_FB21_Msk* = (0x00000001 shl CAN_F7R2_FB21_Pos) ## !< 0x00200000
  CAN_F7R2_FB21* = CAN_F7R2_FB21_Msk
  CAN_F7R2_FB22_Pos* = (22)
  CAN_F7R2_FB22_Msk* = (0x00000001 shl CAN_F7R2_FB22_Pos) ## !< 0x00400000
  CAN_F7R2_FB22* = CAN_F7R2_FB22_Msk
  CAN_F7R2_FB23_Pos* = (23)
  CAN_F7R2_FB23_Msk* = (0x00000001 shl CAN_F7R2_FB23_Pos) ## !< 0x00800000
  CAN_F7R2_FB23* = CAN_F7R2_FB23_Msk
  CAN_F7R2_FB24_Pos* = (24)
  CAN_F7R2_FB24_Msk* = (0x00000001 shl CAN_F7R2_FB24_Pos) ## !< 0x01000000
  CAN_F7R2_FB24* = CAN_F7R2_FB24_Msk
  CAN_F7R2_FB25_Pos* = (25)
  CAN_F7R2_FB25_Msk* = (0x00000001 shl CAN_F7R2_FB25_Pos) ## !< 0x02000000
  CAN_F7R2_FB25* = CAN_F7R2_FB25_Msk
  CAN_F7R2_FB26_Pos* = (26)
  CAN_F7R2_FB26_Msk* = (0x00000001 shl CAN_F7R2_FB26_Pos) ## !< 0x04000000
  CAN_F7R2_FB26* = CAN_F7R2_FB26_Msk
  CAN_F7R2_FB27_Pos* = (27)
  CAN_F7R2_FB27_Msk* = (0x00000001 shl CAN_F7R2_FB27_Pos) ## !< 0x08000000
  CAN_F7R2_FB27* = CAN_F7R2_FB27_Msk
  CAN_F7R2_FB28_Pos* = (28)
  CAN_F7R2_FB28_Msk* = (0x00000001 shl CAN_F7R2_FB28_Pos) ## !< 0x10000000
  CAN_F7R2_FB28* = CAN_F7R2_FB28_Msk
  CAN_F7R2_FB29_Pos* = (29)
  CAN_F7R2_FB29_Msk* = (0x00000001 shl CAN_F7R2_FB29_Pos) ## !< 0x20000000
  CAN_F7R2_FB29* = CAN_F7R2_FB29_Msk
  CAN_F7R2_FB30_Pos* = (30)
  CAN_F7R2_FB30_Msk* = (0x00000001 shl CAN_F7R2_FB30_Pos) ## !< 0x40000000
  CAN_F7R2_FB30* = CAN_F7R2_FB30_Msk
  CAN_F7R2_FB31_Pos* = (31)
  CAN_F7R2_FB31_Msk* = (0x00000001 shl CAN_F7R2_FB31_Pos) ## !< 0x80000000
  CAN_F7R2_FB31* = CAN_F7R2_FB31_Msk

## ******************  Bit definition for CAN_F8R2 register  ******************

const
  CAN_F8R2_FB0_Pos* = (0)
  CAN_F8R2_FB0_Msk* = (0x00000001 shl CAN_F8R2_FB0_Pos) ## !< 0x00000001
  CAN_F8R2_FB0* = CAN_F8R2_FB0_Msk
  CAN_F8R2_FB1_Pos* = (1)
  CAN_F8R2_FB1_Msk* = (0x00000001 shl CAN_F8R2_FB1_Pos) ## !< 0x00000002
  CAN_F8R2_FB1* = CAN_F8R2_FB1_Msk
  CAN_F8R2_FB2_Pos* = (2)
  CAN_F8R2_FB2_Msk* = (0x00000001 shl CAN_F8R2_FB2_Pos) ## !< 0x00000004
  CAN_F8R2_FB2* = CAN_F8R2_FB2_Msk
  CAN_F8R2_FB3_Pos* = (3)
  CAN_F8R2_FB3_Msk* = (0x00000001 shl CAN_F8R2_FB3_Pos) ## !< 0x00000008
  CAN_F8R2_FB3* = CAN_F8R2_FB3_Msk
  CAN_F8R2_FB4_Pos* = (4)
  CAN_F8R2_FB4_Msk* = (0x00000001 shl CAN_F8R2_FB4_Pos) ## !< 0x00000010
  CAN_F8R2_FB4* = CAN_F8R2_FB4_Msk
  CAN_F8R2_FB5_Pos* = (5)
  CAN_F8R2_FB5_Msk* = (0x00000001 shl CAN_F8R2_FB5_Pos) ## !< 0x00000020
  CAN_F8R2_FB5* = CAN_F8R2_FB5_Msk
  CAN_F8R2_FB6_Pos* = (6)
  CAN_F8R2_FB6_Msk* = (0x00000001 shl CAN_F8R2_FB6_Pos) ## !< 0x00000040
  CAN_F8R2_FB6* = CAN_F8R2_FB6_Msk
  CAN_F8R2_FB7_Pos* = (7)
  CAN_F8R2_FB7_Msk* = (0x00000001 shl CAN_F8R2_FB7_Pos) ## !< 0x00000080
  CAN_F8R2_FB7* = CAN_F8R2_FB7_Msk
  CAN_F8R2_FB8_Pos* = (8)
  CAN_F8R2_FB8_Msk* = (0x00000001 shl CAN_F8R2_FB8_Pos) ## !< 0x00000100
  CAN_F8R2_FB8* = CAN_F8R2_FB8_Msk
  CAN_F8R2_FB9_Pos* = (9)
  CAN_F8R2_FB9_Msk* = (0x00000001 shl CAN_F8R2_FB9_Pos) ## !< 0x00000200
  CAN_F8R2_FB9* = CAN_F8R2_FB9_Msk
  CAN_F8R2_FB10_Pos* = (10)
  CAN_F8R2_FB10_Msk* = (0x00000001 shl CAN_F8R2_FB10_Pos) ## !< 0x00000400
  CAN_F8R2_FB10* = CAN_F8R2_FB10_Msk
  CAN_F8R2_FB11_Pos* = (11)
  CAN_F8R2_FB11_Msk* = (0x00000001 shl CAN_F8R2_FB11_Pos) ## !< 0x00000800
  CAN_F8R2_FB11* = CAN_F8R2_FB11_Msk
  CAN_F8R2_FB12_Pos* = (12)
  CAN_F8R2_FB12_Msk* = (0x00000001 shl CAN_F8R2_FB12_Pos) ## !< 0x00001000
  CAN_F8R2_FB12* = CAN_F8R2_FB12_Msk
  CAN_F8R2_FB13_Pos* = (13)
  CAN_F8R2_FB13_Msk* = (0x00000001 shl CAN_F8R2_FB13_Pos) ## !< 0x00002000
  CAN_F8R2_FB13* = CAN_F8R2_FB13_Msk
  CAN_F8R2_FB14_Pos* = (14)
  CAN_F8R2_FB14_Msk* = (0x00000001 shl CAN_F8R2_FB14_Pos) ## !< 0x00004000
  CAN_F8R2_FB14* = CAN_F8R2_FB14_Msk
  CAN_F8R2_FB15_Pos* = (15)
  CAN_F8R2_FB15_Msk* = (0x00000001 shl CAN_F8R2_FB15_Pos) ## !< 0x00008000
  CAN_F8R2_FB15* = CAN_F8R2_FB15_Msk
  CAN_F8R2_FB16_Pos* = (16)
  CAN_F8R2_FB16_Msk* = (0x00000001 shl CAN_F8R2_FB16_Pos) ## !< 0x00010000
  CAN_F8R2_FB16* = CAN_F8R2_FB16_Msk
  CAN_F8R2_FB17_Pos* = (17)
  CAN_F8R2_FB17_Msk* = (0x00000001 shl CAN_F8R2_FB17_Pos) ## !< 0x00020000
  CAN_F8R2_FB17* = CAN_F8R2_FB17_Msk
  CAN_F8R2_FB18_Pos* = (18)
  CAN_F8R2_FB18_Msk* = (0x00000001 shl CAN_F8R2_FB18_Pos) ## !< 0x00040000
  CAN_F8R2_FB18* = CAN_F8R2_FB18_Msk
  CAN_F8R2_FB19_Pos* = (19)
  CAN_F8R2_FB19_Msk* = (0x00000001 shl CAN_F8R2_FB19_Pos) ## !< 0x00080000
  CAN_F8R2_FB19* = CAN_F8R2_FB19_Msk
  CAN_F8R2_FB20_Pos* = (20)
  CAN_F8R2_FB20_Msk* = (0x00000001 shl CAN_F8R2_FB20_Pos) ## !< 0x00100000
  CAN_F8R2_FB20* = CAN_F8R2_FB20_Msk
  CAN_F8R2_FB21_Pos* = (21)
  CAN_F8R2_FB21_Msk* = (0x00000001 shl CAN_F8R2_FB21_Pos) ## !< 0x00200000
  CAN_F8R2_FB21* = CAN_F8R2_FB21_Msk
  CAN_F8R2_FB22_Pos* = (22)
  CAN_F8R2_FB22_Msk* = (0x00000001 shl CAN_F8R2_FB22_Pos) ## !< 0x00400000
  CAN_F8R2_FB22* = CAN_F8R2_FB22_Msk
  CAN_F8R2_FB23_Pos* = (23)
  CAN_F8R2_FB23_Msk* = (0x00000001 shl CAN_F8R2_FB23_Pos) ## !< 0x00800000
  CAN_F8R2_FB23* = CAN_F8R2_FB23_Msk
  CAN_F8R2_FB24_Pos* = (24)
  CAN_F8R2_FB24_Msk* = (0x00000001 shl CAN_F8R2_FB24_Pos) ## !< 0x01000000
  CAN_F8R2_FB24* = CAN_F8R2_FB24_Msk
  CAN_F8R2_FB25_Pos* = (25)
  CAN_F8R2_FB25_Msk* = (0x00000001 shl CAN_F8R2_FB25_Pos) ## !< 0x02000000
  CAN_F8R2_FB25* = CAN_F8R2_FB25_Msk
  CAN_F8R2_FB26_Pos* = (26)
  CAN_F8R2_FB26_Msk* = (0x00000001 shl CAN_F8R2_FB26_Pos) ## !< 0x04000000
  CAN_F8R2_FB26* = CAN_F8R2_FB26_Msk
  CAN_F8R2_FB27_Pos* = (27)
  CAN_F8R2_FB27_Msk* = (0x00000001 shl CAN_F8R2_FB27_Pos) ## !< 0x08000000
  CAN_F8R2_FB27* = CAN_F8R2_FB27_Msk
  CAN_F8R2_FB28_Pos* = (28)
  CAN_F8R2_FB28_Msk* = (0x00000001 shl CAN_F8R2_FB28_Pos) ## !< 0x10000000
  CAN_F8R2_FB28* = CAN_F8R2_FB28_Msk
  CAN_F8R2_FB29_Pos* = (29)
  CAN_F8R2_FB29_Msk* = (0x00000001 shl CAN_F8R2_FB29_Pos) ## !< 0x20000000
  CAN_F8R2_FB29* = CAN_F8R2_FB29_Msk
  CAN_F8R2_FB30_Pos* = (30)
  CAN_F8R2_FB30_Msk* = (0x00000001 shl CAN_F8R2_FB30_Pos) ## !< 0x40000000
  CAN_F8R2_FB30* = CAN_F8R2_FB30_Msk
  CAN_F8R2_FB31_Pos* = (31)
  CAN_F8R2_FB31_Msk* = (0x00000001 shl CAN_F8R2_FB31_Pos) ## !< 0x80000000
  CAN_F8R2_FB31* = CAN_F8R2_FB31_Msk

## ******************  Bit definition for CAN_F9R2 register  ******************

const
  CAN_F9R2_FB0_Pos* = (0)
  CAN_F9R2_FB0_Msk* = (0x00000001 shl CAN_F9R2_FB0_Pos) ## !< 0x00000001
  CAN_F9R2_FB0* = CAN_F9R2_FB0_Msk
  CAN_F9R2_FB1_Pos* = (1)
  CAN_F9R2_FB1_Msk* = (0x00000001 shl CAN_F9R2_FB1_Pos) ## !< 0x00000002
  CAN_F9R2_FB1* = CAN_F9R2_FB1_Msk
  CAN_F9R2_FB2_Pos* = (2)
  CAN_F9R2_FB2_Msk* = (0x00000001 shl CAN_F9R2_FB2_Pos) ## !< 0x00000004
  CAN_F9R2_FB2* = CAN_F9R2_FB2_Msk
  CAN_F9R2_FB3_Pos* = (3)
  CAN_F9R2_FB3_Msk* = (0x00000001 shl CAN_F9R2_FB3_Pos) ## !< 0x00000008
  CAN_F9R2_FB3* = CAN_F9R2_FB3_Msk
  CAN_F9R2_FB4_Pos* = (4)
  CAN_F9R2_FB4_Msk* = (0x00000001 shl CAN_F9R2_FB4_Pos) ## !< 0x00000010
  CAN_F9R2_FB4* = CAN_F9R2_FB4_Msk
  CAN_F9R2_FB5_Pos* = (5)
  CAN_F9R2_FB5_Msk* = (0x00000001 shl CAN_F9R2_FB5_Pos) ## !< 0x00000020
  CAN_F9R2_FB5* = CAN_F9R2_FB5_Msk
  CAN_F9R2_FB6_Pos* = (6)
  CAN_F9R2_FB6_Msk* = (0x00000001 shl CAN_F9R2_FB6_Pos) ## !< 0x00000040
  CAN_F9R2_FB6* = CAN_F9R2_FB6_Msk
  CAN_F9R2_FB7_Pos* = (7)
  CAN_F9R2_FB7_Msk* = (0x00000001 shl CAN_F9R2_FB7_Pos) ## !< 0x00000080
  CAN_F9R2_FB7* = CAN_F9R2_FB7_Msk
  CAN_F9R2_FB8_Pos* = (8)
  CAN_F9R2_FB8_Msk* = (0x00000001 shl CAN_F9R2_FB8_Pos) ## !< 0x00000100
  CAN_F9R2_FB8* = CAN_F9R2_FB8_Msk
  CAN_F9R2_FB9_Pos* = (9)
  CAN_F9R2_FB9_Msk* = (0x00000001 shl CAN_F9R2_FB9_Pos) ## !< 0x00000200
  CAN_F9R2_FB9* = CAN_F9R2_FB9_Msk
  CAN_F9R2_FB10_Pos* = (10)
  CAN_F9R2_FB10_Msk* = (0x00000001 shl CAN_F9R2_FB10_Pos) ## !< 0x00000400
  CAN_F9R2_FB10* = CAN_F9R2_FB10_Msk
  CAN_F9R2_FB11_Pos* = (11)
  CAN_F9R2_FB11_Msk* = (0x00000001 shl CAN_F9R2_FB11_Pos) ## !< 0x00000800
  CAN_F9R2_FB11* = CAN_F9R2_FB11_Msk
  CAN_F9R2_FB12_Pos* = (12)
  CAN_F9R2_FB12_Msk* = (0x00000001 shl CAN_F9R2_FB12_Pos) ## !< 0x00001000
  CAN_F9R2_FB12* = CAN_F9R2_FB12_Msk
  CAN_F9R2_FB13_Pos* = (13)
  CAN_F9R2_FB13_Msk* = (0x00000001 shl CAN_F9R2_FB13_Pos) ## !< 0x00002000
  CAN_F9R2_FB13* = CAN_F9R2_FB13_Msk
  CAN_F9R2_FB14_Pos* = (14)
  CAN_F9R2_FB14_Msk* = (0x00000001 shl CAN_F9R2_FB14_Pos) ## !< 0x00004000
  CAN_F9R2_FB14* = CAN_F9R2_FB14_Msk
  CAN_F9R2_FB15_Pos* = (15)
  CAN_F9R2_FB15_Msk* = (0x00000001 shl CAN_F9R2_FB15_Pos) ## !< 0x00008000
  CAN_F9R2_FB15* = CAN_F9R2_FB15_Msk
  CAN_F9R2_FB16_Pos* = (16)
  CAN_F9R2_FB16_Msk* = (0x00000001 shl CAN_F9R2_FB16_Pos) ## !< 0x00010000
  CAN_F9R2_FB16* = CAN_F9R2_FB16_Msk
  CAN_F9R2_FB17_Pos* = (17)
  CAN_F9R2_FB17_Msk* = (0x00000001 shl CAN_F9R2_FB17_Pos) ## !< 0x00020000
  CAN_F9R2_FB17* = CAN_F9R2_FB17_Msk
  CAN_F9R2_FB18_Pos* = (18)
  CAN_F9R2_FB18_Msk* = (0x00000001 shl CAN_F9R2_FB18_Pos) ## !< 0x00040000
  CAN_F9R2_FB18* = CAN_F9R2_FB18_Msk
  CAN_F9R2_FB19_Pos* = (19)
  CAN_F9R2_FB19_Msk* = (0x00000001 shl CAN_F9R2_FB19_Pos) ## !< 0x00080000
  CAN_F9R2_FB19* = CAN_F9R2_FB19_Msk
  CAN_F9R2_FB20_Pos* = (20)
  CAN_F9R2_FB20_Msk* = (0x00000001 shl CAN_F9R2_FB20_Pos) ## !< 0x00100000
  CAN_F9R2_FB20* = CAN_F9R2_FB20_Msk
  CAN_F9R2_FB21_Pos* = (21)
  CAN_F9R2_FB21_Msk* = (0x00000001 shl CAN_F9R2_FB21_Pos) ## !< 0x00200000
  CAN_F9R2_FB21* = CAN_F9R2_FB21_Msk
  CAN_F9R2_FB22_Pos* = (22)
  CAN_F9R2_FB22_Msk* = (0x00000001 shl CAN_F9R2_FB22_Pos) ## !< 0x00400000
  CAN_F9R2_FB22* = CAN_F9R2_FB22_Msk
  CAN_F9R2_FB23_Pos* = (23)
  CAN_F9R2_FB23_Msk* = (0x00000001 shl CAN_F9R2_FB23_Pos) ## !< 0x00800000
  CAN_F9R2_FB23* = CAN_F9R2_FB23_Msk
  CAN_F9R2_FB24_Pos* = (24)
  CAN_F9R2_FB24_Msk* = (0x00000001 shl CAN_F9R2_FB24_Pos) ## !< 0x01000000
  CAN_F9R2_FB24* = CAN_F9R2_FB24_Msk
  CAN_F9R2_FB25_Pos* = (25)
  CAN_F9R2_FB25_Msk* = (0x00000001 shl CAN_F9R2_FB25_Pos) ## !< 0x02000000
  CAN_F9R2_FB25* = CAN_F9R2_FB25_Msk
  CAN_F9R2_FB26_Pos* = (26)
  CAN_F9R2_FB26_Msk* = (0x00000001 shl CAN_F9R2_FB26_Pos) ## !< 0x04000000
  CAN_F9R2_FB26* = CAN_F9R2_FB26_Msk
  CAN_F9R2_FB27_Pos* = (27)
  CAN_F9R2_FB27_Msk* = (0x00000001 shl CAN_F9R2_FB27_Pos) ## !< 0x08000000
  CAN_F9R2_FB27* = CAN_F9R2_FB27_Msk
  CAN_F9R2_FB28_Pos* = (28)
  CAN_F9R2_FB28_Msk* = (0x00000001 shl CAN_F9R2_FB28_Pos) ## !< 0x10000000
  CAN_F9R2_FB28* = CAN_F9R2_FB28_Msk
  CAN_F9R2_FB29_Pos* = (29)
  CAN_F9R2_FB29_Msk* = (0x00000001 shl CAN_F9R2_FB29_Pos) ## !< 0x20000000
  CAN_F9R2_FB29* = CAN_F9R2_FB29_Msk
  CAN_F9R2_FB30_Pos* = (30)
  CAN_F9R2_FB30_Msk* = (0x00000001 shl CAN_F9R2_FB30_Pos) ## !< 0x40000000
  CAN_F9R2_FB30* = CAN_F9R2_FB30_Msk
  CAN_F9R2_FB31_Pos* = (31)
  CAN_F9R2_FB31_Msk* = (0x00000001 shl CAN_F9R2_FB31_Pos) ## !< 0x80000000
  CAN_F9R2_FB31* = CAN_F9R2_FB31_Msk

## ******************  Bit definition for CAN_F10R2 register  *****************

const
  CAN_F10R2_FB0_Pos* = (0)
  CAN_F10R2_FB0_Msk* = (0x00000001 shl CAN_F10R2_FB0_Pos) ## !< 0x00000001
  CAN_F10R2_FB0* = CAN_F10R2_FB0_Msk
  CAN_F10R2_FB1_Pos* = (1)
  CAN_F10R2_FB1_Msk* = (0x00000001 shl CAN_F10R2_FB1_Pos) ## !< 0x00000002
  CAN_F10R2_FB1* = CAN_F10R2_FB1_Msk
  CAN_F10R2_FB2_Pos* = (2)
  CAN_F10R2_FB2_Msk* = (0x00000001 shl CAN_F10R2_FB2_Pos) ## !< 0x00000004
  CAN_F10R2_FB2* = CAN_F10R2_FB2_Msk
  CAN_F10R2_FB3_Pos* = (3)
  CAN_F10R2_FB3_Msk* = (0x00000001 shl CAN_F10R2_FB3_Pos) ## !< 0x00000008
  CAN_F10R2_FB3* = CAN_F10R2_FB3_Msk
  CAN_F10R2_FB4_Pos* = (4)
  CAN_F10R2_FB4_Msk* = (0x00000001 shl CAN_F10R2_FB4_Pos) ## !< 0x00000010
  CAN_F10R2_FB4* = CAN_F10R2_FB4_Msk
  CAN_F10R2_FB5_Pos* = (5)
  CAN_F10R2_FB5_Msk* = (0x00000001 shl CAN_F10R2_FB5_Pos) ## !< 0x00000020
  CAN_F10R2_FB5* = CAN_F10R2_FB5_Msk
  CAN_F10R2_FB6_Pos* = (6)
  CAN_F10R2_FB6_Msk* = (0x00000001 shl CAN_F10R2_FB6_Pos) ## !< 0x00000040
  CAN_F10R2_FB6* = CAN_F10R2_FB6_Msk
  CAN_F10R2_FB7_Pos* = (7)
  CAN_F10R2_FB7_Msk* = (0x00000001 shl CAN_F10R2_FB7_Pos) ## !< 0x00000080
  CAN_F10R2_FB7* = CAN_F10R2_FB7_Msk
  CAN_F10R2_FB8_Pos* = (8)
  CAN_F10R2_FB8_Msk* = (0x00000001 shl CAN_F10R2_FB8_Pos) ## !< 0x00000100
  CAN_F10R2_FB8* = CAN_F10R2_FB8_Msk
  CAN_F10R2_FB9_Pos* = (9)
  CAN_F10R2_FB9_Msk* = (0x00000001 shl CAN_F10R2_FB9_Pos) ## !< 0x00000200
  CAN_F10R2_FB9* = CAN_F10R2_FB9_Msk
  CAN_F10R2_FB10_Pos* = (10)
  CAN_F10R2_FB10_Msk* = (0x00000001 shl CAN_F10R2_FB10_Pos) ## !< 0x00000400
  CAN_F10R2_FB10* = CAN_F10R2_FB10_Msk
  CAN_F10R2_FB11_Pos* = (11)
  CAN_F10R2_FB11_Msk* = (0x00000001 shl CAN_F10R2_FB11_Pos) ## !< 0x00000800
  CAN_F10R2_FB11* = CAN_F10R2_FB11_Msk
  CAN_F10R2_FB12_Pos* = (12)
  CAN_F10R2_FB12_Msk* = (0x00000001 shl CAN_F10R2_FB12_Pos) ## !< 0x00001000
  CAN_F10R2_FB12* = CAN_F10R2_FB12_Msk
  CAN_F10R2_FB13_Pos* = (13)
  CAN_F10R2_FB13_Msk* = (0x00000001 shl CAN_F10R2_FB13_Pos) ## !< 0x00002000
  CAN_F10R2_FB13* = CAN_F10R2_FB13_Msk
  CAN_F10R2_FB14_Pos* = (14)
  CAN_F10R2_FB14_Msk* = (0x00000001 shl CAN_F10R2_FB14_Pos) ## !< 0x00004000
  CAN_F10R2_FB14* = CAN_F10R2_FB14_Msk
  CAN_F10R2_FB15_Pos* = (15)
  CAN_F10R2_FB15_Msk* = (0x00000001 shl CAN_F10R2_FB15_Pos) ## !< 0x00008000
  CAN_F10R2_FB15* = CAN_F10R2_FB15_Msk
  CAN_F10R2_FB16_Pos* = (16)
  CAN_F10R2_FB16_Msk* = (0x00000001 shl CAN_F10R2_FB16_Pos) ## !< 0x00010000
  CAN_F10R2_FB16* = CAN_F10R2_FB16_Msk
  CAN_F10R2_FB17_Pos* = (17)
  CAN_F10R2_FB17_Msk* = (0x00000001 shl CAN_F10R2_FB17_Pos) ## !< 0x00020000
  CAN_F10R2_FB17* = CAN_F10R2_FB17_Msk
  CAN_F10R2_FB18_Pos* = (18)
  CAN_F10R2_FB18_Msk* = (0x00000001 shl CAN_F10R2_FB18_Pos) ## !< 0x00040000
  CAN_F10R2_FB18* = CAN_F10R2_FB18_Msk
  CAN_F10R2_FB19_Pos* = (19)
  CAN_F10R2_FB19_Msk* = (0x00000001 shl CAN_F10R2_FB19_Pos) ## !< 0x00080000
  CAN_F10R2_FB19* = CAN_F10R2_FB19_Msk
  CAN_F10R2_FB20_Pos* = (20)
  CAN_F10R2_FB20_Msk* = (0x00000001 shl CAN_F10R2_FB20_Pos) ## !< 0x00100000
  CAN_F10R2_FB20* = CAN_F10R2_FB20_Msk
  CAN_F10R2_FB21_Pos* = (21)
  CAN_F10R2_FB21_Msk* = (0x00000001 shl CAN_F10R2_FB21_Pos) ## !< 0x00200000
  CAN_F10R2_FB21* = CAN_F10R2_FB21_Msk
  CAN_F10R2_FB22_Pos* = (22)
  CAN_F10R2_FB22_Msk* = (0x00000001 shl CAN_F10R2_FB22_Pos) ## !< 0x00400000
  CAN_F10R2_FB22* = CAN_F10R2_FB22_Msk
  CAN_F10R2_FB23_Pos* = (23)
  CAN_F10R2_FB23_Msk* = (0x00000001 shl CAN_F10R2_FB23_Pos) ## !< 0x00800000
  CAN_F10R2_FB23* = CAN_F10R2_FB23_Msk
  CAN_F10R2_FB24_Pos* = (24)
  CAN_F10R2_FB24_Msk* = (0x00000001 shl CAN_F10R2_FB24_Pos) ## !< 0x01000000
  CAN_F10R2_FB24* = CAN_F10R2_FB24_Msk
  CAN_F10R2_FB25_Pos* = (25)
  CAN_F10R2_FB25_Msk* = (0x00000001 shl CAN_F10R2_FB25_Pos) ## !< 0x02000000
  CAN_F10R2_FB25* = CAN_F10R2_FB25_Msk
  CAN_F10R2_FB26_Pos* = (26)
  CAN_F10R2_FB26_Msk* = (0x00000001 shl CAN_F10R2_FB26_Pos) ## !< 0x04000000
  CAN_F10R2_FB26* = CAN_F10R2_FB26_Msk
  CAN_F10R2_FB27_Pos* = (27)
  CAN_F10R2_FB27_Msk* = (0x00000001 shl CAN_F10R2_FB27_Pos) ## !< 0x08000000
  CAN_F10R2_FB27* = CAN_F10R2_FB27_Msk
  CAN_F10R2_FB28_Pos* = (28)
  CAN_F10R2_FB28_Msk* = (0x00000001 shl CAN_F10R2_FB28_Pos) ## !< 0x10000000
  CAN_F10R2_FB28* = CAN_F10R2_FB28_Msk
  CAN_F10R2_FB29_Pos* = (29)
  CAN_F10R2_FB29_Msk* = (0x00000001 shl CAN_F10R2_FB29_Pos) ## !< 0x20000000
  CAN_F10R2_FB29* = CAN_F10R2_FB29_Msk
  CAN_F10R2_FB30_Pos* = (30)
  CAN_F10R2_FB30_Msk* = (0x00000001 shl CAN_F10R2_FB30_Pos) ## !< 0x40000000
  CAN_F10R2_FB30* = CAN_F10R2_FB30_Msk
  CAN_F10R2_FB31_Pos* = (31)
  CAN_F10R2_FB31_Msk* = (0x00000001 shl CAN_F10R2_FB31_Pos) ## !< 0x80000000
  CAN_F10R2_FB31* = CAN_F10R2_FB31_Msk

## ******************  Bit definition for CAN_F11R2 register  *****************

const
  CAN_F11R2_FB0_Pos* = (0)
  CAN_F11R2_FB0_Msk* = (0x00000001 shl CAN_F11R2_FB0_Pos) ## !< 0x00000001
  CAN_F11R2_FB0* = CAN_F11R2_FB0_Msk
  CAN_F11R2_FB1_Pos* = (1)
  CAN_F11R2_FB1_Msk* = (0x00000001 shl CAN_F11R2_FB1_Pos) ## !< 0x00000002
  CAN_F11R2_FB1* = CAN_F11R2_FB1_Msk
  CAN_F11R2_FB2_Pos* = (2)
  CAN_F11R2_FB2_Msk* = (0x00000001 shl CAN_F11R2_FB2_Pos) ## !< 0x00000004
  CAN_F11R2_FB2* = CAN_F11R2_FB2_Msk
  CAN_F11R2_FB3_Pos* = (3)
  CAN_F11R2_FB3_Msk* = (0x00000001 shl CAN_F11R2_FB3_Pos) ## !< 0x00000008
  CAN_F11R2_FB3* = CAN_F11R2_FB3_Msk
  CAN_F11R2_FB4_Pos* = (4)
  CAN_F11R2_FB4_Msk* = (0x00000001 shl CAN_F11R2_FB4_Pos) ## !< 0x00000010
  CAN_F11R2_FB4* = CAN_F11R2_FB4_Msk
  CAN_F11R2_FB5_Pos* = (5)
  CAN_F11R2_FB5_Msk* = (0x00000001 shl CAN_F11R2_FB5_Pos) ## !< 0x00000020
  CAN_F11R2_FB5* = CAN_F11R2_FB5_Msk
  CAN_F11R2_FB6_Pos* = (6)
  CAN_F11R2_FB6_Msk* = (0x00000001 shl CAN_F11R2_FB6_Pos) ## !< 0x00000040
  CAN_F11R2_FB6* = CAN_F11R2_FB6_Msk
  CAN_F11R2_FB7_Pos* = (7)
  CAN_F11R2_FB7_Msk* = (0x00000001 shl CAN_F11R2_FB7_Pos) ## !< 0x00000080
  CAN_F11R2_FB7* = CAN_F11R2_FB7_Msk
  CAN_F11R2_FB8_Pos* = (8)
  CAN_F11R2_FB8_Msk* = (0x00000001 shl CAN_F11R2_FB8_Pos) ## !< 0x00000100
  CAN_F11R2_FB8* = CAN_F11R2_FB8_Msk
  CAN_F11R2_FB9_Pos* = (9)
  CAN_F11R2_FB9_Msk* = (0x00000001 shl CAN_F11R2_FB9_Pos) ## !< 0x00000200
  CAN_F11R2_FB9* = CAN_F11R2_FB9_Msk
  CAN_F11R2_FB10_Pos* = (10)
  CAN_F11R2_FB10_Msk* = (0x00000001 shl CAN_F11R2_FB10_Pos) ## !< 0x00000400
  CAN_F11R2_FB10* = CAN_F11R2_FB10_Msk
  CAN_F11R2_FB11_Pos* = (11)
  CAN_F11R2_FB11_Msk* = (0x00000001 shl CAN_F11R2_FB11_Pos) ## !< 0x00000800
  CAN_F11R2_FB11* = CAN_F11R2_FB11_Msk
  CAN_F11R2_FB12_Pos* = (12)
  CAN_F11R2_FB12_Msk* = (0x00000001 shl CAN_F11R2_FB12_Pos) ## !< 0x00001000
  CAN_F11R2_FB12* = CAN_F11R2_FB12_Msk
  CAN_F11R2_FB13_Pos* = (13)
  CAN_F11R2_FB13_Msk* = (0x00000001 shl CAN_F11R2_FB13_Pos) ## !< 0x00002000
  CAN_F11R2_FB13* = CAN_F11R2_FB13_Msk
  CAN_F11R2_FB14_Pos* = (14)
  CAN_F11R2_FB14_Msk* = (0x00000001 shl CAN_F11R2_FB14_Pos) ## !< 0x00004000
  CAN_F11R2_FB14* = CAN_F11R2_FB14_Msk
  CAN_F11R2_FB15_Pos* = (15)
  CAN_F11R2_FB15_Msk* = (0x00000001 shl CAN_F11R2_FB15_Pos) ## !< 0x00008000
  CAN_F11R2_FB15* = CAN_F11R2_FB15_Msk
  CAN_F11R2_FB16_Pos* = (16)
  CAN_F11R2_FB16_Msk* = (0x00000001 shl CAN_F11R2_FB16_Pos) ## !< 0x00010000
  CAN_F11R2_FB16* = CAN_F11R2_FB16_Msk
  CAN_F11R2_FB17_Pos* = (17)
  CAN_F11R2_FB17_Msk* = (0x00000001 shl CAN_F11R2_FB17_Pos) ## !< 0x00020000
  CAN_F11R2_FB17* = CAN_F11R2_FB17_Msk
  CAN_F11R2_FB18_Pos* = (18)
  CAN_F11R2_FB18_Msk* = (0x00000001 shl CAN_F11R2_FB18_Pos) ## !< 0x00040000
  CAN_F11R2_FB18* = CAN_F11R2_FB18_Msk
  CAN_F11R2_FB19_Pos* = (19)
  CAN_F11R2_FB19_Msk* = (0x00000001 shl CAN_F11R2_FB19_Pos) ## !< 0x00080000
  CAN_F11R2_FB19* = CAN_F11R2_FB19_Msk
  CAN_F11R2_FB20_Pos* = (20)
  CAN_F11R2_FB20_Msk* = (0x00000001 shl CAN_F11R2_FB20_Pos) ## !< 0x00100000
  CAN_F11R2_FB20* = CAN_F11R2_FB20_Msk
  CAN_F11R2_FB21_Pos* = (21)
  CAN_F11R2_FB21_Msk* = (0x00000001 shl CAN_F11R2_FB21_Pos) ## !< 0x00200000
  CAN_F11R2_FB21* = CAN_F11R2_FB21_Msk
  CAN_F11R2_FB22_Pos* = (22)
  CAN_F11R2_FB22_Msk* = (0x00000001 shl CAN_F11R2_FB22_Pos) ## !< 0x00400000
  CAN_F11R2_FB22* = CAN_F11R2_FB22_Msk
  CAN_F11R2_FB23_Pos* = (23)
  CAN_F11R2_FB23_Msk* = (0x00000001 shl CAN_F11R2_FB23_Pos) ## !< 0x00800000
  CAN_F11R2_FB23* = CAN_F11R2_FB23_Msk
  CAN_F11R2_FB24_Pos* = (24)
  CAN_F11R2_FB24_Msk* = (0x00000001 shl CAN_F11R2_FB24_Pos) ## !< 0x01000000
  CAN_F11R2_FB24* = CAN_F11R2_FB24_Msk
  CAN_F11R2_FB25_Pos* = (25)
  CAN_F11R2_FB25_Msk* = (0x00000001 shl CAN_F11R2_FB25_Pos) ## !< 0x02000000
  CAN_F11R2_FB25* = CAN_F11R2_FB25_Msk
  CAN_F11R2_FB26_Pos* = (26)
  CAN_F11R2_FB26_Msk* = (0x00000001 shl CAN_F11R2_FB26_Pos) ## !< 0x04000000
  CAN_F11R2_FB26* = CAN_F11R2_FB26_Msk
  CAN_F11R2_FB27_Pos* = (27)
  CAN_F11R2_FB27_Msk* = (0x00000001 shl CAN_F11R2_FB27_Pos) ## !< 0x08000000
  CAN_F11R2_FB27* = CAN_F11R2_FB27_Msk
  CAN_F11R2_FB28_Pos* = (28)
  CAN_F11R2_FB28_Msk* = (0x00000001 shl CAN_F11R2_FB28_Pos) ## !< 0x10000000
  CAN_F11R2_FB28* = CAN_F11R2_FB28_Msk
  CAN_F11R2_FB29_Pos* = (29)
  CAN_F11R2_FB29_Msk* = (0x00000001 shl CAN_F11R2_FB29_Pos) ## !< 0x20000000
  CAN_F11R2_FB29* = CAN_F11R2_FB29_Msk
  CAN_F11R2_FB30_Pos* = (30)
  CAN_F11R2_FB30_Msk* = (0x00000001 shl CAN_F11R2_FB30_Pos) ## !< 0x40000000
  CAN_F11R2_FB30* = CAN_F11R2_FB30_Msk
  CAN_F11R2_FB31_Pos* = (31)
  CAN_F11R2_FB31_Msk* = (0x00000001 shl CAN_F11R2_FB31_Pos) ## !< 0x80000000
  CAN_F11R2_FB31* = CAN_F11R2_FB31_Msk

## ******************  Bit definition for CAN_F12R2 register  *****************

const
  CAN_F12R2_FB0_Pos* = (0)
  CAN_F12R2_FB0_Msk* = (0x00000001 shl CAN_F12R2_FB0_Pos) ## !< 0x00000001
  CAN_F12R2_FB0* = CAN_F12R2_FB0_Msk
  CAN_F12R2_FB1_Pos* = (1)
  CAN_F12R2_FB1_Msk* = (0x00000001 shl CAN_F12R2_FB1_Pos) ## !< 0x00000002
  CAN_F12R2_FB1* = CAN_F12R2_FB1_Msk
  CAN_F12R2_FB2_Pos* = (2)
  CAN_F12R2_FB2_Msk* = (0x00000001 shl CAN_F12R2_FB2_Pos) ## !< 0x00000004
  CAN_F12R2_FB2* = CAN_F12R2_FB2_Msk
  CAN_F12R2_FB3_Pos* = (3)
  CAN_F12R2_FB3_Msk* = (0x00000001 shl CAN_F12R2_FB3_Pos) ## !< 0x00000008
  CAN_F12R2_FB3* = CAN_F12R2_FB3_Msk
  CAN_F12R2_FB4_Pos* = (4)
  CAN_F12R2_FB4_Msk* = (0x00000001 shl CAN_F12R2_FB4_Pos) ## !< 0x00000010
  CAN_F12R2_FB4* = CAN_F12R2_FB4_Msk
  CAN_F12R2_FB5_Pos* = (5)
  CAN_F12R2_FB5_Msk* = (0x00000001 shl CAN_F12R2_FB5_Pos) ## !< 0x00000020
  CAN_F12R2_FB5* = CAN_F12R2_FB5_Msk
  CAN_F12R2_FB6_Pos* = (6)
  CAN_F12R2_FB6_Msk* = (0x00000001 shl CAN_F12R2_FB6_Pos) ## !< 0x00000040
  CAN_F12R2_FB6* = CAN_F12R2_FB6_Msk
  CAN_F12R2_FB7_Pos* = (7)
  CAN_F12R2_FB7_Msk* = (0x00000001 shl CAN_F12R2_FB7_Pos) ## !< 0x00000080
  CAN_F12R2_FB7* = CAN_F12R2_FB7_Msk
  CAN_F12R2_FB8_Pos* = (8)
  CAN_F12R2_FB8_Msk* = (0x00000001 shl CAN_F12R2_FB8_Pos) ## !< 0x00000100
  CAN_F12R2_FB8* = CAN_F12R2_FB8_Msk
  CAN_F12R2_FB9_Pos* = (9)
  CAN_F12R2_FB9_Msk* = (0x00000001 shl CAN_F12R2_FB9_Pos) ## !< 0x00000200
  CAN_F12R2_FB9* = CAN_F12R2_FB9_Msk
  CAN_F12R2_FB10_Pos* = (10)
  CAN_F12R2_FB10_Msk* = (0x00000001 shl CAN_F12R2_FB10_Pos) ## !< 0x00000400
  CAN_F12R2_FB10* = CAN_F12R2_FB10_Msk
  CAN_F12R2_FB11_Pos* = (11)
  CAN_F12R2_FB11_Msk* = (0x00000001 shl CAN_F12R2_FB11_Pos) ## !< 0x00000800
  CAN_F12R2_FB11* = CAN_F12R2_FB11_Msk
  CAN_F12R2_FB12_Pos* = (12)
  CAN_F12R2_FB12_Msk* = (0x00000001 shl CAN_F12R2_FB12_Pos) ## !< 0x00001000
  CAN_F12R2_FB12* = CAN_F12R2_FB12_Msk
  CAN_F12R2_FB13_Pos* = (13)
  CAN_F12R2_FB13_Msk* = (0x00000001 shl CAN_F12R2_FB13_Pos) ## !< 0x00002000
  CAN_F12R2_FB13* = CAN_F12R2_FB13_Msk
  CAN_F12R2_FB14_Pos* = (14)
  CAN_F12R2_FB14_Msk* = (0x00000001 shl CAN_F12R2_FB14_Pos) ## !< 0x00004000
  CAN_F12R2_FB14* = CAN_F12R2_FB14_Msk
  CAN_F12R2_FB15_Pos* = (15)
  CAN_F12R2_FB15_Msk* = (0x00000001 shl CAN_F12R2_FB15_Pos) ## !< 0x00008000
  CAN_F12R2_FB15* = CAN_F12R2_FB15_Msk
  CAN_F12R2_FB16_Pos* = (16)
  CAN_F12R2_FB16_Msk* = (0x00000001 shl CAN_F12R2_FB16_Pos) ## !< 0x00010000
  CAN_F12R2_FB16* = CAN_F12R2_FB16_Msk
  CAN_F12R2_FB17_Pos* = (17)
  CAN_F12R2_FB17_Msk* = (0x00000001 shl CAN_F12R2_FB17_Pos) ## !< 0x00020000
  CAN_F12R2_FB17* = CAN_F12R2_FB17_Msk
  CAN_F12R2_FB18_Pos* = (18)
  CAN_F12R2_FB18_Msk* = (0x00000001 shl CAN_F12R2_FB18_Pos) ## !< 0x00040000
  CAN_F12R2_FB18* = CAN_F12R2_FB18_Msk
  CAN_F12R2_FB19_Pos* = (19)
  CAN_F12R2_FB19_Msk* = (0x00000001 shl CAN_F12R2_FB19_Pos) ## !< 0x00080000
  CAN_F12R2_FB19* = CAN_F12R2_FB19_Msk
  CAN_F12R2_FB20_Pos* = (20)
  CAN_F12R2_FB20_Msk* = (0x00000001 shl CAN_F12R2_FB20_Pos) ## !< 0x00100000
  CAN_F12R2_FB20* = CAN_F12R2_FB20_Msk
  CAN_F12R2_FB21_Pos* = (21)
  CAN_F12R2_FB21_Msk* = (0x00000001 shl CAN_F12R2_FB21_Pos) ## !< 0x00200000
  CAN_F12R2_FB21* = CAN_F12R2_FB21_Msk
  CAN_F12R2_FB22_Pos* = (22)
  CAN_F12R2_FB22_Msk* = (0x00000001 shl CAN_F12R2_FB22_Pos) ## !< 0x00400000
  CAN_F12R2_FB22* = CAN_F12R2_FB22_Msk
  CAN_F12R2_FB23_Pos* = (23)
  CAN_F12R2_FB23_Msk* = (0x00000001 shl CAN_F12R2_FB23_Pos) ## !< 0x00800000
  CAN_F12R2_FB23* = CAN_F12R2_FB23_Msk
  CAN_F12R2_FB24_Pos* = (24)
  CAN_F12R2_FB24_Msk* = (0x00000001 shl CAN_F12R2_FB24_Pos) ## !< 0x01000000
  CAN_F12R2_FB24* = CAN_F12R2_FB24_Msk
  CAN_F12R2_FB25_Pos* = (25)
  CAN_F12R2_FB25_Msk* = (0x00000001 shl CAN_F12R2_FB25_Pos) ## !< 0x02000000
  CAN_F12R2_FB25* = CAN_F12R2_FB25_Msk
  CAN_F12R2_FB26_Pos* = (26)
  CAN_F12R2_FB26_Msk* = (0x00000001 shl CAN_F12R2_FB26_Pos) ## !< 0x04000000
  CAN_F12R2_FB26* = CAN_F12R2_FB26_Msk
  CAN_F12R2_FB27_Pos* = (27)
  CAN_F12R2_FB27_Msk* = (0x00000001 shl CAN_F12R2_FB27_Pos) ## !< 0x08000000
  CAN_F12R2_FB27* = CAN_F12R2_FB27_Msk
  CAN_F12R2_FB28_Pos* = (28)
  CAN_F12R2_FB28_Msk* = (0x00000001 shl CAN_F12R2_FB28_Pos) ## !< 0x10000000
  CAN_F12R2_FB28* = CAN_F12R2_FB28_Msk
  CAN_F12R2_FB29_Pos* = (29)
  CAN_F12R2_FB29_Msk* = (0x00000001 shl CAN_F12R2_FB29_Pos) ## !< 0x20000000
  CAN_F12R2_FB29* = CAN_F12R2_FB29_Msk
  CAN_F12R2_FB30_Pos* = (30)
  CAN_F12R2_FB30_Msk* = (0x00000001 shl CAN_F12R2_FB30_Pos) ## !< 0x40000000
  CAN_F12R2_FB30* = CAN_F12R2_FB30_Msk
  CAN_F12R2_FB31_Pos* = (31)
  CAN_F12R2_FB31_Msk* = (0x00000001 shl CAN_F12R2_FB31_Pos) ## !< 0x80000000
  CAN_F12R2_FB31* = CAN_F12R2_FB31_Msk

## ******************  Bit definition for CAN_F13R2 register  *****************

const
  CAN_F13R2_FB0_Pos* = (0)
  CAN_F13R2_FB0_Msk* = (0x00000001 shl CAN_F13R2_FB0_Pos) ## !< 0x00000001
  CAN_F13R2_FB0* = CAN_F13R2_FB0_Msk
  CAN_F13R2_FB1_Pos* = (1)
  CAN_F13R2_FB1_Msk* = (0x00000001 shl CAN_F13R2_FB1_Pos) ## !< 0x00000002
  CAN_F13R2_FB1* = CAN_F13R2_FB1_Msk
  CAN_F13R2_FB2_Pos* = (2)
  CAN_F13R2_FB2_Msk* = (0x00000001 shl CAN_F13R2_FB2_Pos) ## !< 0x00000004
  CAN_F13R2_FB2* = CAN_F13R2_FB2_Msk
  CAN_F13R2_FB3_Pos* = (3)
  CAN_F13R2_FB3_Msk* = (0x00000001 shl CAN_F13R2_FB3_Pos) ## !< 0x00000008
  CAN_F13R2_FB3* = CAN_F13R2_FB3_Msk
  CAN_F13R2_FB4_Pos* = (4)
  CAN_F13R2_FB4_Msk* = (0x00000001 shl CAN_F13R2_FB4_Pos) ## !< 0x00000010
  CAN_F13R2_FB4* = CAN_F13R2_FB4_Msk
  CAN_F13R2_FB5_Pos* = (5)
  CAN_F13R2_FB5_Msk* = (0x00000001 shl CAN_F13R2_FB5_Pos) ## !< 0x00000020
  CAN_F13R2_FB5* = CAN_F13R2_FB5_Msk
  CAN_F13R2_FB6_Pos* = (6)
  CAN_F13R2_FB6_Msk* = (0x00000001 shl CAN_F13R2_FB6_Pos) ## !< 0x00000040
  CAN_F13R2_FB6* = CAN_F13R2_FB6_Msk
  CAN_F13R2_FB7_Pos* = (7)
  CAN_F13R2_FB7_Msk* = (0x00000001 shl CAN_F13R2_FB7_Pos) ## !< 0x00000080
  CAN_F13R2_FB7* = CAN_F13R2_FB7_Msk
  CAN_F13R2_FB8_Pos* = (8)
  CAN_F13R2_FB8_Msk* = (0x00000001 shl CAN_F13R2_FB8_Pos) ## !< 0x00000100
  CAN_F13R2_FB8* = CAN_F13R2_FB8_Msk
  CAN_F13R2_FB9_Pos* = (9)
  CAN_F13R2_FB9_Msk* = (0x00000001 shl CAN_F13R2_FB9_Pos) ## !< 0x00000200
  CAN_F13R2_FB9* = CAN_F13R2_FB9_Msk
  CAN_F13R2_FB10_Pos* = (10)
  CAN_F13R2_FB10_Msk* = (0x00000001 shl CAN_F13R2_FB10_Pos) ## !< 0x00000400
  CAN_F13R2_FB10* = CAN_F13R2_FB10_Msk
  CAN_F13R2_FB11_Pos* = (11)
  CAN_F13R2_FB11_Msk* = (0x00000001 shl CAN_F13R2_FB11_Pos) ## !< 0x00000800
  CAN_F13R2_FB11* = CAN_F13R2_FB11_Msk
  CAN_F13R2_FB12_Pos* = (12)
  CAN_F13R2_FB12_Msk* = (0x00000001 shl CAN_F13R2_FB12_Pos) ## !< 0x00001000
  CAN_F13R2_FB12* = CAN_F13R2_FB12_Msk
  CAN_F13R2_FB13_Pos* = (13)
  CAN_F13R2_FB13_Msk* = (0x00000001 shl CAN_F13R2_FB13_Pos) ## !< 0x00002000
  CAN_F13R2_FB13* = CAN_F13R2_FB13_Msk
  CAN_F13R2_FB14_Pos* = (14)
  CAN_F13R2_FB14_Msk* = (0x00000001 shl CAN_F13R2_FB14_Pos) ## !< 0x00004000
  CAN_F13R2_FB14* = CAN_F13R2_FB14_Msk
  CAN_F13R2_FB15_Pos* = (15)
  CAN_F13R2_FB15_Msk* = (0x00000001 shl CAN_F13R2_FB15_Pos) ## !< 0x00008000
  CAN_F13R2_FB15* = CAN_F13R2_FB15_Msk
  CAN_F13R2_FB16_Pos* = (16)
  CAN_F13R2_FB16_Msk* = (0x00000001 shl CAN_F13R2_FB16_Pos) ## !< 0x00010000
  CAN_F13R2_FB16* = CAN_F13R2_FB16_Msk
  CAN_F13R2_FB17_Pos* = (17)
  CAN_F13R2_FB17_Msk* = (0x00000001 shl CAN_F13R2_FB17_Pos) ## !< 0x00020000
  CAN_F13R2_FB17* = CAN_F13R2_FB17_Msk
  CAN_F13R2_FB18_Pos* = (18)
  CAN_F13R2_FB18_Msk* = (0x00000001 shl CAN_F13R2_FB18_Pos) ## !< 0x00040000
  CAN_F13R2_FB18* = CAN_F13R2_FB18_Msk
  CAN_F13R2_FB19_Pos* = (19)
  CAN_F13R2_FB19_Msk* = (0x00000001 shl CAN_F13R2_FB19_Pos) ## !< 0x00080000
  CAN_F13R2_FB19* = CAN_F13R2_FB19_Msk
  CAN_F13R2_FB20_Pos* = (20)
  CAN_F13R2_FB20_Msk* = (0x00000001 shl CAN_F13R2_FB20_Pos) ## !< 0x00100000
  CAN_F13R2_FB20* = CAN_F13R2_FB20_Msk
  CAN_F13R2_FB21_Pos* = (21)
  CAN_F13R2_FB21_Msk* = (0x00000001 shl CAN_F13R2_FB21_Pos) ## !< 0x00200000
  CAN_F13R2_FB21* = CAN_F13R2_FB21_Msk
  CAN_F13R2_FB22_Pos* = (22)
  CAN_F13R2_FB22_Msk* = (0x00000001 shl CAN_F13R2_FB22_Pos) ## !< 0x00400000
  CAN_F13R2_FB22* = CAN_F13R2_FB22_Msk
  CAN_F13R2_FB23_Pos* = (23)
  CAN_F13R2_FB23_Msk* = (0x00000001 shl CAN_F13R2_FB23_Pos) ## !< 0x00800000
  CAN_F13R2_FB23* = CAN_F13R2_FB23_Msk
  CAN_F13R2_FB24_Pos* = (24)
  CAN_F13R2_FB24_Msk* = (0x00000001 shl CAN_F13R2_FB24_Pos) ## !< 0x01000000
  CAN_F13R2_FB24* = CAN_F13R2_FB24_Msk
  CAN_F13R2_FB25_Pos* = (25)
  CAN_F13R2_FB25_Msk* = (0x00000001 shl CAN_F13R2_FB25_Pos) ## !< 0x02000000
  CAN_F13R2_FB25* = CAN_F13R2_FB25_Msk
  CAN_F13R2_FB26_Pos* = (26)
  CAN_F13R2_FB26_Msk* = (0x00000001 shl CAN_F13R2_FB26_Pos) ## !< 0x04000000
  CAN_F13R2_FB26* = CAN_F13R2_FB26_Msk
  CAN_F13R2_FB27_Pos* = (27)
  CAN_F13R2_FB27_Msk* = (0x00000001 shl CAN_F13R2_FB27_Pos) ## !< 0x08000000
  CAN_F13R2_FB27* = CAN_F13R2_FB27_Msk
  CAN_F13R2_FB28_Pos* = (28)
  CAN_F13R2_FB28_Msk* = (0x00000001 shl CAN_F13R2_FB28_Pos) ## !< 0x10000000
  CAN_F13R2_FB28* = CAN_F13R2_FB28_Msk
  CAN_F13R2_FB29_Pos* = (29)
  CAN_F13R2_FB29_Msk* = (0x00000001 shl CAN_F13R2_FB29_Pos) ## !< 0x20000000
  CAN_F13R2_FB29* = CAN_F13R2_FB29_Msk
  CAN_F13R2_FB30_Pos* = (30)
  CAN_F13R2_FB30_Msk* = (0x00000001 shl CAN_F13R2_FB30_Pos) ## !< 0x40000000
  CAN_F13R2_FB30* = CAN_F13R2_FB30_Msk
  CAN_F13R2_FB31_Pos* = (31)
  CAN_F13R2_FB31_Msk* = (0x00000001 shl CAN_F13R2_FB31_Pos) ## !< 0x80000000
  CAN_F13R2_FB31* = CAN_F13R2_FB31_Msk

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
##                             Crypto Processor
##
## ****************************************************************************
## ****************** Bits definition for CRYP_CR register  *******************

const
  CRYP_CR_ALGODIR_Pos* = (2)
  CRYP_CR_ALGODIR_Msk* = (0x00000001 shl CRYP_CR_ALGODIR_Pos) ## !< 0x00000004
  CRYP_CR_ALGODIR* = CRYP_CR_ALGODIR_Msk
  CRYP_CR_ALGOMODE_Pos* = (3)
  CRYP_CR_ALGOMODE_Msk* = (0x00010007 shl CRYP_CR_ALGOMODE_Pos) ## !< 0x00080038
  CRYP_CR_ALGOMODE* = CRYP_CR_ALGOMODE_Msk
  CRYP_CR_ALGOMODE_Bit0* = (0x00000001 shl CRYP_CR_ALGOMODE_Pos) ## !< 0x00000008
  CRYP_CR_ALGOMODE_Bit1* = (0x00000002 shl CRYP_CR_ALGOMODE_Pos) ## !< 0x00000010
  CRYP_CR_ALGOMODE_Bit2* = (0x00000004 shl CRYP_CR_ALGOMODE_Pos) ## !< 0x00000020
  CRYP_CR_ALGOMODE_TDES_ECB* = 0x00000000
  CRYP_CR_ALGOMODE_TDES_CBC_Pos* = (3)
  CRYP_CR_ALGOMODE_TDES_CBC_Msk* = (0x00000001 shl CRYP_CR_ALGOMODE_TDES_CBC_Pos) ## !< 0x00000008
  CRYP_CR_ALGOMODE_TDES_CBC* = CRYP_CR_ALGOMODE_TDES_CBC_Msk
  CRYP_CR_ALGOMODE_DES_ECB_Pos* = (4)
  CRYP_CR_ALGOMODE_DES_ECB_Msk* = (0x00000001 shl CRYP_CR_ALGOMODE_DES_ECB_Pos) ## !< 0x00000010
  CRYP_CR_ALGOMODE_DES_ECB* = CRYP_CR_ALGOMODE_DES_ECB_Msk
  CRYP_CR_ALGOMODE_DES_CBC_Pos* = (3)
  CRYP_CR_ALGOMODE_DES_CBC_Msk* = (0x00000003 shl CRYP_CR_ALGOMODE_DES_CBC_Pos) ## !< 0x00000018
  CRYP_CR_ALGOMODE_DES_CBC* = CRYP_CR_ALGOMODE_DES_CBC_Msk
  CRYP_CR_ALGOMODE_AES_ECB_Pos* = (5)
  CRYP_CR_ALGOMODE_AES_ECB_Msk* = (0x00000001 shl CRYP_CR_ALGOMODE_AES_ECB_Pos) ## !< 0x00000020
  CRYP_CR_ALGOMODE_AES_ECB* = CRYP_CR_ALGOMODE_AES_ECB_Msk
  CRYP_CR_ALGOMODE_AES_CBC_Pos* = (3)
  CRYP_CR_ALGOMODE_AES_CBC_Msk* = (0x00000005 shl CRYP_CR_ALGOMODE_AES_CBC_Pos) ## !< 0x00000028
  CRYP_CR_ALGOMODE_AES_CBC* = CRYP_CR_ALGOMODE_AES_CBC_Msk
  CRYP_CR_ALGOMODE_AES_CTR_Pos* = (4)
  CRYP_CR_ALGOMODE_AES_CTR_Msk* = (0x00000003 shl CRYP_CR_ALGOMODE_AES_CTR_Pos) ## !< 0x00000030
  CRYP_CR_ALGOMODE_AES_CTR* = CRYP_CR_ALGOMODE_AES_CTR_Msk
  CRYP_CR_ALGOMODE_AES_KEY_Pos* = (3)
  CRYP_CR_ALGOMODE_AES_KEY_Msk* = (0x00000007 shl CRYP_CR_ALGOMODE_AES_KEY_Pos) ## !< 0x00000038
  CRYP_CR_ALGOMODE_AES_KEY* = CRYP_CR_ALGOMODE_AES_KEY_Msk
  CRYP_CR_DATATYPE_Pos* = (6)
  CRYP_CR_DATATYPE_Msk* = (0x00000003 shl CRYP_CR_DATATYPE_Pos) ## !< 0x000000C0
  CRYP_CR_DATATYPE* = CRYP_CR_DATATYPE_Msk
  CRYP_CR_DATATYPE_Bit0* = (0x00000001 shl CRYP_CR_DATATYPE_Pos) ## !< 0x00000040
  CRYP_CR_DATATYPE_Bit1* = (0x00000002 shl CRYP_CR_DATATYPE_Pos) ## !< 0x00000080
  CRYP_CR_KEYSIZE_Pos* = (8)
  CRYP_CR_KEYSIZE_Msk* = (0x00000003 shl CRYP_CR_KEYSIZE_Pos) ## !< 0x00000300
  CRYP_CR_KEYSIZE* = CRYP_CR_KEYSIZE_Msk
  CRYP_CR_KEYSIZE_Bit0* = (0x00000001 shl CRYP_CR_KEYSIZE_Pos) ## !< 0x00000100
  CRYP_CR_KEYSIZE_Bit1* = (0x00000002 shl CRYP_CR_KEYSIZE_Pos) ## !< 0x00000200
  CRYP_CR_FFLUSH_Pos* = (14)
  CRYP_CR_FFLUSH_Msk* = (0x00000001 shl CRYP_CR_FFLUSH_Pos) ## !< 0x00004000
  CRYP_CR_FFLUSH* = CRYP_CR_FFLUSH_Msk
  CRYP_CR_CRYPEN_Pos* = (15)
  CRYP_CR_CRYPEN_Msk* = (0x00000001 shl CRYP_CR_CRYPEN_Pos) ## !< 0x00008000
  CRYP_CR_CRYPEN* = CRYP_CR_CRYPEN_Msk
  CRYP_CR_GCM_CCMPH_Pos* = (16)
  CRYP_CR_GCM_CCMPH_Msk* = (0x00000003 shl CRYP_CR_GCM_CCMPH_Pos) ## !< 0x00030000
  CRYP_CR_GCM_CCMPH* = CRYP_CR_GCM_CCMPH_Msk
  CRYP_CR_GCM_CCMPH_Bit0* = (0x00000001 shl CRYP_CR_GCM_CCMPH_Pos) ## !< 0x00010000
  CRYP_CR_GCM_CCMPH_Bit1* = (0x00000002 shl CRYP_CR_GCM_CCMPH_Pos) ## !< 0x00020000
  CRYP_CR_ALGOMODE_Bit3* = 0x00080000

## ***************** Bits definition for CRYP_SR register  ********************

const
  CRYP_SR_IFEM_Pos* = (0)
  CRYP_SR_IFEM_Msk* = (0x00000001 shl CRYP_SR_IFEM_Pos) ## !< 0x00000001
  CRYP_SR_IFEM* = CRYP_SR_IFEM_Msk
  CRYP_SR_IFNF_Pos* = (1)
  CRYP_SR_IFNF_Msk* = (0x00000001 shl CRYP_SR_IFNF_Pos) ## !< 0x00000002
  CRYP_SR_IFNF* = CRYP_SR_IFNF_Msk
  CRYP_SR_OFNE_Pos* = (2)
  CRYP_SR_OFNE_Msk* = (0x00000001 shl CRYP_SR_OFNE_Pos) ## !< 0x00000004
  CRYP_SR_OFNE* = CRYP_SR_OFNE_Msk
  CRYP_SR_OFFU_Pos* = (3)
  CRYP_SR_OFFU_Msk* = (0x00000001 shl CRYP_SR_OFFU_Pos) ## !< 0x00000008
  CRYP_SR_OFFU* = CRYP_SR_OFFU_Msk
  CRYP_SR_BUSY_Pos* = (4)
  CRYP_SR_BUSY_Msk* = (0x00000001 shl CRYP_SR_BUSY_Pos) ## !< 0x00000010
  CRYP_SR_BUSY* = CRYP_SR_BUSY_Msk

## ***************** Bits definition for CRYP_DMACR register  *****************

const
  CRYP_DMACR_DIEN_Pos* = (0)
  CRYP_DMACR_DIEN_Msk* = (0x00000001 shl CRYP_DMACR_DIEN_Pos) ## !< 0x00000001
  CRYP_DMACR_DIEN* = CRYP_DMACR_DIEN_Msk
  CRYP_DMACR_DOEN_Pos* = (1)
  CRYP_DMACR_DOEN_Msk* = (0x00000001 shl CRYP_DMACR_DOEN_Pos) ## !< 0x00000002
  CRYP_DMACR_DOEN* = CRYP_DMACR_DOEN_Msk

## ****************  Bits definition for CRYP_IMSCR register  *****************

const
  CRYP_IMSCR_INIM_Pos* = (0)
  CRYP_IMSCR_INIM_Msk* = (0x00000001 shl CRYP_IMSCR_INIM_Pos) ## !< 0x00000001
  CRYP_IMSCR_INIM* = CRYP_IMSCR_INIM_Msk
  CRYP_IMSCR_OUTIM_Pos* = (1)
  CRYP_IMSCR_OUTIM_Msk* = (0x00000001 shl CRYP_IMSCR_OUTIM_Pos) ## !< 0x00000002
  CRYP_IMSCR_OUTIM* = CRYP_IMSCR_OUTIM_Msk

## ***************** Bits definition for CRYP_RISR register  ******************

const
  CRYP_RISR_OUTRIS_Pos* = (0)
  CRYP_RISR_OUTRIS_Msk* = (0x00000001 shl CRYP_RISR_OUTRIS_Pos) ## !< 0x00000001
  CRYP_RISR_OUTRIS* = CRYP_RISR_OUTRIS_Msk
  CRYP_RISR_INRIS_Pos* = (1)
  CRYP_RISR_INRIS_Msk* = (0x00000001 shl CRYP_RISR_INRIS_Pos) ## !< 0x00000002
  CRYP_RISR_INRIS* = CRYP_RISR_INRIS_Msk

## ***************** Bits definition for CRYP_MISR register  ******************

const
  CRYP_MISR_INMIS_Pos* = (0)
  CRYP_MISR_INMIS_Msk* = (0x00000001 shl CRYP_MISR_INMIS_Pos) ## !< 0x00000001
  CRYP_MISR_INMIS* = CRYP_MISR_INMIS_Msk
  CRYP_MISR_OUTMIS_Pos* = (1)
  CRYP_MISR_OUTMIS_Msk* = (0x00000001 shl CRYP_MISR_OUTMIS_Pos) ## !< 0x00000002
  CRYP_MISR_OUTMIS* = CRYP_MISR_OUTMIS_Msk

## ****************************************************************************
##
##                       Digital to Analog Converter
##
## ****************************************************************************
##
##  @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
##

const
  DAC_CHANNEL2_SUPPORT* = true  ## !< DAC feature available only on specific devices: availability of DAC channel 2

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
##                                     DCMI
##
## ****************************************************************************
## *******************  Bits definition for DCMI_CR register  *****************

const
  DCMI_CR_CAPTURE_Pos* = (0)
  DCMI_CR_CAPTURE_Msk* = (0x00000001 shl DCMI_CR_CAPTURE_Pos) ## !< 0x00000001
  DCMI_CR_CAPTURE* = DCMI_CR_CAPTURE_Msk
  DCMI_CR_CM_Pos* = (1)
  DCMI_CR_CM_Msk* = (0x00000001 shl DCMI_CR_CM_Pos) ## !< 0x00000002
  DCMI_CR_CM* = DCMI_CR_CM_Msk
  DCMI_CR_CROP_Pos* = (2)
  DCMI_CR_CROP_Msk* = (0x00000001 shl DCMI_CR_CROP_Pos) ## !< 0x00000004
  DCMI_CR_CROP* = DCMI_CR_CROP_Msk
  DCMI_CR_JPEG_Pos* = (3)
  DCMI_CR_JPEG_Msk* = (0x00000001 shl DCMI_CR_JPEG_Pos) ## !< 0x00000008
  DCMI_CR_JPEG* = DCMI_CR_JPEG_Msk
  DCMI_CR_ESS_Pos* = (4)
  DCMI_CR_ESS_Msk* = (0x00000001 shl DCMI_CR_ESS_Pos) ## !< 0x00000010
  DCMI_CR_ESS* = DCMI_CR_ESS_Msk
  DCMI_CR_PCKPOL_Pos* = (5)
  DCMI_CR_PCKPOL_Msk* = (0x00000001 shl DCMI_CR_PCKPOL_Pos) ## !< 0x00000020
  DCMI_CR_PCKPOL* = DCMI_CR_PCKPOL_Msk
  DCMI_CR_HSPOL_Pos* = (6)
  DCMI_CR_HSPOL_Msk* = (0x00000001 shl DCMI_CR_HSPOL_Pos) ## !< 0x00000040
  DCMI_CR_HSPOL* = DCMI_CR_HSPOL_Msk
  DCMI_CR_VSPOL_Pos* = (7)
  DCMI_CR_VSPOL_Msk* = (0x00000001 shl DCMI_CR_VSPOL_Pos) ## !< 0x00000080
  DCMI_CR_VSPOL* = DCMI_CR_VSPOL_Msk
  DCMI_CR_FCRC_Bit0* = 0x00000100
  DCMI_CR_FCRC_Bit1* = 0x00000200
  DCMI_CR_EDM_Bit0* = 0x00000400
  DCMI_CR_EDM_Bit1* = 0x00000800
  DCMI_CR_OUTEN_Pos* = (13)
  DCMI_CR_OUTEN_Msk* = (0x00000001 shl DCMI_CR_OUTEN_Pos) ## !< 0x00002000
  DCMI_CR_OUTEN* = DCMI_CR_OUTEN_Msk
  DCMI_CR_ENABLE_Pos* = (14)
  DCMI_CR_ENABLE_Msk* = (0x00000001 shl DCMI_CR_ENABLE_Pos) ## !< 0x00004000
  DCMI_CR_ENABLE* = DCMI_CR_ENABLE_Msk
  DCMI_CR_BSM_Bit0* = 0x00010000
  DCMI_CR_BSM_Bit1* = 0x00020000
  DCMI_CR_OEBS_Pos* = (18)
  DCMI_CR_OEBS_Msk* = (0x00000001 shl DCMI_CR_OEBS_Pos) ## !< 0x00040000
  DCMI_CR_OEBS* = DCMI_CR_OEBS_Msk
  DCMI_CR_LSM_Pos* = (19)
  DCMI_CR_LSM_Msk* = (0x00000001 shl DCMI_CR_LSM_Pos) ## !< 0x00080000
  DCMI_CR_LSM* = DCMI_CR_LSM_Msk
  DCMI_CR_OELS_Pos* = (20)
  DCMI_CR_OELS_Msk* = (0x00000001 shl DCMI_CR_OELS_Pos) ## !< 0x00100000
  DCMI_CR_OELS* = DCMI_CR_OELS_Msk

## *******************  Bits definition for DCMI_SR register  *****************

const
  DCMI_SR_HSYNC_Pos* = (0)
  DCMI_SR_HSYNC_Msk* = (0x00000001 shl DCMI_SR_HSYNC_Pos) ## !< 0x00000001
  DCMI_SR_HSYNC* = DCMI_SR_HSYNC_Msk
  DCMI_SR_VSYNC_Pos* = (1)
  DCMI_SR_VSYNC_Msk* = (0x00000001 shl DCMI_SR_VSYNC_Pos) ## !< 0x00000002
  DCMI_SR_VSYNC* = DCMI_SR_VSYNC_Msk
  DCMI_SR_FNE_Pos* = (2)
  DCMI_SR_FNE_Msk* = (0x00000001 shl DCMI_SR_FNE_Pos) ## !< 0x00000004
  DCMI_SR_FNE* = DCMI_SR_FNE_Msk

## *******************  Bits definition for DCMI_RIS register  ****************

const
  DCMI_RIS_FRAME_RIS_Pos* = (0)
  DCMI_RIS_FRAME_RIS_Msk* = (0x00000001 shl DCMI_RIS_FRAME_RIS_Pos) ## !< 0x00000001
  DCMI_RIS_FRAME_RIS* = DCMI_RIS_FRAME_RIS_Msk
  DCMI_RIS_OVR_RIS_Pos* = (1)
  DCMI_RIS_OVR_RIS_Msk* = (0x00000001 shl DCMI_RIS_OVR_RIS_Pos) ## !< 0x00000002
  DCMI_RIS_OVR_RIS* = DCMI_RIS_OVR_RIS_Msk
  DCMI_RIS_ERR_RIS_Pos* = (2)
  DCMI_RIS_ERR_RIS_Msk* = (0x00000001 shl DCMI_RIS_ERR_RIS_Pos) ## !< 0x00000004
  DCMI_RIS_ERR_RIS* = DCMI_RIS_ERR_RIS_Msk
  DCMI_RIS_VSYNC_RIS_Pos* = (3)
  DCMI_RIS_VSYNC_RIS_Msk* = (0x00000001 shl DCMI_RIS_VSYNC_RIS_Pos) ## !< 0x00000008
  DCMI_RIS_VSYNC_RIS* = DCMI_RIS_VSYNC_RIS_Msk
  DCMI_RIS_LINE_RIS_Pos* = (4)
  DCMI_RIS_LINE_RIS_Msk* = (0x00000001 shl DCMI_RIS_LINE_RIS_Pos) ## !< 0x00000010
  DCMI_RIS_LINE_RIS* = DCMI_RIS_LINE_RIS_Msk

##  Legacy defines

const
  DCMI_RISR_FRAME_RIS* = DCMI_RIS_FRAME_RIS
  DCMI_RISR_OVR_RIS* = DCMI_RIS_OVR_RIS
  DCMI_RISR_ERR_RIS* = DCMI_RIS_ERR_RIS
  DCMI_RISR_VSYNC_RIS* = DCMI_RIS_VSYNC_RIS
  DCMI_RISR_LINE_RIS* = DCMI_RIS_LINE_RIS
  DCMI_RISR_OVF_RIS* = DCMI_RIS_OVR_RIS

## *******************  Bits definition for DCMI_IER register  ****************

const
  DCMI_IER_FRAME_IE_Pos* = (0)
  DCMI_IER_FRAME_IE_Msk* = (0x00000001 shl DCMI_IER_FRAME_IE_Pos) ## !< 0x00000001
  DCMI_IER_FRAME_IE* = DCMI_IER_FRAME_IE_Msk
  DCMI_IER_OVR_IE_Pos* = (1)
  DCMI_IER_OVR_IE_Msk* = (0x00000001 shl DCMI_IER_OVR_IE_Pos) ## !< 0x00000002
  DCMI_IER_OVR_IE* = DCMI_IER_OVR_IE_Msk
  DCMI_IER_ERR_IE_Pos* = (2)
  DCMI_IER_ERR_IE_Msk* = (0x00000001 shl DCMI_IER_ERR_IE_Pos) ## !< 0x00000004
  DCMI_IER_ERR_IE* = DCMI_IER_ERR_IE_Msk
  DCMI_IER_VSYNC_IE_Pos* = (3)
  DCMI_IER_VSYNC_IE_Msk* = (0x00000001 shl DCMI_IER_VSYNC_IE_Pos) ## !< 0x00000008
  DCMI_IER_VSYNC_IE* = DCMI_IER_VSYNC_IE_Msk
  DCMI_IER_LINE_IE_Pos* = (4)
  DCMI_IER_LINE_IE_Msk* = (0x00000001 shl DCMI_IER_LINE_IE_Pos) ## !< 0x00000010
  DCMI_IER_LINE_IE* = DCMI_IER_LINE_IE_Msk

##  Legacy defines

const
  DCMI_IER_OVF_IE* = DCMI_IER_OVR_IE

## *******************  Bits definition for DCMI_MIS register  ****************

const
  DCMI_MIS_FRAME_MIS_Pos* = (0)
  DCMI_MIS_FRAME_MIS_Msk* = (0x00000001 shl DCMI_MIS_FRAME_MIS_Pos) ## !< 0x00000001
  DCMI_MIS_FRAME_MIS* = DCMI_MIS_FRAME_MIS_Msk
  DCMI_MIS_OVR_MIS_Pos* = (1)
  DCMI_MIS_OVR_MIS_Msk* = (0x00000001 shl DCMI_MIS_OVR_MIS_Pos) ## !< 0x00000002
  DCMI_MIS_OVR_MIS* = DCMI_MIS_OVR_MIS_Msk
  DCMI_MIS_ERR_MIS_Pos* = (2)
  DCMI_MIS_ERR_MIS_Msk* = (0x00000001 shl DCMI_MIS_ERR_MIS_Pos) ## !< 0x00000004
  DCMI_MIS_ERR_MIS* = DCMI_MIS_ERR_MIS_Msk
  DCMI_MIS_VSYNC_MIS_Pos* = (3)
  DCMI_MIS_VSYNC_MIS_Msk* = (0x00000001 shl DCMI_MIS_VSYNC_MIS_Pos) ## !< 0x00000008
  DCMI_MIS_VSYNC_MIS* = DCMI_MIS_VSYNC_MIS_Msk
  DCMI_MIS_LINE_MIS_Pos* = (4)
  DCMI_MIS_LINE_MIS_Msk* = (0x00000001 shl DCMI_MIS_LINE_MIS_Pos) ## !< 0x00000010
  DCMI_MIS_LINE_MIS* = DCMI_MIS_LINE_MIS_Msk

##  Legacy defines

const
  DCMI_MISR_FRAME_MIS* = DCMI_MIS_FRAME_MIS
  DCMI_MISR_OVF_MIS* = DCMI_MIS_OVR_MIS
  DCMI_MISR_ERR_MIS* = DCMI_MIS_ERR_MIS
  DCMI_MISR_VSYNC_MIS* = DCMI_MIS_VSYNC_MIS
  DCMI_MISR_LINE_MIS* = DCMI_MIS_LINE_MIS

## *******************  Bits definition for DCMI_ICR register  ****************

const
  DCMI_ICR_FRAME_ISC_Pos* = (0)
  DCMI_ICR_FRAME_ISC_Msk* = (0x00000001 shl DCMI_ICR_FRAME_ISC_Pos) ## !< 0x00000001
  DCMI_ICR_FRAME_ISC* = DCMI_ICR_FRAME_ISC_Msk
  DCMI_ICR_OVR_ISC_Pos* = (1)
  DCMI_ICR_OVR_ISC_Msk* = (0x00000001 shl DCMI_ICR_OVR_ISC_Pos) ## !< 0x00000002
  DCMI_ICR_OVR_ISC* = DCMI_ICR_OVR_ISC_Msk
  DCMI_ICR_ERR_ISC_Pos* = (2)
  DCMI_ICR_ERR_ISC_Msk* = (0x00000001 shl DCMI_ICR_ERR_ISC_Pos) ## !< 0x00000004
  DCMI_ICR_ERR_ISC* = DCMI_ICR_ERR_ISC_Msk
  DCMI_ICR_VSYNC_ISC_Pos* = (3)
  DCMI_ICR_VSYNC_ISC_Msk* = (0x00000001 shl DCMI_ICR_VSYNC_ISC_Pos) ## !< 0x00000008
  DCMI_ICR_VSYNC_ISC* = DCMI_ICR_VSYNC_ISC_Msk
  DCMI_ICR_LINE_ISC_Pos* = (4)
  DCMI_ICR_LINE_ISC_Msk* = (0x00000001 shl DCMI_ICR_LINE_ISC_Pos) ## !< 0x00000010
  DCMI_ICR_LINE_ISC* = DCMI_ICR_LINE_ISC_Msk

##  Legacy defines

const
  DCMI_ICR_OVF_ISC* = DCMI_ICR_OVR_ISC

## *******************  Bits definition for DCMI_ESCR register  *****************

const
  DCMI_ESCR_FSC_Pos* = (0)
  DCMI_ESCR_FSC_Msk* = (0x000000FF shl DCMI_ESCR_FSC_Pos) ## !< 0x000000FF
  DCMI_ESCR_FSC* = DCMI_ESCR_FSC_Msk
  DCMI_ESCR_LSC_Pos* = (8)
  DCMI_ESCR_LSC_Msk* = (0x000000FF shl DCMI_ESCR_LSC_Pos) ## !< 0x0000FF00
  DCMI_ESCR_LSC* = DCMI_ESCR_LSC_Msk
  DCMI_ESCR_LEC_Pos* = (16)
  DCMI_ESCR_LEC_Msk* = (0x000000FF shl DCMI_ESCR_LEC_Pos) ## !< 0x00FF0000
  DCMI_ESCR_LEC* = DCMI_ESCR_LEC_Msk
  DCMI_ESCR_FEC_Pos* = (24)
  DCMI_ESCR_FEC_Msk* = (0x000000FF shl DCMI_ESCR_FEC_Pos) ## !< 0xFF000000
  DCMI_ESCR_FEC* = DCMI_ESCR_FEC_Msk

## *******************  Bits definition for DCMI_ESUR register  *****************

const
  DCMI_ESUR_FSU_Pos* = (0)
  DCMI_ESUR_FSU_Msk* = (0x000000FF shl DCMI_ESUR_FSU_Pos) ## !< 0x000000FF
  DCMI_ESUR_FSU* = DCMI_ESUR_FSU_Msk
  DCMI_ESUR_LSU_Pos* = (8)
  DCMI_ESUR_LSU_Msk* = (0x000000FF shl DCMI_ESUR_LSU_Pos) ## !< 0x0000FF00
  DCMI_ESUR_LSU* = DCMI_ESUR_LSU_Msk
  DCMI_ESUR_LEU_Pos* = (16)
  DCMI_ESUR_LEU_Msk* = (0x000000FF shl DCMI_ESUR_LEU_Pos) ## !< 0x00FF0000
  DCMI_ESUR_LEU* = DCMI_ESUR_LEU_Msk
  DCMI_ESUR_FEU_Pos* = (24)
  DCMI_ESUR_FEU_Msk* = (0x000000FF shl DCMI_ESUR_FEU_Pos) ## !< 0xFF000000
  DCMI_ESUR_FEU* = DCMI_ESUR_FEU_Msk

## *******************  Bits definition for DCMI_CWSTRT register  *****************

const
  DCMI_CWSTRT_HOFFCNT_Pos* = (0)
  DCMI_CWSTRT_HOFFCNT_Msk* = (0x00003FFF shl DCMI_CWSTRT_HOFFCNT_Pos) ## !< 0x00003FFF
  DCMI_CWSTRT_HOFFCNT* = DCMI_CWSTRT_HOFFCNT_Msk
  DCMI_CWSTRT_VST_Pos* = (16)
  DCMI_CWSTRT_VST_Msk* = (0x00001FFF shl DCMI_CWSTRT_VST_Pos) ## !< 0x1FFF0000
  DCMI_CWSTRT_VST* = DCMI_CWSTRT_VST_Msk

## *******************  Bits definition for DCMI_CWSIZE register  *****************

const
  DCMI_CWSIZE_CAPCNT_Pos* = (0)
  DCMI_CWSIZE_CAPCNT_Msk* = (0x00003FFF shl DCMI_CWSIZE_CAPCNT_Pos) ## !< 0x00003FFF
  DCMI_CWSIZE_CAPCNT* = DCMI_CWSIZE_CAPCNT_Msk
  DCMI_CWSIZE_VLINE_Pos* = (16)
  DCMI_CWSIZE_VLINE_Msk* = (0x00003FFF shl DCMI_CWSIZE_VLINE_Pos) ## !< 0x3FFF0000
  DCMI_CWSIZE_VLINE* = DCMI_CWSIZE_VLINE_Msk

## *******************  Bits definition for DCMI_DR register  ********************

const
  DCMI_DR_BYTE0_Pos* = (0)
  DCMI_DR_BYTE0_Msk* = (0x000000FF shl DCMI_DR_BYTE0_Pos) ## !< 0x000000FF
  DCMI_DR_BYTE0* = DCMI_DR_BYTE0_Msk
  DCMI_DR_BYTE1_Pos* = (8)
  DCMI_DR_BYTE1_Msk* = (0x000000FF shl DCMI_DR_BYTE1_Pos) ## !< 0x0000FF00
  DCMI_DR_BYTE1* = DCMI_DR_BYTE1_Msk
  DCMI_DR_BYTE2_Pos* = (16)
  DCMI_DR_BYTE2_Msk* = (0x000000FF shl DCMI_DR_BYTE2_Pos) ## !< 0x00FF0000
  DCMI_DR_BYTE2* = DCMI_DR_BYTE2_Msk
  DCMI_DR_BYTE3_Pos* = (24)
  DCMI_DR_BYTE3_Msk* = (0x000000FF shl DCMI_DR_BYTE3_Pos) ## !< 0xFF000000
  DCMI_DR_BYTE3* = DCMI_DR_BYTE3_Msk

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
##                          AHB Master DMA2D Controller (DMA2D)
##
## ****************************************************************************
## *******************  Bit definition for DMA2D_CR register  *****************

const
  DMA2D_CR_START_Pos* = (0)
  DMA2D_CR_START_Msk* = (0x00000001 shl DMA2D_CR_START_Pos) ## !< 0x00000001
  DMA2D_CR_START* = DMA2D_CR_START_Msk
  DMA2D_CR_SUSP_Pos* = (1)
  DMA2D_CR_SUSP_Msk* = (0x00000001 shl DMA2D_CR_SUSP_Pos) ## !< 0x00000002
  DMA2D_CR_SUSP* = DMA2D_CR_SUSP_Msk
  DMA2D_CR_ABORT_Pos* = (2)
  DMA2D_CR_ABORT_Msk* = (0x00000001 shl DMA2D_CR_ABORT_Pos) ## !< 0x00000004
  DMA2D_CR_ABORT* = DMA2D_CR_ABORT_Msk
  DMA2D_CR_TEIE_Pos* = (8)
  DMA2D_CR_TEIE_Msk* = (0x00000001 shl DMA2D_CR_TEIE_Pos) ## !< 0x00000100
  DMA2D_CR_TEIE* = DMA2D_CR_TEIE_Msk
  DMA2D_CR_TCIE_Pos* = (9)
  DMA2D_CR_TCIE_Msk* = (0x00000001 shl DMA2D_CR_TCIE_Pos) ## !< 0x00000200
  DMA2D_CR_TCIE* = DMA2D_CR_TCIE_Msk
  DMA2D_CR_TWIE_Pos* = (10)
  DMA2D_CR_TWIE_Msk* = (0x00000001 shl DMA2D_CR_TWIE_Pos) ## !< 0x00000400
  DMA2D_CR_TWIE* = DMA2D_CR_TWIE_Msk
  DMA2D_CR_CAEIE_Pos* = (11)
  DMA2D_CR_CAEIE_Msk* = (0x00000001 shl DMA2D_CR_CAEIE_Pos) ## !< 0x00000800
  DMA2D_CR_CAEIE* = DMA2D_CR_CAEIE_Msk
  DMA2D_CR_CTCIE_Pos* = (12)
  DMA2D_CR_CTCIE_Msk* = (0x00000001 shl DMA2D_CR_CTCIE_Pos) ## !< 0x00001000
  DMA2D_CR_CTCIE* = DMA2D_CR_CTCIE_Msk
  DMA2D_CR_CEIE_Pos* = (13)
  DMA2D_CR_CEIE_Msk* = (0x00000001 shl DMA2D_CR_CEIE_Pos) ## !< 0x00002000
  DMA2D_CR_CEIE* = DMA2D_CR_CEIE_Msk
  DMA2D_CR_MODE_Pos* = (16)
  DMA2D_CR_MODE_Msk* = (0x00000003 shl DMA2D_CR_MODE_Pos) ## !< 0x00030000
  DMA2D_CR_MODE* = DMA2D_CR_MODE_Msk
  DMA2D_CR_MODE_Bit0* = (0x00000001 shl DMA2D_CR_MODE_Pos) ## !< 0x00010000
  DMA2D_CR_MODE_Bit1* = (0x00000002 shl DMA2D_CR_MODE_Pos) ## !< 0x00020000

## *******************  Bit definition for DMA2D_ISR register  ****************

const
  DMA2D_ISR_TEIF_Pos* = (0)
  DMA2D_ISR_TEIF_Msk* = (0x00000001 shl DMA2D_ISR_TEIF_Pos) ## !< 0x00000001
  DMA2D_ISR_TEIF* = DMA2D_ISR_TEIF_Msk
  DMA2D_ISR_TCIF_Pos* = (1)
  DMA2D_ISR_TCIF_Msk* = (0x00000001 shl DMA2D_ISR_TCIF_Pos) ## !< 0x00000002
  DMA2D_ISR_TCIF* = DMA2D_ISR_TCIF_Msk
  DMA2D_ISR_TWIF_Pos* = (2)
  DMA2D_ISR_TWIF_Msk* = (0x00000001 shl DMA2D_ISR_TWIF_Pos) ## !< 0x00000004
  DMA2D_ISR_TWIF* = DMA2D_ISR_TWIF_Msk
  DMA2D_ISR_CAEIF_Pos* = (3)
  DMA2D_ISR_CAEIF_Msk* = (0x00000001 shl DMA2D_ISR_CAEIF_Pos) ## !< 0x00000008
  DMA2D_ISR_CAEIF* = DMA2D_ISR_CAEIF_Msk
  DMA2D_ISR_CTCIF_Pos* = (4)
  DMA2D_ISR_CTCIF_Msk* = (0x00000001 shl DMA2D_ISR_CTCIF_Pos) ## !< 0x00000010
  DMA2D_ISR_CTCIF* = DMA2D_ISR_CTCIF_Msk
  DMA2D_ISR_CEIF_Pos* = (5)
  DMA2D_ISR_CEIF_Msk* = (0x00000001 shl DMA2D_ISR_CEIF_Pos) ## !< 0x00000020
  DMA2D_ISR_CEIF* = DMA2D_ISR_CEIF_Msk

## *******************  Bit definition for DMA2D_IFCR register  ***************

const
  DMA2D_IFCR_CTEIF_Pos* = (0)
  DMA2D_IFCR_CTEIF_Msk* = (0x00000001 shl DMA2D_IFCR_CTEIF_Pos) ## !< 0x00000001
  DMA2D_IFCR_CTEIF* = DMA2D_IFCR_CTEIF_Msk
  DMA2D_IFCR_CTCIF_Pos* = (1)
  DMA2D_IFCR_CTCIF_Msk* = (0x00000001 shl DMA2D_IFCR_CTCIF_Pos) ## !< 0x00000002
  DMA2D_IFCR_CTCIF* = DMA2D_IFCR_CTCIF_Msk
  DMA2D_IFCR_CTWIF_Pos* = (2)
  DMA2D_IFCR_CTWIF_Msk* = (0x00000001 shl DMA2D_IFCR_CTWIF_Pos) ## !< 0x00000004
  DMA2D_IFCR_CTWIF* = DMA2D_IFCR_CTWIF_Msk
  DMA2D_IFCR_CAECIF_Pos* = (3)
  DMA2D_IFCR_CAECIF_Msk* = (0x00000001 shl DMA2D_IFCR_CAECIF_Pos) ## !< 0x00000008
  DMA2D_IFCR_CAECIF* = DMA2D_IFCR_CAECIF_Msk
  DMA2D_IFCR_CCTCIF_Pos* = (4)
  DMA2D_IFCR_CCTCIF_Msk* = (0x00000001 shl DMA2D_IFCR_CCTCIF_Pos) ## !< 0x00000010
  DMA2D_IFCR_CCTCIF* = DMA2D_IFCR_CCTCIF_Msk
  DMA2D_IFCR_CCEIF_Pos* = (5)
  DMA2D_IFCR_CCEIF_Msk* = (0x00000001 shl DMA2D_IFCR_CCEIF_Pos) ## !< 0x00000020
  DMA2D_IFCR_CCEIF* = DMA2D_IFCR_CCEIF_Msk

##  Legacy defines

const
  DMA2D_IFSR_CTEIF* = DMA2D_IFCR_CTEIF
  DMA2D_IFSR_CTCIF* = DMA2D_IFCR_CTCIF
  DMA2D_IFSR_CTWIF* = DMA2D_IFCR_CTWIF
  DMA2D_IFSR_CCAEIF* = DMA2D_IFCR_CAECIF
  DMA2D_IFSR_CCTCIF* = DMA2D_IFCR_CCTCIF
  DMA2D_IFSR_CCEIF* = DMA2D_IFCR_CCEIF

## *******************  Bit definition for DMA2D_FGMAR register  **************

const
  DMA2D_FGMAR_MA_Pos* = (0)
  DMA2D_FGMAR_MA_Msk* = (0xFFFFFFFF shl DMA2D_FGMAR_MA_Pos) ## !< 0xFFFFFFFF
  DMA2D_FGMAR_MA* = DMA2D_FGMAR_MA_Msk

## *******************  Bit definition for DMA2D_FGOR register  ***************

const
  DMA2D_FGOR_LO_Pos* = (0)
  DMA2D_FGOR_LO_Msk* = (0x00003FFF shl DMA2D_FGOR_LO_Pos) ## !< 0x00003FFF
  DMA2D_FGOR_LO* = DMA2D_FGOR_LO_Msk

## *******************  Bit definition for DMA2D_BGMAR register  **************

const
  DMA2D_BGMAR_MA_Pos* = (0)
  DMA2D_BGMAR_MA_Msk* = (0xFFFFFFFF shl DMA2D_BGMAR_MA_Pos) ## !< 0xFFFFFFFF
  DMA2D_BGMAR_MA* = DMA2D_BGMAR_MA_Msk

## *******************  Bit definition for DMA2D_BGOR register  ***************

const
  DMA2D_BGOR_LO_Pos* = (0)
  DMA2D_BGOR_LO_Msk* = (0x00003FFF shl DMA2D_BGOR_LO_Pos) ## !< 0x00003FFF
  DMA2D_BGOR_LO* = DMA2D_BGOR_LO_Msk

## *******************  Bit definition for DMA2D_FGPFCCR register  ************

const
  DMA2D_FGPFCCR_CM_Pos* = (0)
  DMA2D_FGPFCCR_CM_Msk* = (0x0000000F shl DMA2D_FGPFCCR_CM_Pos) ## !< 0x0000000F
  DMA2D_FGPFCCR_CM* = DMA2D_FGPFCCR_CM_Msk
  DMA2D_FGPFCCR_CM_Bit0* = (0x00000001 shl DMA2D_FGPFCCR_CM_Pos) ## !< 0x00000001
  DMA2D_FGPFCCR_CM_Bit1* = (0x00000002 shl DMA2D_FGPFCCR_CM_Pos) ## !< 0x00000002
  DMA2D_FGPFCCR_CM_Bit2* = (0x00000004 shl DMA2D_FGPFCCR_CM_Pos) ## !< 0x00000004
  DMA2D_FGPFCCR_CM_Bit3* = (0x00000008 shl DMA2D_FGPFCCR_CM_Pos) ## !< 0x00000008
  DMA2D_FGPFCCR_CCM_Pos* = (4)
  DMA2D_FGPFCCR_CCM_Msk* = (0x00000001 shl DMA2D_FGPFCCR_CCM_Pos) ## !< 0x00000010
  DMA2D_FGPFCCR_CCM* = DMA2D_FGPFCCR_CCM_Msk
  DMA2D_FGPFCCR_START_Pos* = (5)
  DMA2D_FGPFCCR_START_Msk* = (0x00000001 shl DMA2D_FGPFCCR_START_Pos) ## !< 0x00000020
  DMA2D_FGPFCCR_START* = DMA2D_FGPFCCR_START_Msk
  DMA2D_FGPFCCR_CS_Pos* = (8)
  DMA2D_FGPFCCR_CS_Msk* = (0x000000FF shl DMA2D_FGPFCCR_CS_Pos) ## !< 0x0000FF00
  DMA2D_FGPFCCR_CS* = DMA2D_FGPFCCR_CS_Msk
  DMA2D_FGPFCCR_AM_Pos* = (16)
  DMA2D_FGPFCCR_AM_Msk* = (0x00000003 shl DMA2D_FGPFCCR_AM_Pos) ## !< 0x00030000
  DMA2D_FGPFCCR_AM* = DMA2D_FGPFCCR_AM_Msk
  DMA2D_FGPFCCR_AM_Bit0* = (0x00000001 shl DMA2D_FGPFCCR_AM_Pos) ## !< 0x00010000
  DMA2D_FGPFCCR_AM_Bit1* = (0x00000002 shl DMA2D_FGPFCCR_AM_Pos) ## !< 0x00020000
  DMA2D_FGPFCCR_ALPHA_Pos* = (24)
  DMA2D_FGPFCCR_ALPHA_Msk* = (0x000000FF shl DMA2D_FGPFCCR_ALPHA_Pos) ## !< 0xFF000000
  DMA2D_FGPFCCR_ALPHA* = DMA2D_FGPFCCR_ALPHA_Msk

## *******************  Bit definition for DMA2D_FGCOLR register  *************

const
  DMA2D_FGCOLR_BLUE_Pos* = (0)
  DMA2D_FGCOLR_BLUE_Msk* = (0x000000FF shl DMA2D_FGCOLR_BLUE_Pos) ## !< 0x000000FF
  DMA2D_FGCOLR_BLUE* = DMA2D_FGCOLR_BLUE_Msk
  DMA2D_FGCOLR_GREEN_Pos* = (8)
  DMA2D_FGCOLR_GREEN_Msk* = (0x000000FF shl DMA2D_FGCOLR_GREEN_Pos) ## !< 0x0000FF00
  DMA2D_FGCOLR_GREEN* = DMA2D_FGCOLR_GREEN_Msk
  DMA2D_FGCOLR_RED_Pos* = (16)
  DMA2D_FGCOLR_RED_Msk* = (0x000000FF shl DMA2D_FGCOLR_RED_Pos) ## !< 0x00FF0000
  DMA2D_FGCOLR_RED* = DMA2D_FGCOLR_RED_Msk

## *******************  Bit definition for DMA2D_BGPFCCR register  ************

const
  DMA2D_BGPFCCR_CM_Pos* = (0)
  DMA2D_BGPFCCR_CM_Msk* = (0x0000000F shl DMA2D_BGPFCCR_CM_Pos) ## !< 0x0000000F
  DMA2D_BGPFCCR_CM* = DMA2D_BGPFCCR_CM_Msk
  DMA2D_BGPFCCR_CM_Bit0* = (0x00000001 shl DMA2D_BGPFCCR_CM_Pos) ## !< 0x00000001
  DMA2D_BGPFCCR_CM_Bit1* = (0x00000002 shl DMA2D_BGPFCCR_CM_Pos) ## !< 0x00000002
  DMA2D_BGPFCCR_CM_Bit2* = (0x00000004 shl DMA2D_BGPFCCR_CM_Pos) ## !< 0x00000004
  DMA2D_BGPFCCR_CM_Bit3* = 0x00000008
  DMA2D_BGPFCCR_CCM_Pos* = (4)
  DMA2D_BGPFCCR_CCM_Msk* = (0x00000001 shl DMA2D_BGPFCCR_CCM_Pos) ## !< 0x00000010
  DMA2D_BGPFCCR_CCM* = DMA2D_BGPFCCR_CCM_Msk
  DMA2D_BGPFCCR_START_Pos* = (5)
  DMA2D_BGPFCCR_START_Msk* = (0x00000001 shl DMA2D_BGPFCCR_START_Pos) ## !< 0x00000020
  DMA2D_BGPFCCR_START* = DMA2D_BGPFCCR_START_Msk
  DMA2D_BGPFCCR_CS_Pos* = (8)
  DMA2D_BGPFCCR_CS_Msk* = (0x000000FF shl DMA2D_BGPFCCR_CS_Pos) ## !< 0x0000FF00
  DMA2D_BGPFCCR_CS* = DMA2D_BGPFCCR_CS_Msk
  DMA2D_BGPFCCR_AM_Pos* = (16)
  DMA2D_BGPFCCR_AM_Msk* = (0x00000003 shl DMA2D_BGPFCCR_AM_Pos) ## !< 0x00030000
  DMA2D_BGPFCCR_AM* = DMA2D_BGPFCCR_AM_Msk
  DMA2D_BGPFCCR_AM_Bit0* = (0x00000001 shl DMA2D_BGPFCCR_AM_Pos) ## !< 0x00010000
  DMA2D_BGPFCCR_AM_Bit1* = (0x00000002 shl DMA2D_BGPFCCR_AM_Pos) ## !< 0x00020000
  DMA2D_BGPFCCR_ALPHA_Pos* = (24)
  DMA2D_BGPFCCR_ALPHA_Msk* = (0x000000FF shl DMA2D_BGPFCCR_ALPHA_Pos) ## !< 0xFF000000
  DMA2D_BGPFCCR_ALPHA* = DMA2D_BGPFCCR_ALPHA_Msk

## *******************  Bit definition for DMA2D_BGCOLR register  *************

const
  DMA2D_BGCOLR_BLUE_Pos* = (0)
  DMA2D_BGCOLR_BLUE_Msk* = (0x000000FF shl DMA2D_BGCOLR_BLUE_Pos) ## !< 0x000000FF
  DMA2D_BGCOLR_BLUE* = DMA2D_BGCOLR_BLUE_Msk
  DMA2D_BGCOLR_GREEN_Pos* = (8)
  DMA2D_BGCOLR_GREEN_Msk* = (0x000000FF shl DMA2D_BGCOLR_GREEN_Pos) ## !< 0x0000FF00
  DMA2D_BGCOLR_GREEN* = DMA2D_BGCOLR_GREEN_Msk
  DMA2D_BGCOLR_RED_Pos* = (16)
  DMA2D_BGCOLR_RED_Msk* = (0x000000FF shl DMA2D_BGCOLR_RED_Pos) ## !< 0x00FF0000
  DMA2D_BGCOLR_RED* = DMA2D_BGCOLR_RED_Msk

## *******************  Bit definition for DMA2D_FGCMAR register  *************

const
  DMA2D_FGCMAR_MA_Pos* = (0)
  DMA2D_FGCMAR_MA_Msk* = (0xFFFFFFFF shl DMA2D_FGCMAR_MA_Pos) ## !< 0xFFFFFFFF
  DMA2D_FGCMAR_MA* = DMA2D_FGCMAR_MA_Msk

## *******************  Bit definition for DMA2D_BGCMAR register  *************

const
  DMA2D_BGCMAR_MA_Pos* = (0)
  DMA2D_BGCMAR_MA_Msk* = (0xFFFFFFFF shl DMA2D_BGCMAR_MA_Pos) ## !< 0xFFFFFFFF
  DMA2D_BGCMAR_MA* = DMA2D_BGCMAR_MA_Msk

## *******************  Bit definition for DMA2D_OPFCCR register  *************

const
  DMA2D_OPFCCR_CM_Pos* = (0)
  DMA2D_OPFCCR_CM_Msk* = (0x00000007 shl DMA2D_OPFCCR_CM_Pos) ## !< 0x00000007
  DMA2D_OPFCCR_CM* = DMA2D_OPFCCR_CM_Msk
  DMA2D_OPFCCR_CM_Bit0* = (0x00000001 shl DMA2D_OPFCCR_CM_Pos) ## !< 0x00000001
  DMA2D_OPFCCR_CM_Bit1* = (0x00000002 shl DMA2D_OPFCCR_CM_Pos) ## !< 0x00000002
  DMA2D_OPFCCR_CM_Bit2* = (0x00000004 shl DMA2D_OPFCCR_CM_Pos) ## !< 0x00000004

## *******************  Bit definition for DMA2D_OCOLR register  **************
## !<Mode_ARGB8888/RGB888

const
  DMA2D_OCOLR_BLUE_Bit1* = 0x000000FF
  DMA2D_OCOLR_GREEN_Bit1* = 0x0000FF00
  DMA2D_OCOLR_RED_Bit1* = 0x00FF0000
  DMA2D_OCOLR_ALPHA_Bit1* = 0xFF000000

## !<Mode_RGB565

const
  DMA2D_OCOLR_BLUE_Bit2* = 0x0000001F
  DMA2D_OCOLR_GREEN_Bit2* = 0x000007E0
  DMA2D_OCOLR_RED_Bit2* = 0x0000F800

## !<Mode_ARGB1555

const
  DMA2D_OCOLR_BLUE_Bit3* = 0x0000001F
  DMA2D_OCOLR_GREEN_Bit3* = 0x000003E0
  DMA2D_OCOLR_RED_Bit3* = 0x00007C00
  DMA2D_OCOLR_ALPHA_Bit3* = 0x00008000

## !<Mode_ARGB4444

const
  DMA2D_OCOLR_BLUE_Bit4* = 0x0000000F
  DMA2D_OCOLR_GREEN_Bit4* = 0x000000F0
  DMA2D_OCOLR_RED_Bit4* = 0x00000F00
  DMA2D_OCOLR_ALPHA_Bit4* = 0x0000F000

## *******************  Bit definition for DMA2D_OMAR register  ***************

const
  DMA2D_OMAR_MA_Pos* = (0)
  DMA2D_OMAR_MA_Msk* = (0xFFFFFFFF shl DMA2D_OMAR_MA_Pos) ## !< 0xFFFFFFFF
  DMA2D_OMAR_MA* = DMA2D_OMAR_MA_Msk

## *******************  Bit definition for DMA2D_OOR register  ****************

const
  DMA2D_OOR_LO_Pos* = (0)
  DMA2D_OOR_LO_Msk* = (0x00003FFF shl DMA2D_OOR_LO_Pos) ## !< 0x00003FFF
  DMA2D_OOR_LO* = DMA2D_OOR_LO_Msk

## *******************  Bit definition for DMA2D_NLR register  ****************

const
  DMA2D_NLR_NL_Pos* = (0)
  DMA2D_NLR_NL_Msk* = (0x0000FFFF shl DMA2D_NLR_NL_Pos) ## !< 0x0000FFFF
  DMA2D_NLR_NL* = DMA2D_NLR_NL_Msk
  DMA2D_NLR_PL_Pos* = (16)
  DMA2D_NLR_PL_Msk* = (0x00003FFF shl DMA2D_NLR_PL_Pos) ## !< 0x3FFF0000
  DMA2D_NLR_PL* = DMA2D_NLR_PL_Msk

## *******************  Bit definition for DMA2D_LWR register  ****************

const
  DMA2D_LWR_LW_Pos* = (0)
  DMA2D_LWR_LW_Msk* = (0x0000FFFF shl DMA2D_LWR_LW_Pos) ## !< 0x0000FFFF
  DMA2D_LWR_LW* = DMA2D_LWR_LW_Msk

## *******************  Bit definition for DMA2D_AMTCR register  **************

const
  DMA2D_AMTCR_EN_Pos* = (0)
  DMA2D_AMTCR_EN_Msk* = (0x00000001 shl DMA2D_AMTCR_EN_Pos) ## !< 0x00000001
  DMA2D_AMTCR_EN* = DMA2D_AMTCR_EN_Msk
  DMA2D_AMTCR_DT_Pos* = (8)
  DMA2D_AMTCR_DT_Msk* = (0x000000FF shl DMA2D_AMTCR_DT_Pos) ## !< 0x0000FF00
  DMA2D_AMTCR_DT* = DMA2D_AMTCR_DT_Msk

## *******************  Bit definition for DMA2D_FGCLUT register  *************
## *******************  Bit definition for DMA2D_BGCLUT register  *************
## ****************************************************************************
##
##                      Display Serial Interface (DSI)
##
## ****************************************************************************
## ******************  Bit definition for DSI_VR register  ****************

const
  DSI_VR_Pos* = (1)
  DSI_VR_Msk* = (0x18999815 shl DSI_VR_Pos) ## !< 0x3133302A
  DSI_VR* = DSI_VR_Msk

## ******************  Bit definition for DSI_CR register  ****************

const
  DSI_CR_EN_Pos* = (0)
  DSI_CR_EN_Msk* = (0x00000001 shl DSI_CR_EN_Pos) ## !< 0x00000001
  DSI_CR_EN* = DSI_CR_EN_Msk

## ******************  Bit definition for DSI_CCR register  ***************

const
  DSI_CCR_TXECKDIV_Pos* = (0)
  DSI_CCR_TXECKDIV_Msk* = (0x000000FF shl DSI_CCR_TXECKDIV_Pos) ## !< 0x000000FF
  DSI_CCR_TXECKDIV* = DSI_CCR_TXECKDIV_Msk
  DSI_CCR_TXECKDIV0_Pos* = (0)
  DSI_CCR_TXECKDIV0_Msk* = (0x00000001 shl DSI_CCR_TXECKDIV0_Pos) ## !< 0x00000001
  DSI_CCR_TXECKDIV0* = DSI_CCR_TXECKDIV0_Msk
  DSI_CCR_TXECKDIV1_Pos* = (1)
  DSI_CCR_TXECKDIV1_Msk* = (0x00000001 shl DSI_CCR_TXECKDIV1_Pos) ## !< 0x00000002
  DSI_CCR_TXECKDIV1* = DSI_CCR_TXECKDIV1_Msk
  DSI_CCR_TXECKDIV2_Pos* = (2)
  DSI_CCR_TXECKDIV2_Msk* = (0x00000001 shl DSI_CCR_TXECKDIV2_Pos) ## !< 0x00000004
  DSI_CCR_TXECKDIV2* = DSI_CCR_TXECKDIV2_Msk
  DSI_CCR_TXECKDIV3_Pos* = (3)
  DSI_CCR_TXECKDIV3_Msk* = (0x00000001 shl DSI_CCR_TXECKDIV3_Pos) ## !< 0x00000008
  DSI_CCR_TXECKDIV3* = DSI_CCR_TXECKDIV3_Msk
  DSI_CCR_TXECKDIV4_Pos* = (4)
  DSI_CCR_TXECKDIV4_Msk* = (0x00000001 shl DSI_CCR_TXECKDIV4_Pos) ## !< 0x00000010
  DSI_CCR_TXECKDIV4* = DSI_CCR_TXECKDIV4_Msk
  DSI_CCR_TXECKDIV5_Pos* = (5)
  DSI_CCR_TXECKDIV5_Msk* = (0x00000001 shl DSI_CCR_TXECKDIV5_Pos) ## !< 0x00000020
  DSI_CCR_TXECKDIV5* = DSI_CCR_TXECKDIV5_Msk
  DSI_CCR_TXECKDIV6_Pos* = (6)
  DSI_CCR_TXECKDIV6_Msk* = (0x00000001 shl DSI_CCR_TXECKDIV6_Pos) ## !< 0x00000040
  DSI_CCR_TXECKDIV6* = DSI_CCR_TXECKDIV6_Msk
  DSI_CCR_TXECKDIV7_Pos* = (7)
  DSI_CCR_TXECKDIV7_Msk* = (0x00000001 shl DSI_CCR_TXECKDIV7_Pos) ## !< 0x00000080
  DSI_CCR_TXECKDIV7* = DSI_CCR_TXECKDIV7_Msk
  DSI_CCR_TOCKDIV_Pos* = (8)
  DSI_CCR_TOCKDIV_Msk* = (0x000000FF shl DSI_CCR_TOCKDIV_Pos) ## !< 0x0000FF00
  DSI_CCR_TOCKDIV* = DSI_CCR_TOCKDIV_Msk
  DSI_CCR_TOCKDIV0_Pos* = (8)
  DSI_CCR_TOCKDIV0_Msk* = (0x00000001 shl DSI_CCR_TOCKDIV0_Pos) ## !< 0x00000100
  DSI_CCR_TOCKDIV0* = DSI_CCR_TOCKDIV0_Msk
  DSI_CCR_TOCKDIV1_Pos* = (9)
  DSI_CCR_TOCKDIV1_Msk* = (0x00000001 shl DSI_CCR_TOCKDIV1_Pos) ## !< 0x00000200
  DSI_CCR_TOCKDIV1* = DSI_CCR_TOCKDIV1_Msk
  DSI_CCR_TOCKDIV2_Pos* = (10)
  DSI_CCR_TOCKDIV2_Msk* = (0x00000001 shl DSI_CCR_TOCKDIV2_Pos) ## !< 0x00000400
  DSI_CCR_TOCKDIV2* = DSI_CCR_TOCKDIV2_Msk
  DSI_CCR_TOCKDIV3_Pos* = (11)
  DSI_CCR_TOCKDIV3_Msk* = (0x00000001 shl DSI_CCR_TOCKDIV3_Pos) ## !< 0x00000800
  DSI_CCR_TOCKDIV3* = DSI_CCR_TOCKDIV3_Msk
  DSI_CCR_TOCKDIV4_Pos* = (12)
  DSI_CCR_TOCKDIV4_Msk* = (0x00000001 shl DSI_CCR_TOCKDIV4_Pos) ## !< 0x00001000
  DSI_CCR_TOCKDIV4* = DSI_CCR_TOCKDIV4_Msk
  DSI_CCR_TOCKDIV5_Pos* = (13)
  DSI_CCR_TOCKDIV5_Msk* = (0x00000001 shl DSI_CCR_TOCKDIV5_Pos) ## !< 0x00002000
  DSI_CCR_TOCKDIV5* = DSI_CCR_TOCKDIV5_Msk
  DSI_CCR_TOCKDIV6_Pos* = (14)
  DSI_CCR_TOCKDIV6_Msk* = (0x00000001 shl DSI_CCR_TOCKDIV6_Pos) ## !< 0x00004000
  DSI_CCR_TOCKDIV6* = DSI_CCR_TOCKDIV6_Msk
  DSI_CCR_TOCKDIV7_Pos* = (15)
  DSI_CCR_TOCKDIV7_Msk* = (0x00000001 shl DSI_CCR_TOCKDIV7_Pos) ## !< 0x00008000
  DSI_CCR_TOCKDIV7* = DSI_CCR_TOCKDIV7_Msk

## ******************  Bit definition for DSI_LVCIDR register  ************

const
  DSI_LVCIDR_VCID_Pos* = (0)
  DSI_LVCIDR_VCID_Msk* = (0x00000003 shl DSI_LVCIDR_VCID_Pos) ## !< 0x00000003
  DSI_LVCIDR_VCID* = DSI_LVCIDR_VCID_Msk
  DSI_LVCIDR_VCID0_Pos* = (0)
  DSI_LVCIDR_VCID0_Msk* = (0x00000001 shl DSI_LVCIDR_VCID0_Pos) ## !< 0x00000001
  DSI_LVCIDR_VCID0* = DSI_LVCIDR_VCID0_Msk
  DSI_LVCIDR_VCID1_Pos* = (1)
  DSI_LVCIDR_VCID1_Msk* = (0x00000001 shl DSI_LVCIDR_VCID1_Pos) ## !< 0x00000002
  DSI_LVCIDR_VCID1* = DSI_LVCIDR_VCID1_Msk

## ******************  Bit definition for DSI_LCOLCR register  ************

const
  DSI_LCOLCR_COLC_Pos* = (0)
  DSI_LCOLCR_COLC_Msk* = (0x0000000F shl DSI_LCOLCR_COLC_Pos) ## !< 0x0000000F
  DSI_LCOLCR_COLC* = DSI_LCOLCR_COLC_Msk
  DSI_LCOLCR_COLC0_Pos* = (0)
  DSI_LCOLCR_COLC0_Msk* = (0x00000001 shl DSI_LCOLCR_COLC0_Pos) ## !< 0x00000001
  DSI_LCOLCR_COLC0* = DSI_LCOLCR_COLC0_Msk
  DSI_LCOLCR_COLC1_Pos* = (5)
  DSI_LCOLCR_COLC1_Msk* = (0x00000001 shl DSI_LCOLCR_COLC1_Pos) ## !< 0x00000020
  DSI_LCOLCR_COLC1* = DSI_LCOLCR_COLC1_Msk
  DSI_LCOLCR_COLC2_Pos* = (6)
  DSI_LCOLCR_COLC2_Msk* = (0x00000001 shl DSI_LCOLCR_COLC2_Pos) ## !< 0x00000040
  DSI_LCOLCR_COLC2* = DSI_LCOLCR_COLC2_Msk
  DSI_LCOLCR_COLC3_Pos* = (7)
  DSI_LCOLCR_COLC3_Msk* = (0x00000001 shl DSI_LCOLCR_COLC3_Pos) ## !< 0x00000080
  DSI_LCOLCR_COLC3* = DSI_LCOLCR_COLC3_Msk
  DSI_LCOLCR_LPE_Pos* = (8)
  DSI_LCOLCR_LPE_Msk* = (0x00000001 shl DSI_LCOLCR_LPE_Pos) ## !< 0x00000100
  DSI_LCOLCR_LPE* = DSI_LCOLCR_LPE_Msk

## ******************  Bit definition for DSI_LPCR register  **************

const
  DSI_LPCR_DEP_Pos* = (0)
  DSI_LPCR_DEP_Msk* = (0x00000001 shl DSI_LPCR_DEP_Pos) ## !< 0x00000001
  DSI_LPCR_DEP* = DSI_LPCR_DEP_Msk
  DSI_LPCR_VSP_Pos* = (1)
  DSI_LPCR_VSP_Msk* = (0x00000001 shl DSI_LPCR_VSP_Pos) ## !< 0x00000002
  DSI_LPCR_VSP* = DSI_LPCR_VSP_Msk
  DSI_LPCR_HSP_Pos* = (2)
  DSI_LPCR_HSP_Msk* = (0x00000001 shl DSI_LPCR_HSP_Pos) ## !< 0x00000004
  DSI_LPCR_HSP* = DSI_LPCR_HSP_Msk

## ******************  Bit definition for DSI_LPMCR register  *************

const
  DSI_LPMCR_VLPSIZE_Pos* = (0)
  DSI_LPMCR_VLPSIZE_Msk* = (0x000000FF shl DSI_LPMCR_VLPSIZE_Pos) ## !< 0x000000FF
  DSI_LPMCR_VLPSIZE* = DSI_LPMCR_VLPSIZE_Msk
  DSI_LPMCR_VLPSIZE0_Pos* = (0)
  DSI_LPMCR_VLPSIZE0_Msk* = (0x00000001 shl DSI_LPMCR_VLPSIZE0_Pos) ## !< 0x00000001
  DSI_LPMCR_VLPSIZE0* = DSI_LPMCR_VLPSIZE0_Msk
  DSI_LPMCR_VLPSIZE1_Pos* = (1)
  DSI_LPMCR_VLPSIZE1_Msk* = (0x00000001 shl DSI_LPMCR_VLPSIZE1_Pos) ## !< 0x00000002
  DSI_LPMCR_VLPSIZE1* = DSI_LPMCR_VLPSIZE1_Msk
  DSI_LPMCR_VLPSIZE2_Pos* = (2)
  DSI_LPMCR_VLPSIZE2_Msk* = (0x00000001 shl DSI_LPMCR_VLPSIZE2_Pos) ## !< 0x00000004
  DSI_LPMCR_VLPSIZE2* = DSI_LPMCR_VLPSIZE2_Msk
  DSI_LPMCR_VLPSIZE3_Pos* = (3)
  DSI_LPMCR_VLPSIZE3_Msk* = (0x00000001 shl DSI_LPMCR_VLPSIZE3_Pos) ## !< 0x00000008
  DSI_LPMCR_VLPSIZE3* = DSI_LPMCR_VLPSIZE3_Msk
  DSI_LPMCR_VLPSIZE4_Pos* = (4)
  DSI_LPMCR_VLPSIZE4_Msk* = (0x00000001 shl DSI_LPMCR_VLPSIZE4_Pos) ## !< 0x00000010
  DSI_LPMCR_VLPSIZE4* = DSI_LPMCR_VLPSIZE4_Msk
  DSI_LPMCR_VLPSIZE5_Pos* = (5)
  DSI_LPMCR_VLPSIZE5_Msk* = (0x00000001 shl DSI_LPMCR_VLPSIZE5_Pos) ## !< 0x00000020
  DSI_LPMCR_VLPSIZE5* = DSI_LPMCR_VLPSIZE5_Msk
  DSI_LPMCR_VLPSIZE6_Pos* = (6)
  DSI_LPMCR_VLPSIZE6_Msk* = (0x00000001 shl DSI_LPMCR_VLPSIZE6_Pos) ## !< 0x00000040
  DSI_LPMCR_VLPSIZE6* = DSI_LPMCR_VLPSIZE6_Msk
  DSI_LPMCR_VLPSIZE7_Pos* = (7)
  DSI_LPMCR_VLPSIZE7_Msk* = (0x00000001 shl DSI_LPMCR_VLPSIZE7_Pos) ## !< 0x00000080
  DSI_LPMCR_VLPSIZE7* = DSI_LPMCR_VLPSIZE7_Msk
  DSI_LPMCR_LPSIZE_Pos* = (16)
  DSI_LPMCR_LPSIZE_Msk* = (0x000000FF shl DSI_LPMCR_LPSIZE_Pos) ## !< 0x00FF0000
  DSI_LPMCR_LPSIZE* = DSI_LPMCR_LPSIZE_Msk
  DSI_LPMCR_LPSIZE0_Pos* = (16)
  DSI_LPMCR_LPSIZE0_Msk* = (0x00000001 shl DSI_LPMCR_LPSIZE0_Pos) ## !< 0x00010000
  DSI_LPMCR_LPSIZE0* = DSI_LPMCR_LPSIZE0_Msk
  DSI_LPMCR_LPSIZE1_Pos* = (17)
  DSI_LPMCR_LPSIZE1_Msk* = (0x00000001 shl DSI_LPMCR_LPSIZE1_Pos) ## !< 0x00020000
  DSI_LPMCR_LPSIZE1* = DSI_LPMCR_LPSIZE1_Msk
  DSI_LPMCR_LPSIZE2_Pos* = (18)
  DSI_LPMCR_LPSIZE2_Msk* = (0x00000001 shl DSI_LPMCR_LPSIZE2_Pos) ## !< 0x00040000
  DSI_LPMCR_LPSIZE2* = DSI_LPMCR_LPSIZE2_Msk
  DSI_LPMCR_LPSIZE3_Pos* = (19)
  DSI_LPMCR_LPSIZE3_Msk* = (0x00000001 shl DSI_LPMCR_LPSIZE3_Pos) ## !< 0x00080000
  DSI_LPMCR_LPSIZE3* = DSI_LPMCR_LPSIZE3_Msk
  DSI_LPMCR_LPSIZE4_Pos* = (20)
  DSI_LPMCR_LPSIZE4_Msk* = (0x00000001 shl DSI_LPMCR_LPSIZE4_Pos) ## !< 0x00100000
  DSI_LPMCR_LPSIZE4* = DSI_LPMCR_LPSIZE4_Msk
  DSI_LPMCR_LPSIZE5_Pos* = (21)
  DSI_LPMCR_LPSIZE5_Msk* = (0x00000001 shl DSI_LPMCR_LPSIZE5_Pos) ## !< 0x00200000
  DSI_LPMCR_LPSIZE5* = DSI_LPMCR_LPSIZE5_Msk
  DSI_LPMCR_LPSIZE6_Pos* = (22)
  DSI_LPMCR_LPSIZE6_Msk* = (0x00000001 shl DSI_LPMCR_LPSIZE6_Pos) ## !< 0x00400000
  DSI_LPMCR_LPSIZE6* = DSI_LPMCR_LPSIZE6_Msk
  DSI_LPMCR_LPSIZE7_Pos* = (23)
  DSI_LPMCR_LPSIZE7_Msk* = (0x00000001 shl DSI_LPMCR_LPSIZE7_Pos) ## !< 0x00800000
  DSI_LPMCR_LPSIZE7* = DSI_LPMCR_LPSIZE7_Msk

## ******************  Bit definition for DSI_PCR register  ***************

const
  DSI_PCR_ETTXE_Pos* = (0)
  DSI_PCR_ETTXE_Msk* = (0x00000001 shl DSI_PCR_ETTXE_Pos) ## !< 0x00000001
  DSI_PCR_ETTXE* = DSI_PCR_ETTXE_Msk
  DSI_PCR_ETRXE_Pos* = (1)
  DSI_PCR_ETRXE_Msk* = (0x00000001 shl DSI_PCR_ETRXE_Pos) ## !< 0x00000002
  DSI_PCR_ETRXE* = DSI_PCR_ETRXE_Msk
  DSI_PCR_BTAE_Pos* = (2)
  DSI_PCR_BTAE_Msk* = (0x00000001 shl DSI_PCR_BTAE_Pos) ## !< 0x00000004
  DSI_PCR_BTAE* = DSI_PCR_BTAE_Msk
  DSI_PCR_ECCRXE_Pos* = (3)
  DSI_PCR_ECCRXE_Msk* = (0x00000001 shl DSI_PCR_ECCRXE_Pos) ## !< 0x00000008
  DSI_PCR_ECCRXE* = DSI_PCR_ECCRXE_Msk
  DSI_PCR_CRCRXE_Pos* = (4)
  DSI_PCR_CRCRXE_Msk* = (0x00000001 shl DSI_PCR_CRCRXE_Pos) ## !< 0x00000010
  DSI_PCR_CRCRXE* = DSI_PCR_CRCRXE_Msk

## ******************  Bit definition for DSI_GVCIDR register  ************

const
  DSI_GVCIDR_VCID_Pos* = (0)
  DSI_GVCIDR_VCID_Msk* = (0x00000003 shl DSI_GVCIDR_VCID_Pos) ## !< 0x00000003
  DSI_GVCIDR_VCID* = DSI_GVCIDR_VCID_Msk
  DSI_GVCIDR_VCID0_Pos* = (0)
  DSI_GVCIDR_VCID0_Msk* = (0x00000001 shl DSI_GVCIDR_VCID0_Pos) ## !< 0x00000001
  DSI_GVCIDR_VCID0* = DSI_GVCIDR_VCID0_Msk
  DSI_GVCIDR_VCID1_Pos* = (1)
  DSI_GVCIDR_VCID1_Msk* = (0x00000001 shl DSI_GVCIDR_VCID1_Pos) ## !< 0x00000002
  DSI_GVCIDR_VCID1* = DSI_GVCIDR_VCID1_Msk

## ******************  Bit definition for DSI_MCR register  ***************

const
  DSI_MCR_CMDM_Pos* = (0)
  DSI_MCR_CMDM_Msk* = (0x00000001 shl DSI_MCR_CMDM_Pos) ## !< 0x00000001
  DSI_MCR_CMDM* = DSI_MCR_CMDM_Msk

## ******************  Bit definition for DSI_VMCR register  **************

const
  DSI_VMCR_VMT_Pos* = (0)
  DSI_VMCR_VMT_Msk* = (0x00000003 shl DSI_VMCR_VMT_Pos) ## !< 0x00000003
  DSI_VMCR_VMT* = DSI_VMCR_VMT_Msk
  DSI_VMCR_VMT0_Pos* = (0)
  DSI_VMCR_VMT0_Msk* = (0x00000001 shl DSI_VMCR_VMT0_Pos) ## !< 0x00000001
  DSI_VMCR_VMT0* = DSI_VMCR_VMT0_Msk
  DSI_VMCR_VMT1_Pos* = (1)
  DSI_VMCR_VMT1_Msk* = (0x00000001 shl DSI_VMCR_VMT1_Pos) ## !< 0x00000002
  DSI_VMCR_VMT1* = DSI_VMCR_VMT1_Msk
  DSI_VMCR_LPVSAE_Pos* = (8)
  DSI_VMCR_LPVSAE_Msk* = (0x00000001 shl DSI_VMCR_LPVSAE_Pos) ## !< 0x00000100
  DSI_VMCR_LPVSAE* = DSI_VMCR_LPVSAE_Msk
  DSI_VMCR_LPVBPE_Pos* = (9)
  DSI_VMCR_LPVBPE_Msk* = (0x00000001 shl DSI_VMCR_LPVBPE_Pos) ## !< 0x00000200
  DSI_VMCR_LPVBPE* = DSI_VMCR_LPVBPE_Msk
  DSI_VMCR_LPVFPE_Pos* = (10)
  DSI_VMCR_LPVFPE_Msk* = (0x00000001 shl DSI_VMCR_LPVFPE_Pos) ## !< 0x00000400
  DSI_VMCR_LPVFPE* = DSI_VMCR_LPVFPE_Msk
  DSI_VMCR_LPVAE_Pos* = (11)
  DSI_VMCR_LPVAE_Msk* = (0x00000001 shl DSI_VMCR_LPVAE_Pos) ## !< 0x00000800
  DSI_VMCR_LPVAE* = DSI_VMCR_LPVAE_Msk
  DSI_VMCR_LPHBPE_Pos* = (12)
  DSI_VMCR_LPHBPE_Msk* = (0x00000001 shl DSI_VMCR_LPHBPE_Pos) ## !< 0x00001000
  DSI_VMCR_LPHBPE* = DSI_VMCR_LPHBPE_Msk
  DSI_VMCR_LPHFPE_Pos* = (13)
  DSI_VMCR_LPHFPE_Msk* = (0x00000001 shl DSI_VMCR_LPHFPE_Pos) ## !< 0x00002000
  DSI_VMCR_LPHFPE* = DSI_VMCR_LPHFPE_Msk
  DSI_VMCR_FBTAAE_Pos* = (14)
  DSI_VMCR_FBTAAE_Msk* = (0x00000001 shl DSI_VMCR_FBTAAE_Pos) ## !< 0x00004000
  DSI_VMCR_FBTAAE* = DSI_VMCR_FBTAAE_Msk
  DSI_VMCR_LPCE_Pos* = (15)
  DSI_VMCR_LPCE_Msk* = (0x00000001 shl DSI_VMCR_LPCE_Pos) ## !< 0x00008000
  DSI_VMCR_LPCE* = DSI_VMCR_LPCE_Msk
  DSI_VMCR_PGE_Pos* = (16)
  DSI_VMCR_PGE_Msk* = (0x00000001 shl DSI_VMCR_PGE_Pos) ## !< 0x00010000
  DSI_VMCR_PGE* = DSI_VMCR_PGE_Msk
  DSI_VMCR_PGM_Pos* = (20)
  DSI_VMCR_PGM_Msk* = (0x00000001 shl DSI_VMCR_PGM_Pos) ## !< 0x00100000
  DSI_VMCR_PGM* = DSI_VMCR_PGM_Msk
  DSI_VMCR_PGO_Pos* = (24)
  DSI_VMCR_PGO_Msk* = (0x00000001 shl DSI_VMCR_PGO_Pos) ## !< 0x01000000
  DSI_VMCR_PGO* = DSI_VMCR_PGO_Msk

## ******************  Bit definition for DSI_VPCR register  **************

const
  DSI_VPCR_VPSIZE_Pos* = (0)
  DSI_VPCR_VPSIZE_Msk* = (0x00003FFF shl DSI_VPCR_VPSIZE_Pos) ## !< 0x00003FFF
  DSI_VPCR_VPSIZE* = DSI_VPCR_VPSIZE_Msk
  DSI_VPCR_VPSIZE0_Pos* = (0)
  DSI_VPCR_VPSIZE0_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE0_Pos) ## !< 0x00000001
  DSI_VPCR_VPSIZE0* = DSI_VPCR_VPSIZE0_Msk
  DSI_VPCR_VPSIZE1_Pos* = (1)
  DSI_VPCR_VPSIZE1_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE1_Pos) ## !< 0x00000002
  DSI_VPCR_VPSIZE1* = DSI_VPCR_VPSIZE1_Msk
  DSI_VPCR_VPSIZE2_Pos* = (2)
  DSI_VPCR_VPSIZE2_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE2_Pos) ## !< 0x00000004
  DSI_VPCR_VPSIZE2* = DSI_VPCR_VPSIZE2_Msk
  DSI_VPCR_VPSIZE3_Pos* = (3)
  DSI_VPCR_VPSIZE3_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE3_Pos) ## !< 0x00000008
  DSI_VPCR_VPSIZE3* = DSI_VPCR_VPSIZE3_Msk
  DSI_VPCR_VPSIZE4_Pos* = (4)
  DSI_VPCR_VPSIZE4_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE4_Pos) ## !< 0x00000010
  DSI_VPCR_VPSIZE4* = DSI_VPCR_VPSIZE4_Msk
  DSI_VPCR_VPSIZE5_Pos* = (5)
  DSI_VPCR_VPSIZE5_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE5_Pos) ## !< 0x00000020
  DSI_VPCR_VPSIZE5* = DSI_VPCR_VPSIZE5_Msk
  DSI_VPCR_VPSIZE6_Pos* = (6)
  DSI_VPCR_VPSIZE6_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE6_Pos) ## !< 0x00000040
  DSI_VPCR_VPSIZE6* = DSI_VPCR_VPSIZE6_Msk
  DSI_VPCR_VPSIZE7_Pos* = (7)
  DSI_VPCR_VPSIZE7_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE7_Pos) ## !< 0x00000080
  DSI_VPCR_VPSIZE7* = DSI_VPCR_VPSIZE7_Msk
  DSI_VPCR_VPSIZE8_Pos* = (8)
  DSI_VPCR_VPSIZE8_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE8_Pos) ## !< 0x00000100
  DSI_VPCR_VPSIZE8* = DSI_VPCR_VPSIZE8_Msk
  DSI_VPCR_VPSIZE9_Pos* = (9)
  DSI_VPCR_VPSIZE9_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE9_Pos) ## !< 0x00000200
  DSI_VPCR_VPSIZE9* = DSI_VPCR_VPSIZE9_Msk
  DSI_VPCR_VPSIZE10_Pos* = (10)
  DSI_VPCR_VPSIZE10_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE10_Pos) ## !< 0x00000400
  DSI_VPCR_VPSIZE10* = DSI_VPCR_VPSIZE10_Msk
  DSI_VPCR_VPSIZE11_Pos* = (11)
  DSI_VPCR_VPSIZE11_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE11_Pos) ## !< 0x00000800
  DSI_VPCR_VPSIZE11* = DSI_VPCR_VPSIZE11_Msk
  DSI_VPCR_VPSIZE12_Pos* = (12)
  DSI_VPCR_VPSIZE12_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE12_Pos) ## !< 0x00001000
  DSI_VPCR_VPSIZE12* = DSI_VPCR_VPSIZE12_Msk
  DSI_VPCR_VPSIZE13_Pos* = (13)
  DSI_VPCR_VPSIZE13_Msk* = (0x00000001 shl DSI_VPCR_VPSIZE13_Pos) ## !< 0x00002000
  DSI_VPCR_VPSIZE13* = DSI_VPCR_VPSIZE13_Msk

## ******************  Bit definition for DSI_VCCR register  **************

const
  DSI_VCCR_NUMC_Pos* = (0)
  DSI_VCCR_NUMC_Msk* = (0x00001FFF shl DSI_VCCR_NUMC_Pos) ## !< 0x00001FFF
  DSI_VCCR_NUMC* = DSI_VCCR_NUMC_Msk
  DSI_VCCR_NUMC0_Pos* = (0)
  DSI_VCCR_NUMC0_Msk* = (0x00000001 shl DSI_VCCR_NUMC0_Pos) ## !< 0x00000001
  DSI_VCCR_NUMC0* = DSI_VCCR_NUMC0_Msk
  DSI_VCCR_NUMC1_Pos* = (1)
  DSI_VCCR_NUMC1_Msk* = (0x00000001 shl DSI_VCCR_NUMC1_Pos) ## !< 0x00000002
  DSI_VCCR_NUMC1* = DSI_VCCR_NUMC1_Msk
  DSI_VCCR_NUMC2_Pos* = (2)
  DSI_VCCR_NUMC2_Msk* = (0x00000001 shl DSI_VCCR_NUMC2_Pos) ## !< 0x00000004
  DSI_VCCR_NUMC2* = DSI_VCCR_NUMC2_Msk
  DSI_VCCR_NUMC3_Pos* = (3)
  DSI_VCCR_NUMC3_Msk* = (0x00000001 shl DSI_VCCR_NUMC3_Pos) ## !< 0x00000008
  DSI_VCCR_NUMC3* = DSI_VCCR_NUMC3_Msk
  DSI_VCCR_NUMC4_Pos* = (4)
  DSI_VCCR_NUMC4_Msk* = (0x00000001 shl DSI_VCCR_NUMC4_Pos) ## !< 0x00000010
  DSI_VCCR_NUMC4* = DSI_VCCR_NUMC4_Msk
  DSI_VCCR_NUMC5_Pos* = (5)
  DSI_VCCR_NUMC5_Msk* = (0x00000001 shl DSI_VCCR_NUMC5_Pos) ## !< 0x00000020
  DSI_VCCR_NUMC5* = DSI_VCCR_NUMC5_Msk
  DSI_VCCR_NUMC6_Pos* = (6)
  DSI_VCCR_NUMC6_Msk* = (0x00000001 shl DSI_VCCR_NUMC6_Pos) ## !< 0x00000040
  DSI_VCCR_NUMC6* = DSI_VCCR_NUMC6_Msk
  DSI_VCCR_NUMC7_Pos* = (7)
  DSI_VCCR_NUMC7_Msk* = (0x00000001 shl DSI_VCCR_NUMC7_Pos) ## !< 0x00000080
  DSI_VCCR_NUMC7* = DSI_VCCR_NUMC7_Msk
  DSI_VCCR_NUMC8_Pos* = (8)
  DSI_VCCR_NUMC8_Msk* = (0x00000001 shl DSI_VCCR_NUMC8_Pos) ## !< 0x00000100
  DSI_VCCR_NUMC8* = DSI_VCCR_NUMC8_Msk
  DSI_VCCR_NUMC9_Pos* = (9)
  DSI_VCCR_NUMC9_Msk* = (0x00000001 shl DSI_VCCR_NUMC9_Pos) ## !< 0x00000200
  DSI_VCCR_NUMC9* = DSI_VCCR_NUMC9_Msk
  DSI_VCCR_NUMC10_Pos* = (10)
  DSI_VCCR_NUMC10_Msk* = (0x00000001 shl DSI_VCCR_NUMC10_Pos) ## !< 0x00000400
  DSI_VCCR_NUMC10* = DSI_VCCR_NUMC10_Msk
  DSI_VCCR_NUMC11_Pos* = (11)
  DSI_VCCR_NUMC11_Msk* = (0x00000001 shl DSI_VCCR_NUMC11_Pos) ## !< 0x00000800
  DSI_VCCR_NUMC11* = DSI_VCCR_NUMC11_Msk
  DSI_VCCR_NUMC12_Pos* = (12)
  DSI_VCCR_NUMC12_Msk* = (0x00000001 shl DSI_VCCR_NUMC12_Pos) ## !< 0x00001000
  DSI_VCCR_NUMC12* = DSI_VCCR_NUMC12_Msk

## ******************  Bit definition for DSI_VNPCR register  *************

const
  DSI_VNPCR_NPSIZE_Pos* = (0)
  DSI_VNPCR_NPSIZE_Msk* = (0x00001FFF shl DSI_VNPCR_NPSIZE_Pos) ## !< 0x00001FFF
  DSI_VNPCR_NPSIZE* = DSI_VNPCR_NPSIZE_Msk
  DSI_VNPCR_NPSIZE0_Pos* = (0)
  DSI_VNPCR_NPSIZE0_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE0_Pos) ## !< 0x00000001
  DSI_VNPCR_NPSIZE0* = DSI_VNPCR_NPSIZE0_Msk
  DSI_VNPCR_NPSIZE1_Pos* = (1)
  DSI_VNPCR_NPSIZE1_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE1_Pos) ## !< 0x00000002
  DSI_VNPCR_NPSIZE1* = DSI_VNPCR_NPSIZE1_Msk
  DSI_VNPCR_NPSIZE2_Pos* = (2)
  DSI_VNPCR_NPSIZE2_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE2_Pos) ## !< 0x00000004
  DSI_VNPCR_NPSIZE2* = DSI_VNPCR_NPSIZE2_Msk
  DSI_VNPCR_NPSIZE3_Pos* = (3)
  DSI_VNPCR_NPSIZE3_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE3_Pos) ## !< 0x00000008
  DSI_VNPCR_NPSIZE3* = DSI_VNPCR_NPSIZE3_Msk
  DSI_VNPCR_NPSIZE4_Pos* = (4)
  DSI_VNPCR_NPSIZE4_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE4_Pos) ## !< 0x00000010
  DSI_VNPCR_NPSIZE4* = DSI_VNPCR_NPSIZE4_Msk
  DSI_VNPCR_NPSIZE5_Pos* = (5)
  DSI_VNPCR_NPSIZE5_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE5_Pos) ## !< 0x00000020
  DSI_VNPCR_NPSIZE5* = DSI_VNPCR_NPSIZE5_Msk
  DSI_VNPCR_NPSIZE6_Pos* = (6)
  DSI_VNPCR_NPSIZE6_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE6_Pos) ## !< 0x00000040
  DSI_VNPCR_NPSIZE6* = DSI_VNPCR_NPSIZE6_Msk
  DSI_VNPCR_NPSIZE7_Pos* = (7)
  DSI_VNPCR_NPSIZE7_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE7_Pos) ## !< 0x00000080
  DSI_VNPCR_NPSIZE7* = DSI_VNPCR_NPSIZE7_Msk
  DSI_VNPCR_NPSIZE8_Pos* = (8)
  DSI_VNPCR_NPSIZE8_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE8_Pos) ## !< 0x00000100
  DSI_VNPCR_NPSIZE8* = DSI_VNPCR_NPSIZE8_Msk
  DSI_VNPCR_NPSIZE9_Pos* = (9)
  DSI_VNPCR_NPSIZE9_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE9_Pos) ## !< 0x00000200
  DSI_VNPCR_NPSIZE9* = DSI_VNPCR_NPSIZE9_Msk
  DSI_VNPCR_NPSIZE10_Pos* = (10)
  DSI_VNPCR_NPSIZE10_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE10_Pos) ## !< 0x00000400
  DSI_VNPCR_NPSIZE10* = DSI_VNPCR_NPSIZE10_Msk
  DSI_VNPCR_NPSIZE11_Pos* = (11)
  DSI_VNPCR_NPSIZE11_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE11_Pos) ## !< 0x00000800
  DSI_VNPCR_NPSIZE11* = DSI_VNPCR_NPSIZE11_Msk
  DSI_VNPCR_NPSIZE12_Pos* = (12)
  DSI_VNPCR_NPSIZE12_Msk* = (0x00000001 shl DSI_VNPCR_NPSIZE12_Pos) ## !< 0x00001000
  DSI_VNPCR_NPSIZE12* = DSI_VNPCR_NPSIZE12_Msk

## ******************  Bit definition for DSI_VHSACR register  ************

const
  DSI_VHSACR_HSA_Pos* = (0)
  DSI_VHSACR_HSA_Msk* = (0x00000FFF shl DSI_VHSACR_HSA_Pos) ## !< 0x00000FFF
  DSI_VHSACR_HSA* = DSI_VHSACR_HSA_Msk
  DSI_VHSACR_HSA0_Pos* = (0)
  DSI_VHSACR_HSA0_Msk* = (0x00000001 shl DSI_VHSACR_HSA0_Pos) ## !< 0x00000001
  DSI_VHSACR_HSA0* = DSI_VHSACR_HSA0_Msk
  DSI_VHSACR_HSA1_Pos* = (1)
  DSI_VHSACR_HSA1_Msk* = (0x00000001 shl DSI_VHSACR_HSA1_Pos) ## !< 0x00000002
  DSI_VHSACR_HSA1* = DSI_VHSACR_HSA1_Msk
  DSI_VHSACR_HSA2_Pos* = (2)
  DSI_VHSACR_HSA2_Msk* = (0x00000001 shl DSI_VHSACR_HSA2_Pos) ## !< 0x00000004
  DSI_VHSACR_HSA2* = DSI_VHSACR_HSA2_Msk
  DSI_VHSACR_HSA3_Pos* = (3)
  DSI_VHSACR_HSA3_Msk* = (0x00000001 shl DSI_VHSACR_HSA3_Pos) ## !< 0x00000008
  DSI_VHSACR_HSA3* = DSI_VHSACR_HSA3_Msk
  DSI_VHSACR_HSA4_Pos* = (4)
  DSI_VHSACR_HSA4_Msk* = (0x00000001 shl DSI_VHSACR_HSA4_Pos) ## !< 0x00000010
  DSI_VHSACR_HSA4* = DSI_VHSACR_HSA4_Msk
  DSI_VHSACR_HSA5_Pos* = (5)
  DSI_VHSACR_HSA5_Msk* = (0x00000001 shl DSI_VHSACR_HSA5_Pos) ## !< 0x00000020
  DSI_VHSACR_HSA5* = DSI_VHSACR_HSA5_Msk
  DSI_VHSACR_HSA6_Pos* = (6)
  DSI_VHSACR_HSA6_Msk* = (0x00000001 shl DSI_VHSACR_HSA6_Pos) ## !< 0x00000040
  DSI_VHSACR_HSA6* = DSI_VHSACR_HSA6_Msk
  DSI_VHSACR_HSA7_Pos* = (7)
  DSI_VHSACR_HSA7_Msk* = (0x00000001 shl DSI_VHSACR_HSA7_Pos) ## !< 0x00000080
  DSI_VHSACR_HSA7* = DSI_VHSACR_HSA7_Msk
  DSI_VHSACR_HSA8_Pos* = (8)
  DSI_VHSACR_HSA8_Msk* = (0x00000001 shl DSI_VHSACR_HSA8_Pos) ## !< 0x00000100
  DSI_VHSACR_HSA8* = DSI_VHSACR_HSA8_Msk
  DSI_VHSACR_HSA9_Pos* = (9)
  DSI_VHSACR_HSA9_Msk* = (0x00000001 shl DSI_VHSACR_HSA9_Pos) ## !< 0x00000200
  DSI_VHSACR_HSA9* = DSI_VHSACR_HSA9_Msk
  DSI_VHSACR_HSA10_Pos* = (10)
  DSI_VHSACR_HSA10_Msk* = (0x00000001 shl DSI_VHSACR_HSA10_Pos) ## !< 0x00000400
  DSI_VHSACR_HSA10* = DSI_VHSACR_HSA10_Msk
  DSI_VHSACR_HSA11_Pos* = (11)
  DSI_VHSACR_HSA11_Msk* = (0x00000001 shl DSI_VHSACR_HSA11_Pos) ## !< 0x00000800
  DSI_VHSACR_HSA11* = DSI_VHSACR_HSA11_Msk

## ******************  Bit definition for DSI_VHBPCR register  ************

const
  DSI_VHBPCR_HBP_Pos* = (0)
  DSI_VHBPCR_HBP_Msk* = (0x00000FFF shl DSI_VHBPCR_HBP_Pos) ## !< 0x00000FFF
  DSI_VHBPCR_HBP* = DSI_VHBPCR_HBP_Msk
  DSI_VHBPCR_HBP0_Pos* = (0)
  DSI_VHBPCR_HBP0_Msk* = (0x00000001 shl DSI_VHBPCR_HBP0_Pos) ## !< 0x00000001
  DSI_VHBPCR_HBP0* = DSI_VHBPCR_HBP0_Msk
  DSI_VHBPCR_HBP1_Pos* = (1)
  DSI_VHBPCR_HBP1_Msk* = (0x00000001 shl DSI_VHBPCR_HBP1_Pos) ## !< 0x00000002
  DSI_VHBPCR_HBP1* = DSI_VHBPCR_HBP1_Msk
  DSI_VHBPCR_HBP2_Pos* = (2)
  DSI_VHBPCR_HBP2_Msk* = (0x00000001 shl DSI_VHBPCR_HBP2_Pos) ## !< 0x00000004
  DSI_VHBPCR_HBP2* = DSI_VHBPCR_HBP2_Msk
  DSI_VHBPCR_HBP3_Pos* = (3)
  DSI_VHBPCR_HBP3_Msk* = (0x00000001 shl DSI_VHBPCR_HBP3_Pos) ## !< 0x00000008
  DSI_VHBPCR_HBP3* = DSI_VHBPCR_HBP3_Msk
  DSI_VHBPCR_HBP4_Pos* = (4)
  DSI_VHBPCR_HBP4_Msk* = (0x00000001 shl DSI_VHBPCR_HBP4_Pos) ## !< 0x00000010
  DSI_VHBPCR_HBP4* = DSI_VHBPCR_HBP4_Msk
  DSI_VHBPCR_HBP5_Pos* = (5)
  DSI_VHBPCR_HBP5_Msk* = (0x00000001 shl DSI_VHBPCR_HBP5_Pos) ## !< 0x00000020
  DSI_VHBPCR_HBP5* = DSI_VHBPCR_HBP5_Msk
  DSI_VHBPCR_HBP6_Pos* = (6)
  DSI_VHBPCR_HBP6_Msk* = (0x00000001 shl DSI_VHBPCR_HBP6_Pos) ## !< 0x00000040
  DSI_VHBPCR_HBP6* = DSI_VHBPCR_HBP6_Msk
  DSI_VHBPCR_HBP7_Pos* = (7)
  DSI_VHBPCR_HBP7_Msk* = (0x00000001 shl DSI_VHBPCR_HBP7_Pos) ## !< 0x00000080
  DSI_VHBPCR_HBP7* = DSI_VHBPCR_HBP7_Msk
  DSI_VHBPCR_HBP8_Pos* = (8)
  DSI_VHBPCR_HBP8_Msk* = (0x00000001 shl DSI_VHBPCR_HBP8_Pos) ## !< 0x00000100
  DSI_VHBPCR_HBP8* = DSI_VHBPCR_HBP8_Msk
  DSI_VHBPCR_HBP9_Pos* = (9)
  DSI_VHBPCR_HBP9_Msk* = (0x00000001 shl DSI_VHBPCR_HBP9_Pos) ## !< 0x00000200
  DSI_VHBPCR_HBP9* = DSI_VHBPCR_HBP9_Msk
  DSI_VHBPCR_HBP10_Pos* = (10)
  DSI_VHBPCR_HBP10_Msk* = (0x00000001 shl DSI_VHBPCR_HBP10_Pos) ## !< 0x00000400
  DSI_VHBPCR_HBP10* = DSI_VHBPCR_HBP10_Msk
  DSI_VHBPCR_HBP11_Pos* = (11)
  DSI_VHBPCR_HBP11_Msk* = (0x00000001 shl DSI_VHBPCR_HBP11_Pos) ## !< 0x00000800
  DSI_VHBPCR_HBP11* = DSI_VHBPCR_HBP11_Msk

## ******************  Bit definition for DSI_VLCR register  **************

const
  DSI_VLCR_HLINE_Pos* = (0)
  DSI_VLCR_HLINE_Msk* = (0x00007FFF shl DSI_VLCR_HLINE_Pos) ## !< 0x00007FFF
  DSI_VLCR_HLINE* = DSI_VLCR_HLINE_Msk
  DSI_VLCR_HLINE0_Pos* = (0)
  DSI_VLCR_HLINE0_Msk* = (0x00000001 shl DSI_VLCR_HLINE0_Pos) ## !< 0x00000001
  DSI_VLCR_HLINE0* = DSI_VLCR_HLINE0_Msk
  DSI_VLCR_HLINE1_Pos* = (1)
  DSI_VLCR_HLINE1_Msk* = (0x00000001 shl DSI_VLCR_HLINE1_Pos) ## !< 0x00000002
  DSI_VLCR_HLINE1* = DSI_VLCR_HLINE1_Msk
  DSI_VLCR_HLINE2_Pos* = (2)
  DSI_VLCR_HLINE2_Msk* = (0x00000001 shl DSI_VLCR_HLINE2_Pos) ## !< 0x00000004
  DSI_VLCR_HLINE2* = DSI_VLCR_HLINE2_Msk
  DSI_VLCR_HLINE3_Pos* = (3)
  DSI_VLCR_HLINE3_Msk* = (0x00000001 shl DSI_VLCR_HLINE3_Pos) ## !< 0x00000008
  DSI_VLCR_HLINE3* = DSI_VLCR_HLINE3_Msk
  DSI_VLCR_HLINE4_Pos* = (4)
  DSI_VLCR_HLINE4_Msk* = (0x00000001 shl DSI_VLCR_HLINE4_Pos) ## !< 0x00000010
  DSI_VLCR_HLINE4* = DSI_VLCR_HLINE4_Msk
  DSI_VLCR_HLINE5_Pos* = (5)
  DSI_VLCR_HLINE5_Msk* = (0x00000001 shl DSI_VLCR_HLINE5_Pos) ## !< 0x00000020
  DSI_VLCR_HLINE5* = DSI_VLCR_HLINE5_Msk
  DSI_VLCR_HLINE6_Pos* = (6)
  DSI_VLCR_HLINE6_Msk* = (0x00000001 shl DSI_VLCR_HLINE6_Pos) ## !< 0x00000040
  DSI_VLCR_HLINE6* = DSI_VLCR_HLINE6_Msk
  DSI_VLCR_HLINE7_Pos* = (7)
  DSI_VLCR_HLINE7_Msk* = (0x00000001 shl DSI_VLCR_HLINE7_Pos) ## !< 0x00000080
  DSI_VLCR_HLINE7* = DSI_VLCR_HLINE7_Msk
  DSI_VLCR_HLINE8_Pos* = (8)
  DSI_VLCR_HLINE8_Msk* = (0x00000001 shl DSI_VLCR_HLINE8_Pos) ## !< 0x00000100
  DSI_VLCR_HLINE8* = DSI_VLCR_HLINE8_Msk
  DSI_VLCR_HLINE9_Pos* = (9)
  DSI_VLCR_HLINE9_Msk* = (0x00000001 shl DSI_VLCR_HLINE9_Pos) ## !< 0x00000200
  DSI_VLCR_HLINE9* = DSI_VLCR_HLINE9_Msk
  DSI_VLCR_HLINE10_Pos* = (10)
  DSI_VLCR_HLINE10_Msk* = (0x00000001 shl DSI_VLCR_HLINE10_Pos) ## !< 0x00000400
  DSI_VLCR_HLINE10* = DSI_VLCR_HLINE10_Msk
  DSI_VLCR_HLINE11_Pos* = (11)
  DSI_VLCR_HLINE11_Msk* = (0x00000001 shl DSI_VLCR_HLINE11_Pos) ## !< 0x00000800
  DSI_VLCR_HLINE11* = DSI_VLCR_HLINE11_Msk
  DSI_VLCR_HLINE12_Pos* = (12)
  DSI_VLCR_HLINE12_Msk* = (0x00000001 shl DSI_VLCR_HLINE12_Pos) ## !< 0x00001000
  DSI_VLCR_HLINE12* = DSI_VLCR_HLINE12_Msk
  DSI_VLCR_HLINE13_Pos* = (13)
  DSI_VLCR_HLINE13_Msk* = (0x00000001 shl DSI_VLCR_HLINE13_Pos) ## !< 0x00002000
  DSI_VLCR_HLINE13* = DSI_VLCR_HLINE13_Msk
  DSI_VLCR_HLINE14_Pos* = (14)
  DSI_VLCR_HLINE14_Msk* = (0x00000001 shl DSI_VLCR_HLINE14_Pos) ## !< 0x00004000
  DSI_VLCR_HLINE14* = DSI_VLCR_HLINE14_Msk

## ******************  Bit definition for DSI_VVSACR register  ************

const
  DSI_VVSACR_VSA_Pos* = (0)
  DSI_VVSACR_VSA_Msk* = (0x000003FF shl DSI_VVSACR_VSA_Pos) ## !< 0x000003FF
  DSI_VVSACR_VSA* = DSI_VVSACR_VSA_Msk
  DSI_VVSACR_VSA0_Pos* = (0)
  DSI_VVSACR_VSA0_Msk* = (0x00000001 shl DSI_VVSACR_VSA0_Pos) ## !< 0x00000001
  DSI_VVSACR_VSA0* = DSI_VVSACR_VSA0_Msk
  DSI_VVSACR_VSA1_Pos* = (1)
  DSI_VVSACR_VSA1_Msk* = (0x00000001 shl DSI_VVSACR_VSA1_Pos) ## !< 0x00000002
  DSI_VVSACR_VSA1* = DSI_VVSACR_VSA1_Msk
  DSI_VVSACR_VSA2_Pos* = (2)
  DSI_VVSACR_VSA2_Msk* = (0x00000001 shl DSI_VVSACR_VSA2_Pos) ## !< 0x00000004
  DSI_VVSACR_VSA2* = DSI_VVSACR_VSA2_Msk
  DSI_VVSACR_VSA3_Pos* = (3)
  DSI_VVSACR_VSA3_Msk* = (0x00000001 shl DSI_VVSACR_VSA3_Pos) ## !< 0x00000008
  DSI_VVSACR_VSA3* = DSI_VVSACR_VSA3_Msk
  DSI_VVSACR_VSA4_Pos* = (4)
  DSI_VVSACR_VSA4_Msk* = (0x00000001 shl DSI_VVSACR_VSA4_Pos) ## !< 0x00000010
  DSI_VVSACR_VSA4* = DSI_VVSACR_VSA4_Msk
  DSI_VVSACR_VSA5_Pos* = (5)
  DSI_VVSACR_VSA5_Msk* = (0x00000001 shl DSI_VVSACR_VSA5_Pos) ## !< 0x00000020
  DSI_VVSACR_VSA5* = DSI_VVSACR_VSA5_Msk
  DSI_VVSACR_VSA6_Pos* = (6)
  DSI_VVSACR_VSA6_Msk* = (0x00000001 shl DSI_VVSACR_VSA6_Pos) ## !< 0x00000040
  DSI_VVSACR_VSA6* = DSI_VVSACR_VSA6_Msk
  DSI_VVSACR_VSA7_Pos* = (7)
  DSI_VVSACR_VSA7_Msk* = (0x00000001 shl DSI_VVSACR_VSA7_Pos) ## !< 0x00000080
  DSI_VVSACR_VSA7* = DSI_VVSACR_VSA7_Msk
  DSI_VVSACR_VSA8_Pos* = (8)
  DSI_VVSACR_VSA8_Msk* = (0x00000001 shl DSI_VVSACR_VSA8_Pos) ## !< 0x00000100
  DSI_VVSACR_VSA8* = DSI_VVSACR_VSA8_Msk
  DSI_VVSACR_VSA9_Pos* = (9)
  DSI_VVSACR_VSA9_Msk* = (0x00000001 shl DSI_VVSACR_VSA9_Pos) ## !< 0x00000200
  DSI_VVSACR_VSA9* = DSI_VVSACR_VSA9_Msk

## ******************  Bit definition for DSI_VVBPCR register  ************

const
  DSI_VVBPCR_VBP_Pos* = (0)
  DSI_VVBPCR_VBP_Msk* = (0x000003FF shl DSI_VVBPCR_VBP_Pos) ## !< 0x000003FF
  DSI_VVBPCR_VBP* = DSI_VVBPCR_VBP_Msk
  DSI_VVBPCR_VBP0_Pos* = (0)
  DSI_VVBPCR_VBP0_Msk* = (0x00000001 shl DSI_VVBPCR_VBP0_Pos) ## !< 0x00000001
  DSI_VVBPCR_VBP0* = DSI_VVBPCR_VBP0_Msk
  DSI_VVBPCR_VBP1_Pos* = (1)
  DSI_VVBPCR_VBP1_Msk* = (0x00000001 shl DSI_VVBPCR_VBP1_Pos) ## !< 0x00000002
  DSI_VVBPCR_VBP1* = DSI_VVBPCR_VBP1_Msk
  DSI_VVBPCR_VBP2_Pos* = (2)
  DSI_VVBPCR_VBP2_Msk* = (0x00000001 shl DSI_VVBPCR_VBP2_Pos) ## !< 0x00000004
  DSI_VVBPCR_VBP2* = DSI_VVBPCR_VBP2_Msk
  DSI_VVBPCR_VBP3_Pos* = (3)
  DSI_VVBPCR_VBP3_Msk* = (0x00000001 shl DSI_VVBPCR_VBP3_Pos) ## !< 0x00000008
  DSI_VVBPCR_VBP3* = DSI_VVBPCR_VBP3_Msk
  DSI_VVBPCR_VBP4_Pos* = (4)
  DSI_VVBPCR_VBP4_Msk* = (0x00000001 shl DSI_VVBPCR_VBP4_Pos) ## !< 0x00000010
  DSI_VVBPCR_VBP4* = DSI_VVBPCR_VBP4_Msk
  DSI_VVBPCR_VBP5_Pos* = (5)
  DSI_VVBPCR_VBP5_Msk* = (0x00000001 shl DSI_VVBPCR_VBP5_Pos) ## !< 0x00000020
  DSI_VVBPCR_VBP5* = DSI_VVBPCR_VBP5_Msk
  DSI_VVBPCR_VBP6_Pos* = (6)
  DSI_VVBPCR_VBP6_Msk* = (0x00000001 shl DSI_VVBPCR_VBP6_Pos) ## !< 0x00000040
  DSI_VVBPCR_VBP6* = DSI_VVBPCR_VBP6_Msk
  DSI_VVBPCR_VBP7_Pos* = (7)
  DSI_VVBPCR_VBP7_Msk* = (0x00000001 shl DSI_VVBPCR_VBP7_Pos) ## !< 0x00000080
  DSI_VVBPCR_VBP7* = DSI_VVBPCR_VBP7_Msk
  DSI_VVBPCR_VBP8_Pos* = (8)
  DSI_VVBPCR_VBP8_Msk* = (0x00000001 shl DSI_VVBPCR_VBP8_Pos) ## !< 0x00000100
  DSI_VVBPCR_VBP8* = DSI_VVBPCR_VBP8_Msk
  DSI_VVBPCR_VBP9_Pos* = (9)
  DSI_VVBPCR_VBP9_Msk* = (0x00000001 shl DSI_VVBPCR_VBP9_Pos) ## !< 0x00000200
  DSI_VVBPCR_VBP9* = DSI_VVBPCR_VBP9_Msk

## ******************  Bit definition for DSI_VVFPCR register  ************

const
  DSI_VVFPCR_VFP_Pos* = (0)
  DSI_VVFPCR_VFP_Msk* = (0x000003FF shl DSI_VVFPCR_VFP_Pos) ## !< 0x000003FF
  DSI_VVFPCR_VFP* = DSI_VVFPCR_VFP_Msk
  DSI_VVFPCR_VFP0_Pos* = (0)
  DSI_VVFPCR_VFP0_Msk* = (0x00000001 shl DSI_VVFPCR_VFP0_Pos) ## !< 0x00000001
  DSI_VVFPCR_VFP0* = DSI_VVFPCR_VFP0_Msk
  DSI_VVFPCR_VFP1_Pos* = (1)
  DSI_VVFPCR_VFP1_Msk* = (0x00000001 shl DSI_VVFPCR_VFP1_Pos) ## !< 0x00000002
  DSI_VVFPCR_VFP1* = DSI_VVFPCR_VFP1_Msk
  DSI_VVFPCR_VFP2_Pos* = (2)
  DSI_VVFPCR_VFP2_Msk* = (0x00000001 shl DSI_VVFPCR_VFP2_Pos) ## !< 0x00000004
  DSI_VVFPCR_VFP2* = DSI_VVFPCR_VFP2_Msk
  DSI_VVFPCR_VFP3_Pos* = (3)
  DSI_VVFPCR_VFP3_Msk* = (0x00000001 shl DSI_VVFPCR_VFP3_Pos) ## !< 0x00000008
  DSI_VVFPCR_VFP3* = DSI_VVFPCR_VFP3_Msk
  DSI_VVFPCR_VFP4_Pos* = (4)
  DSI_VVFPCR_VFP4_Msk* = (0x00000001 shl DSI_VVFPCR_VFP4_Pos) ## !< 0x00000010
  DSI_VVFPCR_VFP4* = DSI_VVFPCR_VFP4_Msk
  DSI_VVFPCR_VFP5_Pos* = (5)
  DSI_VVFPCR_VFP5_Msk* = (0x00000001 shl DSI_VVFPCR_VFP5_Pos) ## !< 0x00000020
  DSI_VVFPCR_VFP5* = DSI_VVFPCR_VFP5_Msk
  DSI_VVFPCR_VFP6_Pos* = (6)
  DSI_VVFPCR_VFP6_Msk* = (0x00000001 shl DSI_VVFPCR_VFP6_Pos) ## !< 0x00000040
  DSI_VVFPCR_VFP6* = DSI_VVFPCR_VFP6_Msk
  DSI_VVFPCR_VFP7_Pos* = (7)
  DSI_VVFPCR_VFP7_Msk* = (0x00000001 shl DSI_VVFPCR_VFP7_Pos) ## !< 0x00000080
  DSI_VVFPCR_VFP7* = DSI_VVFPCR_VFP7_Msk
  DSI_VVFPCR_VFP8_Pos* = (8)
  DSI_VVFPCR_VFP8_Msk* = (0x00000001 shl DSI_VVFPCR_VFP8_Pos) ## !< 0x00000100
  DSI_VVFPCR_VFP8* = DSI_VVFPCR_VFP8_Msk
  DSI_VVFPCR_VFP9_Pos* = (9)
  DSI_VVFPCR_VFP9_Msk* = (0x00000001 shl DSI_VVFPCR_VFP9_Pos) ## !< 0x00000200
  DSI_VVFPCR_VFP9* = DSI_VVFPCR_VFP9_Msk

## ******************  Bit definition for DSI_VVACR register  *************

const
  DSI_VVACR_VA_Pos* = (0)
  DSI_VVACR_VA_Msk* = (0x00003FFF shl DSI_VVACR_VA_Pos) ## !< 0x00003FFF
  DSI_VVACR_VA* = DSI_VVACR_VA_Msk
  DSI_VVACR_VA0_Pos* = (0)
  DSI_VVACR_VA0_Msk* = (0x00000001 shl DSI_VVACR_VA0_Pos) ## !< 0x00000001
  DSI_VVACR_VA0* = DSI_VVACR_VA0_Msk
  DSI_VVACR_VA1_Pos* = (1)
  DSI_VVACR_VA1_Msk* = (0x00000001 shl DSI_VVACR_VA1_Pos) ## !< 0x00000002
  DSI_VVACR_VA1* = DSI_VVACR_VA1_Msk
  DSI_VVACR_VA2_Pos* = (2)
  DSI_VVACR_VA2_Msk* = (0x00000001 shl DSI_VVACR_VA2_Pos) ## !< 0x00000004
  DSI_VVACR_VA2* = DSI_VVACR_VA2_Msk
  DSI_VVACR_VA3_Pos* = (3)
  DSI_VVACR_VA3_Msk* = (0x00000001 shl DSI_VVACR_VA3_Pos) ## !< 0x00000008
  DSI_VVACR_VA3* = DSI_VVACR_VA3_Msk
  DSI_VVACR_VA4_Pos* = (4)
  DSI_VVACR_VA4_Msk* = (0x00000001 shl DSI_VVACR_VA4_Pos) ## !< 0x00000010
  DSI_VVACR_VA4* = DSI_VVACR_VA4_Msk
  DSI_VVACR_VA5_Pos* = (5)
  DSI_VVACR_VA5_Msk* = (0x00000001 shl DSI_VVACR_VA5_Pos) ## !< 0x00000020
  DSI_VVACR_VA5* = DSI_VVACR_VA5_Msk
  DSI_VVACR_VA6_Pos* = (6)
  DSI_VVACR_VA6_Msk* = (0x00000001 shl DSI_VVACR_VA6_Pos) ## !< 0x00000040
  DSI_VVACR_VA6* = DSI_VVACR_VA6_Msk
  DSI_VVACR_VA7_Pos* = (7)
  DSI_VVACR_VA7_Msk* = (0x00000001 shl DSI_VVACR_VA7_Pos) ## !< 0x00000080
  DSI_VVACR_VA7* = DSI_VVACR_VA7_Msk
  DSI_VVACR_VA8_Pos* = (8)
  DSI_VVACR_VA8_Msk* = (0x00000001 shl DSI_VVACR_VA8_Pos) ## !< 0x00000100
  DSI_VVACR_VA8* = DSI_VVACR_VA8_Msk
  DSI_VVACR_VA9_Pos* = (9)
  DSI_VVACR_VA9_Msk* = (0x00000001 shl DSI_VVACR_VA9_Pos) ## !< 0x00000200
  DSI_VVACR_VA9* = DSI_VVACR_VA9_Msk
  DSI_VVACR_VA10_Pos* = (10)
  DSI_VVACR_VA10_Msk* = (0x00000001 shl DSI_VVACR_VA10_Pos) ## !< 0x00000400
  DSI_VVACR_VA10* = DSI_VVACR_VA10_Msk
  DSI_VVACR_VA11_Pos* = (11)
  DSI_VVACR_VA11_Msk* = (0x00000001 shl DSI_VVACR_VA11_Pos) ## !< 0x00000800
  DSI_VVACR_VA11* = DSI_VVACR_VA11_Msk
  DSI_VVACR_VA12_Pos* = (12)
  DSI_VVACR_VA12_Msk* = (0x00000001 shl DSI_VVACR_VA12_Pos) ## !< 0x00001000
  DSI_VVACR_VA12* = DSI_VVACR_VA12_Msk
  DSI_VVACR_VA13_Pos* = (13)
  DSI_VVACR_VA13_Msk* = (0x00000001 shl DSI_VVACR_VA13_Pos) ## !< 0x00002000
  DSI_VVACR_VA13* = DSI_VVACR_VA13_Msk

## ******************  Bit definition for DSI_LCCR register  **************

const
  DSI_LCCR_CMDSIZE_Pos* = (0)
  DSI_LCCR_CMDSIZE_Msk* = (0x0000FFFF shl DSI_LCCR_CMDSIZE_Pos) ## !< 0x0000FFFF
  DSI_LCCR_CMDSIZE* = DSI_LCCR_CMDSIZE_Msk
  DSI_LCCR_CMDSIZE0_Pos* = (0)
  DSI_LCCR_CMDSIZE0_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE0_Pos) ## !< 0x00000001
  DSI_LCCR_CMDSIZE0* = DSI_LCCR_CMDSIZE0_Msk
  DSI_LCCR_CMDSIZE1_Pos* = (1)
  DSI_LCCR_CMDSIZE1_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE1_Pos) ## !< 0x00000002
  DSI_LCCR_CMDSIZE1* = DSI_LCCR_CMDSIZE1_Msk
  DSI_LCCR_CMDSIZE2_Pos* = (2)
  DSI_LCCR_CMDSIZE2_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE2_Pos) ## !< 0x00000004
  DSI_LCCR_CMDSIZE2* = DSI_LCCR_CMDSIZE2_Msk
  DSI_LCCR_CMDSIZE3_Pos* = (3)
  DSI_LCCR_CMDSIZE3_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE3_Pos) ## !< 0x00000008
  DSI_LCCR_CMDSIZE3* = DSI_LCCR_CMDSIZE3_Msk
  DSI_LCCR_CMDSIZE4_Pos* = (4)
  DSI_LCCR_CMDSIZE4_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE4_Pos) ## !< 0x00000010
  DSI_LCCR_CMDSIZE4* = DSI_LCCR_CMDSIZE4_Msk
  DSI_LCCR_CMDSIZE5_Pos* = (5)
  DSI_LCCR_CMDSIZE5_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE5_Pos) ## !< 0x00000020
  DSI_LCCR_CMDSIZE5* = DSI_LCCR_CMDSIZE5_Msk
  DSI_LCCR_CMDSIZE6_Pos* = (6)
  DSI_LCCR_CMDSIZE6_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE6_Pos) ## !< 0x00000040
  DSI_LCCR_CMDSIZE6* = DSI_LCCR_CMDSIZE6_Msk
  DSI_LCCR_CMDSIZE7_Pos* = (7)
  DSI_LCCR_CMDSIZE7_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE7_Pos) ## !< 0x00000080
  DSI_LCCR_CMDSIZE7* = DSI_LCCR_CMDSIZE7_Msk
  DSI_LCCR_CMDSIZE8_Pos* = (8)
  DSI_LCCR_CMDSIZE8_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE8_Pos) ## !< 0x00000100
  DSI_LCCR_CMDSIZE8* = DSI_LCCR_CMDSIZE8_Msk
  DSI_LCCR_CMDSIZE9_Pos* = (9)
  DSI_LCCR_CMDSIZE9_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE9_Pos) ## !< 0x00000200
  DSI_LCCR_CMDSIZE9* = DSI_LCCR_CMDSIZE9_Msk
  DSI_LCCR_CMDSIZE10_Pos* = (10)
  DSI_LCCR_CMDSIZE10_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE10_Pos) ## !< 0x00000400
  DSI_LCCR_CMDSIZE10* = DSI_LCCR_CMDSIZE10_Msk
  DSI_LCCR_CMDSIZE11_Pos* = (11)
  DSI_LCCR_CMDSIZE11_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE11_Pos) ## !< 0x00000800
  DSI_LCCR_CMDSIZE11* = DSI_LCCR_CMDSIZE11_Msk
  DSI_LCCR_CMDSIZE12_Pos* = (12)
  DSI_LCCR_CMDSIZE12_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE12_Pos) ## !< 0x00001000
  DSI_LCCR_CMDSIZE12* = DSI_LCCR_CMDSIZE12_Msk
  DSI_LCCR_CMDSIZE13_Pos* = (13)
  DSI_LCCR_CMDSIZE13_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE13_Pos) ## !< 0x00002000
  DSI_LCCR_CMDSIZE13* = DSI_LCCR_CMDSIZE13_Msk
  DSI_LCCR_CMDSIZE14_Pos* = (14)
  DSI_LCCR_CMDSIZE14_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE14_Pos) ## !< 0x00004000
  DSI_LCCR_CMDSIZE14* = DSI_LCCR_CMDSIZE14_Msk
  DSI_LCCR_CMDSIZE15_Pos* = (15)
  DSI_LCCR_CMDSIZE15_Msk* = (0x00000001 shl DSI_LCCR_CMDSIZE15_Pos) ## !< 0x00008000
  DSI_LCCR_CMDSIZE15* = DSI_LCCR_CMDSIZE15_Msk

## ******************  Bit definition for DSI_CMCR register  **************

const
  DSI_CMCR_TEARE_Pos* = (0)
  DSI_CMCR_TEARE_Msk* = (0x00000001 shl DSI_CMCR_TEARE_Pos) ## !< 0x00000001
  DSI_CMCR_TEARE* = DSI_CMCR_TEARE_Msk
  DSI_CMCR_ARE_Pos* = (1)
  DSI_CMCR_ARE_Msk* = (0x00000001 shl DSI_CMCR_ARE_Pos) ## !< 0x00000002
  DSI_CMCR_ARE* = DSI_CMCR_ARE_Msk
  DSI_CMCR_GSW0TX_Pos* = (8)
  DSI_CMCR_GSW0TX_Msk* = (0x00000001 shl DSI_CMCR_GSW0TX_Pos) ## !< 0x00000100
  DSI_CMCR_GSW0TX* = DSI_CMCR_GSW0TX_Msk
  DSI_CMCR_GSW1TX_Pos* = (9)
  DSI_CMCR_GSW1TX_Msk* = (0x00000001 shl DSI_CMCR_GSW1TX_Pos) ## !< 0x00000200
  DSI_CMCR_GSW1TX* = DSI_CMCR_GSW1TX_Msk
  DSI_CMCR_GSW2TX_Pos* = (10)
  DSI_CMCR_GSW2TX_Msk* = (0x00000001 shl DSI_CMCR_GSW2TX_Pos) ## !< 0x00000400
  DSI_CMCR_GSW2TX* = DSI_CMCR_GSW2TX_Msk
  DSI_CMCR_GSR0TX_Pos* = (11)
  DSI_CMCR_GSR0TX_Msk* = (0x00000001 shl DSI_CMCR_GSR0TX_Pos) ## !< 0x00000800
  DSI_CMCR_GSR0TX* = DSI_CMCR_GSR0TX_Msk
  DSI_CMCR_GSR1TX_Pos* = (12)
  DSI_CMCR_GSR1TX_Msk* = (0x00000001 shl DSI_CMCR_GSR1TX_Pos) ## !< 0x00001000
  DSI_CMCR_GSR1TX* = DSI_CMCR_GSR1TX_Msk
  DSI_CMCR_GSR2TX_Pos* = (13)
  DSI_CMCR_GSR2TX_Msk* = (0x00000001 shl DSI_CMCR_GSR2TX_Pos) ## !< 0x00002000
  DSI_CMCR_GSR2TX* = DSI_CMCR_GSR2TX_Msk
  DSI_CMCR_GLWTX_Pos* = (14)
  DSI_CMCR_GLWTX_Msk* = (0x00000001 shl DSI_CMCR_GLWTX_Pos) ## !< 0x00004000
  DSI_CMCR_GLWTX* = DSI_CMCR_GLWTX_Msk
  DSI_CMCR_DSW0TX_Pos* = (16)
  DSI_CMCR_DSW0TX_Msk* = (0x00000001 shl DSI_CMCR_DSW0TX_Pos) ## !< 0x00010000
  DSI_CMCR_DSW0TX* = DSI_CMCR_DSW0TX_Msk
  DSI_CMCR_DSW1TX_Pos* = (17)
  DSI_CMCR_DSW1TX_Msk* = (0x00000001 shl DSI_CMCR_DSW1TX_Pos) ## !< 0x00020000
  DSI_CMCR_DSW1TX* = DSI_CMCR_DSW1TX_Msk
  DSI_CMCR_DSR0TX_Pos* = (18)
  DSI_CMCR_DSR0TX_Msk* = (0x00000001 shl DSI_CMCR_DSR0TX_Pos) ## !< 0x00040000
  DSI_CMCR_DSR0TX* = DSI_CMCR_DSR0TX_Msk
  DSI_CMCR_DLWTX_Pos* = (19)
  DSI_CMCR_DLWTX_Msk* = (0x00000001 shl DSI_CMCR_DLWTX_Pos) ## !< 0x00080000
  DSI_CMCR_DLWTX* = DSI_CMCR_DLWTX_Msk
  DSI_CMCR_MRDPS_Pos* = (24)
  DSI_CMCR_MRDPS_Msk* = (0x00000001 shl DSI_CMCR_MRDPS_Pos) ## !< 0x01000000
  DSI_CMCR_MRDPS* = DSI_CMCR_MRDPS_Msk

## ******************  Bit definition for DSI_GHCR register  **************

const
  DSI_GHCR_DT_Pos* = (0)
  DSI_GHCR_DT_Msk* = (0x0000003F shl DSI_GHCR_DT_Pos) ## !< 0x0000003F
  DSI_GHCR_DT* = DSI_GHCR_DT_Msk
  DSI_GHCR_DT0_Pos* = (0)
  DSI_GHCR_DT0_Msk* = (0x00000001 shl DSI_GHCR_DT0_Pos) ## !< 0x00000001
  DSI_GHCR_DT0* = DSI_GHCR_DT0_Msk
  DSI_GHCR_DT1_Pos* = (1)
  DSI_GHCR_DT1_Msk* = (0x00000001 shl DSI_GHCR_DT1_Pos) ## !< 0x00000002
  DSI_GHCR_DT1* = DSI_GHCR_DT1_Msk
  DSI_GHCR_DT2_Pos* = (2)
  DSI_GHCR_DT2_Msk* = (0x00000001 shl DSI_GHCR_DT2_Pos) ## !< 0x00000004
  DSI_GHCR_DT2* = DSI_GHCR_DT2_Msk
  DSI_GHCR_DT3_Pos* = (3)
  DSI_GHCR_DT3_Msk* = (0x00000001 shl DSI_GHCR_DT3_Pos) ## !< 0x00000008
  DSI_GHCR_DT3* = DSI_GHCR_DT3_Msk
  DSI_GHCR_DT4_Pos* = (4)
  DSI_GHCR_DT4_Msk* = (0x00000001 shl DSI_GHCR_DT4_Pos) ## !< 0x00000010
  DSI_GHCR_DT4* = DSI_GHCR_DT4_Msk
  DSI_GHCR_DT5_Pos* = (5)
  DSI_GHCR_DT5_Msk* = (0x00000001 shl DSI_GHCR_DT5_Pos) ## !< 0x00000020
  DSI_GHCR_DT5* = DSI_GHCR_DT5_Msk
  DSI_GHCR_VCID_Pos* = (6)
  DSI_GHCR_VCID_Msk* = (0x00000003 shl DSI_GHCR_VCID_Pos) ## !< 0x000000C0
  DSI_GHCR_VCID* = DSI_GHCR_VCID_Msk
  DSI_GHCR_VCID0_Pos* = (6)
  DSI_GHCR_VCID0_Msk* = (0x00000001 shl DSI_GHCR_VCID0_Pos) ## !< 0x00000040
  DSI_GHCR_VCID0* = DSI_GHCR_VCID0_Msk
  DSI_GHCR_VCID1_Pos* = (7)
  DSI_GHCR_VCID1_Msk* = (0x00000001 shl DSI_GHCR_VCID1_Pos) ## !< 0x00000080
  DSI_GHCR_VCID1* = DSI_GHCR_VCID1_Msk
  DSI_GHCR_WCLSB_Pos* = (8)
  DSI_GHCR_WCLSB_Msk* = (0x000000FF shl DSI_GHCR_WCLSB_Pos) ## !< 0x0000FF00
  DSI_GHCR_WCLSB* = DSI_GHCR_WCLSB_Msk
  DSI_GHCR_WCLSB0_Pos* = (8)
  DSI_GHCR_WCLSB0_Msk* = (0x00000001 shl DSI_GHCR_WCLSB0_Pos) ## !< 0x00000100
  DSI_GHCR_WCLSB0* = DSI_GHCR_WCLSB0_Msk
  DSI_GHCR_WCLSB1_Pos* = (9)
  DSI_GHCR_WCLSB1_Msk* = (0x00000001 shl DSI_GHCR_WCLSB1_Pos) ## !< 0x00000200
  DSI_GHCR_WCLSB1* = DSI_GHCR_WCLSB1_Msk
  DSI_GHCR_WCLSB2_Pos* = (10)
  DSI_GHCR_WCLSB2_Msk* = (0x00000001 shl DSI_GHCR_WCLSB2_Pos) ## !< 0x00000400
  DSI_GHCR_WCLSB2* = DSI_GHCR_WCLSB2_Msk
  DSI_GHCR_WCLSB3_Pos* = (11)
  DSI_GHCR_WCLSB3_Msk* = (0x00000001 shl DSI_GHCR_WCLSB3_Pos) ## !< 0x00000800
  DSI_GHCR_WCLSB3* = DSI_GHCR_WCLSB3_Msk
  DSI_GHCR_WCLSB4_Pos* = (12)
  DSI_GHCR_WCLSB4_Msk* = (0x00000001 shl DSI_GHCR_WCLSB4_Pos) ## !< 0x00001000
  DSI_GHCR_WCLSB4* = DSI_GHCR_WCLSB4_Msk
  DSI_GHCR_WCLSB5_Pos* = (13)
  DSI_GHCR_WCLSB5_Msk* = (0x00000001 shl DSI_GHCR_WCLSB5_Pos) ## !< 0x00002000
  DSI_GHCR_WCLSB5* = DSI_GHCR_WCLSB5_Msk
  DSI_GHCR_WCLSB6_Pos* = (14)
  DSI_GHCR_WCLSB6_Msk* = (0x00000001 shl DSI_GHCR_WCLSB6_Pos) ## !< 0x00004000
  DSI_GHCR_WCLSB6* = DSI_GHCR_WCLSB6_Msk
  DSI_GHCR_WCLSB7_Pos* = (15)
  DSI_GHCR_WCLSB7_Msk* = (0x00000001 shl DSI_GHCR_WCLSB7_Pos) ## !< 0x00008000
  DSI_GHCR_WCLSB7* = DSI_GHCR_WCLSB7_Msk
  DSI_GHCR_WCMSB_Pos* = (16)
  DSI_GHCR_WCMSB_Msk* = (0x000000FF shl DSI_GHCR_WCMSB_Pos) ## !< 0x00FF0000
  DSI_GHCR_WCMSB* = DSI_GHCR_WCMSB_Msk
  DSI_GHCR_WCMSB0_Pos* = (16)
  DSI_GHCR_WCMSB0_Msk* = (0x00000001 shl DSI_GHCR_WCMSB0_Pos) ## !< 0x00010000
  DSI_GHCR_WCMSB0* = DSI_GHCR_WCMSB0_Msk
  DSI_GHCR_WCMSB1_Pos* = (17)
  DSI_GHCR_WCMSB1_Msk* = (0x00000001 shl DSI_GHCR_WCMSB1_Pos) ## !< 0x00020000
  DSI_GHCR_WCMSB1* = DSI_GHCR_WCMSB1_Msk
  DSI_GHCR_WCMSB2_Pos* = (18)
  DSI_GHCR_WCMSB2_Msk* = (0x00000001 shl DSI_GHCR_WCMSB2_Pos) ## !< 0x00040000
  DSI_GHCR_WCMSB2* = DSI_GHCR_WCMSB2_Msk
  DSI_GHCR_WCMSB3_Pos* = (19)
  DSI_GHCR_WCMSB3_Msk* = (0x00000001 shl DSI_GHCR_WCMSB3_Pos) ## !< 0x00080000
  DSI_GHCR_WCMSB3* = DSI_GHCR_WCMSB3_Msk
  DSI_GHCR_WCMSB4_Pos* = (20)
  DSI_GHCR_WCMSB4_Msk* = (0x00000001 shl DSI_GHCR_WCMSB4_Pos) ## !< 0x00100000
  DSI_GHCR_WCMSB4* = DSI_GHCR_WCMSB4_Msk
  DSI_GHCR_WCMSB5_Pos* = (21)
  DSI_GHCR_WCMSB5_Msk* = (0x00000001 shl DSI_GHCR_WCMSB5_Pos) ## !< 0x00200000
  DSI_GHCR_WCMSB5* = DSI_GHCR_WCMSB5_Msk
  DSI_GHCR_WCMSB6_Pos* = (22)
  DSI_GHCR_WCMSB6_Msk* = (0x00000001 shl DSI_GHCR_WCMSB6_Pos) ## !< 0x00400000
  DSI_GHCR_WCMSB6* = DSI_GHCR_WCMSB6_Msk
  DSI_GHCR_WCMSB7_Pos* = (23)
  DSI_GHCR_WCMSB7_Msk* = (0x00000001 shl DSI_GHCR_WCMSB7_Pos) ## !< 0x00800000
  DSI_GHCR_WCMSB7* = DSI_GHCR_WCMSB7_Msk

## ******************  Bit definition for DSI_GPDR register  **************

const
  DSI_GPDR_DATA1_Pos* = (0)
  DSI_GPDR_DATA1_Msk* = (0x000000FF shl DSI_GPDR_DATA1_Pos) ## !< 0x000000FF
  DSI_GPDR_DATA1* = DSI_GPDR_DATA1_Msk
  DSI_GPDR_DATA1_Bit0* = (0x00000001 shl DSI_GPDR_DATA1_Pos) ## !< 0x00000001
  DSI_GPDR_DATA1_Bit1* = (0x00000002 shl DSI_GPDR_DATA1_Pos) ## !< 0x00000002
  DSI_GPDR_DATA1_Bit2* = (0x00000004 shl DSI_GPDR_DATA1_Pos) ## !< 0x00000004
  DSI_GPDR_DATA1_Bit3* = (0x00000008 shl DSI_GPDR_DATA1_Pos) ## !< 0x00000008
  DSI_GPDR_DATA1_Bit4* = (0x00000010 shl DSI_GPDR_DATA1_Pos) ## !< 0x00000010
  DSI_GPDR_DATA1_Bit5* = (0x00000020 shl DSI_GPDR_DATA1_Pos) ## !< 0x00000020
  DSI_GPDR_DATA1_Bit6* = (0x00000040 shl DSI_GPDR_DATA1_Pos) ## !< 0x00000040
  DSI_GPDR_DATA1_Bit7* = (0x00000080 shl DSI_GPDR_DATA1_Pos) ## !< 0x00000080
  DSI_GPDR_DATA2_Pos* = (8)
  DSI_GPDR_DATA2_Msk* = (0x000000FF shl DSI_GPDR_DATA2_Pos) ## !< 0x0000FF00
  DSI_GPDR_DATA2* = DSI_GPDR_DATA2_Msk
  DSI_GPDR_DATA2_Bit0* = (0x00000001 shl DSI_GPDR_DATA2_Pos) ## !< 0x00000100
  DSI_GPDR_DATA2_Bit1* = (0x00000002 shl DSI_GPDR_DATA2_Pos) ## !< 0x00000200
  DSI_GPDR_DATA2_Bit2* = (0x00000004 shl DSI_GPDR_DATA2_Pos) ## !< 0x00000400
  DSI_GPDR_DATA2_Bit3* = (0x00000008 shl DSI_GPDR_DATA2_Pos) ## !< 0x00000800
  DSI_GPDR_DATA2_Bit4* = (0x00000010 shl DSI_GPDR_DATA2_Pos) ## !< 0x00001000
  DSI_GPDR_DATA2_Bit5* = (0x00000020 shl DSI_GPDR_DATA2_Pos) ## !< 0x00002000
  DSI_GPDR_DATA2_Bit6* = (0x00000040 shl DSI_GPDR_DATA2_Pos) ## !< 0x00004000
  DSI_GPDR_DATA2_Bit7* = (0x00000080 shl DSI_GPDR_DATA2_Pos) ## !< 0x00008000
  DSI_GPDR_DATA3_Pos* = (16)
  DSI_GPDR_DATA3_Msk* = (0x000000FF shl DSI_GPDR_DATA3_Pos) ## !< 0x00FF0000
  DSI_GPDR_DATA3* = DSI_GPDR_DATA3_Msk
  DSI_GPDR_DATA3_Bit0* = (0x00000001 shl DSI_GPDR_DATA3_Pos) ## !< 0x00010000
  DSI_GPDR_DATA3_Bit1* = (0x00000002 shl DSI_GPDR_DATA3_Pos) ## !< 0x00020000
  DSI_GPDR_DATA3_Bit2* = (0x00000004 shl DSI_GPDR_DATA3_Pos) ## !< 0x00040000
  DSI_GPDR_DATA3_Bit3* = (0x00000008 shl DSI_GPDR_DATA3_Pos) ## !< 0x00080000
  DSI_GPDR_DATA3_Bit4* = (0x00000010 shl DSI_GPDR_DATA3_Pos) ## !< 0x00100000
  DSI_GPDR_DATA3_Bit5* = (0x00000020 shl DSI_GPDR_DATA3_Pos) ## !< 0x00200000
  DSI_GPDR_DATA3_Bit6* = (0x00000040 shl DSI_GPDR_DATA3_Pos) ## !< 0x00400000
  DSI_GPDR_DATA3_Bit7* = (0x00000080 shl DSI_GPDR_DATA3_Pos) ## !< 0x00800000
  DSI_GPDR_DATA4_Pos* = (24)
  DSI_GPDR_DATA4_Msk* = (0x000000FF shl DSI_GPDR_DATA4_Pos) ## !< 0xFF000000
  DSI_GPDR_DATA4* = DSI_GPDR_DATA4_Msk
  DSI_GPDR_DATA4_Bit0* = (0x00000001 shl DSI_GPDR_DATA4_Pos) ## !< 0x01000000
  DSI_GPDR_DATA4_Bit1* = (0x00000002 shl DSI_GPDR_DATA4_Pos) ## !< 0x02000000
  DSI_GPDR_DATA4_Bit2* = (0x00000004 shl DSI_GPDR_DATA4_Pos) ## !< 0x04000000
  DSI_GPDR_DATA4_Bit3* = (0x00000008 shl DSI_GPDR_DATA4_Pos) ## !< 0x08000000
  DSI_GPDR_DATA4_Bit4* = (0x00000010 shl DSI_GPDR_DATA4_Pos) ## !< 0x10000000
  DSI_GPDR_DATA4_Bit5* = (0x00000020 shl DSI_GPDR_DATA4_Pos) ## !< 0x20000000
  DSI_GPDR_DATA4_Bit6* = (0x00000040 shl DSI_GPDR_DATA4_Pos) ## !< 0x40000000
  DSI_GPDR_DATA4_Bit7* = (0x00000080 shl DSI_GPDR_DATA4_Pos) ## !< 0x80000000

## ******************  Bit definition for DSI_GPSR register  **************

const
  DSI_GPSR_CMDFE_Pos* = (0)
  DSI_GPSR_CMDFE_Msk* = (0x00000001 shl DSI_GPSR_CMDFE_Pos) ## !< 0x00000001
  DSI_GPSR_CMDFE* = DSI_GPSR_CMDFE_Msk
  DSI_GPSR_CMDFF_Pos* = (1)
  DSI_GPSR_CMDFF_Msk* = (0x00000001 shl DSI_GPSR_CMDFF_Pos) ## !< 0x00000002
  DSI_GPSR_CMDFF* = DSI_GPSR_CMDFF_Msk
  DSI_GPSR_PWRFE_Pos* = (2)
  DSI_GPSR_PWRFE_Msk* = (0x00000001 shl DSI_GPSR_PWRFE_Pos) ## !< 0x00000004
  DSI_GPSR_PWRFE* = DSI_GPSR_PWRFE_Msk
  DSI_GPSR_PWRFF_Pos* = (3)
  DSI_GPSR_PWRFF_Msk* = (0x00000001 shl DSI_GPSR_PWRFF_Pos) ## !< 0x00000008
  DSI_GPSR_PWRFF* = DSI_GPSR_PWRFF_Msk
  DSI_GPSR_PRDFE_Pos* = (4)
  DSI_GPSR_PRDFE_Msk* = (0x00000001 shl DSI_GPSR_PRDFE_Pos) ## !< 0x00000010
  DSI_GPSR_PRDFE* = DSI_GPSR_PRDFE_Msk
  DSI_GPSR_PRDFF_Pos* = (5)
  DSI_GPSR_PRDFF_Msk* = (0x00000001 shl DSI_GPSR_PRDFF_Pos) ## !< 0x00000020
  DSI_GPSR_PRDFF* = DSI_GPSR_PRDFF_Msk
  DSI_GPSR_RCB_Pos* = (6)
  DSI_GPSR_RCB_Msk* = (0x00000001 shl DSI_GPSR_RCB_Pos) ## !< 0x00000040
  DSI_GPSR_RCB* = DSI_GPSR_RCB_Msk

## ******************  Bit definition for DSI_TCCR0 register  *************

const
  DSI_TCCR0_LPRX_TOCNT_Pos* = (0)
  DSI_TCCR0_LPRX_TOCNT_Msk* = (0x0000FFFF shl DSI_TCCR0_LPRX_TOCNT_Pos) ## !< 0x0000FFFF
  DSI_TCCR0_LPRX_TOCNT* = DSI_TCCR0_LPRX_TOCNT_Msk
  DSI_TCCR0_LPRX_TOCNT0_Pos* = (0)
  DSI_TCCR0_LPRX_TOCNT0_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT0_Pos) ## !< 0x00000001
  DSI_TCCR0_LPRX_TOCNT0* = DSI_TCCR0_LPRX_TOCNT0_Msk
  DSI_TCCR0_LPRX_TOCNT1_Pos* = (1)
  DSI_TCCR0_LPRX_TOCNT1_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT1_Pos) ## !< 0x00000002
  DSI_TCCR0_LPRX_TOCNT1* = DSI_TCCR0_LPRX_TOCNT1_Msk
  DSI_TCCR0_LPRX_TOCNT2_Pos* = (2)
  DSI_TCCR0_LPRX_TOCNT2_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT2_Pos) ## !< 0x00000004
  DSI_TCCR0_LPRX_TOCNT2* = DSI_TCCR0_LPRX_TOCNT2_Msk
  DSI_TCCR0_LPRX_TOCNT3_Pos* = (3)
  DSI_TCCR0_LPRX_TOCNT3_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT3_Pos) ## !< 0x00000008
  DSI_TCCR0_LPRX_TOCNT3* = DSI_TCCR0_LPRX_TOCNT3_Msk
  DSI_TCCR0_LPRX_TOCNT4_Pos* = (4)
  DSI_TCCR0_LPRX_TOCNT4_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT4_Pos) ## !< 0x00000010
  DSI_TCCR0_LPRX_TOCNT4* = DSI_TCCR0_LPRX_TOCNT4_Msk
  DSI_TCCR0_LPRX_TOCNT5_Pos* = (5)
  DSI_TCCR0_LPRX_TOCNT5_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT5_Pos) ## !< 0x00000020
  DSI_TCCR0_LPRX_TOCNT5* = DSI_TCCR0_LPRX_TOCNT5_Msk
  DSI_TCCR0_LPRX_TOCNT6_Pos* = (6)
  DSI_TCCR0_LPRX_TOCNT6_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT6_Pos) ## !< 0x00000040
  DSI_TCCR0_LPRX_TOCNT6* = DSI_TCCR0_LPRX_TOCNT6_Msk
  DSI_TCCR0_LPRX_TOCNT7_Pos* = (7)
  DSI_TCCR0_LPRX_TOCNT7_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT7_Pos) ## !< 0x00000080
  DSI_TCCR0_LPRX_TOCNT7* = DSI_TCCR0_LPRX_TOCNT7_Msk
  DSI_TCCR0_LPRX_TOCNT8_Pos* = (8)
  DSI_TCCR0_LPRX_TOCNT8_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT8_Pos) ## !< 0x00000100
  DSI_TCCR0_LPRX_TOCNT8* = DSI_TCCR0_LPRX_TOCNT8_Msk
  DSI_TCCR0_LPRX_TOCNT9_Pos* = (9)
  DSI_TCCR0_LPRX_TOCNT9_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT9_Pos) ## !< 0x00000200
  DSI_TCCR0_LPRX_TOCNT9* = DSI_TCCR0_LPRX_TOCNT9_Msk
  DSI_TCCR0_LPRX_TOCNT10_Pos* = (10)
  DSI_TCCR0_LPRX_TOCNT10_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT10_Pos) ## !< 0x00000400
  DSI_TCCR0_LPRX_TOCNT10* = DSI_TCCR0_LPRX_TOCNT10_Msk
  DSI_TCCR0_LPRX_TOCNT11_Pos* = (11)
  DSI_TCCR0_LPRX_TOCNT11_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT11_Pos) ## !< 0x00000800
  DSI_TCCR0_LPRX_TOCNT11* = DSI_TCCR0_LPRX_TOCNT11_Msk
  DSI_TCCR0_LPRX_TOCNT12_Pos* = (12)
  DSI_TCCR0_LPRX_TOCNT12_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT12_Pos) ## !< 0x00001000
  DSI_TCCR0_LPRX_TOCNT12* = DSI_TCCR0_LPRX_TOCNT12_Msk
  DSI_TCCR0_LPRX_TOCNT13_Pos* = (13)
  DSI_TCCR0_LPRX_TOCNT13_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT13_Pos) ## !< 0x00002000
  DSI_TCCR0_LPRX_TOCNT13* = DSI_TCCR0_LPRX_TOCNT13_Msk
  DSI_TCCR0_LPRX_TOCNT14_Pos* = (14)
  DSI_TCCR0_LPRX_TOCNT14_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT14_Pos) ## !< 0x00004000
  DSI_TCCR0_LPRX_TOCNT14* = DSI_TCCR0_LPRX_TOCNT14_Msk
  DSI_TCCR0_LPRX_TOCNT15_Pos* = (15)
  DSI_TCCR0_LPRX_TOCNT15_Msk* = (0x00000001 shl DSI_TCCR0_LPRX_TOCNT15_Pos) ## !< 0x00008000
  DSI_TCCR0_LPRX_TOCNT15* = DSI_TCCR0_LPRX_TOCNT15_Msk
  DSI_TCCR0_HSTX_TOCNT_Pos* = (16)
  DSI_TCCR0_HSTX_TOCNT_Msk* = (0x0000FFFF shl DSI_TCCR0_HSTX_TOCNT_Pos) ## !< 0xFFFF0000
  DSI_TCCR0_HSTX_TOCNT* = DSI_TCCR0_HSTX_TOCNT_Msk
  DSI_TCCR0_HSTX_TOCNT0_Pos* = (16)
  DSI_TCCR0_HSTX_TOCNT0_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT0_Pos) ## !< 0x00010000
  DSI_TCCR0_HSTX_TOCNT0* = DSI_TCCR0_HSTX_TOCNT0_Msk
  DSI_TCCR0_HSTX_TOCNT1_Pos* = (17)
  DSI_TCCR0_HSTX_TOCNT1_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT1_Pos) ## !< 0x00020000
  DSI_TCCR0_HSTX_TOCNT1* = DSI_TCCR0_HSTX_TOCNT1_Msk
  DSI_TCCR0_HSTX_TOCNT2_Pos* = (18)
  DSI_TCCR0_HSTX_TOCNT2_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT2_Pos) ## !< 0x00040000
  DSI_TCCR0_HSTX_TOCNT2* = DSI_TCCR0_HSTX_TOCNT2_Msk
  DSI_TCCR0_HSTX_TOCNT3_Pos* = (19)
  DSI_TCCR0_HSTX_TOCNT3_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT3_Pos) ## !< 0x00080000
  DSI_TCCR0_HSTX_TOCNT3* = DSI_TCCR0_HSTX_TOCNT3_Msk
  DSI_TCCR0_HSTX_TOCNT4_Pos* = (20)
  DSI_TCCR0_HSTX_TOCNT4_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT4_Pos) ## !< 0x00100000
  DSI_TCCR0_HSTX_TOCNT4* = DSI_TCCR0_HSTX_TOCNT4_Msk
  DSI_TCCR0_HSTX_TOCNT5_Pos* = (21)
  DSI_TCCR0_HSTX_TOCNT5_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT5_Pos) ## !< 0x00200000
  DSI_TCCR0_HSTX_TOCNT5* = DSI_TCCR0_HSTX_TOCNT5_Msk
  DSI_TCCR0_HSTX_TOCNT6_Pos* = (22)
  DSI_TCCR0_HSTX_TOCNT6_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT6_Pos) ## !< 0x00400000
  DSI_TCCR0_HSTX_TOCNT6* = DSI_TCCR0_HSTX_TOCNT6_Msk
  DSI_TCCR0_HSTX_TOCNT7_Pos* = (23)
  DSI_TCCR0_HSTX_TOCNT7_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT7_Pos) ## !< 0x00800000
  DSI_TCCR0_HSTX_TOCNT7* = DSI_TCCR0_HSTX_TOCNT7_Msk
  DSI_TCCR0_HSTX_TOCNT8_Pos* = (24)
  DSI_TCCR0_HSTX_TOCNT8_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT8_Pos) ## !< 0x01000000
  DSI_TCCR0_HSTX_TOCNT8* = DSI_TCCR0_HSTX_TOCNT8_Msk
  DSI_TCCR0_HSTX_TOCNT9_Pos* = (25)
  DSI_TCCR0_HSTX_TOCNT9_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT9_Pos) ## !< 0x02000000
  DSI_TCCR0_HSTX_TOCNT9* = DSI_TCCR0_HSTX_TOCNT9_Msk
  DSI_TCCR0_HSTX_TOCNT10_Pos* = (26)
  DSI_TCCR0_HSTX_TOCNT10_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT10_Pos) ## !< 0x04000000
  DSI_TCCR0_HSTX_TOCNT10* = DSI_TCCR0_HSTX_TOCNT10_Msk
  DSI_TCCR0_HSTX_TOCNT11_Pos* = (27)
  DSI_TCCR0_HSTX_TOCNT11_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT11_Pos) ## !< 0x08000000
  DSI_TCCR0_HSTX_TOCNT11* = DSI_TCCR0_HSTX_TOCNT11_Msk
  DSI_TCCR0_HSTX_TOCNT12_Pos* = (28)
  DSI_TCCR0_HSTX_TOCNT12_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT12_Pos) ## !< 0x10000000
  DSI_TCCR0_HSTX_TOCNT12* = DSI_TCCR0_HSTX_TOCNT12_Msk
  DSI_TCCR0_HSTX_TOCNT13_Pos* = (29)
  DSI_TCCR0_HSTX_TOCNT13_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT13_Pos) ## !< 0x20000000
  DSI_TCCR0_HSTX_TOCNT13* = DSI_TCCR0_HSTX_TOCNT13_Msk
  DSI_TCCR0_HSTX_TOCNT14_Pos* = (30)
  DSI_TCCR0_HSTX_TOCNT14_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT14_Pos) ## !< 0x40000000
  DSI_TCCR0_HSTX_TOCNT14* = DSI_TCCR0_HSTX_TOCNT14_Msk
  DSI_TCCR0_HSTX_TOCNT15_Pos* = (31)
  DSI_TCCR0_HSTX_TOCNT15_Msk* = (0x00000001 shl DSI_TCCR0_HSTX_TOCNT15_Pos) ## !< 0x80000000
  DSI_TCCR0_HSTX_TOCNT15* = DSI_TCCR0_HSTX_TOCNT15_Msk

## ******************  Bit definition for DSI_TCCR1 register  *************

const
  DSI_TCCR1_HSRD_TOCNT_Pos* = (0)
  DSI_TCCR1_HSRD_TOCNT_Msk* = (0x0000FFFF shl DSI_TCCR1_HSRD_TOCNT_Pos) ## !< 0x0000FFFF
  DSI_TCCR1_HSRD_TOCNT* = DSI_TCCR1_HSRD_TOCNT_Msk
  DSI_TCCR1_HSRD_TOCNT0_Pos* = (0)
  DSI_TCCR1_HSRD_TOCNT0_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT0_Pos) ## !< 0x00000001
  DSI_TCCR1_HSRD_TOCNT0* = DSI_TCCR1_HSRD_TOCNT0_Msk
  DSI_TCCR1_HSRD_TOCNT1_Pos* = (1)
  DSI_TCCR1_HSRD_TOCNT1_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT1_Pos) ## !< 0x00000002
  DSI_TCCR1_HSRD_TOCNT1* = DSI_TCCR1_HSRD_TOCNT1_Msk
  DSI_TCCR1_HSRD_TOCNT2_Pos* = (2)
  DSI_TCCR1_HSRD_TOCNT2_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT2_Pos) ## !< 0x00000004
  DSI_TCCR1_HSRD_TOCNT2* = DSI_TCCR1_HSRD_TOCNT2_Msk
  DSI_TCCR1_HSRD_TOCNT3_Pos* = (3)
  DSI_TCCR1_HSRD_TOCNT3_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT3_Pos) ## !< 0x00000008
  DSI_TCCR1_HSRD_TOCNT3* = DSI_TCCR1_HSRD_TOCNT3_Msk
  DSI_TCCR1_HSRD_TOCNT4_Pos* = (4)
  DSI_TCCR1_HSRD_TOCNT4_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT4_Pos) ## !< 0x00000010
  DSI_TCCR1_HSRD_TOCNT4* = DSI_TCCR1_HSRD_TOCNT4_Msk
  DSI_TCCR1_HSRD_TOCNT5_Pos* = (5)
  DSI_TCCR1_HSRD_TOCNT5_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT5_Pos) ## !< 0x00000020
  DSI_TCCR1_HSRD_TOCNT5* = DSI_TCCR1_HSRD_TOCNT5_Msk
  DSI_TCCR1_HSRD_TOCNT6_Pos* = (6)
  DSI_TCCR1_HSRD_TOCNT6_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT6_Pos) ## !< 0x00000040
  DSI_TCCR1_HSRD_TOCNT6* = DSI_TCCR1_HSRD_TOCNT6_Msk
  DSI_TCCR1_HSRD_TOCNT7_Pos* = (7)
  DSI_TCCR1_HSRD_TOCNT7_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT7_Pos) ## !< 0x00000080
  DSI_TCCR1_HSRD_TOCNT7* = DSI_TCCR1_HSRD_TOCNT7_Msk
  DSI_TCCR1_HSRD_TOCNT8_Pos* = (8)
  DSI_TCCR1_HSRD_TOCNT8_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT8_Pos) ## !< 0x00000100
  DSI_TCCR1_HSRD_TOCNT8* = DSI_TCCR1_HSRD_TOCNT8_Msk
  DSI_TCCR1_HSRD_TOCNT9_Pos* = (9)
  DSI_TCCR1_HSRD_TOCNT9_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT9_Pos) ## !< 0x00000200
  DSI_TCCR1_HSRD_TOCNT9* = DSI_TCCR1_HSRD_TOCNT9_Msk
  DSI_TCCR1_HSRD_TOCNT10_Pos* = (10)
  DSI_TCCR1_HSRD_TOCNT10_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT10_Pos) ## !< 0x00000400
  DSI_TCCR1_HSRD_TOCNT10* = DSI_TCCR1_HSRD_TOCNT10_Msk
  DSI_TCCR1_HSRD_TOCNT11_Pos* = (11)
  DSI_TCCR1_HSRD_TOCNT11_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT11_Pos) ## !< 0x00000800
  DSI_TCCR1_HSRD_TOCNT11* = DSI_TCCR1_HSRD_TOCNT11_Msk
  DSI_TCCR1_HSRD_TOCNT12_Pos* = (12)
  DSI_TCCR1_HSRD_TOCNT12_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT12_Pos) ## !< 0x00001000
  DSI_TCCR1_HSRD_TOCNT12* = DSI_TCCR1_HSRD_TOCNT12_Msk
  DSI_TCCR1_HSRD_TOCNT13_Pos* = (13)
  DSI_TCCR1_HSRD_TOCNT13_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT13_Pos) ## !< 0x00002000
  DSI_TCCR1_HSRD_TOCNT13* = DSI_TCCR1_HSRD_TOCNT13_Msk
  DSI_TCCR1_HSRD_TOCNT14_Pos* = (14)
  DSI_TCCR1_HSRD_TOCNT14_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT14_Pos) ## !< 0x00004000
  DSI_TCCR1_HSRD_TOCNT14* = DSI_TCCR1_HSRD_TOCNT14_Msk
  DSI_TCCR1_HSRD_TOCNT15_Pos* = (15)
  DSI_TCCR1_HSRD_TOCNT15_Msk* = (0x00000001 shl DSI_TCCR1_HSRD_TOCNT15_Pos) ## !< 0x00008000
  DSI_TCCR1_HSRD_TOCNT15* = DSI_TCCR1_HSRD_TOCNT15_Msk

## ******************  Bit definition for DSI_TCCR2 register  *************

const
  DSI_TCCR2_LPRD_TOCNT_Pos* = (0)
  DSI_TCCR2_LPRD_TOCNT_Msk* = (0x0000FFFF shl DSI_TCCR2_LPRD_TOCNT_Pos) ## !< 0x0000FFFF
  DSI_TCCR2_LPRD_TOCNT* = DSI_TCCR2_LPRD_TOCNT_Msk
  DSI_TCCR2_LPRD_TOCNT0_Pos* = (0)
  DSI_TCCR2_LPRD_TOCNT0_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT0_Pos) ## !< 0x00000001
  DSI_TCCR2_LPRD_TOCNT0* = DSI_TCCR2_LPRD_TOCNT0_Msk
  DSI_TCCR2_LPRD_TOCNT1_Pos* = (1)
  DSI_TCCR2_LPRD_TOCNT1_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT1_Pos) ## !< 0x00000002
  DSI_TCCR2_LPRD_TOCNT1* = DSI_TCCR2_LPRD_TOCNT1_Msk
  DSI_TCCR2_LPRD_TOCNT2_Pos* = (2)
  DSI_TCCR2_LPRD_TOCNT2_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT2_Pos) ## !< 0x00000004
  DSI_TCCR2_LPRD_TOCNT2* = DSI_TCCR2_LPRD_TOCNT2_Msk
  DSI_TCCR2_LPRD_TOCNT3_Pos* = (3)
  DSI_TCCR2_LPRD_TOCNT3_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT3_Pos) ## !< 0x00000008
  DSI_TCCR2_LPRD_TOCNT3* = DSI_TCCR2_LPRD_TOCNT3_Msk
  DSI_TCCR2_LPRD_TOCNT4_Pos* = (4)
  DSI_TCCR2_LPRD_TOCNT4_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT4_Pos) ## !< 0x00000010
  DSI_TCCR2_LPRD_TOCNT4* = DSI_TCCR2_LPRD_TOCNT4_Msk
  DSI_TCCR2_LPRD_TOCNT5_Pos* = (5)
  DSI_TCCR2_LPRD_TOCNT5_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT5_Pos) ## !< 0x00000020
  DSI_TCCR2_LPRD_TOCNT5* = DSI_TCCR2_LPRD_TOCNT5_Msk
  DSI_TCCR2_LPRD_TOCNT6_Pos* = (6)
  DSI_TCCR2_LPRD_TOCNT6_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT6_Pos) ## !< 0x00000040
  DSI_TCCR2_LPRD_TOCNT6* = DSI_TCCR2_LPRD_TOCNT6_Msk
  DSI_TCCR2_LPRD_TOCNT7_Pos* = (7)
  DSI_TCCR2_LPRD_TOCNT7_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT7_Pos) ## !< 0x00000080
  DSI_TCCR2_LPRD_TOCNT7* = DSI_TCCR2_LPRD_TOCNT7_Msk
  DSI_TCCR2_LPRD_TOCNT8_Pos* = (8)
  DSI_TCCR2_LPRD_TOCNT8_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT8_Pos) ## !< 0x00000100
  DSI_TCCR2_LPRD_TOCNT8* = DSI_TCCR2_LPRD_TOCNT8_Msk
  DSI_TCCR2_LPRD_TOCNT9_Pos* = (9)
  DSI_TCCR2_LPRD_TOCNT9_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT9_Pos) ## !< 0x00000200
  DSI_TCCR2_LPRD_TOCNT9* = DSI_TCCR2_LPRD_TOCNT9_Msk
  DSI_TCCR2_LPRD_TOCNT10_Pos* = (10)
  DSI_TCCR2_LPRD_TOCNT10_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT10_Pos) ## !< 0x00000400
  DSI_TCCR2_LPRD_TOCNT10* = DSI_TCCR2_LPRD_TOCNT10_Msk
  DSI_TCCR2_LPRD_TOCNT11_Pos* = (11)
  DSI_TCCR2_LPRD_TOCNT11_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT11_Pos) ## !< 0x00000800
  DSI_TCCR2_LPRD_TOCNT11* = DSI_TCCR2_LPRD_TOCNT11_Msk
  DSI_TCCR2_LPRD_TOCNT12_Pos* = (12)
  DSI_TCCR2_LPRD_TOCNT12_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT12_Pos) ## !< 0x00001000
  DSI_TCCR2_LPRD_TOCNT12* = DSI_TCCR2_LPRD_TOCNT12_Msk
  DSI_TCCR2_LPRD_TOCNT13_Pos* = (13)
  DSI_TCCR2_LPRD_TOCNT13_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT13_Pos) ## !< 0x00002000
  DSI_TCCR2_LPRD_TOCNT13* = DSI_TCCR2_LPRD_TOCNT13_Msk
  DSI_TCCR2_LPRD_TOCNT14_Pos* = (14)
  DSI_TCCR2_LPRD_TOCNT14_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT14_Pos) ## !< 0x00004000
  DSI_TCCR2_LPRD_TOCNT14* = DSI_TCCR2_LPRD_TOCNT14_Msk
  DSI_TCCR2_LPRD_TOCNT15_Pos* = (15)
  DSI_TCCR2_LPRD_TOCNT15_Msk* = (0x00000001 shl DSI_TCCR2_LPRD_TOCNT15_Pos) ## !< 0x00008000
  DSI_TCCR2_LPRD_TOCNT15* = DSI_TCCR2_LPRD_TOCNT15_Msk

## ******************  Bit definition for DSI_TCCR3 register  *************

const
  DSI_TCCR3_HSWR_TOCNT_Pos* = (0)
  DSI_TCCR3_HSWR_TOCNT_Msk* = (0x0000FFFF shl DSI_TCCR3_HSWR_TOCNT_Pos) ## !< 0x0000FFFF
  DSI_TCCR3_HSWR_TOCNT* = DSI_TCCR3_HSWR_TOCNT_Msk
  DSI_TCCR3_HSWR_TOCNT0_Pos* = (0)
  DSI_TCCR3_HSWR_TOCNT0_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT0_Pos) ## !< 0x00000001
  DSI_TCCR3_HSWR_TOCNT0* = DSI_TCCR3_HSWR_TOCNT0_Msk
  DSI_TCCR3_HSWR_TOCNT1_Pos* = (1)
  DSI_TCCR3_HSWR_TOCNT1_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT1_Pos) ## !< 0x00000002
  DSI_TCCR3_HSWR_TOCNT1* = DSI_TCCR3_HSWR_TOCNT1_Msk
  DSI_TCCR3_HSWR_TOCNT2_Pos* = (2)
  DSI_TCCR3_HSWR_TOCNT2_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT2_Pos) ## !< 0x00000004
  DSI_TCCR3_HSWR_TOCNT2* = DSI_TCCR3_HSWR_TOCNT2_Msk
  DSI_TCCR3_HSWR_TOCNT3_Pos* = (3)
  DSI_TCCR3_HSWR_TOCNT3_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT3_Pos) ## !< 0x00000008
  DSI_TCCR3_HSWR_TOCNT3* = DSI_TCCR3_HSWR_TOCNT3_Msk
  DSI_TCCR3_HSWR_TOCNT4_Pos* = (4)
  DSI_TCCR3_HSWR_TOCNT4_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT4_Pos) ## !< 0x00000010
  DSI_TCCR3_HSWR_TOCNT4* = DSI_TCCR3_HSWR_TOCNT4_Msk
  DSI_TCCR3_HSWR_TOCNT5_Pos* = (5)
  DSI_TCCR3_HSWR_TOCNT5_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT5_Pos) ## !< 0x00000020
  DSI_TCCR3_HSWR_TOCNT5* = DSI_TCCR3_HSWR_TOCNT5_Msk
  DSI_TCCR3_HSWR_TOCNT6_Pos* = (6)
  DSI_TCCR3_HSWR_TOCNT6_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT6_Pos) ## !< 0x00000040
  DSI_TCCR3_HSWR_TOCNT6* = DSI_TCCR3_HSWR_TOCNT6_Msk
  DSI_TCCR3_HSWR_TOCNT7_Pos* = (7)
  DSI_TCCR3_HSWR_TOCNT7_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT7_Pos) ## !< 0x00000080
  DSI_TCCR3_HSWR_TOCNT7* = DSI_TCCR3_HSWR_TOCNT7_Msk
  DSI_TCCR3_HSWR_TOCNT8_Pos* = (8)
  DSI_TCCR3_HSWR_TOCNT8_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT8_Pos) ## !< 0x00000100
  DSI_TCCR3_HSWR_TOCNT8* = DSI_TCCR3_HSWR_TOCNT8_Msk
  DSI_TCCR3_HSWR_TOCNT9_Pos* = (9)
  DSI_TCCR3_HSWR_TOCNT9_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT9_Pos) ## !< 0x00000200
  DSI_TCCR3_HSWR_TOCNT9* = DSI_TCCR3_HSWR_TOCNT9_Msk
  DSI_TCCR3_HSWR_TOCNT10_Pos* = (10)
  DSI_TCCR3_HSWR_TOCNT10_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT10_Pos) ## !< 0x00000400
  DSI_TCCR3_HSWR_TOCNT10* = DSI_TCCR3_HSWR_TOCNT10_Msk
  DSI_TCCR3_HSWR_TOCNT11_Pos* = (11)
  DSI_TCCR3_HSWR_TOCNT11_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT11_Pos) ## !< 0x00000800
  DSI_TCCR3_HSWR_TOCNT11* = DSI_TCCR3_HSWR_TOCNT11_Msk
  DSI_TCCR3_HSWR_TOCNT12_Pos* = (12)
  DSI_TCCR3_HSWR_TOCNT12_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT12_Pos) ## !< 0x00001000
  DSI_TCCR3_HSWR_TOCNT12* = DSI_TCCR3_HSWR_TOCNT12_Msk
  DSI_TCCR3_HSWR_TOCNT13_Pos* = (13)
  DSI_TCCR3_HSWR_TOCNT13_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT13_Pos) ## !< 0x00002000
  DSI_TCCR3_HSWR_TOCNT13* = DSI_TCCR3_HSWR_TOCNT13_Msk
  DSI_TCCR3_HSWR_TOCNT14_Pos* = (14)
  DSI_TCCR3_HSWR_TOCNT14_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT14_Pos) ## !< 0x00004000
  DSI_TCCR3_HSWR_TOCNT14* = DSI_TCCR3_HSWR_TOCNT14_Msk
  DSI_TCCR3_HSWR_TOCNT15_Pos* = (15)
  DSI_TCCR3_HSWR_TOCNT15_Msk* = (0x00000001 shl DSI_TCCR3_HSWR_TOCNT15_Pos) ## !< 0x00008000
  DSI_TCCR3_HSWR_TOCNT15* = DSI_TCCR3_HSWR_TOCNT15_Msk
  DSI_TCCR3_PM_Pos* = (24)
  DSI_TCCR3_PM_Msk* = (0x00000001 shl DSI_TCCR3_PM_Pos) ## !< 0x01000000
  DSI_TCCR3_PM* = DSI_TCCR3_PM_Msk

## ******************  Bit definition for DSI_TCCR4 register  *************

const
  DSI_TCCR4_LPWR_TOCNT_Pos* = (0)
  DSI_TCCR4_LPWR_TOCNT_Msk* = (0x0000FFFF shl DSI_TCCR4_LPWR_TOCNT_Pos) ## !< 0x0000FFFF
  DSI_TCCR4_LPWR_TOCNT* = DSI_TCCR4_LPWR_TOCNT_Msk
  DSI_TCCR4_LPWR_TOCNT0_Pos* = (0)
  DSI_TCCR4_LPWR_TOCNT0_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT0_Pos) ## !< 0x00000001
  DSI_TCCR4_LPWR_TOCNT0* = DSI_TCCR4_LPWR_TOCNT0_Msk
  DSI_TCCR4_LPWR_TOCNT1_Pos* = (1)
  DSI_TCCR4_LPWR_TOCNT1_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT1_Pos) ## !< 0x00000002
  DSI_TCCR4_LPWR_TOCNT1* = DSI_TCCR4_LPWR_TOCNT1_Msk
  DSI_TCCR4_LPWR_TOCNT2_Pos* = (2)
  DSI_TCCR4_LPWR_TOCNT2_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT2_Pos) ## !< 0x00000004
  DSI_TCCR4_LPWR_TOCNT2* = DSI_TCCR4_LPWR_TOCNT2_Msk
  DSI_TCCR4_LPWR_TOCNT3_Pos* = (3)
  DSI_TCCR4_LPWR_TOCNT3_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT3_Pos) ## !< 0x00000008
  DSI_TCCR4_LPWR_TOCNT3* = DSI_TCCR4_LPWR_TOCNT3_Msk
  DSI_TCCR4_LPWR_TOCNT4_Pos* = (4)
  DSI_TCCR4_LPWR_TOCNT4_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT4_Pos) ## !< 0x00000010
  DSI_TCCR4_LPWR_TOCNT4* = DSI_TCCR4_LPWR_TOCNT4_Msk
  DSI_TCCR4_LPWR_TOCNT5_Pos* = (5)
  DSI_TCCR4_LPWR_TOCNT5_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT5_Pos) ## !< 0x00000020
  DSI_TCCR4_LPWR_TOCNT5* = DSI_TCCR4_LPWR_TOCNT5_Msk
  DSI_TCCR4_LPWR_TOCNT6_Pos* = (6)
  DSI_TCCR4_LPWR_TOCNT6_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT6_Pos) ## !< 0x00000040
  DSI_TCCR4_LPWR_TOCNT6* = DSI_TCCR4_LPWR_TOCNT6_Msk
  DSI_TCCR4_LPWR_TOCNT7_Pos* = (7)
  DSI_TCCR4_LPWR_TOCNT7_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT7_Pos) ## !< 0x00000080
  DSI_TCCR4_LPWR_TOCNT7* = DSI_TCCR4_LPWR_TOCNT7_Msk
  DSI_TCCR4_LPWR_TOCNT8_Pos* = (8)
  DSI_TCCR4_LPWR_TOCNT8_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT8_Pos) ## !< 0x00000100
  DSI_TCCR4_LPWR_TOCNT8* = DSI_TCCR4_LPWR_TOCNT8_Msk
  DSI_TCCR4_LPWR_TOCNT9_Pos* = (9)
  DSI_TCCR4_LPWR_TOCNT9_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT9_Pos) ## !< 0x00000200
  DSI_TCCR4_LPWR_TOCNT9* = DSI_TCCR4_LPWR_TOCNT9_Msk
  DSI_TCCR4_LPWR_TOCNT10_Pos* = (10)
  DSI_TCCR4_LPWR_TOCNT10_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT10_Pos) ## !< 0x00000400
  DSI_TCCR4_LPWR_TOCNT10* = DSI_TCCR4_LPWR_TOCNT10_Msk
  DSI_TCCR4_LPWR_TOCNT11_Pos* = (11)
  DSI_TCCR4_LPWR_TOCNT11_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT11_Pos) ## !< 0x00000800
  DSI_TCCR4_LPWR_TOCNT11* = DSI_TCCR4_LPWR_TOCNT11_Msk
  DSI_TCCR4_LPWR_TOCNT12_Pos* = (12)
  DSI_TCCR4_LPWR_TOCNT12_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT12_Pos) ## !< 0x00001000
  DSI_TCCR4_LPWR_TOCNT12* = DSI_TCCR4_LPWR_TOCNT12_Msk
  DSI_TCCR4_LPWR_TOCNT13_Pos* = (13)
  DSI_TCCR4_LPWR_TOCNT13_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT13_Pos) ## !< 0x00002000
  DSI_TCCR4_LPWR_TOCNT13* = DSI_TCCR4_LPWR_TOCNT13_Msk
  DSI_TCCR4_LPWR_TOCNT14_Pos* = (14)
  DSI_TCCR4_LPWR_TOCNT14_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT14_Pos) ## !< 0x00004000
  DSI_TCCR4_LPWR_TOCNT14* = DSI_TCCR4_LPWR_TOCNT14_Msk
  DSI_TCCR4_LPWR_TOCNT15_Pos* = (15)
  DSI_TCCR4_LPWR_TOCNT15_Msk* = (0x00000001 shl DSI_TCCR4_LPWR_TOCNT15_Pos) ## !< 0x00008000
  DSI_TCCR4_LPWR_TOCNT15* = DSI_TCCR4_LPWR_TOCNT15_Msk

## ******************  Bit definition for DSI_TCCR5 register  *************

const
  DSI_TCCR5_BTA_TOCNT_Pos* = (0)
  DSI_TCCR5_BTA_TOCNT_Msk* = (0x0000FFFF shl DSI_TCCR5_BTA_TOCNT_Pos) ## !< 0x0000FFFF
  DSI_TCCR5_BTA_TOCNT* = DSI_TCCR5_BTA_TOCNT_Msk
  DSI_TCCR5_BTA_TOCNT0_Pos* = (0)
  DSI_TCCR5_BTA_TOCNT0_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT0_Pos) ## !< 0x00000001
  DSI_TCCR5_BTA_TOCNT0* = DSI_TCCR5_BTA_TOCNT0_Msk
  DSI_TCCR5_BTA_TOCNT1_Pos* = (1)
  DSI_TCCR5_BTA_TOCNT1_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT1_Pos) ## !< 0x00000002
  DSI_TCCR5_BTA_TOCNT1* = DSI_TCCR5_BTA_TOCNT1_Msk
  DSI_TCCR5_BTA_TOCNT2_Pos* = (2)
  DSI_TCCR5_BTA_TOCNT2_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT2_Pos) ## !< 0x00000004
  DSI_TCCR5_BTA_TOCNT2* = DSI_TCCR5_BTA_TOCNT2_Msk
  DSI_TCCR5_BTA_TOCNT3_Pos* = (3)
  DSI_TCCR5_BTA_TOCNT3_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT3_Pos) ## !< 0x00000008
  DSI_TCCR5_BTA_TOCNT3* = DSI_TCCR5_BTA_TOCNT3_Msk
  DSI_TCCR5_BTA_TOCNT4_Pos* = (4)
  DSI_TCCR5_BTA_TOCNT4_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT4_Pos) ## !< 0x00000010
  DSI_TCCR5_BTA_TOCNT4* = DSI_TCCR5_BTA_TOCNT4_Msk
  DSI_TCCR5_BTA_TOCNT5_Pos* = (5)
  DSI_TCCR5_BTA_TOCNT5_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT5_Pos) ## !< 0x00000020
  DSI_TCCR5_BTA_TOCNT5* = DSI_TCCR5_BTA_TOCNT5_Msk
  DSI_TCCR5_BTA_TOCNT6_Pos* = (6)
  DSI_TCCR5_BTA_TOCNT6_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT6_Pos) ## !< 0x00000040
  DSI_TCCR5_BTA_TOCNT6* = DSI_TCCR5_BTA_TOCNT6_Msk
  DSI_TCCR5_BTA_TOCNT7_Pos* = (7)
  DSI_TCCR5_BTA_TOCNT7_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT7_Pos) ## !< 0x00000080
  DSI_TCCR5_BTA_TOCNT7* = DSI_TCCR5_BTA_TOCNT7_Msk
  DSI_TCCR5_BTA_TOCNT8_Pos* = (8)
  DSI_TCCR5_BTA_TOCNT8_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT8_Pos) ## !< 0x00000100
  DSI_TCCR5_BTA_TOCNT8* = DSI_TCCR5_BTA_TOCNT8_Msk
  DSI_TCCR5_BTA_TOCNT9_Pos* = (9)
  DSI_TCCR5_BTA_TOCNT9_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT9_Pos) ## !< 0x00000200
  DSI_TCCR5_BTA_TOCNT9* = DSI_TCCR5_BTA_TOCNT9_Msk
  DSI_TCCR5_BTA_TOCNT10_Pos* = (10)
  DSI_TCCR5_BTA_TOCNT10_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT10_Pos) ## !< 0x00000400
  DSI_TCCR5_BTA_TOCNT10* = DSI_TCCR5_BTA_TOCNT10_Msk
  DSI_TCCR5_BTA_TOCNT11_Pos* = (11)
  DSI_TCCR5_BTA_TOCNT11_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT11_Pos) ## !< 0x00000800
  DSI_TCCR5_BTA_TOCNT11* = DSI_TCCR5_BTA_TOCNT11_Msk
  DSI_TCCR5_BTA_TOCNT12_Pos* = (12)
  DSI_TCCR5_BTA_TOCNT12_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT12_Pos) ## !< 0x00001000
  DSI_TCCR5_BTA_TOCNT12* = DSI_TCCR5_BTA_TOCNT12_Msk
  DSI_TCCR5_BTA_TOCNT13_Pos* = (13)
  DSI_TCCR5_BTA_TOCNT13_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT13_Pos) ## !< 0x00002000
  DSI_TCCR5_BTA_TOCNT13* = DSI_TCCR5_BTA_TOCNT13_Msk
  DSI_TCCR5_BTA_TOCNT14_Pos* = (14)
  DSI_TCCR5_BTA_TOCNT14_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT14_Pos) ## !< 0x00004000
  DSI_TCCR5_BTA_TOCNT14* = DSI_TCCR5_BTA_TOCNT14_Msk
  DSI_TCCR5_BTA_TOCNT15_Pos* = (15)
  DSI_TCCR5_BTA_TOCNT15_Msk* = (0x00000001 shl DSI_TCCR5_BTA_TOCNT15_Pos) ## !< 0x00008000
  DSI_TCCR5_BTA_TOCNT15* = DSI_TCCR5_BTA_TOCNT15_Msk

## ******************  Bit definition for DSI_TDCR register  **************

const
  DSI_TDCR_Bit3DM* = 0x00000003
  DSI_TDCR_Bit3DM0* = 0x00000001
  DSI_TDCR_Bit3DM1* = 0x00000002
  DSI_TDCR_Bit3DF* = 0x0000000C
  DSI_TDCR_Bit3DF0* = 0x00000004
  DSI_TDCR_Bit3DF1* = 0x00000008
  DSI_TDCR_SVS_Pos* = (4)
  DSI_TDCR_SVS_Msk* = (0x00000001 shl DSI_TDCR_SVS_Pos) ## !< 0x00000010
  DSI_TDCR_SVS* = DSI_TDCR_SVS_Msk
  DSI_TDCR_RF_Pos* = (5)
  DSI_TDCR_RF_Msk* = (0x00000001 shl DSI_TDCR_RF_Pos) ## !< 0x00000020
  DSI_TDCR_RF* = DSI_TDCR_RF_Msk
  DSI_TDCR_S3DC_Pos* = (16)
  DSI_TDCR_S3DC_Msk* = (0x00000001 shl DSI_TDCR_S3DC_Pos) ## !< 0x00010000
  DSI_TDCR_S3DC* = DSI_TDCR_S3DC_Msk

## ******************  Bit definition for DSI_CLCR register  **************

const
  DSI_CLCR_DPCC_Pos* = (0)
  DSI_CLCR_DPCC_Msk* = (0x00000001 shl DSI_CLCR_DPCC_Pos) ## !< 0x00000001
  DSI_CLCR_DPCC* = DSI_CLCR_DPCC_Msk
  DSI_CLCR_ACR_Pos* = (1)
  DSI_CLCR_ACR_Msk* = (0x00000001 shl DSI_CLCR_ACR_Pos) ## !< 0x00000002
  DSI_CLCR_ACR* = DSI_CLCR_ACR_Msk

## ******************  Bit definition for DSI_CLTCR register  *************

const
  DSI_CLTCR_LP2HS_TIME_Pos* = (0)
  DSI_CLTCR_LP2HS_TIME_Msk* = (0x000003FF shl DSI_CLTCR_LP2HS_TIME_Pos) ## !< 0x000003FF
  DSI_CLTCR_LP2HS_TIME* = DSI_CLTCR_LP2HS_TIME_Msk
  DSI_CLTCR_LP2HS_TIME0_Pos* = (0)
  DSI_CLTCR_LP2HS_TIME0_Msk* = (0x00000001 shl DSI_CLTCR_LP2HS_TIME0_Pos) ## !< 0x00000001
  DSI_CLTCR_LP2HS_TIME0* = DSI_CLTCR_LP2HS_TIME0_Msk
  DSI_CLTCR_LP2HS_TIME1_Pos* = (1)
  DSI_CLTCR_LP2HS_TIME1_Msk* = (0x00000001 shl DSI_CLTCR_LP2HS_TIME1_Pos) ## !< 0x00000002
  DSI_CLTCR_LP2HS_TIME1* = DSI_CLTCR_LP2HS_TIME1_Msk
  DSI_CLTCR_LP2HS_TIME2_Pos* = (2)
  DSI_CLTCR_LP2HS_TIME2_Msk* = (0x00000001 shl DSI_CLTCR_LP2HS_TIME2_Pos) ## !< 0x00000004
  DSI_CLTCR_LP2HS_TIME2* = DSI_CLTCR_LP2HS_TIME2_Msk
  DSI_CLTCR_LP2HS_TIME3_Pos* = (3)
  DSI_CLTCR_LP2HS_TIME3_Msk* = (0x00000001 shl DSI_CLTCR_LP2HS_TIME3_Pos) ## !< 0x00000008
  DSI_CLTCR_LP2HS_TIME3* = DSI_CLTCR_LP2HS_TIME3_Msk
  DSI_CLTCR_LP2HS_TIME4_Pos* = (4)
  DSI_CLTCR_LP2HS_TIME4_Msk* = (0x00000001 shl DSI_CLTCR_LP2HS_TIME4_Pos) ## !< 0x00000010
  DSI_CLTCR_LP2HS_TIME4* = DSI_CLTCR_LP2HS_TIME4_Msk
  DSI_CLTCR_LP2HS_TIME5_Pos* = (5)
  DSI_CLTCR_LP2HS_TIME5_Msk* = (0x00000001 shl DSI_CLTCR_LP2HS_TIME5_Pos) ## !< 0x00000020
  DSI_CLTCR_LP2HS_TIME5* = DSI_CLTCR_LP2HS_TIME5_Msk
  DSI_CLTCR_LP2HS_TIME6_Pos* = (6)
  DSI_CLTCR_LP2HS_TIME6_Msk* = (0x00000001 shl DSI_CLTCR_LP2HS_TIME6_Pos) ## !< 0x00000040
  DSI_CLTCR_LP2HS_TIME6* = DSI_CLTCR_LP2HS_TIME6_Msk
  DSI_CLTCR_LP2HS_TIME7_Pos* = (7)
  DSI_CLTCR_LP2HS_TIME7_Msk* = (0x00000001 shl DSI_CLTCR_LP2HS_TIME7_Pos) ## !< 0x00000080
  DSI_CLTCR_LP2HS_TIME7* = DSI_CLTCR_LP2HS_TIME7_Msk
  DSI_CLTCR_LP2HS_TIME8_Pos* = (8)
  DSI_CLTCR_LP2HS_TIME8_Msk* = (0x00000001 shl DSI_CLTCR_LP2HS_TIME8_Pos) ## !< 0x00000100
  DSI_CLTCR_LP2HS_TIME8* = DSI_CLTCR_LP2HS_TIME8_Msk
  DSI_CLTCR_LP2HS_TIME9_Pos* = (9)
  DSI_CLTCR_LP2HS_TIME9_Msk* = (0x00000001 shl DSI_CLTCR_LP2HS_TIME9_Pos) ## !< 0x00000200
  DSI_CLTCR_LP2HS_TIME9* = DSI_CLTCR_LP2HS_TIME9_Msk
  DSI_CLTCR_HS2LP_TIME_Pos* = (16)
  DSI_CLTCR_HS2LP_TIME_Msk* = (0x000003FF shl DSI_CLTCR_HS2LP_TIME_Pos) ## !< 0x03FF0000
  DSI_CLTCR_HS2LP_TIME* = DSI_CLTCR_HS2LP_TIME_Msk
  DSI_CLTCR_HS2LP_TIME0_Pos* = (16)
  DSI_CLTCR_HS2LP_TIME0_Msk* = (0x00000001 shl DSI_CLTCR_HS2LP_TIME0_Pos) ## !< 0x00010000
  DSI_CLTCR_HS2LP_TIME0* = DSI_CLTCR_HS2LP_TIME0_Msk
  DSI_CLTCR_HS2LP_TIME1_Pos* = (17)
  DSI_CLTCR_HS2LP_TIME1_Msk* = (0x00000001 shl DSI_CLTCR_HS2LP_TIME1_Pos) ## !< 0x00020000
  DSI_CLTCR_HS2LP_TIME1* = DSI_CLTCR_HS2LP_TIME1_Msk
  DSI_CLTCR_HS2LP_TIME2_Pos* = (18)
  DSI_CLTCR_HS2LP_TIME2_Msk* = (0x00000001 shl DSI_CLTCR_HS2LP_TIME2_Pos) ## !< 0x00040000
  DSI_CLTCR_HS2LP_TIME2* = DSI_CLTCR_HS2LP_TIME2_Msk
  DSI_CLTCR_HS2LP_TIME3_Pos* = (19)
  DSI_CLTCR_HS2LP_TIME3_Msk* = (0x00000001 shl DSI_CLTCR_HS2LP_TIME3_Pos) ## !< 0x00080000
  DSI_CLTCR_HS2LP_TIME3* = DSI_CLTCR_HS2LP_TIME3_Msk
  DSI_CLTCR_HS2LP_TIME4_Pos* = (20)
  DSI_CLTCR_HS2LP_TIME4_Msk* = (0x00000001 shl DSI_CLTCR_HS2LP_TIME4_Pos) ## !< 0x00100000
  DSI_CLTCR_HS2LP_TIME4* = DSI_CLTCR_HS2LP_TIME4_Msk
  DSI_CLTCR_HS2LP_TIME5_Pos* = (21)
  DSI_CLTCR_HS2LP_TIME5_Msk* = (0x00000001 shl DSI_CLTCR_HS2LP_TIME5_Pos) ## !< 0x00200000
  DSI_CLTCR_HS2LP_TIME5* = DSI_CLTCR_HS2LP_TIME5_Msk
  DSI_CLTCR_HS2LP_TIME6_Pos* = (22)
  DSI_CLTCR_HS2LP_TIME6_Msk* = (0x00000001 shl DSI_CLTCR_HS2LP_TIME6_Pos) ## !< 0x00400000
  DSI_CLTCR_HS2LP_TIME6* = DSI_CLTCR_HS2LP_TIME6_Msk
  DSI_CLTCR_HS2LP_TIME7_Pos* = (23)
  DSI_CLTCR_HS2LP_TIME7_Msk* = (0x00000001 shl DSI_CLTCR_HS2LP_TIME7_Pos) ## !< 0x00800000
  DSI_CLTCR_HS2LP_TIME7* = DSI_CLTCR_HS2LP_TIME7_Msk
  DSI_CLTCR_HS2LP_TIME8_Pos* = (24)
  DSI_CLTCR_HS2LP_TIME8_Msk* = (0x00000001 shl DSI_CLTCR_HS2LP_TIME8_Pos) ## !< 0x01000000
  DSI_CLTCR_HS2LP_TIME8* = DSI_CLTCR_HS2LP_TIME8_Msk
  DSI_CLTCR_HS2LP_TIME9_Pos* = (25)
  DSI_CLTCR_HS2LP_TIME9_Msk* = (0x00000001 shl DSI_CLTCR_HS2LP_TIME9_Pos) ## !< 0x02000000
  DSI_CLTCR_HS2LP_TIME9* = DSI_CLTCR_HS2LP_TIME9_Msk

## ******************  Bit definition for DSI_DLTCR register  *************

const
  DSI_DLTCR_MRD_TIME_Pos* = (0)
  DSI_DLTCR_MRD_TIME_Msk* = (0x00007FFF shl DSI_DLTCR_MRD_TIME_Pos) ## !< 0x00007FFF
  DSI_DLTCR_MRD_TIME* = DSI_DLTCR_MRD_TIME_Msk
  DSI_DLTCR_MRD_TIME0_Pos* = (0)
  DSI_DLTCR_MRD_TIME0_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME0_Pos) ## !< 0x00000001
  DSI_DLTCR_MRD_TIME0* = DSI_DLTCR_MRD_TIME0_Msk
  DSI_DLTCR_MRD_TIME1_Pos* = (1)
  DSI_DLTCR_MRD_TIME1_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME1_Pos) ## !< 0x00000002
  DSI_DLTCR_MRD_TIME1* = DSI_DLTCR_MRD_TIME1_Msk
  DSI_DLTCR_MRD_TIME2_Pos* = (2)
  DSI_DLTCR_MRD_TIME2_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME2_Pos) ## !< 0x00000004
  DSI_DLTCR_MRD_TIME2* = DSI_DLTCR_MRD_TIME2_Msk
  DSI_DLTCR_MRD_TIME3_Pos* = (3)
  DSI_DLTCR_MRD_TIME3_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME3_Pos) ## !< 0x00000008
  DSI_DLTCR_MRD_TIME3* = DSI_DLTCR_MRD_TIME3_Msk
  DSI_DLTCR_MRD_TIME4_Pos* = (4)
  DSI_DLTCR_MRD_TIME4_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME4_Pos) ## !< 0x00000010
  DSI_DLTCR_MRD_TIME4* = DSI_DLTCR_MRD_TIME4_Msk
  DSI_DLTCR_MRD_TIME5_Pos* = (5)
  DSI_DLTCR_MRD_TIME5_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME5_Pos) ## !< 0x00000020
  DSI_DLTCR_MRD_TIME5* = DSI_DLTCR_MRD_TIME5_Msk
  DSI_DLTCR_MRD_TIME6_Pos* = (6)
  DSI_DLTCR_MRD_TIME6_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME6_Pos) ## !< 0x00000040
  DSI_DLTCR_MRD_TIME6* = DSI_DLTCR_MRD_TIME6_Msk
  DSI_DLTCR_MRD_TIME7_Pos* = (7)
  DSI_DLTCR_MRD_TIME7_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME7_Pos) ## !< 0x00000080
  DSI_DLTCR_MRD_TIME7* = DSI_DLTCR_MRD_TIME7_Msk
  DSI_DLTCR_MRD_TIME8_Pos* = (8)
  DSI_DLTCR_MRD_TIME8_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME8_Pos) ## !< 0x00000100
  DSI_DLTCR_MRD_TIME8* = DSI_DLTCR_MRD_TIME8_Msk
  DSI_DLTCR_MRD_TIME9_Pos* = (9)
  DSI_DLTCR_MRD_TIME9_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME9_Pos) ## !< 0x00000200
  DSI_DLTCR_MRD_TIME9* = DSI_DLTCR_MRD_TIME9_Msk
  DSI_DLTCR_MRD_TIME10_Pos* = (10)
  DSI_DLTCR_MRD_TIME10_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME10_Pos) ## !< 0x00000400
  DSI_DLTCR_MRD_TIME10* = DSI_DLTCR_MRD_TIME10_Msk
  DSI_DLTCR_MRD_TIME11_Pos* = (11)
  DSI_DLTCR_MRD_TIME11_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME11_Pos) ## !< 0x00000800
  DSI_DLTCR_MRD_TIME11* = DSI_DLTCR_MRD_TIME11_Msk
  DSI_DLTCR_MRD_TIME12_Pos* = (12)
  DSI_DLTCR_MRD_TIME12_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME12_Pos) ## !< 0x00001000
  DSI_DLTCR_MRD_TIME12* = DSI_DLTCR_MRD_TIME12_Msk
  DSI_DLTCR_MRD_TIME13_Pos* = (13)
  DSI_DLTCR_MRD_TIME13_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME13_Pos) ## !< 0x00002000
  DSI_DLTCR_MRD_TIME13* = DSI_DLTCR_MRD_TIME13_Msk
  DSI_DLTCR_MRD_TIME14_Pos* = (14)
  DSI_DLTCR_MRD_TIME14_Msk* = (0x00000001 shl DSI_DLTCR_MRD_TIME14_Pos) ## !< 0x00004000
  DSI_DLTCR_MRD_TIME14* = DSI_DLTCR_MRD_TIME14_Msk
  DSI_DLTCR_LP2HS_TIME_Pos* = (16)
  DSI_DLTCR_LP2HS_TIME_Msk* = (0x000000FF shl DSI_DLTCR_LP2HS_TIME_Pos) ## !< 0x00FF0000
  DSI_DLTCR_LP2HS_TIME* = DSI_DLTCR_LP2HS_TIME_Msk
  DSI_DLTCR_LP2HS_TIME0_Pos* = (16)
  DSI_DLTCR_LP2HS_TIME0_Msk* = (0x00000001 shl DSI_DLTCR_LP2HS_TIME0_Pos) ## !< 0x00010000
  DSI_DLTCR_LP2HS_TIME0* = DSI_DLTCR_LP2HS_TIME0_Msk
  DSI_DLTCR_LP2HS_TIME1_Pos* = (17)
  DSI_DLTCR_LP2HS_TIME1_Msk* = (0x00000001 shl DSI_DLTCR_LP2HS_TIME1_Pos) ## !< 0x00020000
  DSI_DLTCR_LP2HS_TIME1* = DSI_DLTCR_LP2HS_TIME1_Msk
  DSI_DLTCR_LP2HS_TIME2_Pos* = (18)
  DSI_DLTCR_LP2HS_TIME2_Msk* = (0x00000001 shl DSI_DLTCR_LP2HS_TIME2_Pos) ## !< 0x00040000
  DSI_DLTCR_LP2HS_TIME2* = DSI_DLTCR_LP2HS_TIME2_Msk
  DSI_DLTCR_LP2HS_TIME3_Pos* = (19)
  DSI_DLTCR_LP2HS_TIME3_Msk* = (0x00000001 shl DSI_DLTCR_LP2HS_TIME3_Pos) ## !< 0x00080000
  DSI_DLTCR_LP2HS_TIME3* = DSI_DLTCR_LP2HS_TIME3_Msk
  DSI_DLTCR_LP2HS_TIME4_Pos* = (20)
  DSI_DLTCR_LP2HS_TIME4_Msk* = (0x00000001 shl DSI_DLTCR_LP2HS_TIME4_Pos) ## !< 0x00100000
  DSI_DLTCR_LP2HS_TIME4* = DSI_DLTCR_LP2HS_TIME4_Msk
  DSI_DLTCR_LP2HS_TIME5_Pos* = (21)
  DSI_DLTCR_LP2HS_TIME5_Msk* = (0x00000001 shl DSI_DLTCR_LP2HS_TIME5_Pos) ## !< 0x00200000
  DSI_DLTCR_LP2HS_TIME5* = DSI_DLTCR_LP2HS_TIME5_Msk
  DSI_DLTCR_LP2HS_TIME6_Pos* = (22)
  DSI_DLTCR_LP2HS_TIME6_Msk* = (0x00000001 shl DSI_DLTCR_LP2HS_TIME6_Pos) ## !< 0x00400000
  DSI_DLTCR_LP2HS_TIME6* = DSI_DLTCR_LP2HS_TIME6_Msk
  DSI_DLTCR_LP2HS_TIME7_Pos* = (23)
  DSI_DLTCR_LP2HS_TIME7_Msk* = (0x00000001 shl DSI_DLTCR_LP2HS_TIME7_Pos) ## !< 0x00800000
  DSI_DLTCR_LP2HS_TIME7* = DSI_DLTCR_LP2HS_TIME7_Msk
  DSI_DLTCR_HS2LP_TIME_Pos* = (24)
  DSI_DLTCR_HS2LP_TIME_Msk* = (0x000000FF shl DSI_DLTCR_HS2LP_TIME_Pos) ## !< 0xFF000000
  DSI_DLTCR_HS2LP_TIME* = DSI_DLTCR_HS2LP_TIME_Msk
  DSI_DLTCR_HS2LP_TIME0_Pos* = (24)
  DSI_DLTCR_HS2LP_TIME0_Msk* = (0x00000001 shl DSI_DLTCR_HS2LP_TIME0_Pos) ## !< 0x01000000
  DSI_DLTCR_HS2LP_TIME0* = DSI_DLTCR_HS2LP_TIME0_Msk
  DSI_DLTCR_HS2LP_TIME1_Pos* = (25)
  DSI_DLTCR_HS2LP_TIME1_Msk* = (0x00000001 shl DSI_DLTCR_HS2LP_TIME1_Pos) ## !< 0x02000000
  DSI_DLTCR_HS2LP_TIME1* = DSI_DLTCR_HS2LP_TIME1_Msk
  DSI_DLTCR_HS2LP_TIME2_Pos* = (26)
  DSI_DLTCR_HS2LP_TIME2_Msk* = (0x00000001 shl DSI_DLTCR_HS2LP_TIME2_Pos) ## !< 0x04000000
  DSI_DLTCR_HS2LP_TIME2* = DSI_DLTCR_HS2LP_TIME2_Msk
  DSI_DLTCR_HS2LP_TIME3_Pos* = (27)
  DSI_DLTCR_HS2LP_TIME3_Msk* = (0x00000001 shl DSI_DLTCR_HS2LP_TIME3_Pos) ## !< 0x08000000
  DSI_DLTCR_HS2LP_TIME3* = DSI_DLTCR_HS2LP_TIME3_Msk
  DSI_DLTCR_HS2LP_TIME4_Pos* = (28)
  DSI_DLTCR_HS2LP_TIME4_Msk* = (0x00000001 shl DSI_DLTCR_HS2LP_TIME4_Pos) ## !< 0x10000000
  DSI_DLTCR_HS2LP_TIME4* = DSI_DLTCR_HS2LP_TIME4_Msk
  DSI_DLTCR_HS2LP_TIME5_Pos* = (29)
  DSI_DLTCR_HS2LP_TIME5_Msk* = (0x00000001 shl DSI_DLTCR_HS2LP_TIME5_Pos) ## !< 0x20000000
  DSI_DLTCR_HS2LP_TIME5* = DSI_DLTCR_HS2LP_TIME5_Msk
  DSI_DLTCR_HS2LP_TIME6_Pos* = (30)
  DSI_DLTCR_HS2LP_TIME6_Msk* = (0x00000001 shl DSI_DLTCR_HS2LP_TIME6_Pos) ## !< 0x40000000
  DSI_DLTCR_HS2LP_TIME6* = DSI_DLTCR_HS2LP_TIME6_Msk
  DSI_DLTCR_HS2LP_TIME7_Pos* = (31)
  DSI_DLTCR_HS2LP_TIME7_Msk* = (0x00000001 shl DSI_DLTCR_HS2LP_TIME7_Pos) ## !< 0x80000000
  DSI_DLTCR_HS2LP_TIME7* = DSI_DLTCR_HS2LP_TIME7_Msk

## ******************  Bit definition for DSI_PCTLR register  *************

const
  DSI_PCTLR_DEN_Pos* = (1)
  DSI_PCTLR_DEN_Msk* = (0x00000001 shl DSI_PCTLR_DEN_Pos) ## !< 0x00000002
  DSI_PCTLR_DEN* = DSI_PCTLR_DEN_Msk
  DSI_PCTLR_CKE_Pos* = (2)
  DSI_PCTLR_CKE_Msk* = (0x00000001 shl DSI_PCTLR_CKE_Pos) ## !< 0x00000004
  DSI_PCTLR_CKE* = DSI_PCTLR_CKE_Msk

## ******************  Bit definition for DSI_PCONFR register  ************

const
  DSI_PCONFR_NL_Pos* = (0)
  DSI_PCONFR_NL_Msk* = (0x00000003 shl DSI_PCONFR_NL_Pos) ## !< 0x00000003
  DSI_PCONFR_NL* = DSI_PCONFR_NL_Msk
  DSI_PCONFR_NL0_Pos* = (0)
  DSI_PCONFR_NL0_Msk* = (0x00000001 shl DSI_PCONFR_NL0_Pos) ## !< 0x00000001
  DSI_PCONFR_NL0* = DSI_PCONFR_NL0_Msk
  DSI_PCONFR_NL1_Pos* = (1)
  DSI_PCONFR_NL1_Msk* = (0x00000001 shl DSI_PCONFR_NL1_Pos) ## !< 0x00000002
  DSI_PCONFR_NL1* = DSI_PCONFR_NL1_Msk
  DSI_PCONFR_SW_TIME_Pos* = (8)
  DSI_PCONFR_SW_TIME_Msk* = (0x000000FF shl DSI_PCONFR_SW_TIME_Pos) ## !< 0x0000FF00
  DSI_PCONFR_SW_TIME* = DSI_PCONFR_SW_TIME_Msk
  DSI_PCONFR_SW_TIME0_Pos* = (8)
  DSI_PCONFR_SW_TIME0_Msk* = (0x00000001 shl DSI_PCONFR_SW_TIME0_Pos) ## !< 0x00000100
  DSI_PCONFR_SW_TIME0* = DSI_PCONFR_SW_TIME0_Msk
  DSI_PCONFR_SW_TIME1_Pos* = (9)
  DSI_PCONFR_SW_TIME1_Msk* = (0x00000001 shl DSI_PCONFR_SW_TIME1_Pos) ## !< 0x00000200
  DSI_PCONFR_SW_TIME1* = DSI_PCONFR_SW_TIME1_Msk
  DSI_PCONFR_SW_TIME2_Pos* = (10)
  DSI_PCONFR_SW_TIME2_Msk* = (0x00000001 shl DSI_PCONFR_SW_TIME2_Pos) ## !< 0x00000400
  DSI_PCONFR_SW_TIME2* = DSI_PCONFR_SW_TIME2_Msk
  DSI_PCONFR_SW_TIME3_Pos* = (11)
  DSI_PCONFR_SW_TIME3_Msk* = (0x00000001 shl DSI_PCONFR_SW_TIME3_Pos) ## !< 0x00000800
  DSI_PCONFR_SW_TIME3* = DSI_PCONFR_SW_TIME3_Msk
  DSI_PCONFR_SW_TIME4_Pos* = (12)
  DSI_PCONFR_SW_TIME4_Msk* = (0x00000001 shl DSI_PCONFR_SW_TIME4_Pos) ## !< 0x00001000
  DSI_PCONFR_SW_TIME4* = DSI_PCONFR_SW_TIME4_Msk
  DSI_PCONFR_SW_TIME5_Pos* = (13)
  DSI_PCONFR_SW_TIME5_Msk* = (0x00000001 shl DSI_PCONFR_SW_TIME5_Pos) ## !< 0x00002000
  DSI_PCONFR_SW_TIME5* = DSI_PCONFR_SW_TIME5_Msk
  DSI_PCONFR_SW_TIME6_Pos* = (14)
  DSI_PCONFR_SW_TIME6_Msk* = (0x00000001 shl DSI_PCONFR_SW_TIME6_Pos) ## !< 0x00004000
  DSI_PCONFR_SW_TIME6* = DSI_PCONFR_SW_TIME6_Msk
  DSI_PCONFR_SW_TIME7_Pos* = (15)
  DSI_PCONFR_SW_TIME7_Msk* = (0x00000001 shl DSI_PCONFR_SW_TIME7_Pos) ## !< 0x00008000
  DSI_PCONFR_SW_TIME7* = DSI_PCONFR_SW_TIME7_Msk

## ******************  Bit definition for DSI_PUCR register  **************

const
  DSI_PUCR_URCL_Pos* = (0)
  DSI_PUCR_URCL_Msk* = (0x00000001 shl DSI_PUCR_URCL_Pos) ## !< 0x00000001
  DSI_PUCR_URCL* = DSI_PUCR_URCL_Msk
  DSI_PUCR_UECL_Pos* = (1)
  DSI_PUCR_UECL_Msk* = (0x00000001 shl DSI_PUCR_UECL_Pos) ## !< 0x00000002
  DSI_PUCR_UECL* = DSI_PUCR_UECL_Msk
  DSI_PUCR_URDL_Pos* = (2)
  DSI_PUCR_URDL_Msk* = (0x00000001 shl DSI_PUCR_URDL_Pos) ## !< 0x00000004
  DSI_PUCR_URDL* = DSI_PUCR_URDL_Msk
  DSI_PUCR_UEDL_Pos* = (3)
  DSI_PUCR_UEDL_Msk* = (0x00000001 shl DSI_PUCR_UEDL_Pos) ## !< 0x00000008
  DSI_PUCR_UEDL* = DSI_PUCR_UEDL_Msk

## ******************  Bit definition for DSI_PTTCR register  *************

const
  DSI_PTTCR_TX_TRIG_Pos* = (0)
  DSI_PTTCR_TX_TRIG_Msk* = (0x0000000F shl DSI_PTTCR_TX_TRIG_Pos) ## !< 0x0000000F
  DSI_PTTCR_TX_TRIG* = DSI_PTTCR_TX_TRIG_Msk
  DSI_PTTCR_TX_TRIG0_Pos* = (0)
  DSI_PTTCR_TX_TRIG0_Msk* = (0x00000001 shl DSI_PTTCR_TX_TRIG0_Pos) ## !< 0x00000001
  DSI_PTTCR_TX_TRIG0* = DSI_PTTCR_TX_TRIG0_Msk
  DSI_PTTCR_TX_TRIG1_Pos* = (1)
  DSI_PTTCR_TX_TRIG1_Msk* = (0x00000001 shl DSI_PTTCR_TX_TRIG1_Pos) ## !< 0x00000002
  DSI_PTTCR_TX_TRIG1* = DSI_PTTCR_TX_TRIG1_Msk
  DSI_PTTCR_TX_TRIG2_Pos* = (2)
  DSI_PTTCR_TX_TRIG2_Msk* = (0x00000001 shl DSI_PTTCR_TX_TRIG2_Pos) ## !< 0x00000004
  DSI_PTTCR_TX_TRIG2* = DSI_PTTCR_TX_TRIG2_Msk
  DSI_PTTCR_TX_TRIG3_Pos* = (3)
  DSI_PTTCR_TX_TRIG3_Msk* = (0x00000001 shl DSI_PTTCR_TX_TRIG3_Pos) ## !< 0x00000008
  DSI_PTTCR_TX_TRIG3* = DSI_PTTCR_TX_TRIG3_Msk

## ******************  Bit definition for DSI_PSR register  ***************

const
  DSI_PSR_PD_Pos* = (1)
  DSI_PSR_PD_Msk* = (0x00000001 shl DSI_PSR_PD_Pos) ## !< 0x00000002
  DSI_PSR_PD* = DSI_PSR_PD_Msk
  DSI_PSR_PSSC_Pos* = (2)
  DSI_PSR_PSSC_Msk* = (0x00000001 shl DSI_PSR_PSSC_Pos) ## !< 0x00000004
  DSI_PSR_PSSC* = DSI_PSR_PSSC_Msk
  DSI_PSR_UANC_Pos* = (3)
  DSI_PSR_UANC_Msk* = (0x00000001 shl DSI_PSR_UANC_Pos) ## !< 0x00000008
  DSI_PSR_UANC* = DSI_PSR_UANC_Msk
  DSI_PSR_PSS0_Pos* = (4)
  DSI_PSR_PSS0_Msk* = (0x00000001 shl DSI_PSR_PSS0_Pos) ## !< 0x00000010
  DSI_PSR_PSS0* = DSI_PSR_PSS0_Msk
  DSI_PSR_UAN0_Pos* = (5)
  DSI_PSR_UAN0_Msk* = (0x00000001 shl DSI_PSR_UAN0_Pos) ## !< 0x00000020
  DSI_PSR_UAN0* = DSI_PSR_UAN0_Msk
  DSI_PSR_RUE0_Pos* = (6)
  DSI_PSR_RUE0_Msk* = (0x00000001 shl DSI_PSR_RUE0_Pos) ## !< 0x00000040
  DSI_PSR_RUE0* = DSI_PSR_RUE0_Msk
  DSI_PSR_PSS1_Pos* = (7)
  DSI_PSR_PSS1_Msk* = (0x00000001 shl DSI_PSR_PSS1_Pos) ## !< 0x00000080
  DSI_PSR_PSS1* = DSI_PSR_PSS1_Msk
  DSI_PSR_UAN1_Pos* = (8)
  DSI_PSR_UAN1_Msk* = (0x00000001 shl DSI_PSR_UAN1_Pos) ## !< 0x00000100
  DSI_PSR_UAN1* = DSI_PSR_UAN1_Msk

## ******************  Bit definition for DSI_ISR0 register  **************

const
  DSI_ISR0_AE0_Pos* = (0)
  DSI_ISR0_AE0_Msk* = (0x00000001 shl DSI_ISR0_AE0_Pos) ## !< 0x00000001
  DSI_ISR0_AE0* = DSI_ISR0_AE0_Msk
  DSI_ISR0_AE1_Pos* = (1)
  DSI_ISR0_AE1_Msk* = (0x00000001 shl DSI_ISR0_AE1_Pos) ## !< 0x00000002
  DSI_ISR0_AE1* = DSI_ISR0_AE1_Msk
  DSI_ISR0_AE2_Pos* = (2)
  DSI_ISR0_AE2_Msk* = (0x00000001 shl DSI_ISR0_AE2_Pos) ## !< 0x00000004
  DSI_ISR0_AE2* = DSI_ISR0_AE2_Msk
  DSI_ISR0_AE3_Pos* = (3)
  DSI_ISR0_AE3_Msk* = (0x00000001 shl DSI_ISR0_AE3_Pos) ## !< 0x00000008
  DSI_ISR0_AE3* = DSI_ISR0_AE3_Msk
  DSI_ISR0_AE4_Pos* = (4)
  DSI_ISR0_AE4_Msk* = (0x00000001 shl DSI_ISR0_AE4_Pos) ## !< 0x00000010
  DSI_ISR0_AE4* = DSI_ISR0_AE4_Msk
  DSI_ISR0_AE5_Pos* = (5)
  DSI_ISR0_AE5_Msk* = (0x00000001 shl DSI_ISR0_AE5_Pos) ## !< 0x00000020
  DSI_ISR0_AE5* = DSI_ISR0_AE5_Msk
  DSI_ISR0_AE6_Pos* = (6)
  DSI_ISR0_AE6_Msk* = (0x00000001 shl DSI_ISR0_AE6_Pos) ## !< 0x00000040
  DSI_ISR0_AE6* = DSI_ISR0_AE6_Msk
  DSI_ISR0_AE7_Pos* = (7)
  DSI_ISR0_AE7_Msk* = (0x00000001 shl DSI_ISR0_AE7_Pos) ## !< 0x00000080
  DSI_ISR0_AE7* = DSI_ISR0_AE7_Msk
  DSI_ISR0_AE8_Pos* = (8)
  DSI_ISR0_AE8_Msk* = (0x00000001 shl DSI_ISR0_AE8_Pos) ## !< 0x00000100
  DSI_ISR0_AE8* = DSI_ISR0_AE8_Msk
  DSI_ISR0_AE9_Pos* = (9)
  DSI_ISR0_AE9_Msk* = (0x00000001 shl DSI_ISR0_AE9_Pos) ## !< 0x00000200
  DSI_ISR0_AE9* = DSI_ISR0_AE9_Msk
  DSI_ISR0_AE10_Pos* = (10)
  DSI_ISR0_AE10_Msk* = (0x00000001 shl DSI_ISR0_AE10_Pos) ## !< 0x00000400
  DSI_ISR0_AE10* = DSI_ISR0_AE10_Msk
  DSI_ISR0_AE11_Pos* = (11)
  DSI_ISR0_AE11_Msk* = (0x00000001 shl DSI_ISR0_AE11_Pos) ## !< 0x00000800
  DSI_ISR0_AE11* = DSI_ISR0_AE11_Msk
  DSI_ISR0_AE12_Pos* = (12)
  DSI_ISR0_AE12_Msk* = (0x00000001 shl DSI_ISR0_AE12_Pos) ## !< 0x00001000
  DSI_ISR0_AE12* = DSI_ISR0_AE12_Msk
  DSI_ISR0_AE13_Pos* = (13)
  DSI_ISR0_AE13_Msk* = (0x00000001 shl DSI_ISR0_AE13_Pos) ## !< 0x00002000
  DSI_ISR0_AE13* = DSI_ISR0_AE13_Msk
  DSI_ISR0_AE14_Pos* = (14)
  DSI_ISR0_AE14_Msk* = (0x00000001 shl DSI_ISR0_AE14_Pos) ## !< 0x00004000
  DSI_ISR0_AE14* = DSI_ISR0_AE14_Msk
  DSI_ISR0_AE15_Pos* = (15)
  DSI_ISR0_AE15_Msk* = (0x00000001 shl DSI_ISR0_AE15_Pos) ## !< 0x00008000
  DSI_ISR0_AE15* = DSI_ISR0_AE15_Msk
  DSI_ISR0_PE0_Pos* = (16)
  DSI_ISR0_PE0_Msk* = (0x00000001 shl DSI_ISR0_PE0_Pos) ## !< 0x00010000
  DSI_ISR0_PE0* = DSI_ISR0_PE0_Msk
  DSI_ISR0_PE1_Pos* = (17)
  DSI_ISR0_PE1_Msk* = (0x00000001 shl DSI_ISR0_PE1_Pos) ## !< 0x00020000
  DSI_ISR0_PE1* = DSI_ISR0_PE1_Msk
  DSI_ISR0_PE2_Pos* = (18)
  DSI_ISR0_PE2_Msk* = (0x00000001 shl DSI_ISR0_PE2_Pos) ## !< 0x00040000
  DSI_ISR0_PE2* = DSI_ISR0_PE2_Msk
  DSI_ISR0_PE3_Pos* = (19)
  DSI_ISR0_PE3_Msk* = (0x00000001 shl DSI_ISR0_PE3_Pos) ## !< 0x00080000
  DSI_ISR0_PE3* = DSI_ISR0_PE3_Msk
  DSI_ISR0_PE4_Pos* = (20)
  DSI_ISR0_PE4_Msk* = (0x00000001 shl DSI_ISR0_PE4_Pos) ## !< 0x00100000
  DSI_ISR0_PE4* = DSI_ISR0_PE4_Msk

## ******************  Bit definition for DSI_ISR1 register  **************

const
  DSI_ISR1_TOHSTX_Pos* = (0)
  DSI_ISR1_TOHSTX_Msk* = (0x00000001 shl DSI_ISR1_TOHSTX_Pos) ## !< 0x00000001
  DSI_ISR1_TOHSTX* = DSI_ISR1_TOHSTX_Msk
  DSI_ISR1_TOLPRX_Pos* = (1)
  DSI_ISR1_TOLPRX_Msk* = (0x00000001 shl DSI_ISR1_TOLPRX_Pos) ## !< 0x00000002
  DSI_ISR1_TOLPRX* = DSI_ISR1_TOLPRX_Msk
  DSI_ISR1_ECCSE_Pos* = (2)
  DSI_ISR1_ECCSE_Msk* = (0x00000001 shl DSI_ISR1_ECCSE_Pos) ## !< 0x00000004
  DSI_ISR1_ECCSE* = DSI_ISR1_ECCSE_Msk
  DSI_ISR1_ECCME_Pos* = (3)
  DSI_ISR1_ECCME_Msk* = (0x00000001 shl DSI_ISR1_ECCME_Pos) ## !< 0x00000008
  DSI_ISR1_ECCME* = DSI_ISR1_ECCME_Msk
  DSI_ISR1_CRCE_Pos* = (4)
  DSI_ISR1_CRCE_Msk* = (0x00000001 shl DSI_ISR1_CRCE_Pos) ## !< 0x00000010
  DSI_ISR1_CRCE* = DSI_ISR1_CRCE_Msk
  DSI_ISR1_PSE_Pos* = (5)
  DSI_ISR1_PSE_Msk* = (0x00000001 shl DSI_ISR1_PSE_Pos) ## !< 0x00000020
  DSI_ISR1_PSE* = DSI_ISR1_PSE_Msk
  DSI_ISR1_EOTPE_Pos* = (6)
  DSI_ISR1_EOTPE_Msk* = (0x00000001 shl DSI_ISR1_EOTPE_Pos) ## !< 0x00000040
  DSI_ISR1_EOTPE* = DSI_ISR1_EOTPE_Msk
  DSI_ISR1_LPWRE_Pos* = (7)
  DSI_ISR1_LPWRE_Msk* = (0x00000001 shl DSI_ISR1_LPWRE_Pos) ## !< 0x00000080
  DSI_ISR1_LPWRE* = DSI_ISR1_LPWRE_Msk
  DSI_ISR1_GCWRE_Pos* = (8)
  DSI_ISR1_GCWRE_Msk* = (0x00000001 shl DSI_ISR1_GCWRE_Pos) ## !< 0x00000100
  DSI_ISR1_GCWRE* = DSI_ISR1_GCWRE_Msk
  DSI_ISR1_GPWRE_Pos* = (9)
  DSI_ISR1_GPWRE_Msk* = (0x00000001 shl DSI_ISR1_GPWRE_Pos) ## !< 0x00000200
  DSI_ISR1_GPWRE* = DSI_ISR1_GPWRE_Msk
  DSI_ISR1_GPTXE_Pos* = (10)
  DSI_ISR1_GPTXE_Msk* = (0x00000001 shl DSI_ISR1_GPTXE_Pos) ## !< 0x00000400
  DSI_ISR1_GPTXE* = DSI_ISR1_GPTXE_Msk
  DSI_ISR1_GPRDE_Pos* = (11)
  DSI_ISR1_GPRDE_Msk* = (0x00000001 shl DSI_ISR1_GPRDE_Pos) ## !< 0x00000800
  DSI_ISR1_GPRDE* = DSI_ISR1_GPRDE_Msk
  DSI_ISR1_GPRXE_Pos* = (12)
  DSI_ISR1_GPRXE_Msk* = (0x00000001 shl DSI_ISR1_GPRXE_Pos) ## !< 0x00001000
  DSI_ISR1_GPRXE* = DSI_ISR1_GPRXE_Msk

## ******************  Bit definition for DSI_IER0 register  **************

const
  DSI_IER0_AE0IE_Pos* = (0)
  DSI_IER0_AE0IE_Msk* = (0x00000001 shl DSI_IER0_AE0IE_Pos) ## !< 0x00000001
  DSI_IER0_AE0IE* = DSI_IER0_AE0IE_Msk
  DSI_IER0_AE1IE_Pos* = (1)
  DSI_IER0_AE1IE_Msk* = (0x00000001 shl DSI_IER0_AE1IE_Pos) ## !< 0x00000002
  DSI_IER0_AE1IE* = DSI_IER0_AE1IE_Msk
  DSI_IER0_AE2IE_Pos* = (2)
  DSI_IER0_AE2IE_Msk* = (0x00000001 shl DSI_IER0_AE2IE_Pos) ## !< 0x00000004
  DSI_IER0_AE2IE* = DSI_IER0_AE2IE_Msk
  DSI_IER0_AE3IE_Pos* = (3)
  DSI_IER0_AE3IE_Msk* = (0x00000001 shl DSI_IER0_AE3IE_Pos) ## !< 0x00000008
  DSI_IER0_AE3IE* = DSI_IER0_AE3IE_Msk
  DSI_IER0_AE4IE_Pos* = (4)
  DSI_IER0_AE4IE_Msk* = (0x00000001 shl DSI_IER0_AE4IE_Pos) ## !< 0x00000010
  DSI_IER0_AE4IE* = DSI_IER0_AE4IE_Msk
  DSI_IER0_AE5IE_Pos* = (5)
  DSI_IER0_AE5IE_Msk* = (0x00000001 shl DSI_IER0_AE5IE_Pos) ## !< 0x00000020
  DSI_IER0_AE5IE* = DSI_IER0_AE5IE_Msk
  DSI_IER0_AE6IE_Pos* = (6)
  DSI_IER0_AE6IE_Msk* = (0x00000001 shl DSI_IER0_AE6IE_Pos) ## !< 0x00000040
  DSI_IER0_AE6IE* = DSI_IER0_AE6IE_Msk
  DSI_IER0_AE7IE_Pos* = (7)
  DSI_IER0_AE7IE_Msk* = (0x00000001 shl DSI_IER0_AE7IE_Pos) ## !< 0x00000080
  DSI_IER0_AE7IE* = DSI_IER0_AE7IE_Msk
  DSI_IER0_AE8IE_Pos* = (8)
  DSI_IER0_AE8IE_Msk* = (0x00000001 shl DSI_IER0_AE8IE_Pos) ## !< 0x00000100
  DSI_IER0_AE8IE* = DSI_IER0_AE8IE_Msk
  DSI_IER0_AE9IE_Pos* = (9)
  DSI_IER0_AE9IE_Msk* = (0x00000001 shl DSI_IER0_AE9IE_Pos) ## !< 0x00000200
  DSI_IER0_AE9IE* = DSI_IER0_AE9IE_Msk
  DSI_IER0_AE10IE_Pos* = (10)
  DSI_IER0_AE10IE_Msk* = (0x00000001 shl DSI_IER0_AE10IE_Pos) ## !< 0x00000400
  DSI_IER0_AE10IE* = DSI_IER0_AE10IE_Msk
  DSI_IER0_AE11IE_Pos* = (11)
  DSI_IER0_AE11IE_Msk* = (0x00000001 shl DSI_IER0_AE11IE_Pos) ## !< 0x00000800
  DSI_IER0_AE11IE* = DSI_IER0_AE11IE_Msk
  DSI_IER0_AE12IE_Pos* = (12)
  DSI_IER0_AE12IE_Msk* = (0x00000001 shl DSI_IER0_AE12IE_Pos) ## !< 0x00001000
  DSI_IER0_AE12IE* = DSI_IER0_AE12IE_Msk
  DSI_IER0_AE13IE_Pos* = (13)
  DSI_IER0_AE13IE_Msk* = (0x00000001 shl DSI_IER0_AE13IE_Pos) ## !< 0x00002000
  DSI_IER0_AE13IE* = DSI_IER0_AE13IE_Msk
  DSI_IER0_AE14IE_Pos* = (14)
  DSI_IER0_AE14IE_Msk* = (0x00000001 shl DSI_IER0_AE14IE_Pos) ## !< 0x00004000
  DSI_IER0_AE14IE* = DSI_IER0_AE14IE_Msk
  DSI_IER0_AE15IE_Pos* = (15)
  DSI_IER0_AE15IE_Msk* = (0x00000001 shl DSI_IER0_AE15IE_Pos) ## !< 0x00008000
  DSI_IER0_AE15IE* = DSI_IER0_AE15IE_Msk
  DSI_IER0_PE0IE_Pos* = (16)
  DSI_IER0_PE0IE_Msk* = (0x00000001 shl DSI_IER0_PE0IE_Pos) ## !< 0x00010000
  DSI_IER0_PE0IE* = DSI_IER0_PE0IE_Msk
  DSI_IER0_PE1IE_Pos* = (17)
  DSI_IER0_PE1IE_Msk* = (0x00000001 shl DSI_IER0_PE1IE_Pos) ## !< 0x00020000
  DSI_IER0_PE1IE* = DSI_IER0_PE1IE_Msk
  DSI_IER0_PE2IE_Pos* = (18)
  DSI_IER0_PE2IE_Msk* = (0x00000001 shl DSI_IER0_PE2IE_Pos) ## !< 0x00040000
  DSI_IER0_PE2IE* = DSI_IER0_PE2IE_Msk
  DSI_IER0_PE3IE_Pos* = (19)
  DSI_IER0_PE3IE_Msk* = (0x00000001 shl DSI_IER0_PE3IE_Pos) ## !< 0x00080000
  DSI_IER0_PE3IE* = DSI_IER0_PE3IE_Msk
  DSI_IER0_PE4IE_Pos* = (20)
  DSI_IER0_PE4IE_Msk* = (0x00000001 shl DSI_IER0_PE4IE_Pos) ## !< 0x00100000
  DSI_IER0_PE4IE* = DSI_IER0_PE4IE_Msk

## ******************  Bit definition for DSI_IER1 register  **************

const
  DSI_IER1_TOHSTXIE_Pos* = (0)
  DSI_IER1_TOHSTXIE_Msk* = (0x00000001 shl DSI_IER1_TOHSTXIE_Pos) ## !< 0x00000001
  DSI_IER1_TOHSTXIE* = DSI_IER1_TOHSTXIE_Msk
  DSI_IER1_TOLPRXIE_Pos* = (1)
  DSI_IER1_TOLPRXIE_Msk* = (0x00000001 shl DSI_IER1_TOLPRXIE_Pos) ## !< 0x00000002
  DSI_IER1_TOLPRXIE* = DSI_IER1_TOLPRXIE_Msk
  DSI_IER1_ECCSEIE_Pos* = (2)
  DSI_IER1_ECCSEIE_Msk* = (0x00000001 shl DSI_IER1_ECCSEIE_Pos) ## !< 0x00000004
  DSI_IER1_ECCSEIE* = DSI_IER1_ECCSEIE_Msk
  DSI_IER1_ECCMEIE_Pos* = (3)
  DSI_IER1_ECCMEIE_Msk* = (0x00000001 shl DSI_IER1_ECCMEIE_Pos) ## !< 0x00000008
  DSI_IER1_ECCMEIE* = DSI_IER1_ECCMEIE_Msk
  DSI_IER1_CRCEIE_Pos* = (4)
  DSI_IER1_CRCEIE_Msk* = (0x00000001 shl DSI_IER1_CRCEIE_Pos) ## !< 0x00000010
  DSI_IER1_CRCEIE* = DSI_IER1_CRCEIE_Msk
  DSI_IER1_PSEIE_Pos* = (5)
  DSI_IER1_PSEIE_Msk* = (0x00000001 shl DSI_IER1_PSEIE_Pos) ## !< 0x00000020
  DSI_IER1_PSEIE* = DSI_IER1_PSEIE_Msk
  DSI_IER1_EOTPEIE_Pos* = (6)
  DSI_IER1_EOTPEIE_Msk* = (0x00000001 shl DSI_IER1_EOTPEIE_Pos) ## !< 0x00000040
  DSI_IER1_EOTPEIE* = DSI_IER1_EOTPEIE_Msk
  DSI_IER1_LPWREIE_Pos* = (7)
  DSI_IER1_LPWREIE_Msk* = (0x00000001 shl DSI_IER1_LPWREIE_Pos) ## !< 0x00000080
  DSI_IER1_LPWREIE* = DSI_IER1_LPWREIE_Msk
  DSI_IER1_GCWREIE_Pos* = (8)
  DSI_IER1_GCWREIE_Msk* = (0x00000001 shl DSI_IER1_GCWREIE_Pos) ## !< 0x00000100
  DSI_IER1_GCWREIE* = DSI_IER1_GCWREIE_Msk
  DSI_IER1_GPWREIE_Pos* = (9)
  DSI_IER1_GPWREIE_Msk* = (0x00000001 shl DSI_IER1_GPWREIE_Pos) ## !< 0x00000200
  DSI_IER1_GPWREIE* = DSI_IER1_GPWREIE_Msk
  DSI_IER1_GPTXEIE_Pos* = (10)
  DSI_IER1_GPTXEIE_Msk* = (0x00000001 shl DSI_IER1_GPTXEIE_Pos) ## !< 0x00000400
  DSI_IER1_GPTXEIE* = DSI_IER1_GPTXEIE_Msk
  DSI_IER1_GPRDEIE_Pos* = (11)
  DSI_IER1_GPRDEIE_Msk* = (0x00000001 shl DSI_IER1_GPRDEIE_Pos) ## !< 0x00000800
  DSI_IER1_GPRDEIE* = DSI_IER1_GPRDEIE_Msk
  DSI_IER1_GPRXEIE_Pos* = (12)
  DSI_IER1_GPRXEIE_Msk* = (0x00000001 shl DSI_IER1_GPRXEIE_Pos) ## !< 0x00001000
  DSI_IER1_GPRXEIE* = DSI_IER1_GPRXEIE_Msk

## ******************  Bit definition for DSI_FIR0 register  **************

const
  DSI_FIR0_FAE0_Pos* = (0)
  DSI_FIR0_FAE0_Msk* = (0x00000001 shl DSI_FIR0_FAE0_Pos) ## !< 0x00000001
  DSI_FIR0_FAE0* = DSI_FIR0_FAE0_Msk
  DSI_FIR0_FAE1_Pos* = (1)
  DSI_FIR0_FAE1_Msk* = (0x00000001 shl DSI_FIR0_FAE1_Pos) ## !< 0x00000002
  DSI_FIR0_FAE1* = DSI_FIR0_FAE1_Msk
  DSI_FIR0_FAE2_Pos* = (2)
  DSI_FIR0_FAE2_Msk* = (0x00000001 shl DSI_FIR0_FAE2_Pos) ## !< 0x00000004
  DSI_FIR0_FAE2* = DSI_FIR0_FAE2_Msk
  DSI_FIR0_FAE3_Pos* = (3)
  DSI_FIR0_FAE3_Msk* = (0x00000001 shl DSI_FIR0_FAE3_Pos) ## !< 0x00000008
  DSI_FIR0_FAE3* = DSI_FIR0_FAE3_Msk
  DSI_FIR0_FAE4_Pos* = (4)
  DSI_FIR0_FAE4_Msk* = (0x00000001 shl DSI_FIR0_FAE4_Pos) ## !< 0x00000010
  DSI_FIR0_FAE4* = DSI_FIR0_FAE4_Msk
  DSI_FIR0_FAE5_Pos* = (5)
  DSI_FIR0_FAE5_Msk* = (0x00000001 shl DSI_FIR0_FAE5_Pos) ## !< 0x00000020
  DSI_FIR0_FAE5* = DSI_FIR0_FAE5_Msk
  DSI_FIR0_FAE6_Pos* = (6)
  DSI_FIR0_FAE6_Msk* = (0x00000001 shl DSI_FIR0_FAE6_Pos) ## !< 0x00000040
  DSI_FIR0_FAE6* = DSI_FIR0_FAE6_Msk
  DSI_FIR0_FAE7_Pos* = (7)
  DSI_FIR0_FAE7_Msk* = (0x00000001 shl DSI_FIR0_FAE7_Pos) ## !< 0x00000080
  DSI_FIR0_FAE7* = DSI_FIR0_FAE7_Msk
  DSI_FIR0_FAE8_Pos* = (8)
  DSI_FIR0_FAE8_Msk* = (0x00000001 shl DSI_FIR0_FAE8_Pos) ## !< 0x00000100
  DSI_FIR0_FAE8* = DSI_FIR0_FAE8_Msk
  DSI_FIR0_FAE9_Pos* = (9)
  DSI_FIR0_FAE9_Msk* = (0x00000001 shl DSI_FIR0_FAE9_Pos) ## !< 0x00000200
  DSI_FIR0_FAE9* = DSI_FIR0_FAE9_Msk
  DSI_FIR0_FAE10_Pos* = (10)
  DSI_FIR0_FAE10_Msk* = (0x00000001 shl DSI_FIR0_FAE10_Pos) ## !< 0x00000400
  DSI_FIR0_FAE10* = DSI_FIR0_FAE10_Msk
  DSI_FIR0_FAE11_Pos* = (11)
  DSI_FIR0_FAE11_Msk* = (0x00000001 shl DSI_FIR0_FAE11_Pos) ## !< 0x00000800
  DSI_FIR0_FAE11* = DSI_FIR0_FAE11_Msk
  DSI_FIR0_FAE12_Pos* = (12)
  DSI_FIR0_FAE12_Msk* = (0x00000001 shl DSI_FIR0_FAE12_Pos) ## !< 0x00001000
  DSI_FIR0_FAE12* = DSI_FIR0_FAE12_Msk
  DSI_FIR0_FAE13_Pos* = (13)
  DSI_FIR0_FAE13_Msk* = (0x00000001 shl DSI_FIR0_FAE13_Pos) ## !< 0x00002000
  DSI_FIR0_FAE13* = DSI_FIR0_FAE13_Msk
  DSI_FIR0_FAE14_Pos* = (14)
  DSI_FIR0_FAE14_Msk* = (0x00000001 shl DSI_FIR0_FAE14_Pos) ## !< 0x00004000
  DSI_FIR0_FAE14* = DSI_FIR0_FAE14_Msk
  DSI_FIR0_FAE15_Pos* = (15)
  DSI_FIR0_FAE15_Msk* = (0x00000001 shl DSI_FIR0_FAE15_Pos) ## !< 0x00008000
  DSI_FIR0_FAE15* = DSI_FIR0_FAE15_Msk
  DSI_FIR0_FPE0_Pos* = (16)
  DSI_FIR0_FPE0_Msk* = (0x00000001 shl DSI_FIR0_FPE0_Pos) ## !< 0x00010000
  DSI_FIR0_FPE0* = DSI_FIR0_FPE0_Msk
  DSI_FIR0_FPE1_Pos* = (17)
  DSI_FIR0_FPE1_Msk* = (0x00000001 shl DSI_FIR0_FPE1_Pos) ## !< 0x00020000
  DSI_FIR0_FPE1* = DSI_FIR0_FPE1_Msk
  DSI_FIR0_FPE2_Pos* = (18)
  DSI_FIR0_FPE2_Msk* = (0x00000001 shl DSI_FIR0_FPE2_Pos) ## !< 0x00040000
  DSI_FIR0_FPE2* = DSI_FIR0_FPE2_Msk
  DSI_FIR0_FPE3_Pos* = (19)
  DSI_FIR0_FPE3_Msk* = (0x00000001 shl DSI_FIR0_FPE3_Pos) ## !< 0x00080000
  DSI_FIR0_FPE3* = DSI_FIR0_FPE3_Msk
  DSI_FIR0_FPE4_Pos* = (20)
  DSI_FIR0_FPE4_Msk* = (0x00000001 shl DSI_FIR0_FPE4_Pos) ## !< 0x00100000
  DSI_FIR0_FPE4* = DSI_FIR0_FPE4_Msk

## ******************  Bit definition for DSI_FIR1 register  **************

const
  DSI_FIR1_FTOHSTX_Pos* = (0)
  DSI_FIR1_FTOHSTX_Msk* = (0x00000001 shl DSI_FIR1_FTOHSTX_Pos) ## !< 0x00000001
  DSI_FIR1_FTOHSTX* = DSI_FIR1_FTOHSTX_Msk
  DSI_FIR1_FTOLPRX_Pos* = (1)
  DSI_FIR1_FTOLPRX_Msk* = (0x00000001 shl DSI_FIR1_FTOLPRX_Pos) ## !< 0x00000002
  DSI_FIR1_FTOLPRX* = DSI_FIR1_FTOLPRX_Msk
  DSI_FIR1_FECCSE_Pos* = (2)
  DSI_FIR1_FECCSE_Msk* = (0x00000001 shl DSI_FIR1_FECCSE_Pos) ## !< 0x00000004
  DSI_FIR1_FECCSE* = DSI_FIR1_FECCSE_Msk
  DSI_FIR1_FECCME_Pos* = (3)
  DSI_FIR1_FECCME_Msk* = (0x00000001 shl DSI_FIR1_FECCME_Pos) ## !< 0x00000008
  DSI_FIR1_FECCME* = DSI_FIR1_FECCME_Msk
  DSI_FIR1_FCRCE_Pos* = (4)
  DSI_FIR1_FCRCE_Msk* = (0x00000001 shl DSI_FIR1_FCRCE_Pos) ## !< 0x00000010
  DSI_FIR1_FCRCE* = DSI_FIR1_FCRCE_Msk
  DSI_FIR1_FPSE_Pos* = (5)
  DSI_FIR1_FPSE_Msk* = (0x00000001 shl DSI_FIR1_FPSE_Pos) ## !< 0x00000020
  DSI_FIR1_FPSE* = DSI_FIR1_FPSE_Msk
  DSI_FIR1_FEOTPE_Pos* = (6)
  DSI_FIR1_FEOTPE_Msk* = (0x00000001 shl DSI_FIR1_FEOTPE_Pos) ## !< 0x00000040
  DSI_FIR1_FEOTPE* = DSI_FIR1_FEOTPE_Msk
  DSI_FIR1_FLPWRE_Pos* = (7)
  DSI_FIR1_FLPWRE_Msk* = (0x00000001 shl DSI_FIR1_FLPWRE_Pos) ## !< 0x00000080
  DSI_FIR1_FLPWRE* = DSI_FIR1_FLPWRE_Msk
  DSI_FIR1_FGCWRE_Pos* = (8)
  DSI_FIR1_FGCWRE_Msk* = (0x00000001 shl DSI_FIR1_FGCWRE_Pos) ## !< 0x00000100
  DSI_FIR1_FGCWRE* = DSI_FIR1_FGCWRE_Msk
  DSI_FIR1_FGPWRE_Pos* = (9)
  DSI_FIR1_FGPWRE_Msk* = (0x00000001 shl DSI_FIR1_FGPWRE_Pos) ## !< 0x00000200
  DSI_FIR1_FGPWRE* = DSI_FIR1_FGPWRE_Msk
  DSI_FIR1_FGPTXE_Pos* = (10)
  DSI_FIR1_FGPTXE_Msk* = (0x00000001 shl DSI_FIR1_FGPTXE_Pos) ## !< 0x00000400
  DSI_FIR1_FGPTXE* = DSI_FIR1_FGPTXE_Msk
  DSI_FIR1_FGPRDE_Pos* = (11)
  DSI_FIR1_FGPRDE_Msk* = (0x00000001 shl DSI_FIR1_FGPRDE_Pos) ## !< 0x00000800
  DSI_FIR1_FGPRDE* = DSI_FIR1_FGPRDE_Msk
  DSI_FIR1_FGPRXE_Pos* = (12)
  DSI_FIR1_FGPRXE_Msk* = (0x00000001 shl DSI_FIR1_FGPRXE_Pos) ## !< 0x00001000
  DSI_FIR1_FGPRXE* = DSI_FIR1_FGPRXE_Msk

## ******************  Bit definition for DSI_VSCR register  **************

const
  DSI_VSCR_EN_Pos* = (0)
  DSI_VSCR_EN_Msk* = (0x00000001 shl DSI_VSCR_EN_Pos) ## !< 0x00000001
  DSI_VSCR_EN* = DSI_VSCR_EN_Msk
  DSI_VSCR_UR_Pos* = (8)
  DSI_VSCR_UR_Msk* = (0x00000001 shl DSI_VSCR_UR_Pos) ## !< 0x00000100
  DSI_VSCR_UR* = DSI_VSCR_UR_Msk

## ******************  Bit definition for DSI_LCVCIDR register  ***********

const
  DSI_LCVCIDR_VCID_Pos* = (0)
  DSI_LCVCIDR_VCID_Msk* = (0x00000003 shl DSI_LCVCIDR_VCID_Pos) ## !< 0x00000003
  DSI_LCVCIDR_VCID* = DSI_LCVCIDR_VCID_Msk
  DSI_LCVCIDR_VCID0_Pos* = (0)
  DSI_LCVCIDR_VCID0_Msk* = (0x00000001 shl DSI_LCVCIDR_VCID0_Pos) ## !< 0x00000001
  DSI_LCVCIDR_VCID0* = DSI_LCVCIDR_VCID0_Msk
  DSI_LCVCIDR_VCID1_Pos* = (1)
  DSI_LCVCIDR_VCID1_Msk* = (0x00000001 shl DSI_LCVCIDR_VCID1_Pos) ## !< 0x00000002
  DSI_LCVCIDR_VCID1* = DSI_LCVCIDR_VCID1_Msk

## ******************  Bit definition for DSI_LCCCR register  *************

const
  DSI_LCCCR_COLC_Pos* = (0)
  DSI_LCCCR_COLC_Msk* = (0x0000000F shl DSI_LCCCR_COLC_Pos) ## !< 0x0000000F
  DSI_LCCCR_COLC* = DSI_LCCCR_COLC_Msk
  DSI_LCCCR_COLC0_Pos* = (0)
  DSI_LCCCR_COLC0_Msk* = (0x00000001 shl DSI_LCCCR_COLC0_Pos) ## !< 0x00000001
  DSI_LCCCR_COLC0* = DSI_LCCCR_COLC0_Msk
  DSI_LCCCR_COLC1_Pos* = (1)
  DSI_LCCCR_COLC1_Msk* = (0x00000001 shl DSI_LCCCR_COLC1_Pos) ## !< 0x00000002
  DSI_LCCCR_COLC1* = DSI_LCCCR_COLC1_Msk
  DSI_LCCCR_COLC2_Pos* = (2)
  DSI_LCCCR_COLC2_Msk* = (0x00000001 shl DSI_LCCCR_COLC2_Pos) ## !< 0x00000004
  DSI_LCCCR_COLC2* = DSI_LCCCR_COLC2_Msk
  DSI_LCCCR_COLC3_Pos* = (3)
  DSI_LCCCR_COLC3_Msk* = (0x00000001 shl DSI_LCCCR_COLC3_Pos) ## !< 0x00000008
  DSI_LCCCR_COLC3* = DSI_LCCCR_COLC3_Msk
  DSI_LCCCR_LPE_Pos* = (8)
  DSI_LCCCR_LPE_Msk* = (0x00000001 shl DSI_LCCCR_LPE_Pos) ## !< 0x00000100
  DSI_LCCCR_LPE* = DSI_LCCCR_LPE_Msk

## ******************  Bit definition for DSI_LPMCCR register  ************

const
  DSI_LPMCCR_VLPSIZE_Pos* = (0)
  DSI_LPMCCR_VLPSIZE_Msk* = (0x000000FF shl DSI_LPMCCR_VLPSIZE_Pos) ## !< 0x000000FF
  DSI_LPMCCR_VLPSIZE* = DSI_LPMCCR_VLPSIZE_Msk
  DSI_LPMCCR_VLPSIZE0_Pos* = (0)
  DSI_LPMCCR_VLPSIZE0_Msk* = (0x00000001 shl DSI_LPMCCR_VLPSIZE0_Pos) ## !< 0x00000001
  DSI_LPMCCR_VLPSIZE0* = DSI_LPMCCR_VLPSIZE0_Msk
  DSI_LPMCCR_VLPSIZE1_Pos* = (1)
  DSI_LPMCCR_VLPSIZE1_Msk* = (0x00000001 shl DSI_LPMCCR_VLPSIZE1_Pos) ## !< 0x00000002
  DSI_LPMCCR_VLPSIZE1* = DSI_LPMCCR_VLPSIZE1_Msk
  DSI_LPMCCR_VLPSIZE2_Pos* = (2)
  DSI_LPMCCR_VLPSIZE2_Msk* = (0x00000001 shl DSI_LPMCCR_VLPSIZE2_Pos) ## !< 0x00000004
  DSI_LPMCCR_VLPSIZE2* = DSI_LPMCCR_VLPSIZE2_Msk
  DSI_LPMCCR_VLPSIZE3_Pos* = (3)
  DSI_LPMCCR_VLPSIZE3_Msk* = (0x00000001 shl DSI_LPMCCR_VLPSIZE3_Pos) ## !< 0x00000008
  DSI_LPMCCR_VLPSIZE3* = DSI_LPMCCR_VLPSIZE3_Msk
  DSI_LPMCCR_VLPSIZE4_Pos* = (4)
  DSI_LPMCCR_VLPSIZE4_Msk* = (0x00000001 shl DSI_LPMCCR_VLPSIZE4_Pos) ## !< 0x00000010
  DSI_LPMCCR_VLPSIZE4* = DSI_LPMCCR_VLPSIZE4_Msk
  DSI_LPMCCR_VLPSIZE5_Pos* = (5)
  DSI_LPMCCR_VLPSIZE5_Msk* = (0x00000001 shl DSI_LPMCCR_VLPSIZE5_Pos) ## !< 0x00000020
  DSI_LPMCCR_VLPSIZE5* = DSI_LPMCCR_VLPSIZE5_Msk
  DSI_LPMCCR_VLPSIZE6_Pos* = (6)
  DSI_LPMCCR_VLPSIZE6_Msk* = (0x00000001 shl DSI_LPMCCR_VLPSIZE6_Pos) ## !< 0x00000040
  DSI_LPMCCR_VLPSIZE6* = DSI_LPMCCR_VLPSIZE6_Msk
  DSI_LPMCCR_VLPSIZE7_Pos* = (7)
  DSI_LPMCCR_VLPSIZE7_Msk* = (0x00000001 shl DSI_LPMCCR_VLPSIZE7_Pos) ## !< 0x00000080
  DSI_LPMCCR_VLPSIZE7* = DSI_LPMCCR_VLPSIZE7_Msk
  DSI_LPMCCR_LPSIZE_Pos* = (16)
  DSI_LPMCCR_LPSIZE_Msk* = (0x000000FF shl DSI_LPMCCR_LPSIZE_Pos) ## !< 0x00FF0000
  DSI_LPMCCR_LPSIZE* = DSI_LPMCCR_LPSIZE_Msk
  DSI_LPMCCR_LPSIZE0_Pos* = (16)
  DSI_LPMCCR_LPSIZE0_Msk* = (0x00000001 shl DSI_LPMCCR_LPSIZE0_Pos) ## !< 0x00010000
  DSI_LPMCCR_LPSIZE0* = DSI_LPMCCR_LPSIZE0_Msk
  DSI_LPMCCR_LPSIZE1_Pos* = (17)
  DSI_LPMCCR_LPSIZE1_Msk* = (0x00000001 shl DSI_LPMCCR_LPSIZE1_Pos) ## !< 0x00020000
  DSI_LPMCCR_LPSIZE1* = DSI_LPMCCR_LPSIZE1_Msk
  DSI_LPMCCR_LPSIZE2_Pos* = (18)
  DSI_LPMCCR_LPSIZE2_Msk* = (0x00000001 shl DSI_LPMCCR_LPSIZE2_Pos) ## !< 0x00040000
  DSI_LPMCCR_LPSIZE2* = DSI_LPMCCR_LPSIZE2_Msk
  DSI_LPMCCR_LPSIZE3_Pos* = (19)
  DSI_LPMCCR_LPSIZE3_Msk* = (0x00000001 shl DSI_LPMCCR_LPSIZE3_Pos) ## !< 0x00080000
  DSI_LPMCCR_LPSIZE3* = DSI_LPMCCR_LPSIZE3_Msk
  DSI_LPMCCR_LPSIZE4_Pos* = (20)
  DSI_LPMCCR_LPSIZE4_Msk* = (0x00000001 shl DSI_LPMCCR_LPSIZE4_Pos) ## !< 0x00100000
  DSI_LPMCCR_LPSIZE4* = DSI_LPMCCR_LPSIZE4_Msk
  DSI_LPMCCR_LPSIZE5_Pos* = (21)
  DSI_LPMCCR_LPSIZE5_Msk* = (0x00000001 shl DSI_LPMCCR_LPSIZE5_Pos) ## !< 0x00200000
  DSI_LPMCCR_LPSIZE5* = DSI_LPMCCR_LPSIZE5_Msk
  DSI_LPMCCR_LPSIZE6_Pos* = (22)
  DSI_LPMCCR_LPSIZE6_Msk* = (0x00000001 shl DSI_LPMCCR_LPSIZE6_Pos) ## !< 0x00400000
  DSI_LPMCCR_LPSIZE6* = DSI_LPMCCR_LPSIZE6_Msk
  DSI_LPMCCR_LPSIZE7_Pos* = (23)
  DSI_LPMCCR_LPSIZE7_Msk* = (0x00000001 shl DSI_LPMCCR_LPSIZE7_Pos) ## !< 0x00800000
  DSI_LPMCCR_LPSIZE7* = DSI_LPMCCR_LPSIZE7_Msk

## ******************  Bit definition for DSI_VMCCR register  *************

const
  DSI_VMCCR_VMT_Pos* = (0)
  DSI_VMCCR_VMT_Msk* = (0x00000003 shl DSI_VMCCR_VMT_Pos) ## !< 0x00000003
  DSI_VMCCR_VMT* = DSI_VMCCR_VMT_Msk
  DSI_VMCCR_VMT0_Pos* = (0)
  DSI_VMCCR_VMT0_Msk* = (0x00000001 shl DSI_VMCCR_VMT0_Pos) ## !< 0x00000001
  DSI_VMCCR_VMT0* = DSI_VMCCR_VMT0_Msk
  DSI_VMCCR_VMT1_Pos* = (1)
  DSI_VMCCR_VMT1_Msk* = (0x00000001 shl DSI_VMCCR_VMT1_Pos) ## !< 0x00000002
  DSI_VMCCR_VMT1* = DSI_VMCCR_VMT1_Msk
  DSI_VMCCR_LPVSAE_Pos* = (8)
  DSI_VMCCR_LPVSAE_Msk* = (0x00000001 shl DSI_VMCCR_LPVSAE_Pos) ## !< 0x00000100
  DSI_VMCCR_LPVSAE* = DSI_VMCCR_LPVSAE_Msk
  DSI_VMCCR_LPVBPE_Pos* = (9)
  DSI_VMCCR_LPVBPE_Msk* = (0x00000001 shl DSI_VMCCR_LPVBPE_Pos) ## !< 0x00000200
  DSI_VMCCR_LPVBPE* = DSI_VMCCR_LPVBPE_Msk
  DSI_VMCCR_LPVFPE_Pos* = (10)
  DSI_VMCCR_LPVFPE_Msk* = (0x00000001 shl DSI_VMCCR_LPVFPE_Pos) ## !< 0x00000400
  DSI_VMCCR_LPVFPE* = DSI_VMCCR_LPVFPE_Msk
  DSI_VMCCR_LPVAE_Pos* = (11)
  DSI_VMCCR_LPVAE_Msk* = (0x00000001 shl DSI_VMCCR_LPVAE_Pos) ## !< 0x00000800
  DSI_VMCCR_LPVAE* = DSI_VMCCR_LPVAE_Msk
  DSI_VMCCR_LPHBPE_Pos* = (12)
  DSI_VMCCR_LPHBPE_Msk* = (0x00000001 shl DSI_VMCCR_LPHBPE_Pos) ## !< 0x00001000
  DSI_VMCCR_LPHBPE* = DSI_VMCCR_LPHBPE_Msk
  DSI_VMCCR_LPHFE_Pos* = (13)
  DSI_VMCCR_LPHFE_Msk* = (0x00000001 shl DSI_VMCCR_LPHFE_Pos) ## !< 0x00002000
  DSI_VMCCR_LPHFE* = DSI_VMCCR_LPHFE_Msk
  DSI_VMCCR_FBTAAE_Pos* = (14)
  DSI_VMCCR_FBTAAE_Msk* = (0x00000001 shl DSI_VMCCR_FBTAAE_Pos) ## !< 0x00004000
  DSI_VMCCR_FBTAAE* = DSI_VMCCR_FBTAAE_Msk
  DSI_VMCCR_LPCE_Pos* = (15)
  DSI_VMCCR_LPCE_Msk* = (0x00000001 shl DSI_VMCCR_LPCE_Pos) ## !< 0x00008000
  DSI_VMCCR_LPCE* = DSI_VMCCR_LPCE_Msk

## ******************  Bit definition for DSI_VPCCR register  *************

const
  DSI_VPCCR_VPSIZE_Pos* = (0)
  DSI_VPCCR_VPSIZE_Msk* = (0x00003FFF shl DSI_VPCCR_VPSIZE_Pos) ## !< 0x00003FFF
  DSI_VPCCR_VPSIZE* = DSI_VPCCR_VPSIZE_Msk
  DSI_VPCCR_VPSIZE0_Pos* = (0)
  DSI_VPCCR_VPSIZE0_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE0_Pos) ## !< 0x00000001
  DSI_VPCCR_VPSIZE0* = DSI_VPCCR_VPSIZE0_Msk
  DSI_VPCCR_VPSIZE1_Pos* = (1)
  DSI_VPCCR_VPSIZE1_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE1_Pos) ## !< 0x00000002
  DSI_VPCCR_VPSIZE1* = DSI_VPCCR_VPSIZE1_Msk
  DSI_VPCCR_VPSIZE2_Pos* = (2)
  DSI_VPCCR_VPSIZE2_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE2_Pos) ## !< 0x00000004
  DSI_VPCCR_VPSIZE2* = DSI_VPCCR_VPSIZE2_Msk
  DSI_VPCCR_VPSIZE3_Pos* = (3)
  DSI_VPCCR_VPSIZE3_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE3_Pos) ## !< 0x00000008
  DSI_VPCCR_VPSIZE3* = DSI_VPCCR_VPSIZE3_Msk
  DSI_VPCCR_VPSIZE4_Pos* = (4)
  DSI_VPCCR_VPSIZE4_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE4_Pos) ## !< 0x00000010
  DSI_VPCCR_VPSIZE4* = DSI_VPCCR_VPSIZE4_Msk
  DSI_VPCCR_VPSIZE5_Pos* = (5)
  DSI_VPCCR_VPSIZE5_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE5_Pos) ## !< 0x00000020
  DSI_VPCCR_VPSIZE5* = DSI_VPCCR_VPSIZE5_Msk
  DSI_VPCCR_VPSIZE6_Pos* = (6)
  DSI_VPCCR_VPSIZE6_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE6_Pos) ## !< 0x00000040
  DSI_VPCCR_VPSIZE6* = DSI_VPCCR_VPSIZE6_Msk
  DSI_VPCCR_VPSIZE7_Pos* = (7)
  DSI_VPCCR_VPSIZE7_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE7_Pos) ## !< 0x00000080
  DSI_VPCCR_VPSIZE7* = DSI_VPCCR_VPSIZE7_Msk
  DSI_VPCCR_VPSIZE8_Pos* = (8)
  DSI_VPCCR_VPSIZE8_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE8_Pos) ## !< 0x00000100
  DSI_VPCCR_VPSIZE8* = DSI_VPCCR_VPSIZE8_Msk
  DSI_VPCCR_VPSIZE9_Pos* = (9)
  DSI_VPCCR_VPSIZE9_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE9_Pos) ## !< 0x00000200
  DSI_VPCCR_VPSIZE9* = DSI_VPCCR_VPSIZE9_Msk
  DSI_VPCCR_VPSIZE10_Pos* = (10)
  DSI_VPCCR_VPSIZE10_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE10_Pos) ## !< 0x00000400
  DSI_VPCCR_VPSIZE10* = DSI_VPCCR_VPSIZE10_Msk
  DSI_VPCCR_VPSIZE11_Pos* = (11)
  DSI_VPCCR_VPSIZE11_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE11_Pos) ## !< 0x00000800
  DSI_VPCCR_VPSIZE11* = DSI_VPCCR_VPSIZE11_Msk
  DSI_VPCCR_VPSIZE12_Pos* = (12)
  DSI_VPCCR_VPSIZE12_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE12_Pos) ## !< 0x00001000
  DSI_VPCCR_VPSIZE12* = DSI_VPCCR_VPSIZE12_Msk
  DSI_VPCCR_VPSIZE13_Pos* = (13)
  DSI_VPCCR_VPSIZE13_Msk* = (0x00000001 shl DSI_VPCCR_VPSIZE13_Pos) ## !< 0x00002000
  DSI_VPCCR_VPSIZE13* = DSI_VPCCR_VPSIZE13_Msk

## ******************  Bit definition for DSI_VCCCR register  *************

const
  DSI_VCCCR_NUMC_Pos* = (0)
  DSI_VCCCR_NUMC_Msk* = (0x00001FFF shl DSI_VCCCR_NUMC_Pos) ## !< 0x00001FFF
  DSI_VCCCR_NUMC* = DSI_VCCCR_NUMC_Msk
  DSI_VCCCR_NUMC0_Pos* = (0)
  DSI_VCCCR_NUMC0_Msk* = (0x00000001 shl DSI_VCCCR_NUMC0_Pos) ## !< 0x00000001
  DSI_VCCCR_NUMC0* = DSI_VCCCR_NUMC0_Msk
  DSI_VCCCR_NUMC1_Pos* = (1)
  DSI_VCCCR_NUMC1_Msk* = (0x00000001 shl DSI_VCCCR_NUMC1_Pos) ## !< 0x00000002
  DSI_VCCCR_NUMC1* = DSI_VCCCR_NUMC1_Msk
  DSI_VCCCR_NUMC2_Pos* = (2)
  DSI_VCCCR_NUMC2_Msk* = (0x00000001 shl DSI_VCCCR_NUMC2_Pos) ## !< 0x00000004
  DSI_VCCCR_NUMC2* = DSI_VCCCR_NUMC2_Msk
  DSI_VCCCR_NUMC3_Pos* = (3)
  DSI_VCCCR_NUMC3_Msk* = (0x00000001 shl DSI_VCCCR_NUMC3_Pos) ## !< 0x00000008
  DSI_VCCCR_NUMC3* = DSI_VCCCR_NUMC3_Msk
  DSI_VCCCR_NUMC4_Pos* = (4)
  DSI_VCCCR_NUMC4_Msk* = (0x00000001 shl DSI_VCCCR_NUMC4_Pos) ## !< 0x00000010
  DSI_VCCCR_NUMC4* = DSI_VCCCR_NUMC4_Msk
  DSI_VCCCR_NUMC5_Pos* = (5)
  DSI_VCCCR_NUMC5_Msk* = (0x00000001 shl DSI_VCCCR_NUMC5_Pos) ## !< 0x00000020
  DSI_VCCCR_NUMC5* = DSI_VCCCR_NUMC5_Msk
  DSI_VCCCR_NUMC6_Pos* = (6)
  DSI_VCCCR_NUMC6_Msk* = (0x00000001 shl DSI_VCCCR_NUMC6_Pos) ## !< 0x00000040
  DSI_VCCCR_NUMC6* = DSI_VCCCR_NUMC6_Msk
  DSI_VCCCR_NUMC7_Pos* = (7)
  DSI_VCCCR_NUMC7_Msk* = (0x00000001 shl DSI_VCCCR_NUMC7_Pos) ## !< 0x00000080
  DSI_VCCCR_NUMC7* = DSI_VCCCR_NUMC7_Msk
  DSI_VCCCR_NUMC8_Pos* = (8)
  DSI_VCCCR_NUMC8_Msk* = (0x00000001 shl DSI_VCCCR_NUMC8_Pos) ## !< 0x00000100
  DSI_VCCCR_NUMC8* = DSI_VCCCR_NUMC8_Msk
  DSI_VCCCR_NUMC9_Pos* = (9)
  DSI_VCCCR_NUMC9_Msk* = (0x00000001 shl DSI_VCCCR_NUMC9_Pos) ## !< 0x00000200
  DSI_VCCCR_NUMC9* = DSI_VCCCR_NUMC9_Msk
  DSI_VCCCR_NUMC10_Pos* = (10)
  DSI_VCCCR_NUMC10_Msk* = (0x00000001 shl DSI_VCCCR_NUMC10_Pos) ## !< 0x00000400
  DSI_VCCCR_NUMC10* = DSI_VCCCR_NUMC10_Msk
  DSI_VCCCR_NUMC11_Pos* = (11)
  DSI_VCCCR_NUMC11_Msk* = (0x00000001 shl DSI_VCCCR_NUMC11_Pos) ## !< 0x00000800
  DSI_VCCCR_NUMC11* = DSI_VCCCR_NUMC11_Msk
  DSI_VCCCR_NUMC12_Pos* = (12)
  DSI_VCCCR_NUMC12_Msk* = (0x00000001 shl DSI_VCCCR_NUMC12_Pos) ## !< 0x00001000
  DSI_VCCCR_NUMC12* = DSI_VCCCR_NUMC12_Msk

## ******************  Bit definition for DSI_VNPCCR register  ************

const
  DSI_VNPCCR_NPSIZE_Pos* = (0)
  DSI_VNPCCR_NPSIZE_Msk* = (0x00001FFF shl DSI_VNPCCR_NPSIZE_Pos) ## !< 0x00001FFF
  DSI_VNPCCR_NPSIZE* = DSI_VNPCCR_NPSIZE_Msk
  DSI_VNPCCR_NPSIZE0_Pos* = (0)
  DSI_VNPCCR_NPSIZE0_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE0_Pos) ## !< 0x00000001
  DSI_VNPCCR_NPSIZE0* = DSI_VNPCCR_NPSIZE0_Msk
  DSI_VNPCCR_NPSIZE1_Pos* = (1)
  DSI_VNPCCR_NPSIZE1_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE1_Pos) ## !< 0x00000002
  DSI_VNPCCR_NPSIZE1* = DSI_VNPCCR_NPSIZE1_Msk
  DSI_VNPCCR_NPSIZE2_Pos* = (2)
  DSI_VNPCCR_NPSIZE2_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE2_Pos) ## !< 0x00000004
  DSI_VNPCCR_NPSIZE2* = DSI_VNPCCR_NPSIZE2_Msk
  DSI_VNPCCR_NPSIZE3_Pos* = (3)
  DSI_VNPCCR_NPSIZE3_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE3_Pos) ## !< 0x00000008
  DSI_VNPCCR_NPSIZE3* = DSI_VNPCCR_NPSIZE3_Msk
  DSI_VNPCCR_NPSIZE4_Pos* = (4)
  DSI_VNPCCR_NPSIZE4_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE4_Pos) ## !< 0x00000010
  DSI_VNPCCR_NPSIZE4* = DSI_VNPCCR_NPSIZE4_Msk
  DSI_VNPCCR_NPSIZE5_Pos* = (5)
  DSI_VNPCCR_NPSIZE5_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE5_Pos) ## !< 0x00000020
  DSI_VNPCCR_NPSIZE5* = DSI_VNPCCR_NPSIZE5_Msk
  DSI_VNPCCR_NPSIZE6_Pos* = (6)
  DSI_VNPCCR_NPSIZE6_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE6_Pos) ## !< 0x00000040
  DSI_VNPCCR_NPSIZE6* = DSI_VNPCCR_NPSIZE6_Msk
  DSI_VNPCCR_NPSIZE7_Pos* = (7)
  DSI_VNPCCR_NPSIZE7_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE7_Pos) ## !< 0x00000080
  DSI_VNPCCR_NPSIZE7* = DSI_VNPCCR_NPSIZE7_Msk
  DSI_VNPCCR_NPSIZE8_Pos* = (8)
  DSI_VNPCCR_NPSIZE8_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE8_Pos) ## !< 0x00000100
  DSI_VNPCCR_NPSIZE8* = DSI_VNPCCR_NPSIZE8_Msk
  DSI_VNPCCR_NPSIZE9_Pos* = (9)
  DSI_VNPCCR_NPSIZE9_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE9_Pos) ## !< 0x00000200
  DSI_VNPCCR_NPSIZE9* = DSI_VNPCCR_NPSIZE9_Msk
  DSI_VNPCCR_NPSIZE10_Pos* = (10)
  DSI_VNPCCR_NPSIZE10_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE10_Pos) ## !< 0x00000400
  DSI_VNPCCR_NPSIZE10* = DSI_VNPCCR_NPSIZE10_Msk
  DSI_VNPCCR_NPSIZE11_Pos* = (11)
  DSI_VNPCCR_NPSIZE11_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE11_Pos) ## !< 0x00000800
  DSI_VNPCCR_NPSIZE11* = DSI_VNPCCR_NPSIZE11_Msk
  DSI_VNPCCR_NPSIZE12_Pos* = (12)
  DSI_VNPCCR_NPSIZE12_Msk* = (0x00000001 shl DSI_VNPCCR_NPSIZE12_Pos) ## !< 0x00001000
  DSI_VNPCCR_NPSIZE12* = DSI_VNPCCR_NPSIZE12_Msk

## ******************  Bit definition for DSI_VHSACCR register  ***********

const
  DSI_VHSACCR_HSA_Pos* = (0)
  DSI_VHSACCR_HSA_Msk* = (0x00000FFF shl DSI_VHSACCR_HSA_Pos) ## !< 0x00000FFF
  DSI_VHSACCR_HSA* = DSI_VHSACCR_HSA_Msk
  DSI_VHSACCR_HSA0_Pos* = (0)
  DSI_VHSACCR_HSA0_Msk* = (0x00000001 shl DSI_VHSACCR_HSA0_Pos) ## !< 0x00000001
  DSI_VHSACCR_HSA0* = DSI_VHSACCR_HSA0_Msk
  DSI_VHSACCR_HSA1_Pos* = (1)
  DSI_VHSACCR_HSA1_Msk* = (0x00000001 shl DSI_VHSACCR_HSA1_Pos) ## !< 0x00000002
  DSI_VHSACCR_HSA1* = DSI_VHSACCR_HSA1_Msk
  DSI_VHSACCR_HSA2_Pos* = (2)
  DSI_VHSACCR_HSA2_Msk* = (0x00000001 shl DSI_VHSACCR_HSA2_Pos) ## !< 0x00000004
  DSI_VHSACCR_HSA2* = DSI_VHSACCR_HSA2_Msk
  DSI_VHSACCR_HSA3_Pos* = (3)
  DSI_VHSACCR_HSA3_Msk* = (0x00000001 shl DSI_VHSACCR_HSA3_Pos) ## !< 0x00000008
  DSI_VHSACCR_HSA3* = DSI_VHSACCR_HSA3_Msk
  DSI_VHSACCR_HSA4_Pos* = (4)
  DSI_VHSACCR_HSA4_Msk* = (0x00000001 shl DSI_VHSACCR_HSA4_Pos) ## !< 0x00000010
  DSI_VHSACCR_HSA4* = DSI_VHSACCR_HSA4_Msk
  DSI_VHSACCR_HSA5_Pos* = (5)
  DSI_VHSACCR_HSA5_Msk* = (0x00000001 shl DSI_VHSACCR_HSA5_Pos) ## !< 0x00000020
  DSI_VHSACCR_HSA5* = DSI_VHSACCR_HSA5_Msk
  DSI_VHSACCR_HSA6_Pos* = (6)
  DSI_VHSACCR_HSA6_Msk* = (0x00000001 shl DSI_VHSACCR_HSA6_Pos) ## !< 0x00000040
  DSI_VHSACCR_HSA6* = DSI_VHSACCR_HSA6_Msk
  DSI_VHSACCR_HSA7_Pos* = (7)
  DSI_VHSACCR_HSA7_Msk* = (0x00000001 shl DSI_VHSACCR_HSA7_Pos) ## !< 0x00000080
  DSI_VHSACCR_HSA7* = DSI_VHSACCR_HSA7_Msk
  DSI_VHSACCR_HSA8_Pos* = (8)
  DSI_VHSACCR_HSA8_Msk* = (0x00000001 shl DSI_VHSACCR_HSA8_Pos) ## !< 0x00000100
  DSI_VHSACCR_HSA8* = DSI_VHSACCR_HSA8_Msk
  DSI_VHSACCR_HSA9_Pos* = (9)
  DSI_VHSACCR_HSA9_Msk* = (0x00000001 shl DSI_VHSACCR_HSA9_Pos) ## !< 0x00000200
  DSI_VHSACCR_HSA9* = DSI_VHSACCR_HSA9_Msk
  DSI_VHSACCR_HSA10_Pos* = (10)
  DSI_VHSACCR_HSA10_Msk* = (0x00000001 shl DSI_VHSACCR_HSA10_Pos) ## !< 0x00000400
  DSI_VHSACCR_HSA10* = DSI_VHSACCR_HSA10_Msk
  DSI_VHSACCR_HSA11_Pos* = (11)
  DSI_VHSACCR_HSA11_Msk* = (0x00000001 shl DSI_VHSACCR_HSA11_Pos) ## !< 0x00000800
  DSI_VHSACCR_HSA11* = DSI_VHSACCR_HSA11_Msk

## ******************  Bit definition for DSI_VHBPCCR register  ***********

const
  DSI_VHBPCCR_HBP_Pos* = (0)
  DSI_VHBPCCR_HBP_Msk* = (0x00000FFF shl DSI_VHBPCCR_HBP_Pos) ## !< 0x00000FFF
  DSI_VHBPCCR_HBP* = DSI_VHBPCCR_HBP_Msk
  DSI_VHBPCCR_HBP0_Pos* = (0)
  DSI_VHBPCCR_HBP0_Msk* = (0x00000001 shl DSI_VHBPCCR_HBP0_Pos) ## !< 0x00000001
  DSI_VHBPCCR_HBP0* = DSI_VHBPCCR_HBP0_Msk
  DSI_VHBPCCR_HBP1_Pos* = (1)
  DSI_VHBPCCR_HBP1_Msk* = (0x00000001 shl DSI_VHBPCCR_HBP1_Pos) ## !< 0x00000002
  DSI_VHBPCCR_HBP1* = DSI_VHBPCCR_HBP1_Msk
  DSI_VHBPCCR_HBP2_Pos* = (2)
  DSI_VHBPCCR_HBP2_Msk* = (0x00000001 shl DSI_VHBPCCR_HBP2_Pos) ## !< 0x00000004
  DSI_VHBPCCR_HBP2* = DSI_VHBPCCR_HBP2_Msk
  DSI_VHBPCCR_HBP3_Pos* = (3)
  DSI_VHBPCCR_HBP3_Msk* = (0x00000001 shl DSI_VHBPCCR_HBP3_Pos) ## !< 0x00000008
  DSI_VHBPCCR_HBP3* = DSI_VHBPCCR_HBP3_Msk
  DSI_VHBPCCR_HBP4_Pos* = (4)
  DSI_VHBPCCR_HBP4_Msk* = (0x00000001 shl DSI_VHBPCCR_HBP4_Pos) ## !< 0x00000010
  DSI_VHBPCCR_HBP4* = DSI_VHBPCCR_HBP4_Msk
  DSI_VHBPCCR_HBP5_Pos* = (5)
  DSI_VHBPCCR_HBP5_Msk* = (0x00000001 shl DSI_VHBPCCR_HBP5_Pos) ## !< 0x00000020
  DSI_VHBPCCR_HBP5* = DSI_VHBPCCR_HBP5_Msk
  DSI_VHBPCCR_HBP6_Pos* = (6)
  DSI_VHBPCCR_HBP6_Msk* = (0x00000001 shl DSI_VHBPCCR_HBP6_Pos) ## !< 0x00000040
  DSI_VHBPCCR_HBP6* = DSI_VHBPCCR_HBP6_Msk
  DSI_VHBPCCR_HBP7_Pos* = (7)
  DSI_VHBPCCR_HBP7_Msk* = (0x00000001 shl DSI_VHBPCCR_HBP7_Pos) ## !< 0x00000080
  DSI_VHBPCCR_HBP7* = DSI_VHBPCCR_HBP7_Msk
  DSI_VHBPCCR_HBP8_Pos* = (8)
  DSI_VHBPCCR_HBP8_Msk* = (0x00000001 shl DSI_VHBPCCR_HBP8_Pos) ## !< 0x00000100
  DSI_VHBPCCR_HBP8* = DSI_VHBPCCR_HBP8_Msk
  DSI_VHBPCCR_HBP9_Pos* = (9)
  DSI_VHBPCCR_HBP9_Msk* = (0x00000001 shl DSI_VHBPCCR_HBP9_Pos) ## !< 0x00000200
  DSI_VHBPCCR_HBP9* = DSI_VHBPCCR_HBP9_Msk
  DSI_VHBPCCR_HBP10_Pos* = (10)
  DSI_VHBPCCR_HBP10_Msk* = (0x00000001 shl DSI_VHBPCCR_HBP10_Pos) ## !< 0x00000400
  DSI_VHBPCCR_HBP10* = DSI_VHBPCCR_HBP10_Msk
  DSI_VHBPCCR_HBP11_Pos* = (11)
  DSI_VHBPCCR_HBP11_Msk* = (0x00000001 shl DSI_VHBPCCR_HBP11_Pos) ## !< 0x00000800
  DSI_VHBPCCR_HBP11* = DSI_VHBPCCR_HBP11_Msk

## ******************  Bit definition for DSI_VLCCR register  *************

const
  DSI_VLCCR_HLINE_Pos* = (0)
  DSI_VLCCR_HLINE_Msk* = (0x00007FFF shl DSI_VLCCR_HLINE_Pos) ## !< 0x00007FFF
  DSI_VLCCR_HLINE* = DSI_VLCCR_HLINE_Msk
  DSI_VLCCR_HLINE0_Pos* = (0)
  DSI_VLCCR_HLINE0_Msk* = (0x00000001 shl DSI_VLCCR_HLINE0_Pos) ## !< 0x00000001
  DSI_VLCCR_HLINE0* = DSI_VLCCR_HLINE0_Msk
  DSI_VLCCR_HLINE1_Pos* = (1)
  DSI_VLCCR_HLINE1_Msk* = (0x00000001 shl DSI_VLCCR_HLINE1_Pos) ## !< 0x00000002
  DSI_VLCCR_HLINE1* = DSI_VLCCR_HLINE1_Msk
  DSI_VLCCR_HLINE2_Pos* = (2)
  DSI_VLCCR_HLINE2_Msk* = (0x00000001 shl DSI_VLCCR_HLINE2_Pos) ## !< 0x00000004
  DSI_VLCCR_HLINE2* = DSI_VLCCR_HLINE2_Msk
  DSI_VLCCR_HLINE3_Pos* = (3)
  DSI_VLCCR_HLINE3_Msk* = (0x00000001 shl DSI_VLCCR_HLINE3_Pos) ## !< 0x00000008
  DSI_VLCCR_HLINE3* = DSI_VLCCR_HLINE3_Msk
  DSI_VLCCR_HLINE4_Pos* = (4)
  DSI_VLCCR_HLINE4_Msk* = (0x00000001 shl DSI_VLCCR_HLINE4_Pos) ## !< 0x00000010
  DSI_VLCCR_HLINE4* = DSI_VLCCR_HLINE4_Msk
  DSI_VLCCR_HLINE5_Pos* = (5)
  DSI_VLCCR_HLINE5_Msk* = (0x00000001 shl DSI_VLCCR_HLINE5_Pos) ## !< 0x00000020
  DSI_VLCCR_HLINE5* = DSI_VLCCR_HLINE5_Msk
  DSI_VLCCR_HLINE6_Pos* = (6)
  DSI_VLCCR_HLINE6_Msk* = (0x00000001 shl DSI_VLCCR_HLINE6_Pos) ## !< 0x00000040
  DSI_VLCCR_HLINE6* = DSI_VLCCR_HLINE6_Msk
  DSI_VLCCR_HLINE7_Pos* = (7)
  DSI_VLCCR_HLINE7_Msk* = (0x00000001 shl DSI_VLCCR_HLINE7_Pos) ## !< 0x00000080
  DSI_VLCCR_HLINE7* = DSI_VLCCR_HLINE7_Msk
  DSI_VLCCR_HLINE8_Pos* = (8)
  DSI_VLCCR_HLINE8_Msk* = (0x00000001 shl DSI_VLCCR_HLINE8_Pos) ## !< 0x00000100
  DSI_VLCCR_HLINE8* = DSI_VLCCR_HLINE8_Msk
  DSI_VLCCR_HLINE9_Pos* = (9)
  DSI_VLCCR_HLINE9_Msk* = (0x00000001 shl DSI_VLCCR_HLINE9_Pos) ## !< 0x00000200
  DSI_VLCCR_HLINE9* = DSI_VLCCR_HLINE9_Msk
  DSI_VLCCR_HLINE10_Pos* = (10)
  DSI_VLCCR_HLINE10_Msk* = (0x00000001 shl DSI_VLCCR_HLINE10_Pos) ## !< 0x00000400
  DSI_VLCCR_HLINE10* = DSI_VLCCR_HLINE10_Msk
  DSI_VLCCR_HLINE11_Pos* = (11)
  DSI_VLCCR_HLINE11_Msk* = (0x00000001 shl DSI_VLCCR_HLINE11_Pos) ## !< 0x00000800
  DSI_VLCCR_HLINE11* = DSI_VLCCR_HLINE11_Msk
  DSI_VLCCR_HLINE12_Pos* = (12)
  DSI_VLCCR_HLINE12_Msk* = (0x00000001 shl DSI_VLCCR_HLINE12_Pos) ## !< 0x00001000
  DSI_VLCCR_HLINE12* = DSI_VLCCR_HLINE12_Msk
  DSI_VLCCR_HLINE13_Pos* = (13)
  DSI_VLCCR_HLINE13_Msk* = (0x00000001 shl DSI_VLCCR_HLINE13_Pos) ## !< 0x00002000
  DSI_VLCCR_HLINE13* = DSI_VLCCR_HLINE13_Msk
  DSI_VLCCR_HLINE14_Pos* = (14)
  DSI_VLCCR_HLINE14_Msk* = (0x00000001 shl DSI_VLCCR_HLINE14_Pos) ## !< 0x00004000
  DSI_VLCCR_HLINE14* = DSI_VLCCR_HLINE14_Msk

## ******************  Bit definition for DSI_VVSACCR register  **************

const
  DSI_VVSACCR_VSA_Pos* = (0)
  DSI_VVSACCR_VSA_Msk* = (0x000003FF shl DSI_VVSACCR_VSA_Pos) ## !< 0x000003FF
  DSI_VVSACCR_VSA* = DSI_VVSACCR_VSA_Msk
  DSI_VVSACCR_VSA0_Pos* = (0)
  DSI_VVSACCR_VSA0_Msk* = (0x00000001 shl DSI_VVSACCR_VSA0_Pos) ## !< 0x00000001
  DSI_VVSACCR_VSA0* = DSI_VVSACCR_VSA0_Msk
  DSI_VVSACCR_VSA1_Pos* = (1)
  DSI_VVSACCR_VSA1_Msk* = (0x00000001 shl DSI_VVSACCR_VSA1_Pos) ## !< 0x00000002
  DSI_VVSACCR_VSA1* = DSI_VVSACCR_VSA1_Msk
  DSI_VVSACCR_VSA2_Pos* = (2)
  DSI_VVSACCR_VSA2_Msk* = (0x00000001 shl DSI_VVSACCR_VSA2_Pos) ## !< 0x00000004
  DSI_VVSACCR_VSA2* = DSI_VVSACCR_VSA2_Msk
  DSI_VVSACCR_VSA3_Pos* = (3)
  DSI_VVSACCR_VSA3_Msk* = (0x00000001 shl DSI_VVSACCR_VSA3_Pos) ## !< 0x00000008
  DSI_VVSACCR_VSA3* = DSI_VVSACCR_VSA3_Msk
  DSI_VVSACCR_VSA4_Pos* = (4)
  DSI_VVSACCR_VSA4_Msk* = (0x00000001 shl DSI_VVSACCR_VSA4_Pos) ## !< 0x00000010
  DSI_VVSACCR_VSA4* = DSI_VVSACCR_VSA4_Msk
  DSI_VVSACCR_VSA5_Pos* = (5)
  DSI_VVSACCR_VSA5_Msk* = (0x00000001 shl DSI_VVSACCR_VSA5_Pos) ## !< 0x00000020
  DSI_VVSACCR_VSA5* = DSI_VVSACCR_VSA5_Msk
  DSI_VVSACCR_VSA6_Pos* = (6)
  DSI_VVSACCR_VSA6_Msk* = (0x00000001 shl DSI_VVSACCR_VSA6_Pos) ## !< 0x00000040
  DSI_VVSACCR_VSA6* = DSI_VVSACCR_VSA6_Msk
  DSI_VVSACCR_VSA7_Pos* = (7)
  DSI_VVSACCR_VSA7_Msk* = (0x00000001 shl DSI_VVSACCR_VSA7_Pos) ## !< 0x00000080
  DSI_VVSACCR_VSA7* = DSI_VVSACCR_VSA7_Msk
  DSI_VVSACCR_VSA8_Pos* = (8)
  DSI_VVSACCR_VSA8_Msk* = (0x00000001 shl DSI_VVSACCR_VSA8_Pos) ## !< 0x00000100
  DSI_VVSACCR_VSA8* = DSI_VVSACCR_VSA8_Msk
  DSI_VVSACCR_VSA9_Pos* = (9)
  DSI_VVSACCR_VSA9_Msk* = (0x00000001 shl DSI_VVSACCR_VSA9_Pos) ## !< 0x00000200
  DSI_VVSACCR_VSA9* = DSI_VVSACCR_VSA9_Msk

## ******************  Bit definition for DSI_VVBPCCR register  ***********

const
  DSI_VVBPCCR_VBP_Pos* = (0)
  DSI_VVBPCCR_VBP_Msk* = (0x000003FF shl DSI_VVBPCCR_VBP_Pos) ## !< 0x000003FF
  DSI_VVBPCCR_VBP* = DSI_VVBPCCR_VBP_Msk
  DSI_VVBPCCR_VBP0_Pos* = (0)
  DSI_VVBPCCR_VBP0_Msk* = (0x00000001 shl DSI_VVBPCCR_VBP0_Pos) ## !< 0x00000001
  DSI_VVBPCCR_VBP0* = DSI_VVBPCCR_VBP0_Msk
  DSI_VVBPCCR_VBP1_Pos* = (1)
  DSI_VVBPCCR_VBP1_Msk* = (0x00000001 shl DSI_VVBPCCR_VBP1_Pos) ## !< 0x00000002
  DSI_VVBPCCR_VBP1* = DSI_VVBPCCR_VBP1_Msk
  DSI_VVBPCCR_VBP2_Pos* = (2)
  DSI_VVBPCCR_VBP2_Msk* = (0x00000001 shl DSI_VVBPCCR_VBP2_Pos) ## !< 0x00000004
  DSI_VVBPCCR_VBP2* = DSI_VVBPCCR_VBP2_Msk
  DSI_VVBPCCR_VBP3_Pos* = (3)
  DSI_VVBPCCR_VBP3_Msk* = (0x00000001 shl DSI_VVBPCCR_VBP3_Pos) ## !< 0x00000008
  DSI_VVBPCCR_VBP3* = DSI_VVBPCCR_VBP3_Msk
  DSI_VVBPCCR_VBP4_Pos* = (4)
  DSI_VVBPCCR_VBP4_Msk* = (0x00000001 shl DSI_VVBPCCR_VBP4_Pos) ## !< 0x00000010
  DSI_VVBPCCR_VBP4* = DSI_VVBPCCR_VBP4_Msk
  DSI_VVBPCCR_VBP5_Pos* = (5)
  DSI_VVBPCCR_VBP5_Msk* = (0x00000001 shl DSI_VVBPCCR_VBP5_Pos) ## !< 0x00000020
  DSI_VVBPCCR_VBP5* = DSI_VVBPCCR_VBP5_Msk
  DSI_VVBPCCR_VBP6_Pos* = (6)
  DSI_VVBPCCR_VBP6_Msk* = (0x00000001 shl DSI_VVBPCCR_VBP6_Pos) ## !< 0x00000040
  DSI_VVBPCCR_VBP6* = DSI_VVBPCCR_VBP6_Msk
  DSI_VVBPCCR_VBP7_Pos* = (7)
  DSI_VVBPCCR_VBP7_Msk* = (0x00000001 shl DSI_VVBPCCR_VBP7_Pos) ## !< 0x00000080
  DSI_VVBPCCR_VBP7* = DSI_VVBPCCR_VBP7_Msk
  DSI_VVBPCCR_VBP8_Pos* = (8)
  DSI_VVBPCCR_VBP8_Msk* = (0x00000001 shl DSI_VVBPCCR_VBP8_Pos) ## !< 0x00000100
  DSI_VVBPCCR_VBP8* = DSI_VVBPCCR_VBP8_Msk
  DSI_VVBPCCR_VBP9_Pos* = (9)
  DSI_VVBPCCR_VBP9_Msk* = (0x00000001 shl DSI_VVBPCCR_VBP9_Pos) ## !< 0x00000200
  DSI_VVBPCCR_VBP9* = DSI_VVBPCCR_VBP9_Msk

## ******************  Bit definition for DSI_VVFPCCR register  ***********

const
  DSI_VVFPCCR_VFP_Pos* = (0)
  DSI_VVFPCCR_VFP_Msk* = (0x000003FF shl DSI_VVFPCCR_VFP_Pos) ## !< 0x000003FF
  DSI_VVFPCCR_VFP* = DSI_VVFPCCR_VFP_Msk
  DSI_VVFPCCR_VFP0_Pos* = (0)
  DSI_VVFPCCR_VFP0_Msk* = (0x00000001 shl DSI_VVFPCCR_VFP0_Pos) ## !< 0x00000001
  DSI_VVFPCCR_VFP0* = DSI_VVFPCCR_VFP0_Msk
  DSI_VVFPCCR_VFP1_Pos* = (1)
  DSI_VVFPCCR_VFP1_Msk* = (0x00000001 shl DSI_VVFPCCR_VFP1_Pos) ## !< 0x00000002
  DSI_VVFPCCR_VFP1* = DSI_VVFPCCR_VFP1_Msk
  DSI_VVFPCCR_VFP2_Pos* = (2)
  DSI_VVFPCCR_VFP2_Msk* = (0x00000001 shl DSI_VVFPCCR_VFP2_Pos) ## !< 0x00000004
  DSI_VVFPCCR_VFP2* = DSI_VVFPCCR_VFP2_Msk
  DSI_VVFPCCR_VFP3_Pos* = (3)
  DSI_VVFPCCR_VFP3_Msk* = (0x00000001 shl DSI_VVFPCCR_VFP3_Pos) ## !< 0x00000008
  DSI_VVFPCCR_VFP3* = DSI_VVFPCCR_VFP3_Msk
  DSI_VVFPCCR_VFP4_Pos* = (4)
  DSI_VVFPCCR_VFP4_Msk* = (0x00000001 shl DSI_VVFPCCR_VFP4_Pos) ## !< 0x00000010
  DSI_VVFPCCR_VFP4* = DSI_VVFPCCR_VFP4_Msk
  DSI_VVFPCCR_VFP5_Pos* = (5)
  DSI_VVFPCCR_VFP5_Msk* = (0x00000001 shl DSI_VVFPCCR_VFP5_Pos) ## !< 0x00000020
  DSI_VVFPCCR_VFP5* = DSI_VVFPCCR_VFP5_Msk
  DSI_VVFPCCR_VFP6_Pos* = (6)
  DSI_VVFPCCR_VFP6_Msk* = (0x00000001 shl DSI_VVFPCCR_VFP6_Pos) ## !< 0x00000040
  DSI_VVFPCCR_VFP6* = DSI_VVFPCCR_VFP6_Msk
  DSI_VVFPCCR_VFP7_Pos* = (7)
  DSI_VVFPCCR_VFP7_Msk* = (0x00000001 shl DSI_VVFPCCR_VFP7_Pos) ## !< 0x00000080
  DSI_VVFPCCR_VFP7* = DSI_VVFPCCR_VFP7_Msk
  DSI_VVFPCCR_VFP8_Pos* = (8)
  DSI_VVFPCCR_VFP8_Msk* = (0x00000001 shl DSI_VVFPCCR_VFP8_Pos) ## !< 0x00000100
  DSI_VVFPCCR_VFP8* = DSI_VVFPCCR_VFP8_Msk
  DSI_VVFPCCR_VFP9_Pos* = (9)
  DSI_VVFPCCR_VFP9_Msk* = (0x00000001 shl DSI_VVFPCCR_VFP9_Pos) ## !< 0x00000200
  DSI_VVFPCCR_VFP9* = DSI_VVFPCCR_VFP9_Msk

## ******************  Bit definition for DSI_VVACCR register  ************

const
  DSI_VVACCR_VA_Pos* = (0)
  DSI_VVACCR_VA_Msk* = (0x00003FFF shl DSI_VVACCR_VA_Pos) ## !< 0x00003FFF
  DSI_VVACCR_VA* = DSI_VVACCR_VA_Msk
  DSI_VVACCR_VA0_Pos* = (0)
  DSI_VVACCR_VA0_Msk* = (0x00000001 shl DSI_VVACCR_VA0_Pos) ## !< 0x00000001
  DSI_VVACCR_VA0* = DSI_VVACCR_VA0_Msk
  DSI_VVACCR_VA1_Pos* = (1)
  DSI_VVACCR_VA1_Msk* = (0x00000001 shl DSI_VVACCR_VA1_Pos) ## !< 0x00000002
  DSI_VVACCR_VA1* = DSI_VVACCR_VA1_Msk
  DSI_VVACCR_VA2_Pos* = (2)
  DSI_VVACCR_VA2_Msk* = (0x00000001 shl DSI_VVACCR_VA2_Pos) ## !< 0x00000004
  DSI_VVACCR_VA2* = DSI_VVACCR_VA2_Msk
  DSI_VVACCR_VA3_Pos* = (3)
  DSI_VVACCR_VA3_Msk* = (0x00000001 shl DSI_VVACCR_VA3_Pos) ## !< 0x00000008
  DSI_VVACCR_VA3* = DSI_VVACCR_VA3_Msk
  DSI_VVACCR_VA4_Pos* = (4)
  DSI_VVACCR_VA4_Msk* = (0x00000001 shl DSI_VVACCR_VA4_Pos) ## !< 0x00000010
  DSI_VVACCR_VA4* = DSI_VVACCR_VA4_Msk
  DSI_VVACCR_VA5_Pos* = (5)
  DSI_VVACCR_VA5_Msk* = (0x00000001 shl DSI_VVACCR_VA5_Pos) ## !< 0x00000020
  DSI_VVACCR_VA5* = DSI_VVACCR_VA5_Msk
  DSI_VVACCR_VA6_Pos* = (6)
  DSI_VVACCR_VA6_Msk* = (0x00000001 shl DSI_VVACCR_VA6_Pos) ## !< 0x00000040
  DSI_VVACCR_VA6* = DSI_VVACCR_VA6_Msk
  DSI_VVACCR_VA7_Pos* = (7)
  DSI_VVACCR_VA7_Msk* = (0x00000001 shl DSI_VVACCR_VA7_Pos) ## !< 0x00000080
  DSI_VVACCR_VA7* = DSI_VVACCR_VA7_Msk
  DSI_VVACCR_VA8_Pos* = (8)
  DSI_VVACCR_VA8_Msk* = (0x00000001 shl DSI_VVACCR_VA8_Pos) ## !< 0x00000100
  DSI_VVACCR_VA8* = DSI_VVACCR_VA8_Msk
  DSI_VVACCR_VA9_Pos* = (9)
  DSI_VVACCR_VA9_Msk* = (0x00000001 shl DSI_VVACCR_VA9_Pos) ## !< 0x00000200
  DSI_VVACCR_VA9* = DSI_VVACCR_VA9_Msk
  DSI_VVACCR_VA10_Pos* = (10)
  DSI_VVACCR_VA10_Msk* = (0x00000001 shl DSI_VVACCR_VA10_Pos) ## !< 0x00000400
  DSI_VVACCR_VA10* = DSI_VVACCR_VA10_Msk
  DSI_VVACCR_VA11_Pos* = (11)
  DSI_VVACCR_VA11_Msk* = (0x00000001 shl DSI_VVACCR_VA11_Pos) ## !< 0x00000800
  DSI_VVACCR_VA11* = DSI_VVACCR_VA11_Msk
  DSI_VVACCR_VA12_Pos* = (12)
  DSI_VVACCR_VA12_Msk* = (0x00000001 shl DSI_VVACCR_VA12_Pos) ## !< 0x00001000
  DSI_VVACCR_VA12* = DSI_VVACCR_VA12_Msk
  DSI_VVACCR_VA13_Pos* = (13)
  DSI_VVACCR_VA13_Msk* = (0x00000001 shl DSI_VVACCR_VA13_Pos) ## !< 0x00002000
  DSI_VVACCR_VA13* = DSI_VVACCR_VA13_Msk

## ******************  Bit definition for DSI_TDCCR register  *************

const
  DSI_TDCCR_Bit3DM* = 0x00000003
  DSI_TDCCR_Bit3DM0* = 0x00000001
  DSI_TDCCR_Bit3DM1* = 0x00000002
  DSI_TDCCR_Bit3DF* = 0x0000000C
  DSI_TDCCR_Bit3DF0* = 0x00000004
  DSI_TDCCR_Bit3DF1* = 0x00000008
  DSI_TDCCR_SVS_Pos* = (4)
  DSI_TDCCR_SVS_Msk* = (0x00000001 shl DSI_TDCCR_SVS_Pos) ## !< 0x00000010
  DSI_TDCCR_SVS* = DSI_TDCCR_SVS_Msk
  DSI_TDCCR_RF_Pos* = (5)
  DSI_TDCCR_RF_Msk* = (0x00000001 shl DSI_TDCCR_RF_Pos) ## !< 0x00000020
  DSI_TDCCR_RF* = DSI_TDCCR_RF_Msk
  DSI_TDCCR_S3DC_Pos* = (16)
  DSI_TDCCR_S3DC_Msk* = (0x00000001 shl DSI_TDCCR_S3DC_Pos) ## !< 0x00010000
  DSI_TDCCR_S3DC* = DSI_TDCCR_S3DC_Msk

## ******************  Bit definition for DSI_WCFGR register  **************

const
  DSI_WCFGR_DSIM_Pos* = (0)
  DSI_WCFGR_DSIM_Msk* = (0x00000001 shl DSI_WCFGR_DSIM_Pos) ## !< 0x00000001
  DSI_WCFGR_DSIM* = DSI_WCFGR_DSIM_Msk
  DSI_WCFGR_COLMUX_Pos* = (1)
  DSI_WCFGR_COLMUX_Msk* = (0x00000007 shl DSI_WCFGR_COLMUX_Pos) ## !< 0x0000000E
  DSI_WCFGR_COLMUX* = DSI_WCFGR_COLMUX_Msk
  DSI_WCFGR_COLMUX0_Pos* = (1)
  DSI_WCFGR_COLMUX0_Msk* = (0x00000001 shl DSI_WCFGR_COLMUX0_Pos) ## !< 0x00000002
  DSI_WCFGR_COLMUX0* = DSI_WCFGR_COLMUX0_Msk
  DSI_WCFGR_COLMUX1_Pos* = (2)
  DSI_WCFGR_COLMUX1_Msk* = (0x00000001 shl DSI_WCFGR_COLMUX1_Pos) ## !< 0x00000004
  DSI_WCFGR_COLMUX1* = DSI_WCFGR_COLMUX1_Msk
  DSI_WCFGR_COLMUX2_Pos* = (3)
  DSI_WCFGR_COLMUX2_Msk* = (0x00000001 shl DSI_WCFGR_COLMUX2_Pos) ## !< 0x00000008
  DSI_WCFGR_COLMUX2* = DSI_WCFGR_COLMUX2_Msk
  DSI_WCFGR_TESRC_Pos* = (4)
  DSI_WCFGR_TESRC_Msk* = (0x00000001 shl DSI_WCFGR_TESRC_Pos) ## !< 0x00000010
  DSI_WCFGR_TESRC* = DSI_WCFGR_TESRC_Msk
  DSI_WCFGR_TEPOL_Pos* = (5)
  DSI_WCFGR_TEPOL_Msk* = (0x00000001 shl DSI_WCFGR_TEPOL_Pos) ## !< 0x00000020
  DSI_WCFGR_TEPOL* = DSI_WCFGR_TEPOL_Msk
  DSI_WCFGR_AR_Pos* = (6)
  DSI_WCFGR_AR_Msk* = (0x00000001 shl DSI_WCFGR_AR_Pos) ## !< 0x00000040
  DSI_WCFGR_AR* = DSI_WCFGR_AR_Msk
  DSI_WCFGR_VSPOL_Pos* = (7)
  DSI_WCFGR_VSPOL_Msk* = (0x00000001 shl DSI_WCFGR_VSPOL_Pos) ## !< 0x00000080
  DSI_WCFGR_VSPOL* = DSI_WCFGR_VSPOL_Msk

## ******************  Bit definition for DSI_WCR register  ****************

const
  DSI_WCR_COLM_Pos* = (0)
  DSI_WCR_COLM_Msk* = (0x00000001 shl DSI_WCR_COLM_Pos) ## !< 0x00000001
  DSI_WCR_COLM* = DSI_WCR_COLM_Msk
  DSI_WCR_SHTDN_Pos* = (1)
  DSI_WCR_SHTDN_Msk* = (0x00000001 shl DSI_WCR_SHTDN_Pos) ## !< 0x00000002
  DSI_WCR_SHTDN* = DSI_WCR_SHTDN_Msk
  DSI_WCR_LTDCEN_Pos* = (2)
  DSI_WCR_LTDCEN_Msk* = (0x00000001 shl DSI_WCR_LTDCEN_Pos) ## !< 0x00000004
  DSI_WCR_LTDCEN* = DSI_WCR_LTDCEN_Msk
  DSI_WCR_DSIEN_Pos* = (3)
  DSI_WCR_DSIEN_Msk* = (0x00000001 shl DSI_WCR_DSIEN_Pos) ## !< 0x00000008
  DSI_WCR_DSIEN* = DSI_WCR_DSIEN_Msk

## ******************  Bit definition for DSI_WIER register  ***************

const
  DSI_WIER_TEIE_Pos* = (0)
  DSI_WIER_TEIE_Msk* = (0x00000001 shl DSI_WIER_TEIE_Pos) ## !< 0x00000001
  DSI_WIER_TEIE* = DSI_WIER_TEIE_Msk
  DSI_WIER_ERIE_Pos* = (1)
  DSI_WIER_ERIE_Msk* = (0x00000001 shl DSI_WIER_ERIE_Pos) ## !< 0x00000002
  DSI_WIER_ERIE* = DSI_WIER_ERIE_Msk
  DSI_WIER_PLLLIE_Pos* = (9)
  DSI_WIER_PLLLIE_Msk* = (0x00000001 shl DSI_WIER_PLLLIE_Pos) ## !< 0x00000200
  DSI_WIER_PLLLIE* = DSI_WIER_PLLLIE_Msk
  DSI_WIER_PLLUIE_Pos* = (10)
  DSI_WIER_PLLUIE_Msk* = (0x00000001 shl DSI_WIER_PLLUIE_Pos) ## !< 0x00000400
  DSI_WIER_PLLUIE* = DSI_WIER_PLLUIE_Msk
  DSI_WIER_RRIE_Pos* = (13)
  DSI_WIER_RRIE_Msk* = (0x00000001 shl DSI_WIER_RRIE_Pos) ## !< 0x00002000
  DSI_WIER_RRIE* = DSI_WIER_RRIE_Msk

## ******************  Bit definition for DSI_WISR register  ***************

const
  DSI_WISR_TEIF_Pos* = (0)
  DSI_WISR_TEIF_Msk* = (0x00000001 shl DSI_WISR_TEIF_Pos) ## !< 0x00000001
  DSI_WISR_TEIF* = DSI_WISR_TEIF_Msk
  DSI_WISR_ERIF_Pos* = (1)
  DSI_WISR_ERIF_Msk* = (0x00000001 shl DSI_WISR_ERIF_Pos) ## !< 0x00000002
  DSI_WISR_ERIF* = DSI_WISR_ERIF_Msk
  DSI_WISR_BUSY_Pos* = (2)
  DSI_WISR_BUSY_Msk* = (0x00000001 shl DSI_WISR_BUSY_Pos) ## !< 0x00000004
  DSI_WISR_BUSY* = DSI_WISR_BUSY_Msk
  DSI_WISR_PLLLS_Pos* = (8)
  DSI_WISR_PLLLS_Msk* = (0x00000001 shl DSI_WISR_PLLLS_Pos) ## !< 0x00000100
  DSI_WISR_PLLLS* = DSI_WISR_PLLLS_Msk
  DSI_WISR_PLLLIF_Pos* = (9)
  DSI_WISR_PLLLIF_Msk* = (0x00000001 shl DSI_WISR_PLLLIF_Pos) ## !< 0x00000200
  DSI_WISR_PLLLIF* = DSI_WISR_PLLLIF_Msk
  DSI_WISR_PLLUIF_Pos* = (10)
  DSI_WISR_PLLUIF_Msk* = (0x00000001 shl DSI_WISR_PLLUIF_Pos) ## !< 0x00000400
  DSI_WISR_PLLUIF* = DSI_WISR_PLLUIF_Msk
  DSI_WISR_RRS_Pos* = (12)
  DSI_WISR_RRS_Msk* = (0x00000001 shl DSI_WISR_RRS_Pos) ## !< 0x00001000
  DSI_WISR_RRS* = DSI_WISR_RRS_Msk
  DSI_WISR_RRIF_Pos* = (13)
  DSI_WISR_RRIF_Msk* = (0x00000001 shl DSI_WISR_RRIF_Pos) ## !< 0x00002000
  DSI_WISR_RRIF* = DSI_WISR_RRIF_Msk

## ******************  Bit definition for DSI_WIFCR register  **************

const
  DSI_WIFCR_CTEIF_Pos* = (0)
  DSI_WIFCR_CTEIF_Msk* = (0x00000001 shl DSI_WIFCR_CTEIF_Pos) ## !< 0x00000001
  DSI_WIFCR_CTEIF* = DSI_WIFCR_CTEIF_Msk
  DSI_WIFCR_CERIF_Pos* = (1)
  DSI_WIFCR_CERIF_Msk* = (0x00000001 shl DSI_WIFCR_CERIF_Pos) ## !< 0x00000002
  DSI_WIFCR_CERIF* = DSI_WIFCR_CERIF_Msk
  DSI_WIFCR_CPLLLIF_Pos* = (9)
  DSI_WIFCR_CPLLLIF_Msk* = (0x00000001 shl DSI_WIFCR_CPLLLIF_Pos) ## !< 0x00000200
  DSI_WIFCR_CPLLLIF* = DSI_WIFCR_CPLLLIF_Msk
  DSI_WIFCR_CPLLUIF_Pos* = (10)
  DSI_WIFCR_CPLLUIF_Msk* = (0x00000001 shl DSI_WIFCR_CPLLUIF_Pos) ## !< 0x00000400
  DSI_WIFCR_CPLLUIF* = DSI_WIFCR_CPLLUIF_Msk
  DSI_WIFCR_CRRIF_Pos* = (13)
  DSI_WIFCR_CRRIF_Msk* = (0x00000001 shl DSI_WIFCR_CRRIF_Pos) ## !< 0x00002000
  DSI_WIFCR_CRRIF* = DSI_WIFCR_CRRIF_Msk

## ******************  Bit definition for DSI_WPCR0 register  **************

const
  DSI_WPCR0_UIX4_Pos* = (0)
  DSI_WPCR0_UIX4_Msk* = (0x0000003F shl DSI_WPCR0_UIX4_Pos) ## !< 0x0000003F
  DSI_WPCR0_UIX4* = DSI_WPCR0_UIX4_Msk
  DSI_WPCR0_UIX4_Bit0* = (0x00000001 shl DSI_WPCR0_UIX4_Pos) ## !< 0x00000001
  DSI_WPCR0_UIX4_Bit1* = (0x00000002 shl DSI_WPCR0_UIX4_Pos) ## !< 0x00000002
  DSI_WPCR0_UIX4_Bit2* = (0x00000004 shl DSI_WPCR0_UIX4_Pos) ## !< 0x00000004
  DSI_WPCR0_UIX4_Bit3* = (0x00000008 shl DSI_WPCR0_UIX4_Pos) ## !< 0x00000008
  DSI_WPCR0_UIX4_Bit4* = (0x00000010 shl DSI_WPCR0_UIX4_Pos) ## !< 0x00000010
  DSI_WPCR0_UIX4_Bit5* = (0x00000020 shl DSI_WPCR0_UIX4_Pos) ## !< 0x00000020
  DSI_WPCR0_SWCL_Pos* = (6)
  DSI_WPCR0_SWCL_Msk* = (0x00000001 shl DSI_WPCR0_SWCL_Pos) ## !< 0x00000040
  DSI_WPCR0_SWCL* = DSI_WPCR0_SWCL_Msk
  DSI_WPCR0_SWDL0_Pos* = (7)
  DSI_WPCR0_SWDL0_Msk* = (0x00000001 shl DSI_WPCR0_SWDL0_Pos) ## !< 0x00000080
  DSI_WPCR0_SWDL0* = DSI_WPCR0_SWDL0_Msk
  DSI_WPCR0_SWDL1_Pos* = (8)
  DSI_WPCR0_SWDL1_Msk* = (0x00000001 shl DSI_WPCR0_SWDL1_Pos) ## !< 0x00000100
  DSI_WPCR0_SWDL1* = DSI_WPCR0_SWDL1_Msk
  DSI_WPCR0_HSICL_Pos* = (9)
  DSI_WPCR0_HSICL_Msk* = (0x00000001 shl DSI_WPCR0_HSICL_Pos) ## !< 0x00000200
  DSI_WPCR0_HSICL* = DSI_WPCR0_HSICL_Msk
  DSI_WPCR0_HSIDL0_Pos* = (10)
  DSI_WPCR0_HSIDL0_Msk* = (0x00000001 shl DSI_WPCR0_HSIDL0_Pos) ## !< 0x00000400
  DSI_WPCR0_HSIDL0* = DSI_WPCR0_HSIDL0_Msk
  DSI_WPCR0_HSIDL1_Pos* = (11)
  DSI_WPCR0_HSIDL1_Msk* = (0x00000001 shl DSI_WPCR0_HSIDL1_Pos) ## !< 0x00000800
  DSI_WPCR0_HSIDL1* = DSI_WPCR0_HSIDL1_Msk
  DSI_WPCR0_FTXSMCL_Pos* = (12)
  DSI_WPCR0_FTXSMCL_Msk* = (0x00000001 shl DSI_WPCR0_FTXSMCL_Pos) ## !< 0x00001000
  DSI_WPCR0_FTXSMCL* = DSI_WPCR0_FTXSMCL_Msk
  DSI_WPCR0_FTXSMDL_Pos* = (13)
  DSI_WPCR0_FTXSMDL_Msk* = (0x00000001 shl DSI_WPCR0_FTXSMDL_Pos) ## !< 0x00002000
  DSI_WPCR0_FTXSMDL* = DSI_WPCR0_FTXSMDL_Msk
  DSI_WPCR0_CDOFFDL_Pos* = (14)
  DSI_WPCR0_CDOFFDL_Msk* = (0x00000001 shl DSI_WPCR0_CDOFFDL_Pos) ## !< 0x00004000
  DSI_WPCR0_CDOFFDL* = DSI_WPCR0_CDOFFDL_Msk
  DSI_WPCR0_TDDL_Pos* = (16)
  DSI_WPCR0_TDDL_Msk* = (0x00000001 shl DSI_WPCR0_TDDL_Pos) ## !< 0x00010000
  DSI_WPCR0_TDDL* = DSI_WPCR0_TDDL_Msk
  DSI_WPCR0_PDEN_Pos* = (18)
  DSI_WPCR0_PDEN_Msk* = (0x00000001 shl DSI_WPCR0_PDEN_Pos) ## !< 0x00040000
  DSI_WPCR0_PDEN* = DSI_WPCR0_PDEN_Msk
  DSI_WPCR0_TCLKPREPEN_Pos* = (19)
  DSI_WPCR0_TCLKPREPEN_Msk* = (0x00000001 shl DSI_WPCR0_TCLKPREPEN_Pos) ## !< 0x00080000
  DSI_WPCR0_TCLKPREPEN* = DSI_WPCR0_TCLKPREPEN_Msk
  DSI_WPCR0_TCLKZEROEN_Pos* = (20)
  DSI_WPCR0_TCLKZEROEN_Msk* = (0x00000001 shl DSI_WPCR0_TCLKZEROEN_Pos) ## !< 0x00100000
  DSI_WPCR0_TCLKZEROEN* = DSI_WPCR0_TCLKZEROEN_Msk
  DSI_WPCR0_THSPREPEN_Pos* = (21)
  DSI_WPCR0_THSPREPEN_Msk* = (0x00000001 shl DSI_WPCR0_THSPREPEN_Pos) ## !< 0x00200000
  DSI_WPCR0_THSPREPEN* = DSI_WPCR0_THSPREPEN_Msk
  DSI_WPCR0_THSTRAILEN_Pos* = (22)
  DSI_WPCR0_THSTRAILEN_Msk* = (0x00000001 shl DSI_WPCR0_THSTRAILEN_Pos) ## !< 0x00400000
  DSI_WPCR0_THSTRAILEN* = DSI_WPCR0_THSTRAILEN_Msk
  DSI_WPCR0_THSZEROEN_Pos* = (23)
  DSI_WPCR0_THSZEROEN_Msk* = (0x00000001 shl DSI_WPCR0_THSZEROEN_Pos) ## !< 0x00800000
  DSI_WPCR0_THSZEROEN* = DSI_WPCR0_THSZEROEN_Msk
  DSI_WPCR0_TLPXDEN_Pos* = (24)
  DSI_WPCR0_TLPXDEN_Msk* = (0x00000001 shl DSI_WPCR0_TLPXDEN_Pos) ## !< 0x01000000
  DSI_WPCR0_TLPXDEN* = DSI_WPCR0_TLPXDEN_Msk
  DSI_WPCR0_THSEXITEN_Pos* = (25)
  DSI_WPCR0_THSEXITEN_Msk* = (0x00000001 shl DSI_WPCR0_THSEXITEN_Pos) ## !< 0x02000000
  DSI_WPCR0_THSEXITEN* = DSI_WPCR0_THSEXITEN_Msk
  DSI_WPCR0_TLPXCEN_Pos* = (26)
  DSI_WPCR0_TLPXCEN_Msk* = (0x00000001 shl DSI_WPCR0_TLPXCEN_Pos) ## !< 0x04000000
  DSI_WPCR0_TLPXCEN* = DSI_WPCR0_TLPXCEN_Msk
  DSI_WPCR0_TCLKPOSTEN_Pos* = (27)
  DSI_WPCR0_TCLKPOSTEN_Msk* = (0x00000001 shl DSI_WPCR0_TCLKPOSTEN_Pos) ## !< 0x08000000
  DSI_WPCR0_TCLKPOSTEN* = DSI_WPCR0_TCLKPOSTEN_Msk

## ******************  Bit definition for DSI_WPCR1 register  **************

const
  DSI_WPCR1_HSTXDCL_Pos* = (0)
  DSI_WPCR1_HSTXDCL_Msk* = (0x00000003 shl DSI_WPCR1_HSTXDCL_Pos) ## !< 0x00000003
  DSI_WPCR1_HSTXDCL* = DSI_WPCR1_HSTXDCL_Msk
  DSI_WPCR1_HSTXDCL0_Pos* = (0)
  DSI_WPCR1_HSTXDCL0_Msk* = (0x00000001 shl DSI_WPCR1_HSTXDCL0_Pos) ## !< 0x00000001
  DSI_WPCR1_HSTXDCL0* = DSI_WPCR1_HSTXDCL0_Msk
  DSI_WPCR1_HSTXDCL1_Pos* = (1)
  DSI_WPCR1_HSTXDCL1_Msk* = (0x00000001 shl DSI_WPCR1_HSTXDCL1_Pos) ## !< 0x00000002
  DSI_WPCR1_HSTXDCL1* = DSI_WPCR1_HSTXDCL1_Msk
  DSI_WPCR1_HSTXDDL_Pos* = (2)
  DSI_WPCR1_HSTXDDL_Msk* = (0x00000003 shl DSI_WPCR1_HSTXDDL_Pos) ## !< 0x0000000C
  DSI_WPCR1_HSTXDDL* = DSI_WPCR1_HSTXDDL_Msk
  DSI_WPCR1_HSTXDDL0_Pos* = (2)
  DSI_WPCR1_HSTXDDL0_Msk* = (0x00000001 shl DSI_WPCR1_HSTXDDL0_Pos) ## !< 0x00000004
  DSI_WPCR1_HSTXDDL0* = DSI_WPCR1_HSTXDDL0_Msk
  DSI_WPCR1_HSTXDDL1_Pos* = (3)
  DSI_WPCR1_HSTXDDL1_Msk* = (0x00000001 shl DSI_WPCR1_HSTXDDL1_Pos) ## !< 0x00000008
  DSI_WPCR1_HSTXDDL1* = DSI_WPCR1_HSTXDDL1_Msk
  DSI_WPCR1_LPSRCCL_Pos* = (6)
  DSI_WPCR1_LPSRCCL_Msk* = (0x00000003 shl DSI_WPCR1_LPSRCCL_Pos) ## !< 0x000000C0
  DSI_WPCR1_LPSRCCL* = DSI_WPCR1_LPSRCCL_Msk
  DSI_WPCR1_LPSRCCL0_Pos* = (6)
  DSI_WPCR1_LPSRCCL0_Msk* = (0x00000001 shl DSI_WPCR1_LPSRCCL0_Pos) ## !< 0x00000040
  DSI_WPCR1_LPSRCCL0* = DSI_WPCR1_LPSRCCL0_Msk
  DSI_WPCR1_LPSRCCL1_Pos* = (7)
  DSI_WPCR1_LPSRCCL1_Msk* = (0x00000001 shl DSI_WPCR1_LPSRCCL1_Pos) ## !< 0x00000080
  DSI_WPCR1_LPSRCCL1* = DSI_WPCR1_LPSRCCL1_Msk
  DSI_WPCR1_LPSRCDL_Pos* = (8)
  DSI_WPCR1_LPSRCDL_Msk* = (0x00000003 shl DSI_WPCR1_LPSRCDL_Pos) ## !< 0x00000300
  DSI_WPCR1_LPSRCDL* = DSI_WPCR1_LPSRCDL_Msk
  DSI_WPCR1_LPSRCDL0_Pos* = (8)
  DSI_WPCR1_LPSRCDL0_Msk* = (0x00000001 shl DSI_WPCR1_LPSRCDL0_Pos) ## !< 0x00000100
  DSI_WPCR1_LPSRCDL0* = DSI_WPCR1_LPSRCDL0_Msk
  DSI_WPCR1_LPSRCDL1_Pos* = (9)
  DSI_WPCR1_LPSRCDL1_Msk* = (0x00000001 shl DSI_WPCR1_LPSRCDL1_Pos) ## !< 0x00000200
  DSI_WPCR1_LPSRCDL1* = DSI_WPCR1_LPSRCDL1_Msk
  DSI_WPCR1_SDDC_Pos* = (12)
  DSI_WPCR1_SDDC_Msk* = (0x00000001 shl DSI_WPCR1_SDDC_Pos) ## !< 0x00001000
  DSI_WPCR1_SDDC* = DSI_WPCR1_SDDC_Msk
  DSI_WPCR1_LPRXVCDL_Pos* = (14)
  DSI_WPCR1_LPRXVCDL_Msk* = (0x00000003 shl DSI_WPCR1_LPRXVCDL_Pos) ## !< 0x0000C000
  DSI_WPCR1_LPRXVCDL* = DSI_WPCR1_LPRXVCDL_Msk
  DSI_WPCR1_LPRXVCDL0_Pos* = (14)
  DSI_WPCR1_LPRXVCDL0_Msk* = (0x00000001 shl DSI_WPCR1_LPRXVCDL0_Pos) ## !< 0x00004000
  DSI_WPCR1_LPRXVCDL0* = DSI_WPCR1_LPRXVCDL0_Msk
  DSI_WPCR1_LPRXVCDL1_Pos* = (15)
  DSI_WPCR1_LPRXVCDL1_Msk* = (0x00000001 shl DSI_WPCR1_LPRXVCDL1_Pos) ## !< 0x00008000
  DSI_WPCR1_LPRXVCDL1* = DSI_WPCR1_LPRXVCDL1_Msk
  DSI_WPCR1_HSTXSRCCL_Pos* = (16)
  DSI_WPCR1_HSTXSRCCL_Msk* = (0x00000003 shl DSI_WPCR1_HSTXSRCCL_Pos) ## !< 0x00030000
  DSI_WPCR1_HSTXSRCCL* = DSI_WPCR1_HSTXSRCCL_Msk
  DSI_WPCR1_HSTXSRCCL0_Pos* = (16)
  DSI_WPCR1_HSTXSRCCL0_Msk* = (0x00000001 shl DSI_WPCR1_HSTXSRCCL0_Pos) ## !< 0x00010000
  DSI_WPCR1_HSTXSRCCL0* = DSI_WPCR1_HSTXSRCCL0_Msk
  DSI_WPCR1_HSTXSRCCL1_Pos* = (17)
  DSI_WPCR1_HSTXSRCCL1_Msk* = (0x00000001 shl DSI_WPCR1_HSTXSRCCL1_Pos) ## !< 0x00020000
  DSI_WPCR1_HSTXSRCCL1* = DSI_WPCR1_HSTXSRCCL1_Msk
  DSI_WPCR1_HSTXSRCDL_Pos* = (18)
  DSI_WPCR1_HSTXSRCDL_Msk* = (0x00000003 shl DSI_WPCR1_HSTXSRCDL_Pos) ## !< 0x000C0000
  DSI_WPCR1_HSTXSRCDL* = DSI_WPCR1_HSTXSRCDL_Msk
  DSI_WPCR1_HSTXSRCDL0_Pos* = (18)
  DSI_WPCR1_HSTXSRCDL0_Msk* = (0x00000001 shl DSI_WPCR1_HSTXSRCDL0_Pos) ## !< 0x00040000
  DSI_WPCR1_HSTXSRCDL0* = DSI_WPCR1_HSTXSRCDL0_Msk
  DSI_WPCR1_HSTXSRCDL1_Pos* = (19)
  DSI_WPCR1_HSTXSRCDL1_Msk* = (0x00000001 shl DSI_WPCR1_HSTXSRCDL1_Pos) ## !< 0x00080000
  DSI_WPCR1_HSTXSRCDL1* = DSI_WPCR1_HSTXSRCDL1_Msk
  DSI_WPCR1_FLPRXLPM_Pos* = (22)
  DSI_WPCR1_FLPRXLPM_Msk* = (0x00000001 shl DSI_WPCR1_FLPRXLPM_Pos) ## !< 0x00400000
  DSI_WPCR1_FLPRXLPM* = DSI_WPCR1_FLPRXLPM_Msk
  DSI_WPCR1_LPRXFT_Pos* = (25)
  DSI_WPCR1_LPRXFT_Msk* = (0x00000003 shl DSI_WPCR1_LPRXFT_Pos) ## !< 0x06000000
  DSI_WPCR1_LPRXFT* = DSI_WPCR1_LPRXFT_Msk
  DSI_WPCR1_LPRXFT0_Pos* = (25)
  DSI_WPCR1_LPRXFT0_Msk* = (0x00000001 shl DSI_WPCR1_LPRXFT0_Pos) ## !< 0x02000000
  DSI_WPCR1_LPRXFT0* = DSI_WPCR1_LPRXFT0_Msk
  DSI_WPCR1_LPRXFT1_Pos* = (26)
  DSI_WPCR1_LPRXFT1_Msk* = (0x00000001 shl DSI_WPCR1_LPRXFT1_Pos) ## !< 0x04000000
  DSI_WPCR1_LPRXFT1* = DSI_WPCR1_LPRXFT1_Msk

## ******************  Bit definition for DSI_WPCR2 register  **************

const
  DSI_WPCR2_TCLKPREP_Pos* = (0)
  DSI_WPCR2_TCLKPREP_Msk* = (0x000000FF shl DSI_WPCR2_TCLKPREP_Pos) ## !< 0x000000FF
  DSI_WPCR2_TCLKPREP* = DSI_WPCR2_TCLKPREP_Msk
  DSI_WPCR2_TCLKPREP0_Pos* = (0)
  DSI_WPCR2_TCLKPREP0_Msk* = (0x00000001 shl DSI_WPCR2_TCLKPREP0_Pos) ## !< 0x00000001
  DSI_WPCR2_TCLKPREP0* = DSI_WPCR2_TCLKPREP0_Msk
  DSI_WPCR2_TCLKPREP1_Pos* = (1)
  DSI_WPCR2_TCLKPREP1_Msk* = (0x00000001 shl DSI_WPCR2_TCLKPREP1_Pos) ## !< 0x00000002
  DSI_WPCR2_TCLKPREP1* = DSI_WPCR2_TCLKPREP1_Msk
  DSI_WPCR2_TCLKPREP2_Pos* = (2)
  DSI_WPCR2_TCLKPREP2_Msk* = (0x00000001 shl DSI_WPCR2_TCLKPREP2_Pos) ## !< 0x00000004
  DSI_WPCR2_TCLKPREP2* = DSI_WPCR2_TCLKPREP2_Msk
  DSI_WPCR2_TCLKPREP3_Pos* = (3)
  DSI_WPCR2_TCLKPREP3_Msk* = (0x00000001 shl DSI_WPCR2_TCLKPREP3_Pos) ## !< 0x00000008
  DSI_WPCR2_TCLKPREP3* = DSI_WPCR2_TCLKPREP3_Msk
  DSI_WPCR2_TCLKPREP4_Pos* = (4)
  DSI_WPCR2_TCLKPREP4_Msk* = (0x00000001 shl DSI_WPCR2_TCLKPREP4_Pos) ## !< 0x00000010
  DSI_WPCR2_TCLKPREP4* = DSI_WPCR2_TCLKPREP4_Msk
  DSI_WPCR2_TCLKPREP5_Pos* = (5)
  DSI_WPCR2_TCLKPREP5_Msk* = (0x00000001 shl DSI_WPCR2_TCLKPREP5_Pos) ## !< 0x00000020
  DSI_WPCR2_TCLKPREP5* = DSI_WPCR2_TCLKPREP5_Msk
  DSI_WPCR2_TCLKPREP6_Pos* = (6)
  DSI_WPCR2_TCLKPREP6_Msk* = (0x00000001 shl DSI_WPCR2_TCLKPREP6_Pos) ## !< 0x00000040
  DSI_WPCR2_TCLKPREP6* = DSI_WPCR2_TCLKPREP6_Msk
  DSI_WPCR2_TCLKPREP7_Pos* = (7)
  DSI_WPCR2_TCLKPREP7_Msk* = (0x00000001 shl DSI_WPCR2_TCLKPREP7_Pos) ## !< 0x00000080
  DSI_WPCR2_TCLKPREP7* = DSI_WPCR2_TCLKPREP7_Msk
  DSI_WPCR2_TCLKZERO_Pos* = (8)
  DSI_WPCR2_TCLKZERO_Msk* = (0x000000FF shl DSI_WPCR2_TCLKZERO_Pos) ## !< 0x0000FF00
  DSI_WPCR2_TCLKZERO* = DSI_WPCR2_TCLKZERO_Msk
  DSI_WPCR2_TCLKZERO0_Pos* = (8)
  DSI_WPCR2_TCLKZERO0_Msk* = (0x00000001 shl DSI_WPCR2_TCLKZERO0_Pos) ## !< 0x00000100
  DSI_WPCR2_TCLKZERO0* = DSI_WPCR2_TCLKZERO0_Msk
  DSI_WPCR2_TCLKZERO1_Pos* = (9)
  DSI_WPCR2_TCLKZERO1_Msk* = (0x00000001 shl DSI_WPCR2_TCLKZERO1_Pos) ## !< 0x00000200
  DSI_WPCR2_TCLKZERO1* = DSI_WPCR2_TCLKZERO1_Msk
  DSI_WPCR2_TCLKZERO2_Pos* = (10)
  DSI_WPCR2_TCLKZERO2_Msk* = (0x00000001 shl DSI_WPCR2_TCLKZERO2_Pos) ## !< 0x00000400
  DSI_WPCR2_TCLKZERO2* = DSI_WPCR2_TCLKZERO2_Msk
  DSI_WPCR2_TCLKZERO3_Pos* = (11)
  DSI_WPCR2_TCLKZERO3_Msk* = (0x00000001 shl DSI_WPCR2_TCLKZERO3_Pos) ## !< 0x00000800
  DSI_WPCR2_TCLKZERO3* = DSI_WPCR2_TCLKZERO3_Msk
  DSI_WPCR2_TCLKZERO4_Pos* = (12)
  DSI_WPCR2_TCLKZERO4_Msk* = (0x00000001 shl DSI_WPCR2_TCLKZERO4_Pos) ## !< 0x00001000
  DSI_WPCR2_TCLKZERO4* = DSI_WPCR2_TCLKZERO4_Msk
  DSI_WPCR2_TCLKZERO5_Pos* = (13)
  DSI_WPCR2_TCLKZERO5_Msk* = (0x00000001 shl DSI_WPCR2_TCLKZERO5_Pos) ## !< 0x00002000
  DSI_WPCR2_TCLKZERO5* = DSI_WPCR2_TCLKZERO5_Msk
  DSI_WPCR2_TCLKZERO6_Pos* = (14)
  DSI_WPCR2_TCLKZERO6_Msk* = (0x00000001 shl DSI_WPCR2_TCLKZERO6_Pos) ## !< 0x00004000
  DSI_WPCR2_TCLKZERO6* = DSI_WPCR2_TCLKZERO6_Msk
  DSI_WPCR2_TCLKZERO7_Pos* = (15)
  DSI_WPCR2_TCLKZERO7_Msk* = (0x00000001 shl DSI_WPCR2_TCLKZERO7_Pos) ## !< 0x00008000
  DSI_WPCR2_TCLKZERO7* = DSI_WPCR2_TCLKZERO7_Msk
  DSI_WPCR2_THSPREP_Pos* = (16)
  DSI_WPCR2_THSPREP_Msk* = (0x000000FF shl DSI_WPCR2_THSPREP_Pos) ## !< 0x00FF0000
  DSI_WPCR2_THSPREP* = DSI_WPCR2_THSPREP_Msk
  DSI_WPCR2_THSPREP0_Pos* = (16)
  DSI_WPCR2_THSPREP0_Msk* = (0x00000001 shl DSI_WPCR2_THSPREP0_Pos) ## !< 0x00010000
  DSI_WPCR2_THSPREP0* = DSI_WPCR2_THSPREP0_Msk
  DSI_WPCR2_THSPREP1_Pos* = (17)
  DSI_WPCR2_THSPREP1_Msk* = (0x00000001 shl DSI_WPCR2_THSPREP1_Pos) ## !< 0x00020000
  DSI_WPCR2_THSPREP1* = DSI_WPCR2_THSPREP1_Msk
  DSI_WPCR2_THSPREP2_Pos* = (18)
  DSI_WPCR2_THSPREP2_Msk* = (0x00000001 shl DSI_WPCR2_THSPREP2_Pos) ## !< 0x00040000
  DSI_WPCR2_THSPREP2* = DSI_WPCR2_THSPREP2_Msk
  DSI_WPCR2_THSPREP3_Pos* = (19)
  DSI_WPCR2_THSPREP3_Msk* = (0x00000001 shl DSI_WPCR2_THSPREP3_Pos) ## !< 0x00080000
  DSI_WPCR2_THSPREP3* = DSI_WPCR2_THSPREP3_Msk
  DSI_WPCR2_THSPREP4_Pos* = (20)
  DSI_WPCR2_THSPREP4_Msk* = (0x00000001 shl DSI_WPCR2_THSPREP4_Pos) ## !< 0x00100000
  DSI_WPCR2_THSPREP4* = DSI_WPCR2_THSPREP4_Msk
  DSI_WPCR2_THSPREP5_Pos* = (21)
  DSI_WPCR2_THSPREP5_Msk* = (0x00000001 shl DSI_WPCR2_THSPREP5_Pos) ## !< 0x00200000
  DSI_WPCR2_THSPREP5* = DSI_WPCR2_THSPREP5_Msk
  DSI_WPCR2_THSPREP6_Pos* = (22)
  DSI_WPCR2_THSPREP6_Msk* = (0x00000001 shl DSI_WPCR2_THSPREP6_Pos) ## !< 0x00400000
  DSI_WPCR2_THSPREP6* = DSI_WPCR2_THSPREP6_Msk
  DSI_WPCR2_THSPREP7_Pos* = (23)
  DSI_WPCR2_THSPREP7_Msk* = (0x00000001 shl DSI_WPCR2_THSPREP7_Pos) ## !< 0x00800000
  DSI_WPCR2_THSPREP7* = DSI_WPCR2_THSPREP7_Msk
  DSI_WPCR2_THSTRAIL_Pos* = (24)
  DSI_WPCR2_THSTRAIL_Msk* = (0x000000FF shl DSI_WPCR2_THSTRAIL_Pos) ## !< 0xFF000000
  DSI_WPCR2_THSTRAIL* = DSI_WPCR2_THSTRAIL_Msk
  DSI_WPCR2_THSTRAIL0_Pos* = (24)
  DSI_WPCR2_THSTRAIL0_Msk* = (0x00000001 shl DSI_WPCR2_THSTRAIL0_Pos) ## !< 0x01000000
  DSI_WPCR2_THSTRAIL0* = DSI_WPCR2_THSTRAIL0_Msk
  DSI_WPCR2_THSTRAIL1_Pos* = (25)
  DSI_WPCR2_THSTRAIL1_Msk* = (0x00000001 shl DSI_WPCR2_THSTRAIL1_Pos) ## !< 0x02000000
  DSI_WPCR2_THSTRAIL1* = DSI_WPCR2_THSTRAIL1_Msk
  DSI_WPCR2_THSTRAIL2_Pos* = (26)
  DSI_WPCR2_THSTRAIL2_Msk* = (0x00000001 shl DSI_WPCR2_THSTRAIL2_Pos) ## !< 0x04000000
  DSI_WPCR2_THSTRAIL2* = DSI_WPCR2_THSTRAIL2_Msk
  DSI_WPCR2_THSTRAIL3_Pos* = (27)
  DSI_WPCR2_THSTRAIL3_Msk* = (0x00000001 shl DSI_WPCR2_THSTRAIL3_Pos) ## !< 0x08000000
  DSI_WPCR2_THSTRAIL3* = DSI_WPCR2_THSTRAIL3_Msk
  DSI_WPCR2_THSTRAIL4_Pos* = (28)
  DSI_WPCR2_THSTRAIL4_Msk* = (0x00000001 shl DSI_WPCR2_THSTRAIL4_Pos) ## !< 0x10000000
  DSI_WPCR2_THSTRAIL4* = DSI_WPCR2_THSTRAIL4_Msk
  DSI_WPCR2_THSTRAIL5_Pos* = (29)
  DSI_WPCR2_THSTRAIL5_Msk* = (0x00000001 shl DSI_WPCR2_THSTRAIL5_Pos) ## !< 0x20000000
  DSI_WPCR2_THSTRAIL5* = DSI_WPCR2_THSTRAIL5_Msk
  DSI_WPCR2_THSTRAIL6_Pos* = (30)
  DSI_WPCR2_THSTRAIL6_Msk* = (0x00000001 shl DSI_WPCR2_THSTRAIL6_Pos) ## !< 0x40000000
  DSI_WPCR2_THSTRAIL6* = DSI_WPCR2_THSTRAIL6_Msk
  DSI_WPCR2_THSTRAIL7_Pos* = (31)
  DSI_WPCR2_THSTRAIL7_Msk* = (0x00000001 shl DSI_WPCR2_THSTRAIL7_Pos) ## !< 0x80000000
  DSI_WPCR2_THSTRAIL7* = DSI_WPCR2_THSTRAIL7_Msk

## ******************  Bit definition for DSI_WPCR3 register  **************

const
  DSI_WPCR3_THSZERO_Pos* = (0)
  DSI_WPCR3_THSZERO_Msk* = (0x000000FF shl DSI_WPCR3_THSZERO_Pos) ## !< 0x000000FF
  DSI_WPCR3_THSZERO* = DSI_WPCR3_THSZERO_Msk
  DSI_WPCR3_THSZERO0_Pos* = (0)
  DSI_WPCR3_THSZERO0_Msk* = (0x00000001 shl DSI_WPCR3_THSZERO0_Pos) ## !< 0x00000001
  DSI_WPCR3_THSZERO0* = DSI_WPCR3_THSZERO0_Msk
  DSI_WPCR3_THSZERO1_Pos* = (1)
  DSI_WPCR3_THSZERO1_Msk* = (0x00000001 shl DSI_WPCR3_THSZERO1_Pos) ## !< 0x00000002
  DSI_WPCR3_THSZERO1* = DSI_WPCR3_THSZERO1_Msk
  DSI_WPCR3_THSZERO2_Pos* = (2)
  DSI_WPCR3_THSZERO2_Msk* = (0x00000001 shl DSI_WPCR3_THSZERO2_Pos) ## !< 0x00000004
  DSI_WPCR3_THSZERO2* = DSI_WPCR3_THSZERO2_Msk
  DSI_WPCR3_THSZERO3_Pos* = (3)
  DSI_WPCR3_THSZERO3_Msk* = (0x00000001 shl DSI_WPCR3_THSZERO3_Pos) ## !< 0x00000008
  DSI_WPCR3_THSZERO3* = DSI_WPCR3_THSZERO3_Msk
  DSI_WPCR3_THSZERO4_Pos* = (4)
  DSI_WPCR3_THSZERO4_Msk* = (0x00000001 shl DSI_WPCR3_THSZERO4_Pos) ## !< 0x00000010
  DSI_WPCR3_THSZERO4* = DSI_WPCR3_THSZERO4_Msk
  DSI_WPCR3_THSZERO5_Pos* = (5)
  DSI_WPCR3_THSZERO5_Msk* = (0x00000001 shl DSI_WPCR3_THSZERO5_Pos) ## !< 0x00000020
  DSI_WPCR3_THSZERO5* = DSI_WPCR3_THSZERO5_Msk
  DSI_WPCR3_THSZERO6_Pos* = (6)
  DSI_WPCR3_THSZERO6_Msk* = (0x00000001 shl DSI_WPCR3_THSZERO6_Pos) ## !< 0x00000040
  DSI_WPCR3_THSZERO6* = DSI_WPCR3_THSZERO6_Msk
  DSI_WPCR3_THSZERO7_Pos* = (7)
  DSI_WPCR3_THSZERO7_Msk* = (0x00000001 shl DSI_WPCR3_THSZERO7_Pos) ## !< 0x00000080
  DSI_WPCR3_THSZERO7* = DSI_WPCR3_THSZERO7_Msk
  DSI_WPCR3_TLPXD_Pos* = (8)
  DSI_WPCR3_TLPXD_Msk* = (0x000000FF shl DSI_WPCR3_TLPXD_Pos) ## !< 0x0000FF00
  DSI_WPCR3_TLPXD* = DSI_WPCR3_TLPXD_Msk
  DSI_WPCR3_TLPXD0_Pos* = (8)
  DSI_WPCR3_TLPXD0_Msk* = (0x00000001 shl DSI_WPCR3_TLPXD0_Pos) ## !< 0x00000100
  DSI_WPCR3_TLPXD0* = DSI_WPCR3_TLPXD0_Msk
  DSI_WPCR3_TLPXD1_Pos* = (9)
  DSI_WPCR3_TLPXD1_Msk* = (0x00000001 shl DSI_WPCR3_TLPXD1_Pos) ## !< 0x00000200
  DSI_WPCR3_TLPXD1* = DSI_WPCR3_TLPXD1_Msk
  DSI_WPCR3_TLPXD2_Pos* = (10)
  DSI_WPCR3_TLPXD2_Msk* = (0x00000001 shl DSI_WPCR3_TLPXD2_Pos) ## !< 0x00000400
  DSI_WPCR3_TLPXD2* = DSI_WPCR3_TLPXD2_Msk
  DSI_WPCR3_TLPXD3_Pos* = (11)
  DSI_WPCR3_TLPXD3_Msk* = (0x00000001 shl DSI_WPCR3_TLPXD3_Pos) ## !< 0x00000800
  DSI_WPCR3_TLPXD3* = DSI_WPCR3_TLPXD3_Msk
  DSI_WPCR3_TLPXD4_Pos* = (12)
  DSI_WPCR3_TLPXD4_Msk* = (0x00000001 shl DSI_WPCR3_TLPXD4_Pos) ## !< 0x00001000
  DSI_WPCR3_TLPXD4* = DSI_WPCR3_TLPXD4_Msk
  DSI_WPCR3_TLPXD5_Pos* = (13)
  DSI_WPCR3_TLPXD5_Msk* = (0x00000001 shl DSI_WPCR3_TLPXD5_Pos) ## !< 0x00002000
  DSI_WPCR3_TLPXD5* = DSI_WPCR3_TLPXD5_Msk
  DSI_WPCR3_TLPXD6_Pos* = (14)
  DSI_WPCR3_TLPXD6_Msk* = (0x00000001 shl DSI_WPCR3_TLPXD6_Pos) ## !< 0x00004000
  DSI_WPCR3_TLPXD6* = DSI_WPCR3_TLPXD6_Msk
  DSI_WPCR3_TLPXD7_Pos* = (15)
  DSI_WPCR3_TLPXD7_Msk* = (0x00000001 shl DSI_WPCR3_TLPXD7_Pos) ## !< 0x00008000
  DSI_WPCR3_TLPXD7* = DSI_WPCR3_TLPXD7_Msk
  DSI_WPCR3_THSEXIT_Pos* = (16)
  DSI_WPCR3_THSEXIT_Msk* = (0x000000FF shl DSI_WPCR3_THSEXIT_Pos) ## !< 0x00FF0000
  DSI_WPCR3_THSEXIT* = DSI_WPCR3_THSEXIT_Msk
  DSI_WPCR3_THSEXIT0_Pos* = (16)
  DSI_WPCR3_THSEXIT0_Msk* = (0x00000001 shl DSI_WPCR3_THSEXIT0_Pos) ## !< 0x00010000
  DSI_WPCR3_THSEXIT0* = DSI_WPCR3_THSEXIT0_Msk
  DSI_WPCR3_THSEXIT1_Pos* = (17)
  DSI_WPCR3_THSEXIT1_Msk* = (0x00000001 shl DSI_WPCR3_THSEXIT1_Pos) ## !< 0x00020000
  DSI_WPCR3_THSEXIT1* = DSI_WPCR3_THSEXIT1_Msk
  DSI_WPCR3_THSEXIT2_Pos* = (18)
  DSI_WPCR3_THSEXIT2_Msk* = (0x00000001 shl DSI_WPCR3_THSEXIT2_Pos) ## !< 0x00040000
  DSI_WPCR3_THSEXIT2* = DSI_WPCR3_THSEXIT2_Msk
  DSI_WPCR3_THSEXIT3_Pos* = (19)
  DSI_WPCR3_THSEXIT3_Msk* = (0x00000001 shl DSI_WPCR3_THSEXIT3_Pos) ## !< 0x00080000
  DSI_WPCR3_THSEXIT3* = DSI_WPCR3_THSEXIT3_Msk
  DSI_WPCR3_THSEXIT4_Pos* = (20)
  DSI_WPCR3_THSEXIT4_Msk* = (0x00000001 shl DSI_WPCR3_THSEXIT4_Pos) ## !< 0x00100000
  DSI_WPCR3_THSEXIT4* = DSI_WPCR3_THSEXIT4_Msk
  DSI_WPCR3_THSEXIT5_Pos* = (21)
  DSI_WPCR3_THSEXIT5_Msk* = (0x00000001 shl DSI_WPCR3_THSEXIT5_Pos) ## !< 0x00200000
  DSI_WPCR3_THSEXIT5* = DSI_WPCR3_THSEXIT5_Msk
  DSI_WPCR3_THSEXIT6_Pos* = (22)
  DSI_WPCR3_THSEXIT6_Msk* = (0x00000001 shl DSI_WPCR3_THSEXIT6_Pos) ## !< 0x00400000
  DSI_WPCR3_THSEXIT6* = DSI_WPCR3_THSEXIT6_Msk
  DSI_WPCR3_THSEXIT7_Pos* = (23)
  DSI_WPCR3_THSEXIT7_Msk* = (0x00000001 shl DSI_WPCR3_THSEXIT7_Pos) ## !< 0x00800000
  DSI_WPCR3_THSEXIT7* = DSI_WPCR3_THSEXIT7_Msk
  DSI_WPCR3_TLPXC_Pos* = (24)
  DSI_WPCR3_TLPXC_Msk* = (0x000000FF shl DSI_WPCR3_TLPXC_Pos) ## !< 0xFF000000
  DSI_WPCR3_TLPXC* = DSI_WPCR3_TLPXC_Msk
  DSI_WPCR3_TLPXC0_Pos* = (24)
  DSI_WPCR3_TLPXC0_Msk* = (0x00000001 shl DSI_WPCR3_TLPXC0_Pos) ## !< 0x01000000
  DSI_WPCR3_TLPXC0* = DSI_WPCR3_TLPXC0_Msk
  DSI_WPCR3_TLPXC1_Pos* = (25)
  DSI_WPCR3_TLPXC1_Msk* = (0x00000001 shl DSI_WPCR3_TLPXC1_Pos) ## !< 0x02000000
  DSI_WPCR3_TLPXC1* = DSI_WPCR3_TLPXC1_Msk
  DSI_WPCR3_TLPXC2_Pos* = (26)
  DSI_WPCR3_TLPXC2_Msk* = (0x00000001 shl DSI_WPCR3_TLPXC2_Pos) ## !< 0x04000000
  DSI_WPCR3_TLPXC2* = DSI_WPCR3_TLPXC2_Msk
  DSI_WPCR3_TLPXC3_Pos* = (27)
  DSI_WPCR3_TLPXC3_Msk* = (0x00000001 shl DSI_WPCR3_TLPXC3_Pos) ## !< 0x08000000
  DSI_WPCR3_TLPXC3* = DSI_WPCR3_TLPXC3_Msk
  DSI_WPCR3_TLPXC4_Pos* = (28)
  DSI_WPCR3_TLPXC4_Msk* = (0x00000001 shl DSI_WPCR3_TLPXC4_Pos) ## !< 0x10000000
  DSI_WPCR3_TLPXC4* = DSI_WPCR3_TLPXC4_Msk
  DSI_WPCR3_TLPXC5_Pos* = (29)
  DSI_WPCR3_TLPXC5_Msk* = (0x00000001 shl DSI_WPCR3_TLPXC5_Pos) ## !< 0x20000000
  DSI_WPCR3_TLPXC5* = DSI_WPCR3_TLPXC5_Msk
  DSI_WPCR3_TLPXC6_Pos* = (30)
  DSI_WPCR3_TLPXC6_Msk* = (0x00000001 shl DSI_WPCR3_TLPXC6_Pos) ## !< 0x40000000
  DSI_WPCR3_TLPXC6* = DSI_WPCR3_TLPXC6_Msk
  DSI_WPCR3_TLPXC7_Pos* = (31)
  DSI_WPCR3_TLPXC7_Msk* = (0x00000001 shl DSI_WPCR3_TLPXC7_Pos) ## !< 0x80000000
  DSI_WPCR3_TLPXC7* = DSI_WPCR3_TLPXC7_Msk

## ******************  Bit definition for DSI_WPCR4 register  **************

const
  DSI_WPCR4_TCLKPOST_Pos* = (0)
  DSI_WPCR4_TCLKPOST_Msk* = (0x000000FF shl DSI_WPCR4_TCLKPOST_Pos) ## !< 0x000000FF
  DSI_WPCR4_TCLKPOST* = DSI_WPCR4_TCLKPOST_Msk
  DSI_WPCR4_TCLKPOST0_Pos* = (0)
  DSI_WPCR4_TCLKPOST0_Msk* = (0x00000001 shl DSI_WPCR4_TCLKPOST0_Pos) ## !< 0x00000001
  DSI_WPCR4_TCLKPOST0* = DSI_WPCR4_TCLKPOST0_Msk
  DSI_WPCR4_TCLKPOST1_Pos* = (1)
  DSI_WPCR4_TCLKPOST1_Msk* = (0x00000001 shl DSI_WPCR4_TCLKPOST1_Pos) ## !< 0x00000002
  DSI_WPCR4_TCLKPOST1* = DSI_WPCR4_TCLKPOST1_Msk
  DSI_WPCR4_TCLKPOST2_Pos* = (2)
  DSI_WPCR4_TCLKPOST2_Msk* = (0x00000001 shl DSI_WPCR4_TCLKPOST2_Pos) ## !< 0x00000004
  DSI_WPCR4_TCLKPOST2* = DSI_WPCR4_TCLKPOST2_Msk
  DSI_WPCR4_TCLKPOST3_Pos* = (3)
  DSI_WPCR4_TCLKPOST3_Msk* = (0x00000001 shl DSI_WPCR4_TCLKPOST3_Pos) ## !< 0x00000008
  DSI_WPCR4_TCLKPOST3* = DSI_WPCR4_TCLKPOST3_Msk
  DSI_WPCR4_TCLKPOST4_Pos* = (4)
  DSI_WPCR4_TCLKPOST4_Msk* = (0x00000001 shl DSI_WPCR4_TCLKPOST4_Pos) ## !< 0x00000010
  DSI_WPCR4_TCLKPOST4* = DSI_WPCR4_TCLKPOST4_Msk
  DSI_WPCR4_TCLKPOST5_Pos* = (5)
  DSI_WPCR4_TCLKPOST5_Msk* = (0x00000001 shl DSI_WPCR4_TCLKPOST5_Pos) ## !< 0x00000020
  DSI_WPCR4_TCLKPOST5* = DSI_WPCR4_TCLKPOST5_Msk
  DSI_WPCR4_TCLKPOST6_Pos* = (6)
  DSI_WPCR4_TCLKPOST6_Msk* = (0x00000001 shl DSI_WPCR4_TCLKPOST6_Pos) ## !< 0x00000040
  DSI_WPCR4_TCLKPOST6* = DSI_WPCR4_TCLKPOST6_Msk
  DSI_WPCR4_TCLKPOST7_Pos* = (7)
  DSI_WPCR4_TCLKPOST7_Msk* = (0x00000001 shl DSI_WPCR4_TCLKPOST7_Pos) ## !< 0x00000080
  DSI_WPCR4_TCLKPOST7* = DSI_WPCR4_TCLKPOST7_Msk

## ******************  Bit definition for DSI_WRPCR register  **************

const
  DSI_WRPCR_PLLEN_Pos* = (0)
  DSI_WRPCR_PLLEN_Msk* = (0x00000001 shl DSI_WRPCR_PLLEN_Pos) ## !< 0x00000001
  DSI_WRPCR_PLLEN* = DSI_WRPCR_PLLEN_Msk
  DSI_WRPCR_PLL_NDIV_Pos* = (2)
  DSI_WRPCR_PLL_NDIV_Msk* = (0x0000007F shl DSI_WRPCR_PLL_NDIV_Pos) ## !< 0x000001FC
  DSI_WRPCR_PLL_NDIV* = DSI_WRPCR_PLL_NDIV_Msk
  DSI_WRPCR_PLL_NDIV0_Pos* = (2)
  DSI_WRPCR_PLL_NDIV0_Msk* = (0x00000001 shl DSI_WRPCR_PLL_NDIV0_Pos) ## !< 0x00000004
  DSI_WRPCR_PLL_NDIV0* = DSI_WRPCR_PLL_NDIV0_Msk
  DSI_WRPCR_PLL_NDIV1_Pos* = (3)
  DSI_WRPCR_PLL_NDIV1_Msk* = (0x00000001 shl DSI_WRPCR_PLL_NDIV1_Pos) ## !< 0x00000008
  DSI_WRPCR_PLL_NDIV1* = DSI_WRPCR_PLL_NDIV1_Msk
  DSI_WRPCR_PLL_NDIV2_Pos* = (4)
  DSI_WRPCR_PLL_NDIV2_Msk* = (0x00000001 shl DSI_WRPCR_PLL_NDIV2_Pos) ## !< 0x00000010
  DSI_WRPCR_PLL_NDIV2* = DSI_WRPCR_PLL_NDIV2_Msk
  DSI_WRPCR_PLL_NDIV3_Pos* = (5)
  DSI_WRPCR_PLL_NDIV3_Msk* = (0x00000001 shl DSI_WRPCR_PLL_NDIV3_Pos) ## !< 0x00000020
  DSI_WRPCR_PLL_NDIV3* = DSI_WRPCR_PLL_NDIV3_Msk
  DSI_WRPCR_PLL_NDIV4_Pos* = (6)
  DSI_WRPCR_PLL_NDIV4_Msk* = (0x00000001 shl DSI_WRPCR_PLL_NDIV4_Pos) ## !< 0x00000040
  DSI_WRPCR_PLL_NDIV4* = DSI_WRPCR_PLL_NDIV4_Msk
  DSI_WRPCR_PLL_NDIV5_Pos* = (7)
  DSI_WRPCR_PLL_NDIV5_Msk* = (0x00000001 shl DSI_WRPCR_PLL_NDIV5_Pos) ## !< 0x00000080
  DSI_WRPCR_PLL_NDIV5* = DSI_WRPCR_PLL_NDIV5_Msk
  DSI_WRPCR_PLL_NDIV6_Pos* = (8)
  DSI_WRPCR_PLL_NDIV6_Msk* = (0x00000001 shl DSI_WRPCR_PLL_NDIV6_Pos) ## !< 0x00000100
  DSI_WRPCR_PLL_NDIV6* = DSI_WRPCR_PLL_NDIV6_Msk
  DSI_WRPCR_PLL_IDF_Pos* = (11)
  DSI_WRPCR_PLL_IDF_Msk* = (0x0000000F shl DSI_WRPCR_PLL_IDF_Pos) ## !< 0x00007800
  DSI_WRPCR_PLL_IDF* = DSI_WRPCR_PLL_IDF_Msk
  DSI_WRPCR_PLL_IDF0_Pos* = (11)
  DSI_WRPCR_PLL_IDF0_Msk* = (0x00000001 shl DSI_WRPCR_PLL_IDF0_Pos) ## !< 0x00000800
  DSI_WRPCR_PLL_IDF0* = DSI_WRPCR_PLL_IDF0_Msk
  DSI_WRPCR_PLL_IDF1_Pos* = (12)
  DSI_WRPCR_PLL_IDF1_Msk* = (0x00000001 shl DSI_WRPCR_PLL_IDF1_Pos) ## !< 0x00001000
  DSI_WRPCR_PLL_IDF1* = DSI_WRPCR_PLL_IDF1_Msk
  DSI_WRPCR_PLL_IDF2_Pos* = (13)
  DSI_WRPCR_PLL_IDF2_Msk* = (0x00000001 shl DSI_WRPCR_PLL_IDF2_Pos) ## !< 0x00002000
  DSI_WRPCR_PLL_IDF2* = DSI_WRPCR_PLL_IDF2_Msk
  DSI_WRPCR_PLL_IDF3_Pos* = (14)
  DSI_WRPCR_PLL_IDF3_Msk* = (0x00000001 shl DSI_WRPCR_PLL_IDF3_Pos) ## !< 0x00004000
  DSI_WRPCR_PLL_IDF3* = DSI_WRPCR_PLL_IDF3_Msk
  DSI_WRPCR_PLL_ODF_Pos* = (16)
  DSI_WRPCR_PLL_ODF_Msk* = (0x00000003 shl DSI_WRPCR_PLL_ODF_Pos) ## !< 0x00030000
  DSI_WRPCR_PLL_ODF* = DSI_WRPCR_PLL_ODF_Msk
  DSI_WRPCR_PLL_ODF0_Pos* = (16)
  DSI_WRPCR_PLL_ODF0_Msk* = (0x00000001 shl DSI_WRPCR_PLL_ODF0_Pos) ## !< 0x00010000
  DSI_WRPCR_PLL_ODF0* = DSI_WRPCR_PLL_ODF0_Msk
  DSI_WRPCR_PLL_ODF1_Pos* = (17)
  DSI_WRPCR_PLL_ODF1_Msk* = (0x00000001 shl DSI_WRPCR_PLL_ODF1_Pos) ## !< 0x00020000
  DSI_WRPCR_PLL_ODF1* = DSI_WRPCR_PLL_ODF1_Msk
  DSI_WRPCR_REGEN_Pos* = (24)
  DSI_WRPCR_REGEN_Msk* = (0x00000001 shl DSI_WRPCR_REGEN_Pos) ## !< 0x01000000
  DSI_WRPCR_REGEN* = DSI_WRPCR_REGEN_Msk

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
  EXTI_IMR_IM_Pos* = (0)
  EXTI_IMR_IM_Msk* = (0x007FFFFF shl EXTI_IMR_IM_Pos) ## !< 0x007FFFFF
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
  FLASH_ACR_LATENCY_Bit8WS* = 0x00000008
  FLASH_ACR_LATENCY_Bit9WS* = 0x00000009
  FLASH_ACR_LATENCY_Bit10WS* = 0x0000000A
  FLASH_ACR_LATENCY_Bit11WS* = 0x0000000B
  FLASH_ACR_LATENCY_Bit12WS* = 0x0000000C
  FLASH_ACR_LATENCY_Bit13WS* = 0x0000000D
  FLASH_ACR_LATENCY_Bit14WS* = 0x0000000E
  FLASH_ACR_LATENCY_Bit15WS* = 0x0000000F
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
  FLASH_CR_MER1* = FLASH_CR_MER
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
  FLASH_CR_MER2_Pos* = (15)
  FLASH_CR_MER2_Msk* = (0x00000001 shl FLASH_CR_MER2_Pos) ## !< 0x00008000
  FLASH_CR_MER2* = FLASH_CR_MER2_Msk
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
  FLASH_OPTCR_BFB2_Pos* = (4)
  FLASH_OPTCR_BFB2_Msk* = (0x00000001 shl FLASH_OPTCR_BFB2_Pos) ## !< 0x00000010
  FLASH_OPTCR_BFB2* = FLASH_OPTCR_BFB2_Msk
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
  FLASH_OPTCR_DB1M_Pos* = (30)
  FLASH_OPTCR_DB1M_Msk* = (0x00000001 shl FLASH_OPTCR_DB1M_Pos) ## !< 0x40000000
  FLASH_OPTCR_DB1M* = FLASH_OPTCR_DB1M_Msk
  FLASH_OPTCR_SPRMOD_Pos* = (31)
  FLASH_OPTCR_SPRMOD_Msk* = (0x00000001 shl FLASH_OPTCR_SPRMOD_Pos) ## !< 0x80000000
  FLASH_OPTCR_SPRMOD* = FLASH_OPTCR_SPRMOD_Msk

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
##                           Flexible Memory Controller
##
## ****************************************************************************
## *****************  Bit definition for FMC_BCR1 register  ******************

const
  FMC_BCR1_MBKEN_Pos* = (0)
  FMC_BCR1_MBKEN_Msk* = (0x00000001 shl FMC_BCR1_MBKEN_Pos) ## !< 0x00000001
  FMC_BCR1_MBKEN* = FMC_BCR1_MBKEN_Msk
  FMC_BCR1_MUXEN_Pos* = (1)
  FMC_BCR1_MUXEN_Msk* = (0x00000001 shl FMC_BCR1_MUXEN_Pos) ## !< 0x00000002
  FMC_BCR1_MUXEN* = FMC_BCR1_MUXEN_Msk
  FMC_BCR1_MTYP_Pos* = (2)
  FMC_BCR1_MTYP_Msk* = (0x00000003 shl FMC_BCR1_MTYP_Pos) ## !< 0x0000000C
  FMC_BCR1_MTYP* = FMC_BCR1_MTYP_Msk
  FMC_BCR1_MTYP_Bit0* = (0x00000001 shl FMC_BCR1_MTYP_Pos) ## !< 0x00000004
  FMC_BCR1_MTYP_Bit1* = (0x00000002 shl FMC_BCR1_MTYP_Pos) ## !< 0x00000008
  FMC_BCR1_MWID_Pos* = (4)
  FMC_BCR1_MWID_Msk* = (0x00000003 shl FMC_BCR1_MWID_Pos) ## !< 0x00000030
  FMC_BCR1_MWID* = FMC_BCR1_MWID_Msk
  FMC_BCR1_MWID_Bit0* = (0x00000001 shl FMC_BCR1_MWID_Pos) ## !< 0x00000010
  FMC_BCR1_MWID_Bit1* = (0x00000002 shl FMC_BCR1_MWID_Pos) ## !< 0x00000020
  FMC_BCR1_FACCEN_Pos* = (6)
  FMC_BCR1_FACCEN_Msk* = (0x00000001 shl FMC_BCR1_FACCEN_Pos) ## !< 0x00000040
  FMC_BCR1_FACCEN* = FMC_BCR1_FACCEN_Msk
  FMC_BCR1_BURSTEN_Pos* = (8)
  FMC_BCR1_BURSTEN_Msk* = (0x00000001 shl FMC_BCR1_BURSTEN_Pos) ## !< 0x00000100
  FMC_BCR1_BURSTEN* = FMC_BCR1_BURSTEN_Msk
  FMC_BCR1_WAITPOL_Pos* = (9)
  FMC_BCR1_WAITPOL_Msk* = (0x00000001 shl FMC_BCR1_WAITPOL_Pos) ## !< 0x00000200
  FMC_BCR1_WAITPOL* = FMC_BCR1_WAITPOL_Msk
  FMC_BCR1_WAITCFG_Pos* = (11)
  FMC_BCR1_WAITCFG_Msk* = (0x00000001 shl FMC_BCR1_WAITCFG_Pos) ## !< 0x00000800
  FMC_BCR1_WAITCFG* = FMC_BCR1_WAITCFG_Msk
  FMC_BCR1_WREN_Pos* = (12)
  FMC_BCR1_WREN_Msk* = (0x00000001 shl FMC_BCR1_WREN_Pos) ## !< 0x00001000
  FMC_BCR1_WREN* = FMC_BCR1_WREN_Msk
  FMC_BCR1_WAITEN_Pos* = (13)
  FMC_BCR1_WAITEN_Msk* = (0x00000001 shl FMC_BCR1_WAITEN_Pos) ## !< 0x00002000
  FMC_BCR1_WAITEN* = FMC_BCR1_WAITEN_Msk
  FMC_BCR1_EXTMOD_Pos* = (14)
  FMC_BCR1_EXTMOD_Msk* = (0x00000001 shl FMC_BCR1_EXTMOD_Pos) ## !< 0x00004000
  FMC_BCR1_EXTMOD* = FMC_BCR1_EXTMOD_Msk
  FMC_BCR1_ASYNCWAIT_Pos* = (15)
  FMC_BCR1_ASYNCWAIT_Msk* = (0x00000001 shl FMC_BCR1_ASYNCWAIT_Pos) ## !< 0x00008000
  FMC_BCR1_ASYNCWAIT* = FMC_BCR1_ASYNCWAIT_Msk
  FMC_BCR1_CPSIZE_Pos* = (16)
  FMC_BCR1_CPSIZE_Msk* = (0x00000007 shl FMC_BCR1_CPSIZE_Pos) ## !< 0x00070000
  FMC_BCR1_CPSIZE* = FMC_BCR1_CPSIZE_Msk
  FMC_BCR1_CPSIZE_Bit0* = (0x00000001 shl FMC_BCR1_CPSIZE_Pos) ## !< 0x00010000
  FMC_BCR1_CPSIZE_Bit1* = (0x00000002 shl FMC_BCR1_CPSIZE_Pos) ## !< 0x00020000
  FMC_BCR1_CPSIZE_Bit2* = (0x00000004 shl FMC_BCR1_CPSIZE_Pos) ## !< 0x00040000
  FMC_BCR1_CBURSTRW_Pos* = (19)
  FMC_BCR1_CBURSTRW_Msk* = (0x00000001 shl FMC_BCR1_CBURSTRW_Pos) ## !< 0x00080000
  FMC_BCR1_CBURSTRW* = FMC_BCR1_CBURSTRW_Msk
  FMC_BCR1_CCLKEN_Pos* = (20)
  FMC_BCR1_CCLKEN_Msk* = (0x00000001 shl FMC_BCR1_CCLKEN_Pos) ## !< 0x00100000
  FMC_BCR1_CCLKEN* = FMC_BCR1_CCLKEN_Msk
  FMC_BCR1_WFDIS_Pos* = (21)
  FMC_BCR1_WFDIS_Msk* = (0x00000001 shl FMC_BCR1_WFDIS_Pos) ## !< 0x00200000
  FMC_BCR1_WFDIS* = FMC_BCR1_WFDIS_Msk

## *****************  Bit definition for FMC_BCR2 register  ******************

const
  FMC_BCR2_MBKEN_Pos* = (0)
  FMC_BCR2_MBKEN_Msk* = (0x00000001 shl FMC_BCR2_MBKEN_Pos) ## !< 0x00000001
  FMC_BCR2_MBKEN* = FMC_BCR2_MBKEN_Msk
  FMC_BCR2_MUXEN_Pos* = (1)
  FMC_BCR2_MUXEN_Msk* = (0x00000001 shl FMC_BCR2_MUXEN_Pos) ## !< 0x00000002
  FMC_BCR2_MUXEN* = FMC_BCR2_MUXEN_Msk
  FMC_BCR2_MTYP_Pos* = (2)
  FMC_BCR2_MTYP_Msk* = (0x00000003 shl FMC_BCR2_MTYP_Pos) ## !< 0x0000000C
  FMC_BCR2_MTYP* = FMC_BCR2_MTYP_Msk
  FMC_BCR2_MTYP_Bit0* = (0x00000001 shl FMC_BCR2_MTYP_Pos) ## !< 0x00000004
  FMC_BCR2_MTYP_Bit1* = (0x00000002 shl FMC_BCR2_MTYP_Pos) ## !< 0x00000008
  FMC_BCR2_MWID_Pos* = (4)
  FMC_BCR2_MWID_Msk* = (0x00000003 shl FMC_BCR2_MWID_Pos) ## !< 0x00000030
  FMC_BCR2_MWID* = FMC_BCR2_MWID_Msk
  FMC_BCR2_MWID_Bit0* = (0x00000001 shl FMC_BCR2_MWID_Pos) ## !< 0x00000010
  FMC_BCR2_MWID_Bit1* = (0x00000002 shl FMC_BCR2_MWID_Pos) ## !< 0x00000020
  FMC_BCR2_FACCEN_Pos* = (6)
  FMC_BCR2_FACCEN_Msk* = (0x00000001 shl FMC_BCR2_FACCEN_Pos) ## !< 0x00000040
  FMC_BCR2_FACCEN* = FMC_BCR2_FACCEN_Msk
  FMC_BCR2_BURSTEN_Pos* = (8)
  FMC_BCR2_BURSTEN_Msk* = (0x00000001 shl FMC_BCR2_BURSTEN_Pos) ## !< 0x00000100
  FMC_BCR2_BURSTEN* = FMC_BCR2_BURSTEN_Msk
  FMC_BCR2_WAITPOL_Pos* = (9)
  FMC_BCR2_WAITPOL_Msk* = (0x00000001 shl FMC_BCR2_WAITPOL_Pos) ## !< 0x00000200
  FMC_BCR2_WAITPOL* = FMC_BCR2_WAITPOL_Msk
  FMC_BCR2_WAITCFG_Pos* = (11)
  FMC_BCR2_WAITCFG_Msk* = (0x00000001 shl FMC_BCR2_WAITCFG_Pos) ## !< 0x00000800
  FMC_BCR2_WAITCFG* = FMC_BCR2_WAITCFG_Msk
  FMC_BCR2_WREN_Pos* = (12)
  FMC_BCR2_WREN_Msk* = (0x00000001 shl FMC_BCR2_WREN_Pos) ## !< 0x00001000
  FMC_BCR2_WREN* = FMC_BCR2_WREN_Msk
  FMC_BCR2_WAITEN_Pos* = (13)
  FMC_BCR2_WAITEN_Msk* = (0x00000001 shl FMC_BCR2_WAITEN_Pos) ## !< 0x00002000
  FMC_BCR2_WAITEN* = FMC_BCR2_WAITEN_Msk
  FMC_BCR2_EXTMOD_Pos* = (14)
  FMC_BCR2_EXTMOD_Msk* = (0x00000001 shl FMC_BCR2_EXTMOD_Pos) ## !< 0x00004000
  FMC_BCR2_EXTMOD* = FMC_BCR2_EXTMOD_Msk
  FMC_BCR2_ASYNCWAIT_Pos* = (15)
  FMC_BCR2_ASYNCWAIT_Msk* = (0x00000001 shl FMC_BCR2_ASYNCWAIT_Pos) ## !< 0x00008000
  FMC_BCR2_ASYNCWAIT* = FMC_BCR2_ASYNCWAIT_Msk
  FMC_BCR2_CBURSTRW_Pos* = (19)
  FMC_BCR2_CBURSTRW_Msk* = (0x00000001 shl FMC_BCR2_CBURSTRW_Pos) ## !< 0x00080000
  FMC_BCR2_CBURSTRW* = FMC_BCR2_CBURSTRW_Msk

## *****************  Bit definition for FMC_BCR3 register  ******************

const
  FMC_BCR3_MBKEN_Pos* = (0)
  FMC_BCR3_MBKEN_Msk* = (0x00000001 shl FMC_BCR3_MBKEN_Pos) ## !< 0x00000001
  FMC_BCR3_MBKEN* = FMC_BCR3_MBKEN_Msk
  FMC_BCR3_MUXEN_Pos* = (1)
  FMC_BCR3_MUXEN_Msk* = (0x00000001 shl FMC_BCR3_MUXEN_Pos) ## !< 0x00000002
  FMC_BCR3_MUXEN* = FMC_BCR3_MUXEN_Msk
  FMC_BCR3_MTYP_Pos* = (2)
  FMC_BCR3_MTYP_Msk* = (0x00000003 shl FMC_BCR3_MTYP_Pos) ## !< 0x0000000C
  FMC_BCR3_MTYP* = FMC_BCR3_MTYP_Msk
  FMC_BCR3_MTYP_Bit0* = (0x00000001 shl FMC_BCR3_MTYP_Pos) ## !< 0x00000004
  FMC_BCR3_MTYP_Bit1* = (0x00000002 shl FMC_BCR3_MTYP_Pos) ## !< 0x00000008
  FMC_BCR3_MWID_Pos* = (4)
  FMC_BCR3_MWID_Msk* = (0x00000003 shl FMC_BCR3_MWID_Pos) ## !< 0x00000030
  FMC_BCR3_MWID* = FMC_BCR3_MWID_Msk
  FMC_BCR3_MWID_Bit0* = (0x00000001 shl FMC_BCR3_MWID_Pos) ## !< 0x00000010
  FMC_BCR3_MWID_Bit1* = (0x00000002 shl FMC_BCR3_MWID_Pos) ## !< 0x00000020
  FMC_BCR3_FACCEN_Pos* = (6)
  FMC_BCR3_FACCEN_Msk* = (0x00000001 shl FMC_BCR3_FACCEN_Pos) ## !< 0x00000040
  FMC_BCR3_FACCEN* = FMC_BCR3_FACCEN_Msk
  FMC_BCR3_BURSTEN_Pos* = (8)
  FMC_BCR3_BURSTEN_Msk* = (0x00000001 shl FMC_BCR3_BURSTEN_Pos) ## !< 0x00000100
  FMC_BCR3_BURSTEN* = FMC_BCR3_BURSTEN_Msk
  FMC_BCR3_WAITPOL_Pos* = (9)
  FMC_BCR3_WAITPOL_Msk* = (0x00000001 shl FMC_BCR3_WAITPOL_Pos) ## !< 0x00000200
  FMC_BCR3_WAITPOL* = FMC_BCR3_WAITPOL_Msk
  FMC_BCR3_WAITCFG_Pos* = (11)
  FMC_BCR3_WAITCFG_Msk* = (0x00000001 shl FMC_BCR3_WAITCFG_Pos) ## !< 0x00000800
  FMC_BCR3_WAITCFG* = FMC_BCR3_WAITCFG_Msk
  FMC_BCR3_WREN_Pos* = (12)
  FMC_BCR3_WREN_Msk* = (0x00000001 shl FMC_BCR3_WREN_Pos) ## !< 0x00001000
  FMC_BCR3_WREN* = FMC_BCR3_WREN_Msk
  FMC_BCR3_WAITEN_Pos* = (13)
  FMC_BCR3_WAITEN_Msk* = (0x00000001 shl FMC_BCR3_WAITEN_Pos) ## !< 0x00002000
  FMC_BCR3_WAITEN* = FMC_BCR3_WAITEN_Msk
  FMC_BCR3_EXTMOD_Pos* = (14)
  FMC_BCR3_EXTMOD_Msk* = (0x00000001 shl FMC_BCR3_EXTMOD_Pos) ## !< 0x00004000
  FMC_BCR3_EXTMOD* = FMC_BCR3_EXTMOD_Msk
  FMC_BCR3_ASYNCWAIT_Pos* = (15)
  FMC_BCR3_ASYNCWAIT_Msk* = (0x00000001 shl FMC_BCR3_ASYNCWAIT_Pos) ## !< 0x00008000
  FMC_BCR3_ASYNCWAIT* = FMC_BCR3_ASYNCWAIT_Msk
  FMC_BCR3_CBURSTRW_Pos* = (19)
  FMC_BCR3_CBURSTRW_Msk* = (0x00000001 shl FMC_BCR3_CBURSTRW_Pos) ## !< 0x00080000
  FMC_BCR3_CBURSTRW* = FMC_BCR3_CBURSTRW_Msk

## *****************  Bit definition for FMC_BCR4 register  ******************

const
  FMC_BCR4_MBKEN_Pos* = (0)
  FMC_BCR4_MBKEN_Msk* = (0x00000001 shl FMC_BCR4_MBKEN_Pos) ## !< 0x00000001
  FMC_BCR4_MBKEN* = FMC_BCR4_MBKEN_Msk
  FMC_BCR4_MUXEN_Pos* = (1)
  FMC_BCR4_MUXEN_Msk* = (0x00000001 shl FMC_BCR4_MUXEN_Pos) ## !< 0x00000002
  FMC_BCR4_MUXEN* = FMC_BCR4_MUXEN_Msk
  FMC_BCR4_MTYP_Pos* = (2)
  FMC_BCR4_MTYP_Msk* = (0x00000003 shl FMC_BCR4_MTYP_Pos) ## !< 0x0000000C
  FMC_BCR4_MTYP* = FMC_BCR4_MTYP_Msk
  FMC_BCR4_MTYP_Bit0* = (0x00000001 shl FMC_BCR4_MTYP_Pos) ## !< 0x00000004
  FMC_BCR4_MTYP_Bit1* = (0x00000002 shl FMC_BCR4_MTYP_Pos) ## !< 0x00000008
  FMC_BCR4_MWID_Pos* = (4)
  FMC_BCR4_MWID_Msk* = (0x00000003 shl FMC_BCR4_MWID_Pos) ## !< 0x00000030
  FMC_BCR4_MWID* = FMC_BCR4_MWID_Msk
  FMC_BCR4_MWID_Bit0* = (0x00000001 shl FMC_BCR4_MWID_Pos) ## !< 0x00000010
  FMC_BCR4_MWID_Bit1* = (0x00000002 shl FMC_BCR4_MWID_Pos) ## !< 0x00000020
  FMC_BCR4_FACCEN_Pos* = (6)
  FMC_BCR4_FACCEN_Msk* = (0x00000001 shl FMC_BCR4_FACCEN_Pos) ## !< 0x00000040
  FMC_BCR4_FACCEN* = FMC_BCR4_FACCEN_Msk
  FMC_BCR4_BURSTEN_Pos* = (8)
  FMC_BCR4_BURSTEN_Msk* = (0x00000001 shl FMC_BCR4_BURSTEN_Pos) ## !< 0x00000100
  FMC_BCR4_BURSTEN* = FMC_BCR4_BURSTEN_Msk
  FMC_BCR4_WAITPOL_Pos* = (9)
  FMC_BCR4_WAITPOL_Msk* = (0x00000001 shl FMC_BCR4_WAITPOL_Pos) ## !< 0x00000200
  FMC_BCR4_WAITPOL* = FMC_BCR4_WAITPOL_Msk
  FMC_BCR4_WAITCFG_Pos* = (11)
  FMC_BCR4_WAITCFG_Msk* = (0x00000001 shl FMC_BCR4_WAITCFG_Pos) ## !< 0x00000800
  FMC_BCR4_WAITCFG* = FMC_BCR4_WAITCFG_Msk
  FMC_BCR4_WREN_Pos* = (12)
  FMC_BCR4_WREN_Msk* = (0x00000001 shl FMC_BCR4_WREN_Pos) ## !< 0x00001000
  FMC_BCR4_WREN* = FMC_BCR4_WREN_Msk
  FMC_BCR4_WAITEN_Pos* = (13)
  FMC_BCR4_WAITEN_Msk* = (0x00000001 shl FMC_BCR4_WAITEN_Pos) ## !< 0x00002000
  FMC_BCR4_WAITEN* = FMC_BCR4_WAITEN_Msk
  FMC_BCR4_EXTMOD_Pos* = (14)
  FMC_BCR4_EXTMOD_Msk* = (0x00000001 shl FMC_BCR4_EXTMOD_Pos) ## !< 0x00004000
  FMC_BCR4_EXTMOD* = FMC_BCR4_EXTMOD_Msk
  FMC_BCR4_ASYNCWAIT_Pos* = (15)
  FMC_BCR4_ASYNCWAIT_Msk* = (0x00000001 shl FMC_BCR4_ASYNCWAIT_Pos) ## !< 0x00008000
  FMC_BCR4_ASYNCWAIT* = FMC_BCR4_ASYNCWAIT_Msk
  FMC_BCR4_CBURSTRW_Pos* = (19)
  FMC_BCR4_CBURSTRW_Msk* = (0x00000001 shl FMC_BCR4_CBURSTRW_Pos) ## !< 0x00080000
  FMC_BCR4_CBURSTRW* = FMC_BCR4_CBURSTRW_Msk

## *****************  Bit definition for FMC_BTR1 register  *****************

const
  FMC_BTR1_ADDSET_Pos* = (0)
  FMC_BTR1_ADDSET_Msk* = (0x0000000F shl FMC_BTR1_ADDSET_Pos) ## !< 0x0000000F
  FMC_BTR1_ADDSET* = FMC_BTR1_ADDSET_Msk
  FMC_BTR1_ADDSET_Bit0* = (0x00000001 shl FMC_BTR1_ADDSET_Pos) ## !< 0x00000001
  FMC_BTR1_ADDSET_Bit1* = (0x00000002 shl FMC_BTR1_ADDSET_Pos) ## !< 0x00000002
  FMC_BTR1_ADDSET_Bit2* = (0x00000004 shl FMC_BTR1_ADDSET_Pos) ## !< 0x00000004
  FMC_BTR1_ADDSET_Bit3* = (0x00000008 shl FMC_BTR1_ADDSET_Pos) ## !< 0x00000008
  FMC_BTR1_ADDHLD_Pos* = (4)
  FMC_BTR1_ADDHLD_Msk* = (0x0000000F shl FMC_BTR1_ADDHLD_Pos) ## !< 0x000000F0
  FMC_BTR1_ADDHLD* = FMC_BTR1_ADDHLD_Msk
  FMC_BTR1_ADDHLD_Bit0* = (0x00000001 shl FMC_BTR1_ADDHLD_Pos) ## !< 0x00000010
  FMC_BTR1_ADDHLD_Bit1* = (0x00000002 shl FMC_BTR1_ADDHLD_Pos) ## !< 0x00000020
  FMC_BTR1_ADDHLD_Bit2* = (0x00000004 shl FMC_BTR1_ADDHLD_Pos) ## !< 0x00000040
  FMC_BTR1_ADDHLD_Bit3* = (0x00000008 shl FMC_BTR1_ADDHLD_Pos) ## !< 0x00000080
  FMC_BTR1_DATAST_Pos* = (8)
  FMC_BTR1_DATAST_Msk* = (0x000000FF shl FMC_BTR1_DATAST_Pos) ## !< 0x0000FF00
  FMC_BTR1_DATAST* = FMC_BTR1_DATAST_Msk
  FMC_BTR1_DATAST_Bit0* = (0x00000001 shl FMC_BTR1_DATAST_Pos) ## !< 0x00000100
  FMC_BTR1_DATAST_Bit1* = (0x00000002 shl FMC_BTR1_DATAST_Pos) ## !< 0x00000200
  FMC_BTR1_DATAST_Bit2* = (0x00000004 shl FMC_BTR1_DATAST_Pos) ## !< 0x00000400
  FMC_BTR1_DATAST_Bit3* = (0x00000008 shl FMC_BTR1_DATAST_Pos) ## !< 0x00000800
  FMC_BTR1_DATAST_Bit4* = (0x00000010 shl FMC_BTR1_DATAST_Pos) ## !< 0x00001000
  FMC_BTR1_DATAST_Bit5* = (0x00000020 shl FMC_BTR1_DATAST_Pos) ## !< 0x00002000
  FMC_BTR1_DATAST_Bit6* = (0x00000040 shl FMC_BTR1_DATAST_Pos) ## !< 0x00004000
  FMC_BTR1_DATAST_Bit7* = (0x00000080 shl FMC_BTR1_DATAST_Pos) ## !< 0x00008000
  FMC_BTR1_BUSTURN_Pos* = (16)
  FMC_BTR1_BUSTURN_Msk* = (0x0000000F shl FMC_BTR1_BUSTURN_Pos) ## !< 0x000F0000
  FMC_BTR1_BUSTURN* = FMC_BTR1_BUSTURN_Msk
  FMC_BTR1_BUSTURN_Bit0* = (0x00000001 shl FMC_BTR1_BUSTURN_Pos) ## !< 0x00010000
  FMC_BTR1_BUSTURN_Bit1* = (0x00000002 shl FMC_BTR1_BUSTURN_Pos) ## !< 0x00020000
  FMC_BTR1_BUSTURN_Bit2* = (0x00000004 shl FMC_BTR1_BUSTURN_Pos) ## !< 0x00040000
  FMC_BTR1_BUSTURN_Bit3* = (0x00000008 shl FMC_BTR1_BUSTURN_Pos) ## !< 0x00080000
  FMC_BTR1_CLKDIV_Pos* = (20)
  FMC_BTR1_CLKDIV_Msk* = (0x0000000F shl FMC_BTR1_CLKDIV_Pos) ## !< 0x00F00000
  FMC_BTR1_CLKDIV* = FMC_BTR1_CLKDIV_Msk
  FMC_BTR1_CLKDIV_Bit0* = (0x00000001 shl FMC_BTR1_CLKDIV_Pos) ## !< 0x00100000
  FMC_BTR1_CLKDIV_Bit1* = (0x00000002 shl FMC_BTR1_CLKDIV_Pos) ## !< 0x00200000
  FMC_BTR1_CLKDIV_Bit2* = (0x00000004 shl FMC_BTR1_CLKDIV_Pos) ## !< 0x00400000
  FMC_BTR1_CLKDIV_Bit3* = (0x00000008 shl FMC_BTR1_CLKDIV_Pos) ## !< 0x00800000
  FMC_BTR1_DATLAT_Pos* = (24)
  FMC_BTR1_DATLAT_Msk* = (0x0000000F shl FMC_BTR1_DATLAT_Pos) ## !< 0x0F000000
  FMC_BTR1_DATLAT* = FMC_BTR1_DATLAT_Msk
  FMC_BTR1_DATLAT_Bit0* = (0x00000001 shl FMC_BTR1_DATLAT_Pos) ## !< 0x01000000
  FMC_BTR1_DATLAT_Bit1* = (0x00000002 shl FMC_BTR1_DATLAT_Pos) ## !< 0x02000000
  FMC_BTR1_DATLAT_Bit2* = (0x00000004 shl FMC_BTR1_DATLAT_Pos) ## !< 0x04000000
  FMC_BTR1_DATLAT_Bit3* = (0x00000008 shl FMC_BTR1_DATLAT_Pos) ## !< 0x08000000
  FMC_BTR1_ACCMOD_Pos* = (28)
  FMC_BTR1_ACCMOD_Msk* = (0x00000003 shl FMC_BTR1_ACCMOD_Pos) ## !< 0x30000000
  FMC_BTR1_ACCMOD* = FMC_BTR1_ACCMOD_Msk
  FMC_BTR1_ACCMOD_Bit0* = (0x00000001 shl FMC_BTR1_ACCMOD_Pos) ## !< 0x10000000
  FMC_BTR1_ACCMOD_Bit1* = (0x00000002 shl FMC_BTR1_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FMC_BTR2 register  ******************

const
  FMC_BTR2_ADDSET_Pos* = (0)
  FMC_BTR2_ADDSET_Msk* = (0x0000000F shl FMC_BTR2_ADDSET_Pos) ## !< 0x0000000F
  FMC_BTR2_ADDSET* = FMC_BTR2_ADDSET_Msk
  FMC_BTR2_ADDSET_Bit0* = (0x00000001 shl FMC_BTR2_ADDSET_Pos) ## !< 0x00000001
  FMC_BTR2_ADDSET_Bit1* = (0x00000002 shl FMC_BTR2_ADDSET_Pos) ## !< 0x00000002
  FMC_BTR2_ADDSET_Bit2* = (0x00000004 shl FMC_BTR2_ADDSET_Pos) ## !< 0x00000004
  FMC_BTR2_ADDSET_Bit3* = (0x00000008 shl FMC_BTR2_ADDSET_Pos) ## !< 0x00000008
  FMC_BTR2_ADDHLD_Pos* = (4)
  FMC_BTR2_ADDHLD_Msk* = (0x0000000F shl FMC_BTR2_ADDHLD_Pos) ## !< 0x000000F0
  FMC_BTR2_ADDHLD* = FMC_BTR2_ADDHLD_Msk
  FMC_BTR2_ADDHLD_Bit0* = (0x00000001 shl FMC_BTR2_ADDHLD_Pos) ## !< 0x00000010
  FMC_BTR2_ADDHLD_Bit1* = (0x00000002 shl FMC_BTR2_ADDHLD_Pos) ## !< 0x00000020
  FMC_BTR2_ADDHLD_Bit2* = (0x00000004 shl FMC_BTR2_ADDHLD_Pos) ## !< 0x00000040
  FMC_BTR2_ADDHLD_Bit3* = (0x00000008 shl FMC_BTR2_ADDHLD_Pos) ## !< 0x00000080
  FMC_BTR2_DATAST_Pos* = (8)
  FMC_BTR2_DATAST_Msk* = (0x000000FF shl FMC_BTR2_DATAST_Pos) ## !< 0x0000FF00
  FMC_BTR2_DATAST* = FMC_BTR2_DATAST_Msk
  FMC_BTR2_DATAST_Bit0* = (0x00000001 shl FMC_BTR2_DATAST_Pos) ## !< 0x00000100
  FMC_BTR2_DATAST_Bit1* = (0x00000002 shl FMC_BTR2_DATAST_Pos) ## !< 0x00000200
  FMC_BTR2_DATAST_Bit2* = (0x00000004 shl FMC_BTR2_DATAST_Pos) ## !< 0x00000400
  FMC_BTR2_DATAST_Bit3* = (0x00000008 shl FMC_BTR2_DATAST_Pos) ## !< 0x00000800
  FMC_BTR2_DATAST_Bit4* = (0x00000010 shl FMC_BTR2_DATAST_Pos) ## !< 0x00001000
  FMC_BTR2_DATAST_Bit5* = (0x00000020 shl FMC_BTR2_DATAST_Pos) ## !< 0x00002000
  FMC_BTR2_DATAST_Bit6* = (0x00000040 shl FMC_BTR2_DATAST_Pos) ## !< 0x00004000
  FMC_BTR2_DATAST_Bit7* = (0x00000080 shl FMC_BTR2_DATAST_Pos) ## !< 0x00008000
  FMC_BTR2_BUSTURN_Pos* = (16)
  FMC_BTR2_BUSTURN_Msk* = (0x0000000F shl FMC_BTR2_BUSTURN_Pos) ## !< 0x000F0000
  FMC_BTR2_BUSTURN* = FMC_BTR2_BUSTURN_Msk
  FMC_BTR2_BUSTURN_Bit0* = (0x00000001 shl FMC_BTR2_BUSTURN_Pos) ## !< 0x00010000
  FMC_BTR2_BUSTURN_Bit1* = (0x00000002 shl FMC_BTR2_BUSTURN_Pos) ## !< 0x00020000
  FMC_BTR2_BUSTURN_Bit2* = (0x00000004 shl FMC_BTR2_BUSTURN_Pos) ## !< 0x00040000
  FMC_BTR2_BUSTURN_Bit3* = (0x00000008 shl FMC_BTR2_BUSTURN_Pos) ## !< 0x00080000
  FMC_BTR2_CLKDIV_Pos* = (20)
  FMC_BTR2_CLKDIV_Msk* = (0x0000000F shl FMC_BTR2_CLKDIV_Pos) ## !< 0x00F00000
  FMC_BTR2_CLKDIV* = FMC_BTR2_CLKDIV_Msk
  FMC_BTR2_CLKDIV_Bit0* = (0x00000001 shl FMC_BTR2_CLKDIV_Pos) ## !< 0x00100000
  FMC_BTR2_CLKDIV_Bit1* = (0x00000002 shl FMC_BTR2_CLKDIV_Pos) ## !< 0x00200000
  FMC_BTR2_CLKDIV_Bit2* = (0x00000004 shl FMC_BTR2_CLKDIV_Pos) ## !< 0x00400000
  FMC_BTR2_CLKDIV_Bit3* = (0x00000008 shl FMC_BTR2_CLKDIV_Pos) ## !< 0x00800000
  FMC_BTR2_DATLAT_Pos* = (24)
  FMC_BTR2_DATLAT_Msk* = (0x0000000F shl FMC_BTR2_DATLAT_Pos) ## !< 0x0F000000
  FMC_BTR2_DATLAT* = FMC_BTR2_DATLAT_Msk
  FMC_BTR2_DATLAT_Bit0* = (0x00000001 shl FMC_BTR2_DATLAT_Pos) ## !< 0x01000000
  FMC_BTR2_DATLAT_Bit1* = (0x00000002 shl FMC_BTR2_DATLAT_Pos) ## !< 0x02000000
  FMC_BTR2_DATLAT_Bit2* = (0x00000004 shl FMC_BTR2_DATLAT_Pos) ## !< 0x04000000
  FMC_BTR2_DATLAT_Bit3* = (0x00000008 shl FMC_BTR2_DATLAT_Pos) ## !< 0x08000000
  FMC_BTR2_ACCMOD_Pos* = (28)
  FMC_BTR2_ACCMOD_Msk* = (0x00000003 shl FMC_BTR2_ACCMOD_Pos) ## !< 0x30000000
  FMC_BTR2_ACCMOD* = FMC_BTR2_ACCMOD_Msk
  FMC_BTR2_ACCMOD_Bit0* = (0x00000001 shl FMC_BTR2_ACCMOD_Pos) ## !< 0x10000000
  FMC_BTR2_ACCMOD_Bit1* = (0x00000002 shl FMC_BTR2_ACCMOD_Pos) ## !< 0x20000000

## ******************  Bit definition for FMC_BTR3 register  ******************

const
  FMC_BTR3_ADDSET_Pos* = (0)
  FMC_BTR3_ADDSET_Msk* = (0x0000000F shl FMC_BTR3_ADDSET_Pos) ## !< 0x0000000F
  FMC_BTR3_ADDSET* = FMC_BTR3_ADDSET_Msk
  FMC_BTR3_ADDSET_Bit0* = (0x00000001 shl FMC_BTR3_ADDSET_Pos) ## !< 0x00000001
  FMC_BTR3_ADDSET_Bit1* = (0x00000002 shl FMC_BTR3_ADDSET_Pos) ## !< 0x00000002
  FMC_BTR3_ADDSET_Bit2* = (0x00000004 shl FMC_BTR3_ADDSET_Pos) ## !< 0x00000004
  FMC_BTR3_ADDSET_Bit3* = (0x00000008 shl FMC_BTR3_ADDSET_Pos) ## !< 0x00000008
  FMC_BTR3_ADDHLD_Pos* = (4)
  FMC_BTR3_ADDHLD_Msk* = (0x0000000F shl FMC_BTR3_ADDHLD_Pos) ## !< 0x000000F0
  FMC_BTR3_ADDHLD* = FMC_BTR3_ADDHLD_Msk
  FMC_BTR3_ADDHLD_Bit0* = (0x00000001 shl FMC_BTR3_ADDHLD_Pos) ## !< 0x00000010
  FMC_BTR3_ADDHLD_Bit1* = (0x00000002 shl FMC_BTR3_ADDHLD_Pos) ## !< 0x00000020
  FMC_BTR3_ADDHLD_Bit2* = (0x00000004 shl FMC_BTR3_ADDHLD_Pos) ## !< 0x00000040
  FMC_BTR3_ADDHLD_Bit3* = (0x00000008 shl FMC_BTR3_ADDHLD_Pos) ## !< 0x00000080
  FMC_BTR3_DATAST_Pos* = (8)
  FMC_BTR3_DATAST_Msk* = (0x000000FF shl FMC_BTR3_DATAST_Pos) ## !< 0x0000FF00
  FMC_BTR3_DATAST* = FMC_BTR3_DATAST_Msk
  FMC_BTR3_DATAST_Bit0* = (0x00000001 shl FMC_BTR3_DATAST_Pos) ## !< 0x00000100
  FMC_BTR3_DATAST_Bit1* = (0x00000002 shl FMC_BTR3_DATAST_Pos) ## !< 0x00000200
  FMC_BTR3_DATAST_Bit2* = (0x00000004 shl FMC_BTR3_DATAST_Pos) ## !< 0x00000400
  FMC_BTR3_DATAST_Bit3* = (0x00000008 shl FMC_BTR3_DATAST_Pos) ## !< 0x00000800
  FMC_BTR3_DATAST_Bit4* = (0x00000010 shl FMC_BTR3_DATAST_Pos) ## !< 0x00001000
  FMC_BTR3_DATAST_Bit5* = (0x00000020 shl FMC_BTR3_DATAST_Pos) ## !< 0x00002000
  FMC_BTR3_DATAST_Bit6* = (0x00000040 shl FMC_BTR3_DATAST_Pos) ## !< 0x00004000
  FMC_BTR3_DATAST_Bit7* = (0x00000080 shl FMC_BTR3_DATAST_Pos) ## !< 0x00008000
  FMC_BTR3_BUSTURN_Pos* = (16)
  FMC_BTR3_BUSTURN_Msk* = (0x0000000F shl FMC_BTR3_BUSTURN_Pos) ## !< 0x000F0000
  FMC_BTR3_BUSTURN* = FMC_BTR3_BUSTURN_Msk
  FMC_BTR3_BUSTURN_Bit0* = (0x00000001 shl FMC_BTR3_BUSTURN_Pos) ## !< 0x00010000
  FMC_BTR3_BUSTURN_Bit1* = (0x00000002 shl FMC_BTR3_BUSTURN_Pos) ## !< 0x00020000
  FMC_BTR3_BUSTURN_Bit2* = (0x00000004 shl FMC_BTR3_BUSTURN_Pos) ## !< 0x00040000
  FMC_BTR3_BUSTURN_Bit3* = (0x00000008 shl FMC_BTR3_BUSTURN_Pos) ## !< 0x00080000
  FMC_BTR3_CLKDIV_Pos* = (20)
  FMC_BTR3_CLKDIV_Msk* = (0x0000000F shl FMC_BTR3_CLKDIV_Pos) ## !< 0x00F00000
  FMC_BTR3_CLKDIV* = FMC_BTR3_CLKDIV_Msk
  FMC_BTR3_CLKDIV_Bit0* = (0x00000001 shl FMC_BTR3_CLKDIV_Pos) ## !< 0x00100000
  FMC_BTR3_CLKDIV_Bit1* = (0x00000002 shl FMC_BTR3_CLKDIV_Pos) ## !< 0x00200000
  FMC_BTR3_CLKDIV_Bit2* = (0x00000004 shl FMC_BTR3_CLKDIV_Pos) ## !< 0x00400000
  FMC_BTR3_CLKDIV_Bit3* = (0x00000008 shl FMC_BTR3_CLKDIV_Pos) ## !< 0x00800000
  FMC_BTR3_DATLAT_Pos* = (24)
  FMC_BTR3_DATLAT_Msk* = (0x0000000F shl FMC_BTR3_DATLAT_Pos) ## !< 0x0F000000
  FMC_BTR3_DATLAT* = FMC_BTR3_DATLAT_Msk
  FMC_BTR3_DATLAT_Bit0* = (0x00000001 shl FMC_BTR3_DATLAT_Pos) ## !< 0x01000000
  FMC_BTR3_DATLAT_Bit1* = (0x00000002 shl FMC_BTR3_DATLAT_Pos) ## !< 0x02000000
  FMC_BTR3_DATLAT_Bit2* = (0x00000004 shl FMC_BTR3_DATLAT_Pos) ## !< 0x04000000
  FMC_BTR3_DATLAT_Bit3* = (0x00000008 shl FMC_BTR3_DATLAT_Pos) ## !< 0x08000000
  FMC_BTR3_ACCMOD_Pos* = (28)
  FMC_BTR3_ACCMOD_Msk* = (0x00000003 shl FMC_BTR3_ACCMOD_Pos) ## !< 0x30000000
  FMC_BTR3_ACCMOD* = FMC_BTR3_ACCMOD_Msk
  FMC_BTR3_ACCMOD_Bit0* = (0x00000001 shl FMC_BTR3_ACCMOD_Pos) ## !< 0x10000000
  FMC_BTR3_ACCMOD_Bit1* = (0x00000002 shl FMC_BTR3_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FMC_BTR4 register  ******************

const
  FMC_BTR4_ADDSET_Pos* = (0)
  FMC_BTR4_ADDSET_Msk* = (0x0000000F shl FMC_BTR4_ADDSET_Pos) ## !< 0x0000000F
  FMC_BTR4_ADDSET* = FMC_BTR4_ADDSET_Msk
  FMC_BTR4_ADDSET_Bit0* = (0x00000001 shl FMC_BTR4_ADDSET_Pos) ## !< 0x00000001
  FMC_BTR4_ADDSET_Bit1* = (0x00000002 shl FMC_BTR4_ADDSET_Pos) ## !< 0x00000002
  FMC_BTR4_ADDSET_Bit2* = (0x00000004 shl FMC_BTR4_ADDSET_Pos) ## !< 0x00000004
  FMC_BTR4_ADDSET_Bit3* = (0x00000008 shl FMC_BTR4_ADDSET_Pos) ## !< 0x00000008
  FMC_BTR4_ADDHLD_Pos* = (4)
  FMC_BTR4_ADDHLD_Msk* = (0x0000000F shl FMC_BTR4_ADDHLD_Pos) ## !< 0x000000F0
  FMC_BTR4_ADDHLD* = FMC_BTR4_ADDHLD_Msk
  FMC_BTR4_ADDHLD_Bit0* = (0x00000001 shl FMC_BTR4_ADDHLD_Pos) ## !< 0x00000010
  FMC_BTR4_ADDHLD_Bit1* = (0x00000002 shl FMC_BTR4_ADDHLD_Pos) ## !< 0x00000020
  FMC_BTR4_ADDHLD_Bit2* = (0x00000004 shl FMC_BTR4_ADDHLD_Pos) ## !< 0x00000040
  FMC_BTR4_ADDHLD_Bit3* = (0x00000008 shl FMC_BTR4_ADDHLD_Pos) ## !< 0x00000080
  FMC_BTR4_DATAST_Pos* = (8)
  FMC_BTR4_DATAST_Msk* = (0x000000FF shl FMC_BTR4_DATAST_Pos) ## !< 0x0000FF00
  FMC_BTR4_DATAST* = FMC_BTR4_DATAST_Msk
  FMC_BTR4_DATAST_Bit0* = (0x00000001 shl FMC_BTR4_DATAST_Pos) ## !< 0x00000100
  FMC_BTR4_DATAST_Bit1* = (0x00000002 shl FMC_BTR4_DATAST_Pos) ## !< 0x00000200
  FMC_BTR4_DATAST_Bit2* = (0x00000004 shl FMC_BTR4_DATAST_Pos) ## !< 0x00000400
  FMC_BTR4_DATAST_Bit3* = (0x00000008 shl FMC_BTR4_DATAST_Pos) ## !< 0x00000800
  FMC_BTR4_DATAST_Bit4* = (0x00000010 shl FMC_BTR4_DATAST_Pos) ## !< 0x00001000
  FMC_BTR4_DATAST_Bit5* = (0x00000020 shl FMC_BTR4_DATAST_Pos) ## !< 0x00002000
  FMC_BTR4_DATAST_Bit6* = (0x00000040 shl FMC_BTR4_DATAST_Pos) ## !< 0x00004000
  FMC_BTR4_DATAST_Bit7* = (0x00000080 shl FMC_BTR4_DATAST_Pos) ## !< 0x00008000
  FMC_BTR4_BUSTURN_Pos* = (16)
  FMC_BTR4_BUSTURN_Msk* = (0x0000000F shl FMC_BTR4_BUSTURN_Pos) ## !< 0x000F0000
  FMC_BTR4_BUSTURN* = FMC_BTR4_BUSTURN_Msk
  FMC_BTR4_BUSTURN_Bit0* = (0x00000001 shl FMC_BTR4_BUSTURN_Pos) ## !< 0x00010000
  FMC_BTR4_BUSTURN_Bit1* = (0x00000002 shl FMC_BTR4_BUSTURN_Pos) ## !< 0x00020000
  FMC_BTR4_BUSTURN_Bit2* = (0x00000004 shl FMC_BTR4_BUSTURN_Pos) ## !< 0x00040000
  FMC_BTR4_BUSTURN_Bit3* = (0x00000008 shl FMC_BTR4_BUSTURN_Pos) ## !< 0x00080000
  FMC_BTR4_CLKDIV_Pos* = (20)
  FMC_BTR4_CLKDIV_Msk* = (0x0000000F shl FMC_BTR4_CLKDIV_Pos) ## !< 0x00F00000
  FMC_BTR4_CLKDIV* = FMC_BTR4_CLKDIV_Msk
  FMC_BTR4_CLKDIV_Bit0* = (0x00000001 shl FMC_BTR4_CLKDIV_Pos) ## !< 0x00100000
  FMC_BTR4_CLKDIV_Bit1* = (0x00000002 shl FMC_BTR4_CLKDIV_Pos) ## !< 0x00200000
  FMC_BTR4_CLKDIV_Bit2* = (0x00000004 shl FMC_BTR4_CLKDIV_Pos) ## !< 0x00400000
  FMC_BTR4_CLKDIV_Bit3* = (0x00000008 shl FMC_BTR4_CLKDIV_Pos) ## !< 0x00800000
  FMC_BTR4_DATLAT_Pos* = (24)
  FMC_BTR4_DATLAT_Msk* = (0x0000000F shl FMC_BTR4_DATLAT_Pos) ## !< 0x0F000000
  FMC_BTR4_DATLAT* = FMC_BTR4_DATLAT_Msk
  FMC_BTR4_DATLAT_Bit0* = (0x00000001 shl FMC_BTR4_DATLAT_Pos) ## !< 0x01000000
  FMC_BTR4_DATLAT_Bit1* = (0x00000002 shl FMC_BTR4_DATLAT_Pos) ## !< 0x02000000
  FMC_BTR4_DATLAT_Bit2* = (0x00000004 shl FMC_BTR4_DATLAT_Pos) ## !< 0x04000000
  FMC_BTR4_DATLAT_Bit3* = (0x00000008 shl FMC_BTR4_DATLAT_Pos) ## !< 0x08000000
  FMC_BTR4_ACCMOD_Pos* = (28)
  FMC_BTR4_ACCMOD_Msk* = (0x00000003 shl FMC_BTR4_ACCMOD_Pos) ## !< 0x30000000
  FMC_BTR4_ACCMOD* = FMC_BTR4_ACCMOD_Msk
  FMC_BTR4_ACCMOD_Bit0* = (0x00000001 shl FMC_BTR4_ACCMOD_Pos) ## !< 0x10000000
  FMC_BTR4_ACCMOD_Bit1* = (0x00000002 shl FMC_BTR4_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FMC_BWTR1 register  *****************

const
  FMC_BWTR1_ADDSET_Pos* = (0)
  FMC_BWTR1_ADDSET_Msk* = (0x0000000F shl FMC_BWTR1_ADDSET_Pos) ## !< 0x0000000F
  FMC_BWTR1_ADDSET* = FMC_BWTR1_ADDSET_Msk
  FMC_BWTR1_ADDSET_Bit0* = (0x00000001 shl FMC_BWTR1_ADDSET_Pos) ## !< 0x00000001
  FMC_BWTR1_ADDSET_Bit1* = (0x00000002 shl FMC_BWTR1_ADDSET_Pos) ## !< 0x00000002
  FMC_BWTR1_ADDSET_Bit2* = (0x00000004 shl FMC_BWTR1_ADDSET_Pos) ## !< 0x00000004
  FMC_BWTR1_ADDSET_Bit3* = (0x00000008 shl FMC_BWTR1_ADDSET_Pos) ## !< 0x00000008
  FMC_BWTR1_ADDHLD_Pos* = (4)
  FMC_BWTR1_ADDHLD_Msk* = (0x0000000F shl FMC_BWTR1_ADDHLD_Pos) ## !< 0x000000F0
  FMC_BWTR1_ADDHLD* = FMC_BWTR1_ADDHLD_Msk
  FMC_BWTR1_ADDHLD_Bit0* = (0x00000001 shl FMC_BWTR1_ADDHLD_Pos) ## !< 0x00000010
  FMC_BWTR1_ADDHLD_Bit1* = (0x00000002 shl FMC_BWTR1_ADDHLD_Pos) ## !< 0x00000020
  FMC_BWTR1_ADDHLD_Bit2* = (0x00000004 shl FMC_BWTR1_ADDHLD_Pos) ## !< 0x00000040
  FMC_BWTR1_ADDHLD_Bit3* = (0x00000008 shl FMC_BWTR1_ADDHLD_Pos) ## !< 0x00000080
  FMC_BWTR1_DATAST_Pos* = (8)
  FMC_BWTR1_DATAST_Msk* = (0x000000FF shl FMC_BWTR1_DATAST_Pos) ## !< 0x0000FF00
  FMC_BWTR1_DATAST* = FMC_BWTR1_DATAST_Msk
  FMC_BWTR1_DATAST_Bit0* = (0x00000001 shl FMC_BWTR1_DATAST_Pos) ## !< 0x00000100
  FMC_BWTR1_DATAST_Bit1* = (0x00000002 shl FMC_BWTR1_DATAST_Pos) ## !< 0x00000200
  FMC_BWTR1_DATAST_Bit2* = (0x00000004 shl FMC_BWTR1_DATAST_Pos) ## !< 0x00000400
  FMC_BWTR1_DATAST_Bit3* = (0x00000008 shl FMC_BWTR1_DATAST_Pos) ## !< 0x00000800
  FMC_BWTR1_DATAST_Bit4* = (0x00000010 shl FMC_BWTR1_DATAST_Pos) ## !< 0x00001000
  FMC_BWTR1_DATAST_Bit5* = (0x00000020 shl FMC_BWTR1_DATAST_Pos) ## !< 0x00002000
  FMC_BWTR1_DATAST_Bit6* = (0x00000040 shl FMC_BWTR1_DATAST_Pos) ## !< 0x00004000
  FMC_BWTR1_DATAST_Bit7* = (0x00000080 shl FMC_BWTR1_DATAST_Pos) ## !< 0x00008000
  FMC_BWTR1_BUSTURN_Pos* = (16)
  FMC_BWTR1_BUSTURN_Msk* = (0x0000000F shl FMC_BWTR1_BUSTURN_Pos) ## !< 0x000F0000
  FMC_BWTR1_BUSTURN* = FMC_BWTR1_BUSTURN_Msk
  FMC_BWTR1_BUSTURN_Bit0* = (0x00000001 shl FMC_BWTR1_BUSTURN_Pos) ## !< 0x00010000
  FMC_BWTR1_BUSTURN_Bit1* = (0x00000002 shl FMC_BWTR1_BUSTURN_Pos) ## !< 0x00020000
  FMC_BWTR1_BUSTURN_Bit2* = (0x00000004 shl FMC_BWTR1_BUSTURN_Pos) ## !< 0x00040000
  FMC_BWTR1_BUSTURN_Bit3* = (0x00000008 shl FMC_BWTR1_BUSTURN_Pos) ## !< 0x00080000
  FMC_BWTR1_ACCMOD_Pos* = (28)
  FMC_BWTR1_ACCMOD_Msk* = (0x00000003 shl FMC_BWTR1_ACCMOD_Pos) ## !< 0x30000000
  FMC_BWTR1_ACCMOD* = FMC_BWTR1_ACCMOD_Msk
  FMC_BWTR1_ACCMOD_Bit0* = (0x00000001 shl FMC_BWTR1_ACCMOD_Pos) ## !< 0x10000000
  FMC_BWTR1_ACCMOD_Bit1* = (0x00000002 shl FMC_BWTR1_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FMC_BWTR2 register  *****************

const
  FMC_BWTR2_ADDSET_Pos* = (0)
  FMC_BWTR2_ADDSET_Msk* = (0x0000000F shl FMC_BWTR2_ADDSET_Pos) ## !< 0x0000000F
  FMC_BWTR2_ADDSET* = FMC_BWTR2_ADDSET_Msk
  FMC_BWTR2_ADDSET_Bit0* = (0x00000001 shl FMC_BWTR2_ADDSET_Pos) ## !< 0x00000001
  FMC_BWTR2_ADDSET_Bit1* = (0x00000002 shl FMC_BWTR2_ADDSET_Pos) ## !< 0x00000002
  FMC_BWTR2_ADDSET_Bit2* = (0x00000004 shl FMC_BWTR2_ADDSET_Pos) ## !< 0x00000004
  FMC_BWTR2_ADDSET_Bit3* = (0x00000008 shl FMC_BWTR2_ADDSET_Pos) ## !< 0x00000008
  FMC_BWTR2_ADDHLD_Pos* = (4)
  FMC_BWTR2_ADDHLD_Msk* = (0x0000000F shl FMC_BWTR2_ADDHLD_Pos) ## !< 0x000000F0
  FMC_BWTR2_ADDHLD* = FMC_BWTR2_ADDHLD_Msk
  FMC_BWTR2_ADDHLD_Bit0* = (0x00000001 shl FMC_BWTR2_ADDHLD_Pos) ## !< 0x00000010
  FMC_BWTR2_ADDHLD_Bit1* = (0x00000002 shl FMC_BWTR2_ADDHLD_Pos) ## !< 0x00000020
  FMC_BWTR2_ADDHLD_Bit2* = (0x00000004 shl FMC_BWTR2_ADDHLD_Pos) ## !< 0x00000040
  FMC_BWTR2_ADDHLD_Bit3* = (0x00000008 shl FMC_BWTR2_ADDHLD_Pos) ## !< 0x00000080
  FMC_BWTR2_DATAST_Pos* = (8)
  FMC_BWTR2_DATAST_Msk* = (0x000000FF shl FMC_BWTR2_DATAST_Pos) ## !< 0x0000FF00
  FMC_BWTR2_DATAST* = FMC_BWTR2_DATAST_Msk
  FMC_BWTR2_DATAST_Bit0* = (0x00000001 shl FMC_BWTR2_DATAST_Pos) ## !< 0x00000100
  FMC_BWTR2_DATAST_Bit1* = (0x00000002 shl FMC_BWTR2_DATAST_Pos) ## !< 0x00000200
  FMC_BWTR2_DATAST_Bit2* = (0x00000004 shl FMC_BWTR2_DATAST_Pos) ## !< 0x00000400
  FMC_BWTR2_DATAST_Bit3* = (0x00000008 shl FMC_BWTR2_DATAST_Pos) ## !< 0x00000800
  FMC_BWTR2_DATAST_Bit4* = (0x00000010 shl FMC_BWTR2_DATAST_Pos) ## !< 0x00001000
  FMC_BWTR2_DATAST_Bit5* = (0x00000020 shl FMC_BWTR2_DATAST_Pos) ## !< 0x00002000
  FMC_BWTR2_DATAST_Bit6* = (0x00000040 shl FMC_BWTR2_DATAST_Pos) ## !< 0x00004000
  FMC_BWTR2_DATAST_Bit7* = (0x00000080 shl FMC_BWTR2_DATAST_Pos) ## !< 0x00008000
  FMC_BWTR2_BUSTURN_Pos* = (16)
  FMC_BWTR2_BUSTURN_Msk* = (0x0000000F shl FMC_BWTR2_BUSTURN_Pos) ## !< 0x000F0000
  FMC_BWTR2_BUSTURN* = FMC_BWTR2_BUSTURN_Msk
  FMC_BWTR2_BUSTURN_Bit0* = (0x00000001 shl FMC_BWTR2_BUSTURN_Pos) ## !< 0x00010000
  FMC_BWTR2_BUSTURN_Bit1* = (0x00000002 shl FMC_BWTR2_BUSTURN_Pos) ## !< 0x00020000
  FMC_BWTR2_BUSTURN_Bit2* = (0x00000004 shl FMC_BWTR2_BUSTURN_Pos) ## !< 0x00040000
  FMC_BWTR2_BUSTURN_Bit3* = (0x00000008 shl FMC_BWTR2_BUSTURN_Pos) ## !< 0x00080000
  FMC_BWTR2_ACCMOD_Pos* = (28)
  FMC_BWTR2_ACCMOD_Msk* = (0x00000003 shl FMC_BWTR2_ACCMOD_Pos) ## !< 0x30000000
  FMC_BWTR2_ACCMOD* = FMC_BWTR2_ACCMOD_Msk
  FMC_BWTR2_ACCMOD_Bit0* = (0x00000001 shl FMC_BWTR2_ACCMOD_Pos) ## !< 0x10000000
  FMC_BWTR2_ACCMOD_Bit1* = (0x00000002 shl FMC_BWTR2_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FMC_BWTR3 register  *****************

const
  FMC_BWTR3_ADDSET_Pos* = (0)
  FMC_BWTR3_ADDSET_Msk* = (0x0000000F shl FMC_BWTR3_ADDSET_Pos) ## !< 0x0000000F
  FMC_BWTR3_ADDSET* = FMC_BWTR3_ADDSET_Msk
  FMC_BWTR3_ADDSET_Bit0* = (0x00000001 shl FMC_BWTR3_ADDSET_Pos) ## !< 0x00000001
  FMC_BWTR3_ADDSET_Bit1* = (0x00000002 shl FMC_BWTR3_ADDSET_Pos) ## !< 0x00000002
  FMC_BWTR3_ADDSET_Bit2* = (0x00000004 shl FMC_BWTR3_ADDSET_Pos) ## !< 0x00000004
  FMC_BWTR3_ADDSET_Bit3* = (0x00000008 shl FMC_BWTR3_ADDSET_Pos) ## !< 0x00000008
  FMC_BWTR3_ADDHLD_Pos* = (4)
  FMC_BWTR3_ADDHLD_Msk* = (0x0000000F shl FMC_BWTR3_ADDHLD_Pos) ## !< 0x000000F0
  FMC_BWTR3_ADDHLD* = FMC_BWTR3_ADDHLD_Msk
  FMC_BWTR3_ADDHLD_Bit0* = (0x00000001 shl FMC_BWTR3_ADDHLD_Pos) ## !< 0x00000010
  FMC_BWTR3_ADDHLD_Bit1* = (0x00000002 shl FMC_BWTR3_ADDHLD_Pos) ## !< 0x00000020
  FMC_BWTR3_ADDHLD_Bit2* = (0x00000004 shl FMC_BWTR3_ADDHLD_Pos) ## !< 0x00000040
  FMC_BWTR3_ADDHLD_Bit3* = (0x00000008 shl FMC_BWTR3_ADDHLD_Pos) ## !< 0x00000080
  FMC_BWTR3_DATAST_Pos* = (8)
  FMC_BWTR3_DATAST_Msk* = (0x000000FF shl FMC_BWTR3_DATAST_Pos) ## !< 0x0000FF00
  FMC_BWTR3_DATAST* = FMC_BWTR3_DATAST_Msk
  FMC_BWTR3_DATAST_Bit0* = (0x00000001 shl FMC_BWTR3_DATAST_Pos) ## !< 0x00000100
  FMC_BWTR3_DATAST_Bit1* = (0x00000002 shl FMC_BWTR3_DATAST_Pos) ## !< 0x00000200
  FMC_BWTR3_DATAST_Bit2* = (0x00000004 shl FMC_BWTR3_DATAST_Pos) ## !< 0x00000400
  FMC_BWTR3_DATAST_Bit3* = (0x00000008 shl FMC_BWTR3_DATAST_Pos) ## !< 0x00000800
  FMC_BWTR3_DATAST_Bit4* = (0x00000010 shl FMC_BWTR3_DATAST_Pos) ## !< 0x00001000
  FMC_BWTR3_DATAST_Bit5* = (0x00000020 shl FMC_BWTR3_DATAST_Pos) ## !< 0x00002000
  FMC_BWTR3_DATAST_Bit6* = (0x00000040 shl FMC_BWTR3_DATAST_Pos) ## !< 0x00004000
  FMC_BWTR3_DATAST_Bit7* = (0x00000080 shl FMC_BWTR3_DATAST_Pos) ## !< 0x00008000
  FMC_BWTR3_BUSTURN_Pos* = (16)
  FMC_BWTR3_BUSTURN_Msk* = (0x0000000F shl FMC_BWTR3_BUSTURN_Pos) ## !< 0x000F0000
  FMC_BWTR3_BUSTURN* = FMC_BWTR3_BUSTURN_Msk
  FMC_BWTR3_BUSTURN_Bit0* = (0x00000001 shl FMC_BWTR3_BUSTURN_Pos) ## !< 0x00010000
  FMC_BWTR3_BUSTURN_Bit1* = (0x00000002 shl FMC_BWTR3_BUSTURN_Pos) ## !< 0x00020000
  FMC_BWTR3_BUSTURN_Bit2* = (0x00000004 shl FMC_BWTR3_BUSTURN_Pos) ## !< 0x00040000
  FMC_BWTR3_BUSTURN_Bit3* = (0x00000008 shl FMC_BWTR3_BUSTURN_Pos) ## !< 0x00080000
  FMC_BWTR3_ACCMOD_Pos* = (28)
  FMC_BWTR3_ACCMOD_Msk* = (0x00000003 shl FMC_BWTR3_ACCMOD_Pos) ## !< 0x30000000
  FMC_BWTR3_ACCMOD* = FMC_BWTR3_ACCMOD_Msk
  FMC_BWTR3_ACCMOD_Bit0* = (0x00000001 shl FMC_BWTR3_ACCMOD_Pos) ## !< 0x10000000
  FMC_BWTR3_ACCMOD_Bit1* = (0x00000002 shl FMC_BWTR3_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FMC_BWTR4 register  *****************

const
  FMC_BWTR4_ADDSET_Pos* = (0)
  FMC_BWTR4_ADDSET_Msk* = (0x0000000F shl FMC_BWTR4_ADDSET_Pos) ## !< 0x0000000F
  FMC_BWTR4_ADDSET* = FMC_BWTR4_ADDSET_Msk
  FMC_BWTR4_ADDSET_Bit0* = (0x00000001 shl FMC_BWTR4_ADDSET_Pos) ## !< 0x00000001
  FMC_BWTR4_ADDSET_Bit1* = (0x00000002 shl FMC_BWTR4_ADDSET_Pos) ## !< 0x00000002
  FMC_BWTR4_ADDSET_Bit2* = (0x00000004 shl FMC_BWTR4_ADDSET_Pos) ## !< 0x00000004
  FMC_BWTR4_ADDSET_Bit3* = (0x00000008 shl FMC_BWTR4_ADDSET_Pos) ## !< 0x00000008
  FMC_BWTR4_ADDHLD_Pos* = (4)
  FMC_BWTR4_ADDHLD_Msk* = (0x0000000F shl FMC_BWTR4_ADDHLD_Pos) ## !< 0x000000F0
  FMC_BWTR4_ADDHLD* = FMC_BWTR4_ADDHLD_Msk
  FMC_BWTR4_ADDHLD_Bit0* = (0x00000001 shl FMC_BWTR4_ADDHLD_Pos) ## !< 0x00000010
  FMC_BWTR4_ADDHLD_Bit1* = (0x00000002 shl FMC_BWTR4_ADDHLD_Pos) ## !< 0x00000020
  FMC_BWTR4_ADDHLD_Bit2* = (0x00000004 shl FMC_BWTR4_ADDHLD_Pos) ## !< 0x00000040
  FMC_BWTR4_ADDHLD_Bit3* = (0x00000008 shl FMC_BWTR4_ADDHLD_Pos) ## !< 0x00000080
  FMC_BWTR4_DATAST_Pos* = (8)
  FMC_BWTR4_DATAST_Msk* = (0x000000FF shl FMC_BWTR4_DATAST_Pos) ## !< 0x0000FF00
  FMC_BWTR4_DATAST* = FMC_BWTR4_DATAST_Msk
  FMC_BWTR4_DATAST_Bit0* = (0x00000001 shl FMC_BWTR4_DATAST_Pos) ## !< 0x00000100
  FMC_BWTR4_DATAST_Bit1* = (0x00000002 shl FMC_BWTR4_DATAST_Pos) ## !< 0x00000200
  FMC_BWTR4_DATAST_Bit2* = (0x00000004 shl FMC_BWTR4_DATAST_Pos) ## !< 0x00000400
  FMC_BWTR4_DATAST_Bit3* = (0x00000008 shl FMC_BWTR4_DATAST_Pos) ## !< 0x00000800
  FMC_BWTR4_DATAST_Bit4* = (0x00000010 shl FMC_BWTR4_DATAST_Pos) ## !< 0x00001000
  FMC_BWTR4_DATAST_Bit5* = (0x00000020 shl FMC_BWTR4_DATAST_Pos) ## !< 0x00002000
  FMC_BWTR4_DATAST_Bit6* = (0x00000040 shl FMC_BWTR4_DATAST_Pos) ## !< 0x00004000
  FMC_BWTR4_DATAST_Bit7* = (0x00000080 shl FMC_BWTR4_DATAST_Pos) ## !< 0x00008000
  FMC_BWTR4_BUSTURN_Pos* = (16)
  FMC_BWTR4_BUSTURN_Msk* = (0x0000000F shl FMC_BWTR4_BUSTURN_Pos) ## !< 0x000F0000
  FMC_BWTR4_BUSTURN* = FMC_BWTR4_BUSTURN_Msk
  FMC_BWTR4_BUSTURN_Bit0* = (0x00000001 shl FMC_BWTR4_BUSTURN_Pos) ## !< 0x00010000
  FMC_BWTR4_BUSTURN_Bit1* = (0x00000002 shl FMC_BWTR4_BUSTURN_Pos) ## !< 0x00020000
  FMC_BWTR4_BUSTURN_Bit2* = (0x00000004 shl FMC_BWTR4_BUSTURN_Pos) ## !< 0x00040000
  FMC_BWTR4_BUSTURN_Bit3* = (0x00000008 shl FMC_BWTR4_BUSTURN_Pos) ## !< 0x00080000
  FMC_BWTR4_ACCMOD_Pos* = (28)
  FMC_BWTR4_ACCMOD_Msk* = (0x00000003 shl FMC_BWTR4_ACCMOD_Pos) ## !< 0x30000000
  FMC_BWTR4_ACCMOD* = FMC_BWTR4_ACCMOD_Msk
  FMC_BWTR4_ACCMOD_Bit0* = (0x00000001 shl FMC_BWTR4_ACCMOD_Pos) ## !< 0x10000000
  FMC_BWTR4_ACCMOD_Bit1* = (0x00000002 shl FMC_BWTR4_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FMC_PCR register  ******************

const
  FMC_PCR_PWAITEN_Pos* = (1)
  FMC_PCR_PWAITEN_Msk* = (0x00000001 shl FMC_PCR_PWAITEN_Pos) ## !< 0x00000002
  FMC_PCR_PWAITEN* = FMC_PCR_PWAITEN_Msk
  FMC_PCR_PBKEN_Pos* = (2)
  FMC_PCR_PBKEN_Msk* = (0x00000001 shl FMC_PCR_PBKEN_Pos) ## !< 0x00000004
  FMC_PCR_PBKEN* = FMC_PCR_PBKEN_Msk
  FMC_PCR_PTYP_Pos* = (3)
  FMC_PCR_PTYP_Msk* = (0x00000001 shl FMC_PCR_PTYP_Pos) ## !< 0x00000008
  FMC_PCR_PTYP* = FMC_PCR_PTYP_Msk
  FMC_PCR_PWID_Pos* = (4)
  FMC_PCR_PWID_Msk* = (0x00000003 shl FMC_PCR_PWID_Pos) ## !< 0x00000030
  FMC_PCR_PWID* = FMC_PCR_PWID_Msk
  FMC_PCR_PWID_Bit0* = (0x00000001 shl FMC_PCR_PWID_Pos) ## !< 0x00000010
  FMC_PCR_PWID_Bit1* = (0x00000002 shl FMC_PCR_PWID_Pos) ## !< 0x00000020
  FMC_PCR_ECCEN_Pos* = (6)
  FMC_PCR_ECCEN_Msk* = (0x00000001 shl FMC_PCR_ECCEN_Pos) ## !< 0x00000040
  FMC_PCR_ECCEN* = FMC_PCR_ECCEN_Msk
  FMC_PCR_TCLR_Pos* = (9)
  FMC_PCR_TCLR_Msk* = (0x0000000F shl FMC_PCR_TCLR_Pos) ## !< 0x00001E00
  FMC_PCR_TCLR* = FMC_PCR_TCLR_Msk
  FMC_PCR_TCLR_Bit0* = (0x00000001 shl FMC_PCR_TCLR_Pos) ## !< 0x00000200
  FMC_PCR_TCLR_Bit1* = (0x00000002 shl FMC_PCR_TCLR_Pos) ## !< 0x00000400
  FMC_PCR_TCLR_Bit2* = (0x00000004 shl FMC_PCR_TCLR_Pos) ## !< 0x00000800
  FMC_PCR_TCLR_Bit3* = (0x00000008 shl FMC_PCR_TCLR_Pos) ## !< 0x00001000
  FMC_PCR_TAR_Pos* = (13)
  FMC_PCR_TAR_Msk* = (0x0000000F shl FMC_PCR_TAR_Pos) ## !< 0x0001E000
  FMC_PCR_TAR* = FMC_PCR_TAR_Msk
  FMC_PCR_TAR_Bit0* = (0x00000001 shl FMC_PCR_TAR_Pos) ## !< 0x00002000
  FMC_PCR_TAR_Bit1* = (0x00000002 shl FMC_PCR_TAR_Pos) ## !< 0x00004000
  FMC_PCR_TAR_Bit2* = (0x00000004 shl FMC_PCR_TAR_Pos) ## !< 0x00008000
  FMC_PCR_TAR_Bit3* = (0x00000008 shl FMC_PCR_TAR_Pos) ## !< 0x00010000
  FMC_PCR_ECCPS_Pos* = (17)
  FMC_PCR_ECCPS_Msk* = (0x00000007 shl FMC_PCR_ECCPS_Pos) ## !< 0x000E0000
  FMC_PCR_ECCPS* = FMC_PCR_ECCPS_Msk
  FMC_PCR_ECCPS_Bit0* = (0x00000001 shl FMC_PCR_ECCPS_Pos) ## !< 0x00020000
  FMC_PCR_ECCPS_Bit1* = (0x00000002 shl FMC_PCR_ECCPS_Pos) ## !< 0x00040000
  FMC_PCR_ECCPS_Bit2* = (0x00000004 shl FMC_PCR_ECCPS_Pos) ## !< 0x00080000

## ******************  Bit definition for FMC_SR register  ******************

const
  FMC_SR_IRS_Pos* = (0)
  FMC_SR_IRS_Msk* = (0x00000001 shl FMC_SR_IRS_Pos) ## !< 0x00000001
  FMC_SR_IRS* = FMC_SR_IRS_Msk
  FMC_SR_ILS_Pos* = (1)
  FMC_SR_ILS_Msk* = (0x00000001 shl FMC_SR_ILS_Pos) ## !< 0x00000002
  FMC_SR_ILS* = FMC_SR_ILS_Msk
  FMC_SR_IFS_Pos* = (2)
  FMC_SR_IFS_Msk* = (0x00000001 shl FMC_SR_IFS_Pos) ## !< 0x00000004
  FMC_SR_IFS* = FMC_SR_IFS_Msk
  FMC_SR_IREN_Pos* = (3)
  FMC_SR_IREN_Msk* = (0x00000001 shl FMC_SR_IREN_Pos) ## !< 0x00000008
  FMC_SR_IREN* = FMC_SR_IREN_Msk
  FMC_SR_ILEN_Pos* = (4)
  FMC_SR_ILEN_Msk* = (0x00000001 shl FMC_SR_ILEN_Pos) ## !< 0x00000010
  FMC_SR_ILEN* = FMC_SR_ILEN_Msk
  FMC_SR_IFEN_Pos* = (5)
  FMC_SR_IFEN_Msk* = (0x00000001 shl FMC_SR_IFEN_Pos) ## !< 0x00000020
  FMC_SR_IFEN* = FMC_SR_IFEN_Msk
  FMC_SR_FEMPT_Pos* = (6)
  FMC_SR_FEMPT_Msk* = (0x00000001 shl FMC_SR_FEMPT_Pos) ## !< 0x00000040
  FMC_SR_FEMPT* = FMC_SR_FEMPT_Msk

## *****************  Bit definition for FMC_PMEM register  *****************

const
  FMC_PMEM_MEMSET2_Pos* = (0)
  FMC_PMEM_MEMSET2_Msk* = (0x000000FF shl FMC_PMEM_MEMSET2_Pos) ## !< 0x000000FF
  FMC_PMEM_MEMSET2* = FMC_PMEM_MEMSET2_Msk
  FMC_PMEM_MEMSET2_Bit0* = (0x00000001 shl FMC_PMEM_MEMSET2_Pos) ## !< 0x00000001
  FMC_PMEM_MEMSET2_Bit1* = (0x00000002 shl FMC_PMEM_MEMSET2_Pos) ## !< 0x00000002
  FMC_PMEM_MEMSET2_Bit2* = (0x00000004 shl FMC_PMEM_MEMSET2_Pos) ## !< 0x00000004
  FMC_PMEM_MEMSET2_Bit3* = (0x00000008 shl FMC_PMEM_MEMSET2_Pos) ## !< 0x00000008
  FMC_PMEM_MEMSET2_Bit4* = (0x00000010 shl FMC_PMEM_MEMSET2_Pos) ## !< 0x00000010
  FMC_PMEM_MEMSET2_Bit5* = (0x00000020 shl FMC_PMEM_MEMSET2_Pos) ## !< 0x00000020
  FMC_PMEM_MEMSET2_Bit6* = (0x00000040 shl FMC_PMEM_MEMSET2_Pos) ## !< 0x00000040
  FMC_PMEM_MEMSET2_Bit7* = (0x00000080 shl FMC_PMEM_MEMSET2_Pos) ## !< 0x00000080
  FMC_PMEM_MEMWAIT2_Pos* = (8)
  FMC_PMEM_MEMWAIT2_Msk* = (0x000000FF shl FMC_PMEM_MEMWAIT2_Pos) ## !< 0x0000FF00
  FMC_PMEM_MEMWAIT2* = FMC_PMEM_MEMWAIT2_Msk
  FMC_PMEM_MEMWAIT2_Bit0* = (0x00000001 shl FMC_PMEM_MEMWAIT2_Pos) ## !< 0x00000100
  FMC_PMEM_MEMWAIT2_Bit1* = (0x00000002 shl FMC_PMEM_MEMWAIT2_Pos) ## !< 0x00000200
  FMC_PMEM_MEMWAIT2_Bit2* = (0x00000004 shl FMC_PMEM_MEMWAIT2_Pos) ## !< 0x00000400
  FMC_PMEM_MEMWAIT2_Bit3* = (0x00000008 shl FMC_PMEM_MEMWAIT2_Pos) ## !< 0x00000800
  FMC_PMEM_MEMWAIT2_Bit4* = (0x00000010 shl FMC_PMEM_MEMWAIT2_Pos) ## !< 0x00001000
  FMC_PMEM_MEMWAIT2_Bit5* = (0x00000020 shl FMC_PMEM_MEMWAIT2_Pos) ## !< 0x00002000
  FMC_PMEM_MEMWAIT2_Bit6* = (0x00000040 shl FMC_PMEM_MEMWAIT2_Pos) ## !< 0x00004000
  FMC_PMEM_MEMWAIT2_Bit7* = (0x00000080 shl FMC_PMEM_MEMWAIT2_Pos) ## !< 0x00008000
  FMC_PMEM_MEMHOLD2_Pos* = (16)
  FMC_PMEM_MEMHOLD2_Msk* = (0x000000FF shl FMC_PMEM_MEMHOLD2_Pos) ## !< 0x00FF0000
  FMC_PMEM_MEMHOLD2* = FMC_PMEM_MEMHOLD2_Msk
  FMC_PMEM_MEMHOLD2_Bit0* = (0x00000001 shl FMC_PMEM_MEMHOLD2_Pos) ## !< 0x00010000
  FMC_PMEM_MEMHOLD2_Bit1* = (0x00000002 shl FMC_PMEM_MEMHOLD2_Pos) ## !< 0x00020000
  FMC_PMEM_MEMHOLD2_Bit2* = (0x00000004 shl FMC_PMEM_MEMHOLD2_Pos) ## !< 0x00040000
  FMC_PMEM_MEMHOLD2_Bit3* = (0x00000008 shl FMC_PMEM_MEMHOLD2_Pos) ## !< 0x00080000
  FMC_PMEM_MEMHOLD2_Bit4* = (0x00000010 shl FMC_PMEM_MEMHOLD2_Pos) ## !< 0x00100000
  FMC_PMEM_MEMHOLD2_Bit5* = (0x00000020 shl FMC_PMEM_MEMHOLD2_Pos) ## !< 0x00200000
  FMC_PMEM_MEMHOLD2_Bit6* = (0x00000040 shl FMC_PMEM_MEMHOLD2_Pos) ## !< 0x00400000
  FMC_PMEM_MEMHOLD2_Bit7* = (0x00000080 shl FMC_PMEM_MEMHOLD2_Pos) ## !< 0x00800000
  FMC_PMEM_MEMHIZ2_Pos* = (24)
  FMC_PMEM_MEMHIZ2_Msk* = (0x000000FF shl FMC_PMEM_MEMHIZ2_Pos) ## !< 0xFF000000
  FMC_PMEM_MEMHIZ2* = FMC_PMEM_MEMHIZ2_Msk
  FMC_PMEM_MEMHIZ2_Bit0* = (0x00000001 shl FMC_PMEM_MEMHIZ2_Pos) ## !< 0x01000000
  FMC_PMEM_MEMHIZ2_Bit1* = (0x00000002 shl FMC_PMEM_MEMHIZ2_Pos) ## !< 0x02000000
  FMC_PMEM_MEMHIZ2_Bit2* = (0x00000004 shl FMC_PMEM_MEMHIZ2_Pos) ## !< 0x04000000
  FMC_PMEM_MEMHIZ2_Bit3* = (0x00000008 shl FMC_PMEM_MEMHIZ2_Pos) ## !< 0x08000000
  FMC_PMEM_MEMHIZ2_Bit4* = (0x00000010 shl FMC_PMEM_MEMHIZ2_Pos) ## !< 0x10000000
  FMC_PMEM_MEMHIZ2_Bit5* = (0x00000020 shl FMC_PMEM_MEMHIZ2_Pos) ## !< 0x20000000
  FMC_PMEM_MEMHIZ2_Bit6* = (0x00000040 shl FMC_PMEM_MEMHIZ2_Pos) ## !< 0x40000000
  FMC_PMEM_MEMHIZ2_Bit7* = (0x00000080 shl FMC_PMEM_MEMHIZ2_Pos) ## !< 0x80000000

## *****************  Bit definition for FMC_PATT register  *****************

const
  FMC_PATT_ATTSET2_Pos* = (0)
  FMC_PATT_ATTSET2_Msk* = (0x000000FF shl FMC_PATT_ATTSET2_Pos) ## !< 0x000000FF
  FMC_PATT_ATTSET2* = FMC_PATT_ATTSET2_Msk
  FMC_PATT_ATTSET2_Bit0* = (0x00000001 shl FMC_PATT_ATTSET2_Pos) ## !< 0x00000001
  FMC_PATT_ATTSET2_Bit1* = (0x00000002 shl FMC_PATT_ATTSET2_Pos) ## !< 0x00000002
  FMC_PATT_ATTSET2_Bit2* = (0x00000004 shl FMC_PATT_ATTSET2_Pos) ## !< 0x00000004
  FMC_PATT_ATTSET2_Bit3* = (0x00000008 shl FMC_PATT_ATTSET2_Pos) ## !< 0x00000008
  FMC_PATT_ATTSET2_Bit4* = (0x00000010 shl FMC_PATT_ATTSET2_Pos) ## !< 0x00000010
  FMC_PATT_ATTSET2_Bit5* = (0x00000020 shl FMC_PATT_ATTSET2_Pos) ## !< 0x00000020
  FMC_PATT_ATTSET2_Bit6* = (0x00000040 shl FMC_PATT_ATTSET2_Pos) ## !< 0x00000040
  FMC_PATT_ATTSET2_Bit7* = (0x00000080 shl FMC_PATT_ATTSET2_Pos) ## !< 0x00000080
  FMC_PATT_ATTWAIT2_Pos* = (8)
  FMC_PATT_ATTWAIT2_Msk* = (0x000000FF shl FMC_PATT_ATTWAIT2_Pos) ## !< 0x0000FF00
  FMC_PATT_ATTWAIT2* = FMC_PATT_ATTWAIT2_Msk
  FMC_PATT_ATTWAIT2_Bit0* = (0x00000001 shl FMC_PATT_ATTWAIT2_Pos) ## !< 0x00000100
  FMC_PATT_ATTWAIT2_Bit1* = (0x00000002 shl FMC_PATT_ATTWAIT2_Pos) ## !< 0x00000200
  FMC_PATT_ATTWAIT2_Bit2* = (0x00000004 shl FMC_PATT_ATTWAIT2_Pos) ## !< 0x00000400
  FMC_PATT_ATTWAIT2_Bit3* = (0x00000008 shl FMC_PATT_ATTWAIT2_Pos) ## !< 0x00000800
  FMC_PATT_ATTWAIT2_Bit4* = (0x00000010 shl FMC_PATT_ATTWAIT2_Pos) ## !< 0x00001000
  FMC_PATT_ATTWAIT2_Bit5* = (0x00000020 shl FMC_PATT_ATTWAIT2_Pos) ## !< 0x00002000
  FMC_PATT_ATTWAIT2_Bit6* = (0x00000040 shl FMC_PATT_ATTWAIT2_Pos) ## !< 0x00004000
  FMC_PATT_ATTWAIT2_Bit7* = (0x00000080 shl FMC_PATT_ATTWAIT2_Pos) ## !< 0x00008000
  FMC_PATT_ATTHOLD2_Pos* = (16)
  FMC_PATT_ATTHOLD2_Msk* = (0x000000FF shl FMC_PATT_ATTHOLD2_Pos) ## !< 0x00FF0000
  FMC_PATT_ATTHOLD2* = FMC_PATT_ATTHOLD2_Msk
  FMC_PATT_ATTHOLD2_Bit0* = (0x00000001 shl FMC_PATT_ATTHOLD2_Pos) ## !< 0x00010000
  FMC_PATT_ATTHOLD2_Bit1* = (0x00000002 shl FMC_PATT_ATTHOLD2_Pos) ## !< 0x00020000
  FMC_PATT_ATTHOLD2_Bit2* = (0x00000004 shl FMC_PATT_ATTHOLD2_Pos) ## !< 0x00040000
  FMC_PATT_ATTHOLD2_Bit3* = (0x00000008 shl FMC_PATT_ATTHOLD2_Pos) ## !< 0x00080000
  FMC_PATT_ATTHOLD2_Bit4* = (0x00000010 shl FMC_PATT_ATTHOLD2_Pos) ## !< 0x00100000
  FMC_PATT_ATTHOLD2_Bit5* = (0x00000020 shl FMC_PATT_ATTHOLD2_Pos) ## !< 0x00200000
  FMC_PATT_ATTHOLD2_Bit6* = (0x00000040 shl FMC_PATT_ATTHOLD2_Pos) ## !< 0x00400000
  FMC_PATT_ATTHOLD2_Bit7* = (0x00000080 shl FMC_PATT_ATTHOLD2_Pos) ## !< 0x00800000
  FMC_PATT_ATTHIZ2_Pos* = (24)
  FMC_PATT_ATTHIZ2_Msk* = (0x000000FF shl FMC_PATT_ATTHIZ2_Pos) ## !< 0xFF000000
  FMC_PATT_ATTHIZ2* = FMC_PATT_ATTHIZ2_Msk
  FMC_PATT_ATTHIZ2_Bit0* = (0x00000001 shl FMC_PATT_ATTHIZ2_Pos) ## !< 0x01000000
  FMC_PATT_ATTHIZ2_Bit1* = (0x00000002 shl FMC_PATT_ATTHIZ2_Pos) ## !< 0x02000000
  FMC_PATT_ATTHIZ2_Bit2* = (0x00000004 shl FMC_PATT_ATTHIZ2_Pos) ## !< 0x04000000
  FMC_PATT_ATTHIZ2_Bit3* = (0x00000008 shl FMC_PATT_ATTHIZ2_Pos) ## !< 0x08000000
  FMC_PATT_ATTHIZ2_Bit4* = (0x00000010 shl FMC_PATT_ATTHIZ2_Pos) ## !< 0x10000000
  FMC_PATT_ATTHIZ2_Bit5* = (0x00000020 shl FMC_PATT_ATTHIZ2_Pos) ## !< 0x20000000
  FMC_PATT_ATTHIZ2_Bit6* = (0x00000040 shl FMC_PATT_ATTHIZ2_Pos) ## !< 0x40000000
  FMC_PATT_ATTHIZ2_Bit7* = (0x00000080 shl FMC_PATT_ATTHIZ2_Pos) ## !< 0x80000000

## *****************  Bit definition for FMC_ECCR register  *****************

const
  FMC_ECCR_ECC2_Pos* = (0)
  FMC_ECCR_ECC2_Msk* = (0xFFFFFFFF shl FMC_ECCR_ECC2_Pos) ## !< 0xFFFFFFFF
  FMC_ECCR_ECC2* = FMC_ECCR_ECC2_Msk

## *****************  Bit definition for FMC_SDCR1 register  *****************

const
  FMC_SDCR1_NC_Pos* = (0)
  FMC_SDCR1_NC_Msk* = (0x00000003 shl FMC_SDCR1_NC_Pos) ## !< 0x00000003
  FMC_SDCR1_NC* = FMC_SDCR1_NC_Msk
  FMC_SDCR1_NC_Bit0* = (0x00000001 shl FMC_SDCR1_NC_Pos) ## !< 0x00000001
  FMC_SDCR1_NC_Bit1* = (0x00000002 shl FMC_SDCR1_NC_Pos) ## !< 0x00000002
  FMC_SDCR1_NR_Pos* = (2)
  FMC_SDCR1_NR_Msk* = (0x00000003 shl FMC_SDCR1_NR_Pos) ## !< 0x0000000C
  FMC_SDCR1_NR* = FMC_SDCR1_NR_Msk
  FMC_SDCR1_NR_Bit0* = (0x00000001 shl FMC_SDCR1_NR_Pos) ## !< 0x00000004
  FMC_SDCR1_NR_Bit1* = (0x00000002 shl FMC_SDCR1_NR_Pos) ## !< 0x00000008
  FMC_SDCR1_MWID_Pos* = (4)
  FMC_SDCR1_MWID_Msk* = (0x00000003 shl FMC_SDCR1_MWID_Pos) ## !< 0x00000030
  FMC_SDCR1_MWID* = FMC_SDCR1_MWID_Msk
  FMC_SDCR1_MWID_Bit0* = (0x00000001 shl FMC_SDCR1_MWID_Pos) ## !< 0x00000010
  FMC_SDCR1_MWID_Bit1* = (0x00000002 shl FMC_SDCR1_MWID_Pos) ## !< 0x00000020
  FMC_SDCR1_NB_Pos* = (6)
  FMC_SDCR1_NB_Msk* = (0x00000001 shl FMC_SDCR1_NB_Pos) ## !< 0x00000040
  FMC_SDCR1_NB* = FMC_SDCR1_NB_Msk
  FMC_SDCR1_CAS_Pos* = (7)
  FMC_SDCR1_CAS_Msk* = (0x00000003 shl FMC_SDCR1_CAS_Pos) ## !< 0x00000180
  FMC_SDCR1_CAS* = FMC_SDCR1_CAS_Msk
  FMC_SDCR1_CAS_Bit0* = (0x00000001 shl FMC_SDCR1_CAS_Pos) ## !< 0x00000080
  FMC_SDCR1_CAS_Bit1* = (0x00000002 shl FMC_SDCR1_CAS_Pos) ## !< 0x00000100
  FMC_SDCR1_WP_Pos* = (9)
  FMC_SDCR1_WP_Msk* = (0x00000001 shl FMC_SDCR1_WP_Pos) ## !< 0x00000200
  FMC_SDCR1_WP* = FMC_SDCR1_WP_Msk
  FMC_SDCR1_SDCLK_Pos* = (10)
  FMC_SDCR1_SDCLK_Msk* = (0x00000003 shl FMC_SDCR1_SDCLK_Pos) ## !< 0x00000C00
  FMC_SDCR1_SDCLK* = FMC_SDCR1_SDCLK_Msk
  FMC_SDCR1_SDCLK_Bit0* = (0x00000001 shl FMC_SDCR1_SDCLK_Pos) ## !< 0x00000400
  FMC_SDCR1_SDCLK_Bit1* = (0x00000002 shl FMC_SDCR1_SDCLK_Pos) ## !< 0x00000800
  FMC_SDCR1_RBURST_Pos* = (12)
  FMC_SDCR1_RBURST_Msk* = (0x00000001 shl FMC_SDCR1_RBURST_Pos) ## !< 0x00001000
  FMC_SDCR1_RBURST* = FMC_SDCR1_RBURST_Msk
  FMC_SDCR1_RPIPE_Pos* = (13)
  FMC_SDCR1_RPIPE_Msk* = (0x00000003 shl FMC_SDCR1_RPIPE_Pos) ## !< 0x00006000
  FMC_SDCR1_RPIPE* = FMC_SDCR1_RPIPE_Msk
  FMC_SDCR1_RPIPE_Bit0* = (0x00000001 shl FMC_SDCR1_RPIPE_Pos) ## !< 0x00002000
  FMC_SDCR1_RPIPE_Bit1* = (0x00000002 shl FMC_SDCR1_RPIPE_Pos) ## !< 0x00004000

## *****************  Bit definition for FMC_SDCR2 register  *****************

const
  FMC_SDCR2_NC_Pos* = (0)
  FMC_SDCR2_NC_Msk* = (0x00000003 shl FMC_SDCR2_NC_Pos) ## !< 0x00000003
  FMC_SDCR2_NC* = FMC_SDCR2_NC_Msk
  FMC_SDCR2_NC_Bit0* = (0x00000001 shl FMC_SDCR2_NC_Pos) ## !< 0x00000001
  FMC_SDCR2_NC_Bit1* = (0x00000002 shl FMC_SDCR2_NC_Pos) ## !< 0x00000002
  FMC_SDCR2_NR_Pos* = (2)
  FMC_SDCR2_NR_Msk* = (0x00000003 shl FMC_SDCR2_NR_Pos) ## !< 0x0000000C
  FMC_SDCR2_NR* = FMC_SDCR2_NR_Msk
  FMC_SDCR2_NR_Bit0* = (0x00000001 shl FMC_SDCR2_NR_Pos) ## !< 0x00000004
  FMC_SDCR2_NR_Bit1* = (0x00000002 shl FMC_SDCR2_NR_Pos) ## !< 0x00000008
  FMC_SDCR2_MWID_Pos* = (4)
  FMC_SDCR2_MWID_Msk* = (0x00000003 shl FMC_SDCR2_MWID_Pos) ## !< 0x00000030
  FMC_SDCR2_MWID* = FMC_SDCR2_MWID_Msk
  FMC_SDCR2_MWID_Bit0* = (0x00000001 shl FMC_SDCR2_MWID_Pos) ## !< 0x00000010
  FMC_SDCR2_MWID_Bit1* = (0x00000002 shl FMC_SDCR2_MWID_Pos) ## !< 0x00000020
  FMC_SDCR2_NB_Pos* = (6)
  FMC_SDCR2_NB_Msk* = (0x00000001 shl FMC_SDCR2_NB_Pos) ## !< 0x00000040
  FMC_SDCR2_NB* = FMC_SDCR2_NB_Msk
  FMC_SDCR2_CAS_Pos* = (7)
  FMC_SDCR2_CAS_Msk* = (0x00000003 shl FMC_SDCR2_CAS_Pos) ## !< 0x00000180
  FMC_SDCR2_CAS* = FMC_SDCR2_CAS_Msk
  FMC_SDCR2_CAS_Bit0* = (0x00000001 shl FMC_SDCR2_CAS_Pos) ## !< 0x00000080
  FMC_SDCR2_CAS_Bit1* = (0x00000002 shl FMC_SDCR2_CAS_Pos) ## !< 0x00000100
  FMC_SDCR2_WP_Pos* = (9)
  FMC_SDCR2_WP_Msk* = (0x00000001 shl FMC_SDCR2_WP_Pos) ## !< 0x00000200
  FMC_SDCR2_WP* = FMC_SDCR2_WP_Msk
  FMC_SDCR2_SDCLK_Pos* = (10)
  FMC_SDCR2_SDCLK_Msk* = (0x00000003 shl FMC_SDCR2_SDCLK_Pos) ## !< 0x00000C00
  FMC_SDCR2_SDCLK* = FMC_SDCR2_SDCLK_Msk
  FMC_SDCR2_SDCLK_Bit0* = (0x00000001 shl FMC_SDCR2_SDCLK_Pos) ## !< 0x00000400
  FMC_SDCR2_SDCLK_Bit1* = (0x00000002 shl FMC_SDCR2_SDCLK_Pos) ## !< 0x00000800
  FMC_SDCR2_RBURST_Pos* = (12)
  FMC_SDCR2_RBURST_Msk* = (0x00000001 shl FMC_SDCR2_RBURST_Pos) ## !< 0x00001000
  FMC_SDCR2_RBURST* = FMC_SDCR2_RBURST_Msk
  FMC_SDCR2_RPIPE_Pos* = (13)
  FMC_SDCR2_RPIPE_Msk* = (0x00000003 shl FMC_SDCR2_RPIPE_Pos) ## !< 0x00006000
  FMC_SDCR2_RPIPE* = FMC_SDCR2_RPIPE_Msk
  FMC_SDCR2_RPIPE_Bit0* = (0x00000001 shl FMC_SDCR2_RPIPE_Pos) ## !< 0x00002000
  FMC_SDCR2_RPIPE_Bit1* = (0x00000002 shl FMC_SDCR2_RPIPE_Pos) ## !< 0x00004000

## *****************  Bit definition for FMC_SDTR1 register  *****************

const
  FMC_SDTR1_TMRD_Pos* = (0)
  FMC_SDTR1_TMRD_Msk* = (0x0000000F shl FMC_SDTR1_TMRD_Pos) ## !< 0x0000000F
  FMC_SDTR1_TMRD* = FMC_SDTR1_TMRD_Msk
  FMC_SDTR1_TMRD_Bit0* = (0x00000001 shl FMC_SDTR1_TMRD_Pos) ## !< 0x00000001
  FMC_SDTR1_TMRD_Bit1* = (0x00000002 shl FMC_SDTR1_TMRD_Pos) ## !< 0x00000002
  FMC_SDTR1_TMRD_Bit2* = (0x00000004 shl FMC_SDTR1_TMRD_Pos) ## !< 0x00000004
  FMC_SDTR1_TMRD_Bit3* = (0x00000008 shl FMC_SDTR1_TMRD_Pos) ## !< 0x00000008
  FMC_SDTR1_TXSR_Pos* = (4)
  FMC_SDTR1_TXSR_Msk* = (0x0000000F shl FMC_SDTR1_TXSR_Pos) ## !< 0x000000F0
  FMC_SDTR1_TXSR* = FMC_SDTR1_TXSR_Msk
  FMC_SDTR1_TXSR_Bit0* = (0x00000001 shl FMC_SDTR1_TXSR_Pos) ## !< 0x00000010
  FMC_SDTR1_TXSR_Bit1* = (0x00000002 shl FMC_SDTR1_TXSR_Pos) ## !< 0x00000020
  FMC_SDTR1_TXSR_Bit2* = (0x00000004 shl FMC_SDTR1_TXSR_Pos) ## !< 0x00000040
  FMC_SDTR1_TXSR_Bit3* = (0x00000008 shl FMC_SDTR1_TXSR_Pos) ## !< 0x00000080
  FMC_SDTR1_TRAS_Pos* = (8)
  FMC_SDTR1_TRAS_Msk* = (0x0000000F shl FMC_SDTR1_TRAS_Pos) ## !< 0x00000F00
  FMC_SDTR1_TRAS* = FMC_SDTR1_TRAS_Msk
  FMC_SDTR1_TRAS_Bit0* = (0x00000001 shl FMC_SDTR1_TRAS_Pos) ## !< 0x00000100
  FMC_SDTR1_TRAS_Bit1* = (0x00000002 shl FMC_SDTR1_TRAS_Pos) ## !< 0x00000200
  FMC_SDTR1_TRAS_Bit2* = (0x00000004 shl FMC_SDTR1_TRAS_Pos) ## !< 0x00000400
  FMC_SDTR1_TRAS_Bit3* = (0x00000008 shl FMC_SDTR1_TRAS_Pos) ## !< 0x00000800
  FMC_SDTR1_TRC_Pos* = (12)
  FMC_SDTR1_TRC_Msk* = (0x0000000F shl FMC_SDTR1_TRC_Pos) ## !< 0x0000F000
  FMC_SDTR1_TRC* = FMC_SDTR1_TRC_Msk
  FMC_SDTR1_TRC_Bit0* = (0x00000001 shl FMC_SDTR1_TRC_Pos) ## !< 0x00001000
  FMC_SDTR1_TRC_Bit1* = (0x00000002 shl FMC_SDTR1_TRC_Pos) ## !< 0x00002000
  FMC_SDTR1_TRC_Bit2* = (0x00000004 shl FMC_SDTR1_TRC_Pos) ## !< 0x00004000
  FMC_SDTR1_TWR_Pos* = (16)
  FMC_SDTR1_TWR_Msk* = (0x0000000F shl FMC_SDTR1_TWR_Pos) ## !< 0x000F0000
  FMC_SDTR1_TWR* = FMC_SDTR1_TWR_Msk
  FMC_SDTR1_TWR_Bit0* = (0x00000001 shl FMC_SDTR1_TWR_Pos) ## !< 0x00010000
  FMC_SDTR1_TWR_Bit1* = (0x00000002 shl FMC_SDTR1_TWR_Pos) ## !< 0x00020000
  FMC_SDTR1_TWR_Bit2* = (0x00000004 shl FMC_SDTR1_TWR_Pos) ## !< 0x00040000
  FMC_SDTR1_TRP_Pos* = (20)
  FMC_SDTR1_TRP_Msk* = (0x0000000F shl FMC_SDTR1_TRP_Pos) ## !< 0x00F00000
  FMC_SDTR1_TRP* = FMC_SDTR1_TRP_Msk
  FMC_SDTR1_TRP_Bit0* = (0x00000001 shl FMC_SDTR1_TRP_Pos) ## !< 0x00100000
  FMC_SDTR1_TRP_Bit1* = (0x00000002 shl FMC_SDTR1_TRP_Pos) ## !< 0x00200000
  FMC_SDTR1_TRP_Bit2* = (0x00000004 shl FMC_SDTR1_TRP_Pos) ## !< 0x00400000
  FMC_SDTR1_TRCD_Pos* = (24)
  FMC_SDTR1_TRCD_Msk* = (0x0000000F shl FMC_SDTR1_TRCD_Pos) ## !< 0x0F000000
  FMC_SDTR1_TRCD* = FMC_SDTR1_TRCD_Msk
  FMC_SDTR1_TRCD_Bit0* = (0x00000001 shl FMC_SDTR1_TRCD_Pos) ## !< 0x01000000
  FMC_SDTR1_TRCD_Bit1* = (0x00000002 shl FMC_SDTR1_TRCD_Pos) ## !< 0x02000000
  FMC_SDTR1_TRCD_Bit2* = (0x00000004 shl FMC_SDTR1_TRCD_Pos) ## !< 0x04000000

## *****************  Bit definition for FMC_SDTR2 register  *****************

const
  FMC_SDTR2_TMRD_Pos* = (0)
  FMC_SDTR2_TMRD_Msk* = (0x0000000F shl FMC_SDTR2_TMRD_Pos) ## !< 0x0000000F
  FMC_SDTR2_TMRD* = FMC_SDTR2_TMRD_Msk
  FMC_SDTR2_TMRD_Bit0* = (0x00000001 shl FMC_SDTR2_TMRD_Pos) ## !< 0x00000001
  FMC_SDTR2_TMRD_Bit1* = (0x00000002 shl FMC_SDTR2_TMRD_Pos) ## !< 0x00000002
  FMC_SDTR2_TMRD_Bit2* = (0x00000004 shl FMC_SDTR2_TMRD_Pos) ## !< 0x00000004
  FMC_SDTR2_TMRD_Bit3* = (0x00000008 shl FMC_SDTR2_TMRD_Pos) ## !< 0x00000008
  FMC_SDTR2_TXSR_Pos* = (4)
  FMC_SDTR2_TXSR_Msk* = (0x0000000F shl FMC_SDTR2_TXSR_Pos) ## !< 0x000000F0
  FMC_SDTR2_TXSR* = FMC_SDTR2_TXSR_Msk
  FMC_SDTR2_TXSR_Bit0* = (0x00000001 shl FMC_SDTR2_TXSR_Pos) ## !< 0x00000010
  FMC_SDTR2_TXSR_Bit1* = (0x00000002 shl FMC_SDTR2_TXSR_Pos) ## !< 0x00000020
  FMC_SDTR2_TXSR_Bit2* = (0x00000004 shl FMC_SDTR2_TXSR_Pos) ## !< 0x00000040
  FMC_SDTR2_TXSR_Bit3* = (0x00000008 shl FMC_SDTR2_TXSR_Pos) ## !< 0x00000080
  FMC_SDTR2_TRAS_Pos* = (8)
  FMC_SDTR2_TRAS_Msk* = (0x0000000F shl FMC_SDTR2_TRAS_Pos) ## !< 0x00000F00
  FMC_SDTR2_TRAS* = FMC_SDTR2_TRAS_Msk
  FMC_SDTR2_TRAS_Bit0* = (0x00000001 shl FMC_SDTR2_TRAS_Pos) ## !< 0x00000100
  FMC_SDTR2_TRAS_Bit1* = (0x00000002 shl FMC_SDTR2_TRAS_Pos) ## !< 0x00000200
  FMC_SDTR2_TRAS_Bit2* = (0x00000004 shl FMC_SDTR2_TRAS_Pos) ## !< 0x00000400
  FMC_SDTR2_TRAS_Bit3* = (0x00000008 shl FMC_SDTR2_TRAS_Pos) ## !< 0x00000800
  FMC_SDTR2_TRC_Pos* = (12)
  FMC_SDTR2_TRC_Msk* = (0x0000000F shl FMC_SDTR2_TRC_Pos) ## !< 0x0000F000
  FMC_SDTR2_TRC* = FMC_SDTR2_TRC_Msk
  FMC_SDTR2_TRC_Bit0* = (0x00000001 shl FMC_SDTR2_TRC_Pos) ## !< 0x00001000
  FMC_SDTR2_TRC_Bit1* = (0x00000002 shl FMC_SDTR2_TRC_Pos) ## !< 0x00002000
  FMC_SDTR2_TRC_Bit2* = (0x00000004 shl FMC_SDTR2_TRC_Pos) ## !< 0x00004000
  FMC_SDTR2_TWR_Pos* = (16)
  FMC_SDTR2_TWR_Msk* = (0x0000000F shl FMC_SDTR2_TWR_Pos) ## !< 0x000F0000
  FMC_SDTR2_TWR* = FMC_SDTR2_TWR_Msk
  FMC_SDTR2_TWR_Bit0* = (0x00000001 shl FMC_SDTR2_TWR_Pos) ## !< 0x00010000
  FMC_SDTR2_TWR_Bit1* = (0x00000002 shl FMC_SDTR2_TWR_Pos) ## !< 0x00020000
  FMC_SDTR2_TWR_Bit2* = (0x00000004 shl FMC_SDTR2_TWR_Pos) ## !< 0x00040000
  FMC_SDTR2_TRP_Pos* = (20)
  FMC_SDTR2_TRP_Msk* = (0x0000000F shl FMC_SDTR2_TRP_Pos) ## !< 0x00F00000
  FMC_SDTR2_TRP* = FMC_SDTR2_TRP_Msk
  FMC_SDTR2_TRP_Bit0* = (0x00000001 shl FMC_SDTR2_TRP_Pos) ## !< 0x00100000
  FMC_SDTR2_TRP_Bit1* = (0x00000002 shl FMC_SDTR2_TRP_Pos) ## !< 0x00200000
  FMC_SDTR2_TRP_Bit2* = (0x00000004 shl FMC_SDTR2_TRP_Pos) ## !< 0x00400000
  FMC_SDTR2_TRCD_Pos* = (24)
  FMC_SDTR2_TRCD_Msk* = (0x0000000F shl FMC_SDTR2_TRCD_Pos) ## !< 0x0F000000
  FMC_SDTR2_TRCD* = FMC_SDTR2_TRCD_Msk
  FMC_SDTR2_TRCD_Bit0* = (0x00000001 shl FMC_SDTR2_TRCD_Pos) ## !< 0x01000000
  FMC_SDTR2_TRCD_Bit1* = (0x00000002 shl FMC_SDTR2_TRCD_Pos) ## !< 0x02000000
  FMC_SDTR2_TRCD_Bit2* = (0x00000004 shl FMC_SDTR2_TRCD_Pos) ## !< 0x04000000

## *****************  Bit definition for FMC_SDCMR register  *****************

const
  FMC_SDCMR_MODE_Pos* = (0)
  FMC_SDCMR_MODE_Msk* = (0x00000007 shl FMC_SDCMR_MODE_Pos) ## !< 0x00000007
  FMC_SDCMR_MODE* = FMC_SDCMR_MODE_Msk
  FMC_SDCMR_MODE_Bit0* = (0x00000001 shl FMC_SDCMR_MODE_Pos) ## !< 0x00000001
  FMC_SDCMR_MODE_Bit1* = (0x00000002 shl FMC_SDCMR_MODE_Pos) ## !< 0x00000002
  FMC_SDCMR_MODE_Bit2* = (0x00000004 shl FMC_SDCMR_MODE_Pos) ## !< 0x00000004
  FMC_SDCMR_CTB2_Pos* = (3)
  FMC_SDCMR_CTB2_Msk* = (0x00000001 shl FMC_SDCMR_CTB2_Pos) ## !< 0x00000008
  FMC_SDCMR_CTB2* = FMC_SDCMR_CTB2_Msk
  FMC_SDCMR_CTB1_Pos* = (4)
  FMC_SDCMR_CTB1_Msk* = (0x00000001 shl FMC_SDCMR_CTB1_Pos) ## !< 0x00000010
  FMC_SDCMR_CTB1* = FMC_SDCMR_CTB1_Msk
  FMC_SDCMR_NRFS_Pos* = (5)
  FMC_SDCMR_NRFS_Msk* = (0x0000000F shl FMC_SDCMR_NRFS_Pos) ## !< 0x000001E0
  FMC_SDCMR_NRFS* = FMC_SDCMR_NRFS_Msk
  FMC_SDCMR_NRFS_Bit0* = (0x00000001 shl FMC_SDCMR_NRFS_Pos) ## !< 0x00000020
  FMC_SDCMR_NRFS_Bit1* = (0x00000002 shl FMC_SDCMR_NRFS_Pos) ## !< 0x00000040
  FMC_SDCMR_NRFS_Bit2* = (0x00000004 shl FMC_SDCMR_NRFS_Pos) ## !< 0x00000080
  FMC_SDCMR_NRFS_Bit3* = (0x00000008 shl FMC_SDCMR_NRFS_Pos) ## !< 0x00000100
  FMC_SDCMR_MRD_Pos* = (9)
  FMC_SDCMR_MRD_Msk* = (0x00001FFF shl FMC_SDCMR_MRD_Pos) ## !< 0x003FFE00
  FMC_SDCMR_MRD* = FMC_SDCMR_MRD_Msk

## *****************  Bit definition for FMC_SDRTR register  *****************

const
  FMC_SDRTR_CRE_Pos* = (0)
  FMC_SDRTR_CRE_Msk* = (0x00000001 shl FMC_SDRTR_CRE_Pos) ## !< 0x00000001
  FMC_SDRTR_CRE* = FMC_SDRTR_CRE_Msk
  FMC_SDRTR_COUNT_Pos* = (1)
  FMC_SDRTR_COUNT_Msk* = (0x00001FFF shl FMC_SDRTR_COUNT_Pos) ## !< 0x00003FFE
  FMC_SDRTR_COUNT* = FMC_SDRTR_COUNT_Msk
  FMC_SDRTR_REIE_Pos* = (14)
  FMC_SDRTR_REIE_Msk* = (0x00000001 shl FMC_SDRTR_REIE_Pos) ## !< 0x00004000
  FMC_SDRTR_REIE* = FMC_SDRTR_REIE_Msk

## *****************  Bit definition for FMC_SDSR register  *****************

const
  FMC_SDSR_RE_Pos* = (0)
  FMC_SDSR_RE_Msk* = (0x00000001 shl FMC_SDSR_RE_Pos) ## !< 0x00000001
  FMC_SDSR_RE* = FMC_SDSR_RE_Msk
  FMC_SDSR_MODES1_Pos* = (1)
  FMC_SDSR_MODES1_Msk* = (0x00000003 shl FMC_SDSR_MODES1_Pos) ## !< 0x00000006
  FMC_SDSR_MODES1* = FMC_SDSR_MODES1_Msk
  FMC_SDSR_MODES1_Bit0* = (0x00000001 shl FMC_SDSR_MODES1_Pos) ## !< 0x00000002
  FMC_SDSR_MODES1_Bit1* = (0x00000002 shl FMC_SDSR_MODES1_Pos) ## !< 0x00000004
  FMC_SDSR_MODES2_Pos* = (3)
  FMC_SDSR_MODES2_Msk* = (0x00000003 shl FMC_SDSR_MODES2_Pos) ## !< 0x00000018
  FMC_SDSR_MODES2* = FMC_SDSR_MODES2_Msk
  FMC_SDSR_MODES2_Bit0* = (0x00000001 shl FMC_SDSR_MODES2_Pos) ## !< 0x00000008
  FMC_SDSR_MODES2_Bit1* = (0x00000002 shl FMC_SDSR_MODES2_Pos) ## !< 0x00000010
  FMC_SDSR_BUSY_Pos* = (5)
  FMC_SDSR_BUSY_Msk* = (0x00000001 shl FMC_SDSR_BUSY_Pos) ## !< 0x00000020
  FMC_SDSR_BUSY* = FMC_SDSR_BUSY_Msk

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
##                                     HASH
##
## ****************************************************************************
## *****************  Bits definition for HASH_CR register  *******************

const
  HASH_CR_INIT_Pos* = (2)
  HASH_CR_INIT_Msk* = (0x00000001 shl HASH_CR_INIT_Pos) ## !< 0x00000004
  HASH_CR_INIT* = HASH_CR_INIT_Msk
  HASH_CR_DMAE_Pos* = (3)
  HASH_CR_DMAE_Msk* = (0x00000001 shl HASH_CR_DMAE_Pos) ## !< 0x00000008
  HASH_CR_DMAE* = HASH_CR_DMAE_Msk
  HASH_CR_DATATYPE_Pos* = (4)
  HASH_CR_DATATYPE_Msk* = (0x00000003 shl HASH_CR_DATATYPE_Pos) ## !< 0x00000030
  HASH_CR_DATATYPE* = HASH_CR_DATATYPE_Msk
  HASH_CR_DATATYPE_Bit0* = (0x00000001 shl HASH_CR_DATATYPE_Pos) ## !< 0x00000010
  HASH_CR_DATATYPE_Bit1* = (0x00000002 shl HASH_CR_DATATYPE_Pos) ## !< 0x00000020
  HASH_CR_MODE_Pos* = (6)
  HASH_CR_MODE_Msk* = (0x00000001 shl HASH_CR_MODE_Pos) ## !< 0x00000040
  HASH_CR_MODE* = HASH_CR_MODE_Msk
  HASH_CR_ALGO_Pos* = (7)
  HASH_CR_ALGO_Msk* = (0x00000801 shl HASH_CR_ALGO_Pos) ## !< 0x00040080
  HASH_CR_ALGO* = HASH_CR_ALGO_Msk
  HASH_CR_ALGO_Bit0* = (0x00000001 shl HASH_CR_ALGO_Pos) ## !< 0x00000080
  HASH_CR_ALGO_Bit1* = (0x00000800 shl HASH_CR_ALGO_Pos) ## !< 0x00040000
  HASH_CR_NBW_Pos* = (8)
  HASH_CR_NBW_Msk* = (0x0000000F shl HASH_CR_NBW_Pos) ## !< 0x00000F00
  HASH_CR_NBW* = HASH_CR_NBW_Msk
  HASH_CR_NBW_Bit0* = (0x00000001 shl HASH_CR_NBW_Pos) ## !< 0x00000100
  HASH_CR_NBW_Bit1* = (0x00000002 shl HASH_CR_NBW_Pos) ## !< 0x00000200
  HASH_CR_NBW_Bit2* = (0x00000004 shl HASH_CR_NBW_Pos) ## !< 0x00000400
  HASH_CR_NBW_Bit3* = (0x00000008 shl HASH_CR_NBW_Pos) ## !< 0x00000800
  HASH_CR_DINNE_Pos* = (12)
  HASH_CR_DINNE_Msk* = (0x00000001 shl HASH_CR_DINNE_Pos) ## !< 0x00001000
  HASH_CR_DINNE* = HASH_CR_DINNE_Msk
  HASH_CR_MDMAT_Pos* = (13)
  HASH_CR_MDMAT_Msk* = (0x00000001 shl HASH_CR_MDMAT_Pos) ## !< 0x00002000
  HASH_CR_MDMAT* = HASH_CR_MDMAT_Msk
  HASH_CR_LKEY_Pos* = (16)
  HASH_CR_LKEY_Msk* = (0x00000001 shl HASH_CR_LKEY_Pos) ## !< 0x00010000
  HASH_CR_LKEY* = HASH_CR_LKEY_Msk

## *****************  Bits definition for HASH_STR register  ******************

const
  HASH_STR_NBLW_Pos* = (0)
  HASH_STR_NBLW_Msk* = (0x0000001F shl HASH_STR_NBLW_Pos) ## !< 0x0000001F
  HASH_STR_NBLW* = HASH_STR_NBLW_Msk
  HASH_STR_NBLW_Bit0* = (0x00000001 shl HASH_STR_NBLW_Pos) ## !< 0x00000001
  HASH_STR_NBLW_Bit1* = (0x00000002 shl HASH_STR_NBLW_Pos) ## !< 0x00000002
  HASH_STR_NBLW_Bit2* = (0x00000004 shl HASH_STR_NBLW_Pos) ## !< 0x00000004
  HASH_STR_NBLW_Bit3* = (0x00000008 shl HASH_STR_NBLW_Pos) ## !< 0x00000008
  HASH_STR_NBLW_Bit4* = (0x00000010 shl HASH_STR_NBLW_Pos) ## !< 0x00000010
  HASH_STR_DCAL_Pos* = (8)
  HASH_STR_DCAL_Msk* = (0x00000001 shl HASH_STR_DCAL_Pos) ## !< 0x00000100
  HASH_STR_DCAL* = HASH_STR_DCAL_Msk

## *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE***
