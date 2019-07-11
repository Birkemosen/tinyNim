## *
## *****************************************************************************
##  @file    stm32f417xx.h
##  @author  MCD Application Team
##  @brief   CMSIS STM32F417xx Device Peripheral Access Layer Header File.
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
## * @addtogroup stm32f417xx
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
    FSMC_IRQn = 48,             ## !< FSMC global Interrupt
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
    FPU_IRQn = 81


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
##  @brief Flexible Static Memory Controller
##

type
  FSMC_Bank1_TypeDef* {.bycopy.} = object
    BTCR*: array[8, uint32]     ## !< NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C


## *
##  @brief Flexible Static Memory Controller Bank1E
##

type
  FSMC_Bank1E_TypeDef* {.bycopy.} = object
    BWTR*: array[7, uint32]     ## !< NOR/PSRAM write timing registers, Address offset: 0x104-0x11C


## *
##  @brief Flexible Static Memory Controller Bank2
##

type
  FSMC_Bank2_Bit3_TypeDef* {.bycopy.} = object
    PCR2*: uint32              ## !< NAND Flash control register 2,                       Address offset: 0x60
    SR2*: uint32               ## !< NAND Flash FIFO status and interrupt register 2,     Address offset: 0x64
    PMEM2*: uint32             ## !< NAND Flash Common memory space timing register 2,    Address offset: 0x68
    PATT2*: uint32             ## !< NAND Flash Attribute memory space timing register 2, Address offset: 0x6C
    RESERVED0*: uint32         ## !< Reserved, 0x70
    ECCR2*: uint32             ## !< NAND Flash ECC result registers 2,                   Address offset: 0x74
    RESERVED1*: uint32         ## !< Reserved, 0x78
    RESERVED2*: uint32         ## !< Reserved, 0x7C
    PCR3*: uint32              ## !< NAND Flash control register 3,                       Address offset: 0x80
    SR3*: uint32               ## !< NAND Flash FIFO status and interrupt register 3,     Address offset: 0x84
    PMEM3*: uint32             ## !< NAND Flash Common memory space timing register 3,    Address offset: 0x88
    PATT3*: uint32             ## !< NAND Flash Attribute memory space timing register 3, Address offset: 0x8C
    RESERVED3*: uint32         ## !< Reserved, 0x90
    ECCR3*: uint32             ## !< NAND Flash ECC result registers 3,                   Address offset: 0x94


## *
##  @brief Flexible Static Memory Controller Bank4
##

type
  FSMC_Bank4_TypeDef* {.bycopy.} = object
    PCR4*: uint32              ## !< PC Card  control register 4,                       Address offset: 0xA0
    SR4*: uint32               ## !< PC Card  FIFO status and interrupt register 4,     Address offset: 0xA4
    PMEM4*: uint32             ## !< PC Card  Common memory space timing register 4,    Address offset: 0xA8
    PATT4*: uint32             ## !< PC Card  Attribute memory space timing register 4, Address offset: 0xAC
    PIO4*: uint32              ## !< PC Card  I/O space timing register 4,              Address offset: 0xB0


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
    Reserved40*: array[48, uint32] ## !< Reserved                                0x40-0xFF
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
  SRAM2_BASE* = 0x2001C000
  PERIPH_BASE* = 0x40000000
  BKPSRAM_BASE* = 0x40024000
  FSMC_R_BASE* = 0xA0000000
  SRAM1_BB_BASE* = 0x22000000
  SRAM2_BB_BASE* = 0x22380000
  PERIPH_BB_BASE* = 0x42000000
  BKPSRAM_BB_BASE* = 0x42480000
  FLASH_END* = 0x080FFFFF
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
  SYSCFG_BASE* = (APB2PERIPH_BASE + 0x00003800)
  EXTI_BASE* = (APB2PERIPH_BASE + 0x00003C00)
  TIM9_BASE* = (APB2PERIPH_BASE + 0x00004000)
  TIM10_BASE* = (APB2PERIPH_BASE + 0x00004400)
  TIM11_BASE* = (APB2PERIPH_BASE + 0x00004800)

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

## !< AHB2 peripherals

const
  DCMI_BASE* = (AHB2PERIPH_BASE + 0x00050000)
  CRYP_BASE* = (AHB2PERIPH_BASE + 0x00060000)
  HASH_BASE* = (AHB2PERIPH_BASE + 0x00060400)
  HASH_DIGEST_BASE* = (AHB2PERIPH_BASE + 0x00060710)
  RNG_BASE* = (AHB2PERIPH_BASE + 0x00060800)

## !< FSMC Bankx registers base address

const
  FSMC_Bank1_R_BASE* = (FSMC_R_BASE + 0x00000000)
  FSMC_Bank1E_R_BASE* = (FSMC_R_BASE + 0x00000104)
  FSMC_Bank2_Bit3_R_BASE* = (FSMC_R_BASE + 0x00000060)
  FSMC_Bank4_R_BASE* = (FSMC_R_BASE + 0x000000A0)

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
  SYSCFG* = (cast[ptr SYSCFG_TypeDef](SYSCFG_BASE))
  EXTI* = (cast[ptr EXTI_TypeDef](EXTI_BASE))
  TIM9* = (cast[ptr TIM_TypeDef](TIM9_BASE))
  TIM10* = (cast[ptr TIM_TypeDef](TIM10_BASE))
  TIM11* = (cast[ptr TIM_TypeDef](TIM11_BASE))
  GPIOA* = (cast[ptr GPIO_TypeDef](GPIOA_BASE))
  GPIOB* = (cast[ptr GPIO_TypeDef](GPIOB_BASE))
  GPIOC* = (cast[ptr GPIO_TypeDef](GPIOC_BASE))
  GPIOD* = (cast[ptr GPIO_TypeDef](GPIOD_BASE))
  GPIOE* = (cast[ptr GPIO_TypeDef](GPIOE_BASE))
  GPIOF* = (cast[ptr GPIO_TypeDef](GPIOF_BASE))
  GPIOG* = (cast[ptr GPIO_TypeDef](GPIOG_BASE))
  GPIOH* = (cast[ptr GPIO_TypeDef](GPIOH_BASE))
  GPIOI* = (cast[ptr GPIO_TypeDef](GPIOI_BASE))
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
  DCMI* = (cast[ptr DCMI_TypeDef](DCMI_BASE))
  CRYP* = (cast[ptr CRYP_TypeDef](CRYP_BASE))
  HASH* = (cast[ptr HASH_TypeDef](HASH_BASE))
  HASH_DIGEST* = (cast[ptr HASH_DIGEST_TypeDef](HASH_DIGEST_BASE))
  RNG* = (cast[ptr RNG_TypeDef](RNG_BASE))
  FSMC_Bank1* = (cast[ptr FSMC_Bank1_TypeDef](FSMC_Bank1_R_BASE))
  FSMC_Bank1E* = (cast[ptr FSMC_Bank1E_TypeDef](FSMC_Bank1E_R_BASE))
  FSMC_Bank2_Bit3* = (cast[ptr FSMC_Bank2_Bit3_TypeDef](FSMC_Bank2_Bit3_R_BASE))
  FSMC_Bank4* = (cast[ptr FSMC_Bank4_TypeDef](FSMC_Bank4_R_BASE))
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
  DCMI_CR_ENABLE_Pos* = (14)
  DCMI_CR_ENABLE_Msk* = (0x00000001 shl DCMI_CR_ENABLE_Pos) ## !< 0x00004000
  DCMI_CR_ENABLE* = DCMI_CR_ENABLE_Msk

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
##                    Flexible Static Memory Controller
##
## ****************************************************************************
## *****************  Bit definition for FSMC_BCR1 register  ******************

const
  FSMC_BCR1_MBKEN_Pos* = (0)
  FSMC_BCR1_MBKEN_Msk* = (0x00000001 shl FSMC_BCR1_MBKEN_Pos) ## !< 0x00000001
  FSMC_BCR1_MBKEN* = FSMC_BCR1_MBKEN_Msk
  FSMC_BCR1_MUXEN_Pos* = (1)
  FSMC_BCR1_MUXEN_Msk* = (0x00000001 shl FSMC_BCR1_MUXEN_Pos) ## !< 0x00000002
  FSMC_BCR1_MUXEN* = FSMC_BCR1_MUXEN_Msk
  FSMC_BCR1_MTYP_Pos* = (2)
  FSMC_BCR1_MTYP_Msk* = (0x00000003 shl FSMC_BCR1_MTYP_Pos) ## !< 0x0000000C
  FSMC_BCR1_MTYP* = FSMC_BCR1_MTYP_Msk
  FSMC_BCR1_MTYP_Bit0* = (0x00000001 shl FSMC_BCR1_MTYP_Pos) ## !< 0x00000004
  FSMC_BCR1_MTYP_Bit1* = (0x00000002 shl FSMC_BCR1_MTYP_Pos) ## !< 0x00000008
  FSMC_BCR1_MWID_Pos* = (4)
  FSMC_BCR1_MWID_Msk* = (0x00000003 shl FSMC_BCR1_MWID_Pos) ## !< 0x00000030
  FSMC_BCR1_MWID* = FSMC_BCR1_MWID_Msk
  FSMC_BCR1_MWID_Bit0* = (0x00000001 shl FSMC_BCR1_MWID_Pos) ## !< 0x00000010
  FSMC_BCR1_MWID_Bit1* = (0x00000002 shl FSMC_BCR1_MWID_Pos) ## !< 0x00000020
  FSMC_BCR1_FACCEN_Pos* = (6)
  FSMC_BCR1_FACCEN_Msk* = (0x00000001 shl FSMC_BCR1_FACCEN_Pos) ## !< 0x00000040
  FSMC_BCR1_FACCEN* = FSMC_BCR1_FACCEN_Msk
  FSMC_BCR1_BURSTEN_Pos* = (8)
  FSMC_BCR1_BURSTEN_Msk* = (0x00000001 shl FSMC_BCR1_BURSTEN_Pos) ## !< 0x00000100
  FSMC_BCR1_BURSTEN* = FSMC_BCR1_BURSTEN_Msk
  FSMC_BCR1_WAITPOL_Pos* = (9)
  FSMC_BCR1_WAITPOL_Msk* = (0x00000001 shl FSMC_BCR1_WAITPOL_Pos) ## !< 0x00000200
  FSMC_BCR1_WAITPOL* = FSMC_BCR1_WAITPOL_Msk
  FSMC_BCR1_WRAPMOD_Pos* = (10)
  FSMC_BCR1_WRAPMOD_Msk* = (0x00000001 shl FSMC_BCR1_WRAPMOD_Pos) ## !< 0x00000400
  FSMC_BCR1_WRAPMOD* = FSMC_BCR1_WRAPMOD_Msk
  FSMC_BCR1_WAITCFG_Pos* = (11)
  FSMC_BCR1_WAITCFG_Msk* = (0x00000001 shl FSMC_BCR1_WAITCFG_Pos) ## !< 0x00000800
  FSMC_BCR1_WAITCFG* = FSMC_BCR1_WAITCFG_Msk
  FSMC_BCR1_WREN_Pos* = (12)
  FSMC_BCR1_WREN_Msk* = (0x00000001 shl FSMC_BCR1_WREN_Pos) ## !< 0x00001000
  FSMC_BCR1_WREN* = FSMC_BCR1_WREN_Msk
  FSMC_BCR1_WAITEN_Pos* = (13)
  FSMC_BCR1_WAITEN_Msk* = (0x00000001 shl FSMC_BCR1_WAITEN_Pos) ## !< 0x00002000
  FSMC_BCR1_WAITEN* = FSMC_BCR1_WAITEN_Msk
  FSMC_BCR1_EXTMOD_Pos* = (14)
  FSMC_BCR1_EXTMOD_Msk* = (0x00000001 shl FSMC_BCR1_EXTMOD_Pos) ## !< 0x00004000
  FSMC_BCR1_EXTMOD* = FSMC_BCR1_EXTMOD_Msk
  FSMC_BCR1_ASYNCWAIT_Pos* = (15)
  FSMC_BCR1_ASYNCWAIT_Msk* = (0x00000001 shl FSMC_BCR1_ASYNCWAIT_Pos) ## !< 0x00008000
  FSMC_BCR1_ASYNCWAIT* = FSMC_BCR1_ASYNCWAIT_Msk
  FSMC_BCR1_CPSIZE_Pos* = (16)
  FSMC_BCR1_CPSIZE_Msk* = (0x00000007 shl FSMC_BCR1_CPSIZE_Pos) ## !< 0x00070000
  FSMC_BCR1_CPSIZE* = FSMC_BCR1_CPSIZE_Msk
  FSMC_BCR1_CPSIZE_Bit0* = (0x00000001 shl FSMC_BCR1_CPSIZE_Pos) ## !< 0x00010000
  FSMC_BCR1_CPSIZE_Bit1* = (0x00000002 shl FSMC_BCR1_CPSIZE_Pos) ## !< 0x00020000
  FSMC_BCR1_CPSIZE_Bit2* = (0x00000004 shl FSMC_BCR1_CPSIZE_Pos) ## !< 0x00040000
  FSMC_BCR1_CBURSTRW_Pos* = (19)
  FSMC_BCR1_CBURSTRW_Msk* = (0x00000001 shl FSMC_BCR1_CBURSTRW_Pos) ## !< 0x00080000
  FSMC_BCR1_CBURSTRW* = FSMC_BCR1_CBURSTRW_Msk

## *****************  Bit definition for FSMC_BCR2 register  ******************

const
  FSMC_BCR2_MBKEN_Pos* = (0)
  FSMC_BCR2_MBKEN_Msk* = (0x00000001 shl FSMC_BCR2_MBKEN_Pos) ## !< 0x00000001
  FSMC_BCR2_MBKEN* = FSMC_BCR2_MBKEN_Msk
  FSMC_BCR2_MUXEN_Pos* = (1)
  FSMC_BCR2_MUXEN_Msk* = (0x00000001 shl FSMC_BCR2_MUXEN_Pos) ## !< 0x00000002
  FSMC_BCR2_MUXEN* = FSMC_BCR2_MUXEN_Msk
  FSMC_BCR2_MTYP_Pos* = (2)
  FSMC_BCR2_MTYP_Msk* = (0x00000003 shl FSMC_BCR2_MTYP_Pos) ## !< 0x0000000C
  FSMC_BCR2_MTYP* = FSMC_BCR2_MTYP_Msk
  FSMC_BCR2_MTYP_Bit0* = (0x00000001 shl FSMC_BCR2_MTYP_Pos) ## !< 0x00000004
  FSMC_BCR2_MTYP_Bit1* = (0x00000002 shl FSMC_BCR2_MTYP_Pos) ## !< 0x00000008
  FSMC_BCR2_MWID_Pos* = (4)
  FSMC_BCR2_MWID_Msk* = (0x00000003 shl FSMC_BCR2_MWID_Pos) ## !< 0x00000030
  FSMC_BCR2_MWID* = FSMC_BCR2_MWID_Msk
  FSMC_BCR2_MWID_Bit0* = (0x00000001 shl FSMC_BCR2_MWID_Pos) ## !< 0x00000010
  FSMC_BCR2_MWID_Bit1* = (0x00000002 shl FSMC_BCR2_MWID_Pos) ## !< 0x00000020
  FSMC_BCR2_FACCEN_Pos* = (6)
  FSMC_BCR2_FACCEN_Msk* = (0x00000001 shl FSMC_BCR2_FACCEN_Pos) ## !< 0x00000040
  FSMC_BCR2_FACCEN* = FSMC_BCR2_FACCEN_Msk
  FSMC_BCR2_BURSTEN_Pos* = (8)
  FSMC_BCR2_BURSTEN_Msk* = (0x00000001 shl FSMC_BCR2_BURSTEN_Pos) ## !< 0x00000100
  FSMC_BCR2_BURSTEN* = FSMC_BCR2_BURSTEN_Msk
  FSMC_BCR2_WAITPOL_Pos* = (9)
  FSMC_BCR2_WAITPOL_Msk* = (0x00000001 shl FSMC_BCR2_WAITPOL_Pos) ## !< 0x00000200
  FSMC_BCR2_WAITPOL* = FSMC_BCR2_WAITPOL_Msk
  FSMC_BCR2_WRAPMOD_Pos* = (10)
  FSMC_BCR2_WRAPMOD_Msk* = (0x00000001 shl FSMC_BCR2_WRAPMOD_Pos) ## !< 0x00000400
  FSMC_BCR2_WRAPMOD* = FSMC_BCR2_WRAPMOD_Msk
  FSMC_BCR2_WAITCFG_Pos* = (11)
  FSMC_BCR2_WAITCFG_Msk* = (0x00000001 shl FSMC_BCR2_WAITCFG_Pos) ## !< 0x00000800
  FSMC_BCR2_WAITCFG* = FSMC_BCR2_WAITCFG_Msk
  FSMC_BCR2_WREN_Pos* = (12)
  FSMC_BCR2_WREN_Msk* = (0x00000001 shl FSMC_BCR2_WREN_Pos) ## !< 0x00001000
  FSMC_BCR2_WREN* = FSMC_BCR2_WREN_Msk
  FSMC_BCR2_WAITEN_Pos* = (13)
  FSMC_BCR2_WAITEN_Msk* = (0x00000001 shl FSMC_BCR2_WAITEN_Pos) ## !< 0x00002000
  FSMC_BCR2_WAITEN* = FSMC_BCR2_WAITEN_Msk
  FSMC_BCR2_EXTMOD_Pos* = (14)
  FSMC_BCR2_EXTMOD_Msk* = (0x00000001 shl FSMC_BCR2_EXTMOD_Pos) ## !< 0x00004000
  FSMC_BCR2_EXTMOD* = FSMC_BCR2_EXTMOD_Msk
  FSMC_BCR2_ASYNCWAIT_Pos* = (15)
  FSMC_BCR2_ASYNCWAIT_Msk* = (0x00000001 shl FSMC_BCR2_ASYNCWAIT_Pos) ## !< 0x00008000
  FSMC_BCR2_ASYNCWAIT* = FSMC_BCR2_ASYNCWAIT_Msk
  FSMC_BCR2_CPSIZE_Pos* = (16)
  FSMC_BCR2_CPSIZE_Msk* = (0x00000007 shl FSMC_BCR2_CPSIZE_Pos) ## !< 0x00070000
  FSMC_BCR2_CPSIZE* = FSMC_BCR2_CPSIZE_Msk
  FSMC_BCR2_CPSIZE_Bit0* = (0x00000001 shl FSMC_BCR2_CPSIZE_Pos) ## !< 0x00010000
  FSMC_BCR2_CPSIZE_Bit1* = (0x00000002 shl FSMC_BCR2_CPSIZE_Pos) ## !< 0x00020000
  FSMC_BCR2_CPSIZE_Bit2* = (0x00000004 shl FSMC_BCR2_CPSIZE_Pos) ## !< 0x00040000
  FSMC_BCR2_CBURSTRW_Pos* = (19)
  FSMC_BCR2_CBURSTRW_Msk* = (0x00000001 shl FSMC_BCR2_CBURSTRW_Pos) ## !< 0x00080000
  FSMC_BCR2_CBURSTRW* = FSMC_BCR2_CBURSTRW_Msk

## *****************  Bit definition for FSMC_BCR3 register  ******************

const
  FSMC_BCR3_MBKEN_Pos* = (0)
  FSMC_BCR3_MBKEN_Msk* = (0x00000001 shl FSMC_BCR3_MBKEN_Pos) ## !< 0x00000001
  FSMC_BCR3_MBKEN* = FSMC_BCR3_MBKEN_Msk
  FSMC_BCR3_MUXEN_Pos* = (1)
  FSMC_BCR3_MUXEN_Msk* = (0x00000001 shl FSMC_BCR3_MUXEN_Pos) ## !< 0x00000002
  FSMC_BCR3_MUXEN* = FSMC_BCR3_MUXEN_Msk
  FSMC_BCR3_MTYP_Pos* = (2)
  FSMC_BCR3_MTYP_Msk* = (0x00000003 shl FSMC_BCR3_MTYP_Pos) ## !< 0x0000000C
  FSMC_BCR3_MTYP* = FSMC_BCR3_MTYP_Msk
  FSMC_BCR3_MTYP_Bit0* = (0x00000001 shl FSMC_BCR3_MTYP_Pos) ## !< 0x00000004
  FSMC_BCR3_MTYP_Bit1* = (0x00000002 shl FSMC_BCR3_MTYP_Pos) ## !< 0x00000008
  FSMC_BCR3_MWID_Pos* = (4)
  FSMC_BCR3_MWID_Msk* = (0x00000003 shl FSMC_BCR3_MWID_Pos) ## !< 0x00000030
  FSMC_BCR3_MWID* = FSMC_BCR3_MWID_Msk
  FSMC_BCR3_MWID_Bit0* = (0x00000001 shl FSMC_BCR3_MWID_Pos) ## !< 0x00000010
  FSMC_BCR3_MWID_Bit1* = (0x00000002 shl FSMC_BCR3_MWID_Pos) ## !< 0x00000020
  FSMC_BCR3_FACCEN_Pos* = (6)
  FSMC_BCR3_FACCEN_Msk* = (0x00000001 shl FSMC_BCR3_FACCEN_Pos) ## !< 0x00000040
  FSMC_BCR3_FACCEN* = FSMC_BCR3_FACCEN_Msk
  FSMC_BCR3_BURSTEN_Pos* = (8)
  FSMC_BCR3_BURSTEN_Msk* = (0x00000001 shl FSMC_BCR3_BURSTEN_Pos) ## !< 0x00000100
  FSMC_BCR3_BURSTEN* = FSMC_BCR3_BURSTEN_Msk
  FSMC_BCR3_WAITPOL_Pos* = (9)
  FSMC_BCR3_WAITPOL_Msk* = (0x00000001 shl FSMC_BCR3_WAITPOL_Pos) ## !< 0x00000200
  FSMC_BCR3_WAITPOL* = FSMC_BCR3_WAITPOL_Msk
  FSMC_BCR3_WRAPMOD_Pos* = (10)
  FSMC_BCR3_WRAPMOD_Msk* = (0x00000001 shl FSMC_BCR3_WRAPMOD_Pos) ## !< 0x00000400
  FSMC_BCR3_WRAPMOD* = FSMC_BCR3_WRAPMOD_Msk
  FSMC_BCR3_WAITCFG_Pos* = (11)
  FSMC_BCR3_WAITCFG_Msk* = (0x00000001 shl FSMC_BCR3_WAITCFG_Pos) ## !< 0x00000800
  FSMC_BCR3_WAITCFG* = FSMC_BCR3_WAITCFG_Msk
  FSMC_BCR3_WREN_Pos* = (12)
  FSMC_BCR3_WREN_Msk* = (0x00000001 shl FSMC_BCR3_WREN_Pos) ## !< 0x00001000
  FSMC_BCR3_WREN* = FSMC_BCR3_WREN_Msk
  FSMC_BCR3_WAITEN_Pos* = (13)
  FSMC_BCR3_WAITEN_Msk* = (0x00000001 shl FSMC_BCR3_WAITEN_Pos) ## !< 0x00002000
  FSMC_BCR3_WAITEN* = FSMC_BCR3_WAITEN_Msk
  FSMC_BCR3_EXTMOD_Pos* = (14)
  FSMC_BCR3_EXTMOD_Msk* = (0x00000001 shl FSMC_BCR3_EXTMOD_Pos) ## !< 0x00004000
  FSMC_BCR3_EXTMOD* = FSMC_BCR3_EXTMOD_Msk
  FSMC_BCR3_ASYNCWAIT_Pos* = (15)
  FSMC_BCR3_ASYNCWAIT_Msk* = (0x00000001 shl FSMC_BCR3_ASYNCWAIT_Pos) ## !< 0x00008000
  FSMC_BCR3_ASYNCWAIT* = FSMC_BCR3_ASYNCWAIT_Msk
  FSMC_BCR3_CPSIZE_Pos* = (16)
  FSMC_BCR3_CPSIZE_Msk* = (0x00000007 shl FSMC_BCR3_CPSIZE_Pos) ## !< 0x00070000
  FSMC_BCR3_CPSIZE* = FSMC_BCR3_CPSIZE_Msk
  FSMC_BCR3_CPSIZE_Bit0* = (0x00000001 shl FSMC_BCR3_CPSIZE_Pos) ## !< 0x00010000
  FSMC_BCR3_CPSIZE_Bit1* = (0x00000002 shl FSMC_BCR3_CPSIZE_Pos) ## !< 0x00020000
  FSMC_BCR3_CPSIZE_Bit2* = (0x00000004 shl FSMC_BCR3_CPSIZE_Pos) ## !< 0x00040000
  FSMC_BCR3_CBURSTRW_Pos* = (19)
  FSMC_BCR3_CBURSTRW_Msk* = (0x00000001 shl FSMC_BCR3_CBURSTRW_Pos) ## !< 0x00080000
  FSMC_BCR3_CBURSTRW* = FSMC_BCR3_CBURSTRW_Msk

## *****************  Bit definition for FSMC_BCR4 register  ******************

const
  FSMC_BCR4_MBKEN_Pos* = (0)
  FSMC_BCR4_MBKEN_Msk* = (0x00000001 shl FSMC_BCR4_MBKEN_Pos) ## !< 0x00000001
  FSMC_BCR4_MBKEN* = FSMC_BCR4_MBKEN_Msk
  FSMC_BCR4_MUXEN_Pos* = (1)
  FSMC_BCR4_MUXEN_Msk* = (0x00000001 shl FSMC_BCR4_MUXEN_Pos) ## !< 0x00000002
  FSMC_BCR4_MUXEN* = FSMC_BCR4_MUXEN_Msk
  FSMC_BCR4_MTYP_Pos* = (2)
  FSMC_BCR4_MTYP_Msk* = (0x00000003 shl FSMC_BCR4_MTYP_Pos) ## !< 0x0000000C
  FSMC_BCR4_MTYP* = FSMC_BCR4_MTYP_Msk
  FSMC_BCR4_MTYP_Bit0* = (0x00000001 shl FSMC_BCR4_MTYP_Pos) ## !< 0x00000004
  FSMC_BCR4_MTYP_Bit1* = (0x00000002 shl FSMC_BCR4_MTYP_Pos) ## !< 0x00000008
  FSMC_BCR4_MWID_Pos* = (4)
  FSMC_BCR4_MWID_Msk* = (0x00000003 shl FSMC_BCR4_MWID_Pos) ## !< 0x00000030
  FSMC_BCR4_MWID* = FSMC_BCR4_MWID_Msk
  FSMC_BCR4_MWID_Bit0* = (0x00000001 shl FSMC_BCR4_MWID_Pos) ## !< 0x00000010
  FSMC_BCR4_MWID_Bit1* = (0x00000002 shl FSMC_BCR4_MWID_Pos) ## !< 0x00000020
  FSMC_BCR4_FACCEN_Pos* = (6)
  FSMC_BCR4_FACCEN_Msk* = (0x00000001 shl FSMC_BCR4_FACCEN_Pos) ## !< 0x00000040
  FSMC_BCR4_FACCEN* = FSMC_BCR4_FACCEN_Msk
  FSMC_BCR4_BURSTEN_Pos* = (8)
  FSMC_BCR4_BURSTEN_Msk* = (0x00000001 shl FSMC_BCR4_BURSTEN_Pos) ## !< 0x00000100
  FSMC_BCR4_BURSTEN* = FSMC_BCR4_BURSTEN_Msk
  FSMC_BCR4_WAITPOL_Pos* = (9)
  FSMC_BCR4_WAITPOL_Msk* = (0x00000001 shl FSMC_BCR4_WAITPOL_Pos) ## !< 0x00000200
  FSMC_BCR4_WAITPOL* = FSMC_BCR4_WAITPOL_Msk
  FSMC_BCR4_WRAPMOD_Pos* = (10)
  FSMC_BCR4_WRAPMOD_Msk* = (0x00000001 shl FSMC_BCR4_WRAPMOD_Pos) ## !< 0x00000400
  FSMC_BCR4_WRAPMOD* = FSMC_BCR4_WRAPMOD_Msk
  FSMC_BCR4_WAITCFG_Pos* = (11)
  FSMC_BCR4_WAITCFG_Msk* = (0x00000001 shl FSMC_BCR4_WAITCFG_Pos) ## !< 0x00000800
  FSMC_BCR4_WAITCFG* = FSMC_BCR4_WAITCFG_Msk
  FSMC_BCR4_WREN_Pos* = (12)
  FSMC_BCR4_WREN_Msk* = (0x00000001 shl FSMC_BCR4_WREN_Pos) ## !< 0x00001000
  FSMC_BCR4_WREN* = FSMC_BCR4_WREN_Msk
  FSMC_BCR4_WAITEN_Pos* = (13)
  FSMC_BCR4_WAITEN_Msk* = (0x00000001 shl FSMC_BCR4_WAITEN_Pos) ## !< 0x00002000
  FSMC_BCR4_WAITEN* = FSMC_BCR4_WAITEN_Msk
  FSMC_BCR4_EXTMOD_Pos* = (14)
  FSMC_BCR4_EXTMOD_Msk* = (0x00000001 shl FSMC_BCR4_EXTMOD_Pos) ## !< 0x00004000
  FSMC_BCR4_EXTMOD* = FSMC_BCR4_EXTMOD_Msk
  FSMC_BCR4_ASYNCWAIT_Pos* = (15)
  FSMC_BCR4_ASYNCWAIT_Msk* = (0x00000001 shl FSMC_BCR4_ASYNCWAIT_Pos) ## !< 0x00008000
  FSMC_BCR4_ASYNCWAIT* = FSMC_BCR4_ASYNCWAIT_Msk
  FSMC_BCR4_CPSIZE_Pos* = (16)
  FSMC_BCR4_CPSIZE_Msk* = (0x00000007 shl FSMC_BCR4_CPSIZE_Pos) ## !< 0x00070000
  FSMC_BCR4_CPSIZE* = FSMC_BCR4_CPSIZE_Msk
  FSMC_BCR4_CPSIZE_Bit0* = (0x00000001 shl FSMC_BCR4_CPSIZE_Pos) ## !< 0x00010000
  FSMC_BCR4_CPSIZE_Bit1* = (0x00000002 shl FSMC_BCR4_CPSIZE_Pos) ## !< 0x00020000
  FSMC_BCR4_CPSIZE_Bit2* = (0x00000004 shl FSMC_BCR4_CPSIZE_Pos) ## !< 0x00040000
  FSMC_BCR4_CBURSTRW_Pos* = (19)
  FSMC_BCR4_CBURSTRW_Msk* = (0x00000001 shl FSMC_BCR4_CBURSTRW_Pos) ## !< 0x00080000
  FSMC_BCR4_CBURSTRW* = FSMC_BCR4_CBURSTRW_Msk

## *****************  Bit definition for FSMC_BTR1 register  *****************

const
  FSMC_BTR1_ADDSET_Pos* = (0)
  FSMC_BTR1_ADDSET_Msk* = (0x0000000F shl FSMC_BTR1_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BTR1_ADDSET* = FSMC_BTR1_ADDSET_Msk
  FSMC_BTR1_ADDSET_Bit0* = (0x00000001 shl FSMC_BTR1_ADDSET_Pos) ## !< 0x00000001
  FSMC_BTR1_ADDSET_Bit1* = (0x00000002 shl FSMC_BTR1_ADDSET_Pos) ## !< 0x00000002
  FSMC_BTR1_ADDSET_Bit2* = (0x00000004 shl FSMC_BTR1_ADDSET_Pos) ## !< 0x00000004
  FSMC_BTR1_ADDSET_Bit3* = (0x00000008 shl FSMC_BTR1_ADDSET_Pos) ## !< 0x00000008
  FSMC_BTR1_ADDHLD_Pos* = (4)
  FSMC_BTR1_ADDHLD_Msk* = (0x0000000F shl FSMC_BTR1_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BTR1_ADDHLD* = FSMC_BTR1_ADDHLD_Msk
  FSMC_BTR1_ADDHLD_Bit0* = (0x00000001 shl FSMC_BTR1_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BTR1_ADDHLD_Bit1* = (0x00000002 shl FSMC_BTR1_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BTR1_ADDHLD_Bit2* = (0x00000004 shl FSMC_BTR1_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BTR1_ADDHLD_Bit3* = (0x00000008 shl FSMC_BTR1_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BTR1_DATAST_Pos* = (8)
  FSMC_BTR1_DATAST_Msk* = (0x000000FF shl FSMC_BTR1_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BTR1_DATAST* = FSMC_BTR1_DATAST_Msk
  FSMC_BTR1_DATAST_Bit0* = (0x00000001 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00000100
  FSMC_BTR1_DATAST_Bit1* = (0x00000002 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00000200
  FSMC_BTR1_DATAST_Bit2* = (0x00000004 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00000400
  FSMC_BTR1_DATAST_Bit3* = (0x00000008 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00000800
  FSMC_BTR1_DATAST_Bit4* = (0x00000010 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00001000
  FSMC_BTR1_DATAST_Bit5* = (0x00000020 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00002000
  FSMC_BTR1_DATAST_Bit6* = (0x00000040 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00004000
  FSMC_BTR1_DATAST_Bit7* = (0x00000080 shl FSMC_BTR1_DATAST_Pos) ## !< 0x00008000
  FSMC_BTR1_BUSTURN_Pos* = (16)
  FSMC_BTR1_BUSTURN_Msk* = (0x0000000F shl FSMC_BTR1_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BTR1_BUSTURN* = FSMC_BTR1_BUSTURN_Msk
  FSMC_BTR1_BUSTURN_Bit0* = (0x00000001 shl FSMC_BTR1_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BTR1_BUSTURN_Bit1* = (0x00000002 shl FSMC_BTR1_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BTR1_BUSTURN_Bit2* = (0x00000004 shl FSMC_BTR1_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BTR1_BUSTURN_Bit3* = (0x00000008 shl FSMC_BTR1_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BTR1_CLKDIV_Pos* = (20)
  FSMC_BTR1_CLKDIV_Msk* = (0x0000000F shl FSMC_BTR1_CLKDIV_Pos) ## !< 0x00F00000
  FSMC_BTR1_CLKDIV* = FSMC_BTR1_CLKDIV_Msk
  FSMC_BTR1_CLKDIV_Bit0* = (0x00000001 shl FSMC_BTR1_CLKDIV_Pos) ## !< 0x00100000
  FSMC_BTR1_CLKDIV_Bit1* = (0x00000002 shl FSMC_BTR1_CLKDIV_Pos) ## !< 0x00200000
  FSMC_BTR1_CLKDIV_Bit2* = (0x00000004 shl FSMC_BTR1_CLKDIV_Pos) ## !< 0x00400000
  FSMC_BTR1_CLKDIV_Bit3* = (0x00000008 shl FSMC_BTR1_CLKDIV_Pos) ## !< 0x00800000
  FSMC_BTR1_DATLAT_Pos* = (24)
  FSMC_BTR1_DATLAT_Msk* = (0x0000000F shl FSMC_BTR1_DATLAT_Pos) ## !< 0x0F000000
  FSMC_BTR1_DATLAT* = FSMC_BTR1_DATLAT_Msk
  FSMC_BTR1_DATLAT_Bit0* = (0x00000001 shl FSMC_BTR1_DATLAT_Pos) ## !< 0x01000000
  FSMC_BTR1_DATLAT_Bit1* = (0x00000002 shl FSMC_BTR1_DATLAT_Pos) ## !< 0x02000000
  FSMC_BTR1_DATLAT_Bit2* = (0x00000004 shl FSMC_BTR1_DATLAT_Pos) ## !< 0x04000000
  FSMC_BTR1_DATLAT_Bit3* = (0x00000008 shl FSMC_BTR1_DATLAT_Pos) ## !< 0x08000000
  FSMC_BTR1_ACCMOD_Pos* = (28)
  FSMC_BTR1_ACCMOD_Msk* = (0x00000003 shl FSMC_BTR1_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BTR1_ACCMOD* = FSMC_BTR1_ACCMOD_Msk
  FSMC_BTR1_ACCMOD_Bit0* = (0x00000001 shl FSMC_BTR1_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BTR1_ACCMOD_Bit1* = (0x00000002 shl FSMC_BTR1_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_BTR2 register  ******************

const
  FSMC_BTR2_ADDSET_Pos* = (0)
  FSMC_BTR2_ADDSET_Msk* = (0x0000000F shl FSMC_BTR2_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BTR2_ADDSET* = FSMC_BTR2_ADDSET_Msk
  FSMC_BTR2_ADDSET_Bit0* = (0x00000001 shl FSMC_BTR2_ADDSET_Pos) ## !< 0x00000001
  FSMC_BTR2_ADDSET_Bit1* = (0x00000002 shl FSMC_BTR2_ADDSET_Pos) ## !< 0x00000002
  FSMC_BTR2_ADDSET_Bit2* = (0x00000004 shl FSMC_BTR2_ADDSET_Pos) ## !< 0x00000004
  FSMC_BTR2_ADDSET_Bit3* = (0x00000008 shl FSMC_BTR2_ADDSET_Pos) ## !< 0x00000008
  FSMC_BTR2_ADDHLD_Pos* = (4)
  FSMC_BTR2_ADDHLD_Msk* = (0x0000000F shl FSMC_BTR2_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BTR2_ADDHLD* = FSMC_BTR2_ADDHLD_Msk
  FSMC_BTR2_ADDHLD_Bit0* = (0x00000001 shl FSMC_BTR2_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BTR2_ADDHLD_Bit1* = (0x00000002 shl FSMC_BTR2_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BTR2_ADDHLD_Bit2* = (0x00000004 shl FSMC_BTR2_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BTR2_ADDHLD_Bit3* = (0x00000008 shl FSMC_BTR2_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BTR2_DATAST_Pos* = (8)
  FSMC_BTR2_DATAST_Msk* = (0x000000FF shl FSMC_BTR2_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BTR2_DATAST* = FSMC_BTR2_DATAST_Msk
  FSMC_BTR2_DATAST_Bit0* = (0x00000001 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00000100
  FSMC_BTR2_DATAST_Bit1* = (0x00000002 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00000200
  FSMC_BTR2_DATAST_Bit2* = (0x00000004 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00000400
  FSMC_BTR2_DATAST_Bit3* = (0x00000008 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00000800
  FSMC_BTR2_DATAST_Bit4* = (0x00000010 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00001000
  FSMC_BTR2_DATAST_Bit5* = (0x00000020 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00002000
  FSMC_BTR2_DATAST_Bit6* = (0x00000040 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00004000
  FSMC_BTR2_DATAST_Bit7* = (0x00000080 shl FSMC_BTR2_DATAST_Pos) ## !< 0x00008000
  FSMC_BTR2_BUSTURN_Pos* = (16)
  FSMC_BTR2_BUSTURN_Msk* = (0x0000000F shl FSMC_BTR2_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BTR2_BUSTURN* = FSMC_BTR2_BUSTURN_Msk
  FSMC_BTR2_BUSTURN_Bit0* = (0x00000001 shl FSMC_BTR2_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BTR2_BUSTURN_Bit1* = (0x00000002 shl FSMC_BTR2_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BTR2_BUSTURN_Bit2* = (0x00000004 shl FSMC_BTR2_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BTR2_BUSTURN_Bit3* = (0x00000008 shl FSMC_BTR2_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BTR2_CLKDIV_Pos* = (20)
  FSMC_BTR2_CLKDIV_Msk* = (0x0000000F shl FSMC_BTR2_CLKDIV_Pos) ## !< 0x00F00000
  FSMC_BTR2_CLKDIV* = FSMC_BTR2_CLKDIV_Msk
  FSMC_BTR2_CLKDIV_Bit0* = (0x00000001 shl FSMC_BTR2_CLKDIV_Pos) ## !< 0x00100000
  FSMC_BTR2_CLKDIV_Bit1* = (0x00000002 shl FSMC_BTR2_CLKDIV_Pos) ## !< 0x00200000
  FSMC_BTR2_CLKDIV_Bit2* = (0x00000004 shl FSMC_BTR2_CLKDIV_Pos) ## !< 0x00400000
  FSMC_BTR2_CLKDIV_Bit3* = (0x00000008 shl FSMC_BTR2_CLKDIV_Pos) ## !< 0x00800000
  FSMC_BTR2_DATLAT_Pos* = (24)
  FSMC_BTR2_DATLAT_Msk* = (0x0000000F shl FSMC_BTR2_DATLAT_Pos) ## !< 0x0F000000
  FSMC_BTR2_DATLAT* = FSMC_BTR2_DATLAT_Msk
  FSMC_BTR2_DATLAT_Bit0* = (0x00000001 shl FSMC_BTR2_DATLAT_Pos) ## !< 0x01000000
  FSMC_BTR2_DATLAT_Bit1* = (0x00000002 shl FSMC_BTR2_DATLAT_Pos) ## !< 0x02000000
  FSMC_BTR2_DATLAT_Bit2* = (0x00000004 shl FSMC_BTR2_DATLAT_Pos) ## !< 0x04000000
  FSMC_BTR2_DATLAT_Bit3* = (0x00000008 shl FSMC_BTR2_DATLAT_Pos) ## !< 0x08000000
  FSMC_BTR2_ACCMOD_Pos* = (28)
  FSMC_BTR2_ACCMOD_Msk* = (0x00000003 shl FSMC_BTR2_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BTR2_ACCMOD* = FSMC_BTR2_ACCMOD_Msk
  FSMC_BTR2_ACCMOD_Bit0* = (0x00000001 shl FSMC_BTR2_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BTR2_ACCMOD_Bit1* = (0x00000002 shl FSMC_BTR2_ACCMOD_Pos) ## !< 0x20000000

## ******************  Bit definition for FSMC_BTR3 register  ******************

const
  FSMC_BTR3_ADDSET_Pos* = (0)
  FSMC_BTR3_ADDSET_Msk* = (0x0000000F shl FSMC_BTR3_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BTR3_ADDSET* = FSMC_BTR3_ADDSET_Msk
  FSMC_BTR3_ADDSET_Bit0* = (0x00000001 shl FSMC_BTR3_ADDSET_Pos) ## !< 0x00000001
  FSMC_BTR3_ADDSET_Bit1* = (0x00000002 shl FSMC_BTR3_ADDSET_Pos) ## !< 0x00000002
  FSMC_BTR3_ADDSET_Bit2* = (0x00000004 shl FSMC_BTR3_ADDSET_Pos) ## !< 0x00000004
  FSMC_BTR3_ADDSET_Bit3* = (0x00000008 shl FSMC_BTR3_ADDSET_Pos) ## !< 0x00000008
  FSMC_BTR3_ADDHLD_Pos* = (4)
  FSMC_BTR3_ADDHLD_Msk* = (0x0000000F shl FSMC_BTR3_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BTR3_ADDHLD* = FSMC_BTR3_ADDHLD_Msk
  FSMC_BTR3_ADDHLD_Bit0* = (0x00000001 shl FSMC_BTR3_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BTR3_ADDHLD_Bit1* = (0x00000002 shl FSMC_BTR3_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BTR3_ADDHLD_Bit2* = (0x00000004 shl FSMC_BTR3_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BTR3_ADDHLD_Bit3* = (0x00000008 shl FSMC_BTR3_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BTR3_DATAST_Pos* = (8)
  FSMC_BTR3_DATAST_Msk* = (0x000000FF shl FSMC_BTR3_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BTR3_DATAST* = FSMC_BTR3_DATAST_Msk
  FSMC_BTR3_DATAST_Bit0* = (0x00000001 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00000100
  FSMC_BTR3_DATAST_Bit1* = (0x00000002 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00000200
  FSMC_BTR3_DATAST_Bit2* = (0x00000004 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00000400
  FSMC_BTR3_DATAST_Bit3* = (0x00000008 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00000800
  FSMC_BTR3_DATAST_Bit4* = (0x00000010 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00001000
  FSMC_BTR3_DATAST_Bit5* = (0x00000020 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00002000
  FSMC_BTR3_DATAST_Bit6* = (0x00000040 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00004000
  FSMC_BTR3_DATAST_Bit7* = (0x00000080 shl FSMC_BTR3_DATAST_Pos) ## !< 0x00008000
  FSMC_BTR3_BUSTURN_Pos* = (16)
  FSMC_BTR3_BUSTURN_Msk* = (0x0000000F shl FSMC_BTR3_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BTR3_BUSTURN* = FSMC_BTR3_BUSTURN_Msk
  FSMC_BTR3_BUSTURN_Bit0* = (0x00000001 shl FSMC_BTR3_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BTR3_BUSTURN_Bit1* = (0x00000002 shl FSMC_BTR3_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BTR3_BUSTURN_Bit2* = (0x00000004 shl FSMC_BTR3_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BTR3_BUSTURN_Bit3* = (0x00000008 shl FSMC_BTR3_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BTR3_CLKDIV_Pos* = (20)
  FSMC_BTR3_CLKDIV_Msk* = (0x0000000F shl FSMC_BTR3_CLKDIV_Pos) ## !< 0x00F00000
  FSMC_BTR3_CLKDIV* = FSMC_BTR3_CLKDIV_Msk
  FSMC_BTR3_CLKDIV_Bit0* = (0x00000001 shl FSMC_BTR3_CLKDIV_Pos) ## !< 0x00100000
  FSMC_BTR3_CLKDIV_Bit1* = (0x00000002 shl FSMC_BTR3_CLKDIV_Pos) ## !< 0x00200000
  FSMC_BTR3_CLKDIV_Bit2* = (0x00000004 shl FSMC_BTR3_CLKDIV_Pos) ## !< 0x00400000
  FSMC_BTR3_CLKDIV_Bit3* = (0x00000008 shl FSMC_BTR3_CLKDIV_Pos) ## !< 0x00800000
  FSMC_BTR3_DATLAT_Pos* = (24)
  FSMC_BTR3_DATLAT_Msk* = (0x0000000F shl FSMC_BTR3_DATLAT_Pos) ## !< 0x0F000000
  FSMC_BTR3_DATLAT* = FSMC_BTR3_DATLAT_Msk
  FSMC_BTR3_DATLAT_Bit0* = (0x00000001 shl FSMC_BTR3_DATLAT_Pos) ## !< 0x01000000
  FSMC_BTR3_DATLAT_Bit1* = (0x00000002 shl FSMC_BTR3_DATLAT_Pos) ## !< 0x02000000
  FSMC_BTR3_DATLAT_Bit2* = (0x00000004 shl FSMC_BTR3_DATLAT_Pos) ## !< 0x04000000
  FSMC_BTR3_DATLAT_Bit3* = (0x00000008 shl FSMC_BTR3_DATLAT_Pos) ## !< 0x08000000
  FSMC_BTR3_ACCMOD_Pos* = (28)
  FSMC_BTR3_ACCMOD_Msk* = (0x00000003 shl FSMC_BTR3_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BTR3_ACCMOD* = FSMC_BTR3_ACCMOD_Msk
  FSMC_BTR3_ACCMOD_Bit0* = (0x00000001 shl FSMC_BTR3_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BTR3_ACCMOD_Bit1* = (0x00000002 shl FSMC_BTR3_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_BTR4 register  ******************

const
  FSMC_BTR4_ADDSET_Pos* = (0)
  FSMC_BTR4_ADDSET_Msk* = (0x0000000F shl FSMC_BTR4_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BTR4_ADDSET* = FSMC_BTR4_ADDSET_Msk
  FSMC_BTR4_ADDSET_Bit0* = (0x00000001 shl FSMC_BTR4_ADDSET_Pos) ## !< 0x00000001
  FSMC_BTR4_ADDSET_Bit1* = (0x00000002 shl FSMC_BTR4_ADDSET_Pos) ## !< 0x00000002
  FSMC_BTR4_ADDSET_Bit2* = (0x00000004 shl FSMC_BTR4_ADDSET_Pos) ## !< 0x00000004
  FSMC_BTR4_ADDSET_Bit3* = (0x00000008 shl FSMC_BTR4_ADDSET_Pos) ## !< 0x00000008
  FSMC_BTR4_ADDHLD_Pos* = (4)
  FSMC_BTR4_ADDHLD_Msk* = (0x0000000F shl FSMC_BTR4_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BTR4_ADDHLD* = FSMC_BTR4_ADDHLD_Msk
  FSMC_BTR4_ADDHLD_Bit0* = (0x00000001 shl FSMC_BTR4_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BTR4_ADDHLD_Bit1* = (0x00000002 shl FSMC_BTR4_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BTR4_ADDHLD_Bit2* = (0x00000004 shl FSMC_BTR4_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BTR4_ADDHLD_Bit3* = (0x00000008 shl FSMC_BTR4_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BTR4_DATAST_Pos* = (8)
  FSMC_BTR4_DATAST_Msk* = (0x000000FF shl FSMC_BTR4_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BTR4_DATAST* = FSMC_BTR4_DATAST_Msk
  FSMC_BTR4_DATAST_Bit0* = (0x00000001 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00000100
  FSMC_BTR4_DATAST_Bit1* = (0x00000002 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00000200
  FSMC_BTR4_DATAST_Bit2* = (0x00000004 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00000400
  FSMC_BTR4_DATAST_Bit3* = (0x00000008 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00000800
  FSMC_BTR4_DATAST_Bit4* = (0x00000010 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00001000
  FSMC_BTR4_DATAST_Bit5* = (0x00000020 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00002000
  FSMC_BTR4_DATAST_Bit6* = (0x00000040 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00004000
  FSMC_BTR4_DATAST_Bit7* = (0x00000080 shl FSMC_BTR4_DATAST_Pos) ## !< 0x00008000
  FSMC_BTR4_BUSTURN_Pos* = (16)
  FSMC_BTR4_BUSTURN_Msk* = (0x0000000F shl FSMC_BTR4_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BTR4_BUSTURN* = FSMC_BTR4_BUSTURN_Msk
  FSMC_BTR4_BUSTURN_Bit0* = (0x00000001 shl FSMC_BTR4_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BTR4_BUSTURN_Bit1* = (0x00000002 shl FSMC_BTR4_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BTR4_BUSTURN_Bit2* = (0x00000004 shl FSMC_BTR4_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BTR4_BUSTURN_Bit3* = (0x00000008 shl FSMC_BTR4_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BTR4_CLKDIV_Pos* = (20)
  FSMC_BTR4_CLKDIV_Msk* = (0x0000000F shl FSMC_BTR4_CLKDIV_Pos) ## !< 0x00F00000
  FSMC_BTR4_CLKDIV* = FSMC_BTR4_CLKDIV_Msk
  FSMC_BTR4_CLKDIV_Bit0* = (0x00000001 shl FSMC_BTR4_CLKDIV_Pos) ## !< 0x00100000
  FSMC_BTR4_CLKDIV_Bit1* = (0x00000002 shl FSMC_BTR4_CLKDIV_Pos) ## !< 0x00200000
  FSMC_BTR4_CLKDIV_Bit2* = (0x00000004 shl FSMC_BTR4_CLKDIV_Pos) ## !< 0x00400000
  FSMC_BTR4_CLKDIV_Bit3* = (0x00000008 shl FSMC_BTR4_CLKDIV_Pos) ## !< 0x00800000
  FSMC_BTR4_DATLAT_Pos* = (24)
  FSMC_BTR4_DATLAT_Msk* = (0x0000000F shl FSMC_BTR4_DATLAT_Pos) ## !< 0x0F000000
  FSMC_BTR4_DATLAT* = FSMC_BTR4_DATLAT_Msk
  FSMC_BTR4_DATLAT_Bit0* = (0x00000001 shl FSMC_BTR4_DATLAT_Pos) ## !< 0x01000000
  FSMC_BTR4_DATLAT_Bit1* = (0x00000002 shl FSMC_BTR4_DATLAT_Pos) ## !< 0x02000000
  FSMC_BTR4_DATLAT_Bit2* = (0x00000004 shl FSMC_BTR4_DATLAT_Pos) ## !< 0x04000000
  FSMC_BTR4_DATLAT_Bit3* = (0x00000008 shl FSMC_BTR4_DATLAT_Pos) ## !< 0x08000000
  FSMC_BTR4_ACCMOD_Pos* = (28)
  FSMC_BTR4_ACCMOD_Msk* = (0x00000003 shl FSMC_BTR4_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BTR4_ACCMOD* = FSMC_BTR4_ACCMOD_Msk
  FSMC_BTR4_ACCMOD_Bit0* = (0x00000001 shl FSMC_BTR4_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BTR4_ACCMOD_Bit1* = (0x00000002 shl FSMC_BTR4_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_BWTR1 register  *****************

const
  FSMC_BWTR1_ADDSET_Pos* = (0)
  FSMC_BWTR1_ADDSET_Msk* = (0x0000000F shl FSMC_BWTR1_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BWTR1_ADDSET* = FSMC_BWTR1_ADDSET_Msk
  FSMC_BWTR1_ADDSET_Bit0* = (0x00000001 shl FSMC_BWTR1_ADDSET_Pos) ## !< 0x00000001
  FSMC_BWTR1_ADDSET_Bit1* = (0x00000002 shl FSMC_BWTR1_ADDSET_Pos) ## !< 0x00000002
  FSMC_BWTR1_ADDSET_Bit2* = (0x00000004 shl FSMC_BWTR1_ADDSET_Pos) ## !< 0x00000004
  FSMC_BWTR1_ADDSET_Bit3* = (0x00000008 shl FSMC_BWTR1_ADDSET_Pos) ## !< 0x00000008
  FSMC_BWTR1_ADDHLD_Pos* = (4)
  FSMC_BWTR1_ADDHLD_Msk* = (0x0000000F shl FSMC_BWTR1_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BWTR1_ADDHLD* = FSMC_BWTR1_ADDHLD_Msk
  FSMC_BWTR1_ADDHLD_Bit0* = (0x00000001 shl FSMC_BWTR1_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BWTR1_ADDHLD_Bit1* = (0x00000002 shl FSMC_BWTR1_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BWTR1_ADDHLD_Bit2* = (0x00000004 shl FSMC_BWTR1_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BWTR1_ADDHLD_Bit3* = (0x00000008 shl FSMC_BWTR1_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BWTR1_DATAST_Pos* = (8)
  FSMC_BWTR1_DATAST_Msk* = (0x000000FF shl FSMC_BWTR1_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BWTR1_DATAST* = FSMC_BWTR1_DATAST_Msk
  FSMC_BWTR1_DATAST_Bit0* = (0x00000001 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00000100
  FSMC_BWTR1_DATAST_Bit1* = (0x00000002 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00000200
  FSMC_BWTR1_DATAST_Bit2* = (0x00000004 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00000400
  FSMC_BWTR1_DATAST_Bit3* = (0x00000008 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00000800
  FSMC_BWTR1_DATAST_Bit4* = (0x00000010 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00001000
  FSMC_BWTR1_DATAST_Bit5* = (0x00000020 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00002000
  FSMC_BWTR1_DATAST_Bit6* = (0x00000040 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00004000
  FSMC_BWTR1_DATAST_Bit7* = (0x00000080 shl FSMC_BWTR1_DATAST_Pos) ## !< 0x00008000
  FSMC_BWTR1_BUSTURN_Pos* = (16)
  FSMC_BWTR1_BUSTURN_Msk* = (0x0000000F shl FSMC_BWTR1_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BWTR1_BUSTURN* = FSMC_BWTR1_BUSTURN_Msk
  FSMC_BWTR1_BUSTURN_Bit0* = (0x00000001 shl FSMC_BWTR1_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BWTR1_BUSTURN_Bit1* = (0x00000002 shl FSMC_BWTR1_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BWTR1_BUSTURN_Bit2* = (0x00000004 shl FSMC_BWTR1_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BWTR1_BUSTURN_Bit3* = (0x00000008 shl FSMC_BWTR1_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BWTR1_ACCMOD_Pos* = (28)
  FSMC_BWTR1_ACCMOD_Msk* = (0x00000003 shl FSMC_BWTR1_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BWTR1_ACCMOD* = FSMC_BWTR1_ACCMOD_Msk
  FSMC_BWTR1_ACCMOD_Bit0* = (0x00000001 shl FSMC_BWTR1_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BWTR1_ACCMOD_Bit1* = (0x00000002 shl FSMC_BWTR1_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_BWTR2 register  *****************

const
  FSMC_BWTR2_ADDSET_Pos* = (0)
  FSMC_BWTR2_ADDSET_Msk* = (0x0000000F shl FSMC_BWTR2_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BWTR2_ADDSET* = FSMC_BWTR2_ADDSET_Msk
  FSMC_BWTR2_ADDSET_Bit0* = (0x00000001 shl FSMC_BWTR2_ADDSET_Pos) ## !< 0x00000001
  FSMC_BWTR2_ADDSET_Bit1* = (0x00000002 shl FSMC_BWTR2_ADDSET_Pos) ## !< 0x00000002
  FSMC_BWTR2_ADDSET_Bit2* = (0x00000004 shl FSMC_BWTR2_ADDSET_Pos) ## !< 0x00000004
  FSMC_BWTR2_ADDSET_Bit3* = (0x00000008 shl FSMC_BWTR2_ADDSET_Pos) ## !< 0x00000008
  FSMC_BWTR2_ADDHLD_Pos* = (4)
  FSMC_BWTR2_ADDHLD_Msk* = (0x0000000F shl FSMC_BWTR2_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BWTR2_ADDHLD* = FSMC_BWTR2_ADDHLD_Msk
  FSMC_BWTR2_ADDHLD_Bit0* = (0x00000001 shl FSMC_BWTR2_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BWTR2_ADDHLD_Bit1* = (0x00000002 shl FSMC_BWTR2_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BWTR2_ADDHLD_Bit2* = (0x00000004 shl FSMC_BWTR2_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BWTR2_ADDHLD_Bit3* = (0x00000008 shl FSMC_BWTR2_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BWTR2_DATAST_Pos* = (8)
  FSMC_BWTR2_DATAST_Msk* = (0x000000FF shl FSMC_BWTR2_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BWTR2_DATAST* = FSMC_BWTR2_DATAST_Msk
  FSMC_BWTR2_DATAST_Bit0* = (0x00000001 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00000100
  FSMC_BWTR2_DATAST_Bit1* = (0x00000002 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00000200
  FSMC_BWTR2_DATAST_Bit2* = (0x00000004 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00000400
  FSMC_BWTR2_DATAST_Bit3* = (0x00000008 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00000800
  FSMC_BWTR2_DATAST_Bit4* = (0x00000010 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00001000
  FSMC_BWTR2_DATAST_Bit5* = (0x00000020 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00002000
  FSMC_BWTR2_DATAST_Bit6* = (0x00000040 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00004000
  FSMC_BWTR2_DATAST_Bit7* = (0x00000080 shl FSMC_BWTR2_DATAST_Pos) ## !< 0x00008000
  FSMC_BWTR2_BUSTURN_Pos* = (16)
  FSMC_BWTR2_BUSTURN_Msk* = (0x0000000F shl FSMC_BWTR2_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BWTR2_BUSTURN* = FSMC_BWTR2_BUSTURN_Msk
  FSMC_BWTR2_BUSTURN_Bit0* = (0x00000001 shl FSMC_BWTR2_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BWTR2_BUSTURN_Bit1* = (0x00000002 shl FSMC_BWTR2_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BWTR2_BUSTURN_Bit2* = (0x00000004 shl FSMC_BWTR2_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BWTR2_BUSTURN_Bit3* = (0x00000008 shl FSMC_BWTR2_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BWTR2_ACCMOD_Pos* = (28)
  FSMC_BWTR2_ACCMOD_Msk* = (0x00000003 shl FSMC_BWTR2_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BWTR2_ACCMOD* = FSMC_BWTR2_ACCMOD_Msk
  FSMC_BWTR2_ACCMOD_Bit0* = (0x00000001 shl FSMC_BWTR2_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BWTR2_ACCMOD_Bit1* = (0x00000002 shl FSMC_BWTR2_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_BWTR3 register  *****************

const
  FSMC_BWTR3_ADDSET_Pos* = (0)
  FSMC_BWTR3_ADDSET_Msk* = (0x0000000F shl FSMC_BWTR3_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BWTR3_ADDSET* = FSMC_BWTR3_ADDSET_Msk
  FSMC_BWTR3_ADDSET_Bit0* = (0x00000001 shl FSMC_BWTR3_ADDSET_Pos) ## !< 0x00000001
  FSMC_BWTR3_ADDSET_Bit1* = (0x00000002 shl FSMC_BWTR3_ADDSET_Pos) ## !< 0x00000002
  FSMC_BWTR3_ADDSET_Bit2* = (0x00000004 shl FSMC_BWTR3_ADDSET_Pos) ## !< 0x00000004
  FSMC_BWTR3_ADDSET_Bit3* = (0x00000008 shl FSMC_BWTR3_ADDSET_Pos) ## !< 0x00000008
  FSMC_BWTR3_ADDHLD_Pos* = (4)
  FSMC_BWTR3_ADDHLD_Msk* = (0x0000000F shl FSMC_BWTR3_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BWTR3_ADDHLD* = FSMC_BWTR3_ADDHLD_Msk
  FSMC_BWTR3_ADDHLD_Bit0* = (0x00000001 shl FSMC_BWTR3_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BWTR3_ADDHLD_Bit1* = (0x00000002 shl FSMC_BWTR3_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BWTR3_ADDHLD_Bit2* = (0x00000004 shl FSMC_BWTR3_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BWTR3_ADDHLD_Bit3* = (0x00000008 shl FSMC_BWTR3_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BWTR3_DATAST_Pos* = (8)
  FSMC_BWTR3_DATAST_Msk* = (0x000000FF shl FSMC_BWTR3_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BWTR3_DATAST* = FSMC_BWTR3_DATAST_Msk
  FSMC_BWTR3_DATAST_Bit0* = (0x00000001 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00000100
  FSMC_BWTR3_DATAST_Bit1* = (0x00000002 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00000200
  FSMC_BWTR3_DATAST_Bit2* = (0x00000004 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00000400
  FSMC_BWTR3_DATAST_Bit3* = (0x00000008 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00000800
  FSMC_BWTR3_DATAST_Bit4* = (0x00000010 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00001000
  FSMC_BWTR3_DATAST_Bit5* = (0x00000020 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00002000
  FSMC_BWTR3_DATAST_Bit6* = (0x00000040 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00004000
  FSMC_BWTR3_DATAST_Bit7* = (0x00000080 shl FSMC_BWTR3_DATAST_Pos) ## !< 0x00008000
  FSMC_BWTR3_BUSTURN_Pos* = (16)
  FSMC_BWTR3_BUSTURN_Msk* = (0x0000000F shl FSMC_BWTR3_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BWTR3_BUSTURN* = FSMC_BWTR3_BUSTURN_Msk
  FSMC_BWTR3_BUSTURN_Bit0* = (0x00000001 shl FSMC_BWTR3_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BWTR3_BUSTURN_Bit1* = (0x00000002 shl FSMC_BWTR3_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BWTR3_BUSTURN_Bit2* = (0x00000004 shl FSMC_BWTR3_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BWTR3_BUSTURN_Bit3* = (0x00000008 shl FSMC_BWTR3_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BWTR3_ACCMOD_Pos* = (28)
  FSMC_BWTR3_ACCMOD_Msk* = (0x00000003 shl FSMC_BWTR3_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BWTR3_ACCMOD* = FSMC_BWTR3_ACCMOD_Msk
  FSMC_BWTR3_ACCMOD_Bit0* = (0x00000001 shl FSMC_BWTR3_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BWTR3_ACCMOD_Bit1* = (0x00000002 shl FSMC_BWTR3_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_BWTR4 register  *****************

const
  FSMC_BWTR4_ADDSET_Pos* = (0)
  FSMC_BWTR4_ADDSET_Msk* = (0x0000000F shl FSMC_BWTR4_ADDSET_Pos) ## !< 0x0000000F
  FSMC_BWTR4_ADDSET* = FSMC_BWTR4_ADDSET_Msk
  FSMC_BWTR4_ADDSET_Bit0* = (0x00000001 shl FSMC_BWTR4_ADDSET_Pos) ## !< 0x00000001
  FSMC_BWTR4_ADDSET_Bit1* = (0x00000002 shl FSMC_BWTR4_ADDSET_Pos) ## !< 0x00000002
  FSMC_BWTR4_ADDSET_Bit2* = (0x00000004 shl FSMC_BWTR4_ADDSET_Pos) ## !< 0x00000004
  FSMC_BWTR4_ADDSET_Bit3* = (0x00000008 shl FSMC_BWTR4_ADDSET_Pos) ## !< 0x00000008
  FSMC_BWTR4_ADDHLD_Pos* = (4)
  FSMC_BWTR4_ADDHLD_Msk* = (0x0000000F shl FSMC_BWTR4_ADDHLD_Pos) ## !< 0x000000F0
  FSMC_BWTR4_ADDHLD* = FSMC_BWTR4_ADDHLD_Msk
  FSMC_BWTR4_ADDHLD_Bit0* = (0x00000001 shl FSMC_BWTR4_ADDHLD_Pos) ## !< 0x00000010
  FSMC_BWTR4_ADDHLD_Bit1* = (0x00000002 shl FSMC_BWTR4_ADDHLD_Pos) ## !< 0x00000020
  FSMC_BWTR4_ADDHLD_Bit2* = (0x00000004 shl FSMC_BWTR4_ADDHLD_Pos) ## !< 0x00000040
  FSMC_BWTR4_ADDHLD_Bit3* = (0x00000008 shl FSMC_BWTR4_ADDHLD_Pos) ## !< 0x00000080
  FSMC_BWTR4_DATAST_Pos* = (8)
  FSMC_BWTR4_DATAST_Msk* = (0x000000FF shl FSMC_BWTR4_DATAST_Pos) ## !< 0x0000FF00
  FSMC_BWTR4_DATAST* = FSMC_BWTR4_DATAST_Msk
  FSMC_BWTR4_DATAST_Bit0* = 0x00000100
  FSMC_BWTR4_DATAST_Bit1* = 0x00000200
  FSMC_BWTR4_DATAST_Bit2* = 0x00000400
  FSMC_BWTR4_DATAST_Bit3* = 0x00000800
  FSMC_BWTR4_DATAST_Bit4* = 0x00001000
  FSMC_BWTR4_DATAST_Bit5* = 0x00002000
  FSMC_BWTR4_DATAST_Bit6* = 0x00004000
  FSMC_BWTR4_DATAST_Bit7* = 0x00008000
  FSMC_BWTR4_BUSTURN_Pos* = (16)
  FSMC_BWTR4_BUSTURN_Msk* = (0x0000000F shl FSMC_BWTR4_BUSTURN_Pos) ## !< 0x000F0000
  FSMC_BWTR4_BUSTURN* = FSMC_BWTR4_BUSTURN_Msk
  FSMC_BWTR4_BUSTURN_Bit0* = (0x00000001 shl FSMC_BWTR4_BUSTURN_Pos) ## !< 0x00010000
  FSMC_BWTR4_BUSTURN_Bit1* = (0x00000002 shl FSMC_BWTR4_BUSTURN_Pos) ## !< 0x00020000
  FSMC_BWTR4_BUSTURN_Bit2* = (0x00000004 shl FSMC_BWTR4_BUSTURN_Pos) ## !< 0x00040000
  FSMC_BWTR4_BUSTURN_Bit3* = (0x00000008 shl FSMC_BWTR4_BUSTURN_Pos) ## !< 0x00080000
  FSMC_BWTR4_ACCMOD_Pos* = (28)
  FSMC_BWTR4_ACCMOD_Msk* = (0x00000003 shl FSMC_BWTR4_ACCMOD_Pos) ## !< 0x30000000
  FSMC_BWTR4_ACCMOD* = FSMC_BWTR4_ACCMOD_Msk
  FSMC_BWTR4_ACCMOD_Bit0* = (0x00000001 shl FSMC_BWTR4_ACCMOD_Pos) ## !< 0x10000000
  FSMC_BWTR4_ACCMOD_Bit1* = (0x00000002 shl FSMC_BWTR4_ACCMOD_Pos) ## !< 0x20000000

## *****************  Bit definition for FSMC_PCR2 register  ******************

const
  FSMC_PCR2_PWAITEN_Pos* = (1)
  FSMC_PCR2_PWAITEN_Msk* = (0x00000001 shl FSMC_PCR2_PWAITEN_Pos) ## !< 0x00000002
  FSMC_PCR2_PWAITEN* = FSMC_PCR2_PWAITEN_Msk
  FSMC_PCR2_PBKEN_Pos* = (2)
  FSMC_PCR2_PBKEN_Msk* = (0x00000001 shl FSMC_PCR2_PBKEN_Pos) ## !< 0x00000004
  FSMC_PCR2_PBKEN* = FSMC_PCR2_PBKEN_Msk
  FSMC_PCR2_PTYP_Pos* = (3)
  FSMC_PCR2_PTYP_Msk* = (0x00000001 shl FSMC_PCR2_PTYP_Pos) ## !< 0x00000008
  FSMC_PCR2_PTYP* = FSMC_PCR2_PTYP_Msk
  FSMC_PCR2_PWID_Pos* = (4)
  FSMC_PCR2_PWID_Msk* = (0x00000003 shl FSMC_PCR2_PWID_Pos) ## !< 0x00000030
  FSMC_PCR2_PWID* = FSMC_PCR2_PWID_Msk
  FSMC_PCR2_PWID_Bit0* = (0x00000001 shl FSMC_PCR2_PWID_Pos) ## !< 0x00000010
  FSMC_PCR2_PWID_Bit1* = (0x00000002 shl FSMC_PCR2_PWID_Pos) ## !< 0x00000020
  FSMC_PCR2_ECCEN_Pos* = (6)
  FSMC_PCR2_ECCEN_Msk* = (0x00000001 shl FSMC_PCR2_ECCEN_Pos) ## !< 0x00000040
  FSMC_PCR2_ECCEN* = FSMC_PCR2_ECCEN_Msk
  FSMC_PCR2_TCLR_Pos* = (9)
  FSMC_PCR2_TCLR_Msk* = (0x0000000F shl FSMC_PCR2_TCLR_Pos) ## !< 0x00001E00
  FSMC_PCR2_TCLR* = FSMC_PCR2_TCLR_Msk
  FSMC_PCR2_TCLR_Bit0* = (0x00000001 shl FSMC_PCR2_TCLR_Pos) ## !< 0x00000200
  FSMC_PCR2_TCLR_Bit1* = (0x00000002 shl FSMC_PCR2_TCLR_Pos) ## !< 0x00000400
  FSMC_PCR2_TCLR_Bit2* = (0x00000004 shl FSMC_PCR2_TCLR_Pos) ## !< 0x00000800
  FSMC_PCR2_TCLR_Bit3* = (0x00000008 shl FSMC_PCR2_TCLR_Pos) ## !< 0x00001000
  FSMC_PCR2_TAR_Pos* = (13)
  FSMC_PCR2_TAR_Msk* = (0x0000000F shl FSMC_PCR2_TAR_Pos) ## !< 0x0001E000
  FSMC_PCR2_TAR* = FSMC_PCR2_TAR_Msk
  FSMC_PCR2_TAR_Bit0* = (0x00000001 shl FSMC_PCR2_TAR_Pos) ## !< 0x00002000
  FSMC_PCR2_TAR_Bit1* = (0x00000002 shl FSMC_PCR2_TAR_Pos) ## !< 0x00004000
  FSMC_PCR2_TAR_Bit2* = (0x00000004 shl FSMC_PCR2_TAR_Pos) ## !< 0x00008000
  FSMC_PCR2_TAR_Bit3* = (0x00000008 shl FSMC_PCR2_TAR_Pos) ## !< 0x00010000
  FSMC_PCR2_ECCPS_Pos* = (17)
  FSMC_PCR2_ECCPS_Msk* = (0x00000007 shl FSMC_PCR2_ECCPS_Pos) ## !< 0x000E0000
  FSMC_PCR2_ECCPS* = FSMC_PCR2_ECCPS_Msk
  FSMC_PCR2_ECCPS_Bit0* = (0x00000001 shl FSMC_PCR2_ECCPS_Pos) ## !< 0x00020000
  FSMC_PCR2_ECCPS_Bit1* = (0x00000002 shl FSMC_PCR2_ECCPS_Pos) ## !< 0x00040000
  FSMC_PCR2_ECCPS_Bit2* = (0x00000004 shl FSMC_PCR2_ECCPS_Pos) ## !< 0x00080000

## *****************  Bit definition for FSMC_PCR3 register  ******************

const
  FSMC_PCR3_PWAITEN_Pos* = (1)
  FSMC_PCR3_PWAITEN_Msk* = (0x00000001 shl FSMC_PCR3_PWAITEN_Pos) ## !< 0x00000002
  FSMC_PCR3_PWAITEN* = FSMC_PCR3_PWAITEN_Msk
  FSMC_PCR3_PBKEN_Pos* = (2)
  FSMC_PCR3_PBKEN_Msk* = (0x00000001 shl FSMC_PCR3_PBKEN_Pos) ## !< 0x00000004
  FSMC_PCR3_PBKEN* = FSMC_PCR3_PBKEN_Msk
  FSMC_PCR3_PTYP_Pos* = (3)
  FSMC_PCR3_PTYP_Msk* = (0x00000001 shl FSMC_PCR3_PTYP_Pos) ## !< 0x00000008
  FSMC_PCR3_PTYP* = FSMC_PCR3_PTYP_Msk
  FSMC_PCR3_PWID_Pos* = (4)
  FSMC_PCR3_PWID_Msk* = (0x00000003 shl FSMC_PCR3_PWID_Pos) ## !< 0x00000030
  FSMC_PCR3_PWID* = FSMC_PCR3_PWID_Msk
  FSMC_PCR3_PWID_Bit0* = (0x00000001 shl FSMC_PCR3_PWID_Pos) ## !< 0x00000010
  FSMC_PCR3_PWID_Bit1* = (0x00000002 shl FSMC_PCR3_PWID_Pos) ## !< 0x00000020
  FSMC_PCR3_ECCEN_Pos* = (6)
  FSMC_PCR3_ECCEN_Msk* = (0x00000001 shl FSMC_PCR3_ECCEN_Pos) ## !< 0x00000040
  FSMC_PCR3_ECCEN* = FSMC_PCR3_ECCEN_Msk
  FSMC_PCR3_TCLR_Pos* = (9)
  FSMC_PCR3_TCLR_Msk* = (0x0000000F shl FSMC_PCR3_TCLR_Pos) ## !< 0x00001E00
  FSMC_PCR3_TCLR* = FSMC_PCR3_TCLR_Msk
  FSMC_PCR3_TCLR_Bit0* = (0x00000001 shl FSMC_PCR3_TCLR_Pos) ## !< 0x00000200
  FSMC_PCR3_TCLR_Bit1* = (0x00000002 shl FSMC_PCR3_TCLR_Pos) ## !< 0x00000400
  FSMC_PCR3_TCLR_Bit2* = (0x00000004 shl FSMC_PCR3_TCLR_Pos) ## !< 0x00000800
  FSMC_PCR3_TCLR_Bit3* = (0x00000008 shl FSMC_PCR3_TCLR_Pos) ## !< 0x00001000
  FSMC_PCR3_TAR_Pos* = (13)
  FSMC_PCR3_TAR_Msk* = (0x0000000F shl FSMC_PCR3_TAR_Pos) ## !< 0x0001E000
  FSMC_PCR3_TAR* = FSMC_PCR3_TAR_Msk
  FSMC_PCR3_TAR_Bit0* = (0x00000001 shl FSMC_PCR3_TAR_Pos) ## !< 0x00002000
  FSMC_PCR3_TAR_Bit1* = (0x00000002 shl FSMC_PCR3_TAR_Pos) ## !< 0x00004000
  FSMC_PCR3_TAR_Bit2* = (0x00000004 shl FSMC_PCR3_TAR_Pos) ## !< 0x00008000
  FSMC_PCR3_TAR_Bit3* = (0x00000008 shl FSMC_PCR3_TAR_Pos) ## !< 0x00010000
  FSMC_PCR3_ECCPS_Pos* = (17)
  FSMC_PCR3_ECCPS_Msk* = (0x00000007 shl FSMC_PCR3_ECCPS_Pos) ## !< 0x000E0000
  FSMC_PCR3_ECCPS* = FSMC_PCR3_ECCPS_Msk
  FSMC_PCR3_ECCPS_Bit0* = (0x00000001 shl FSMC_PCR3_ECCPS_Pos) ## !< 0x00020000
  FSMC_PCR3_ECCPS_Bit1* = (0x00000002 shl FSMC_PCR3_ECCPS_Pos) ## !< 0x00040000
  FSMC_PCR3_ECCPS_Bit2* = (0x00000004 shl FSMC_PCR3_ECCPS_Pos) ## !< 0x00080000

## *****************  Bit definition for FSMC_PCR4 register  ******************

const
  FSMC_PCR4_PWAITEN_Pos* = (1)
  FSMC_PCR4_PWAITEN_Msk* = (0x00000001 shl FSMC_PCR4_PWAITEN_Pos) ## !< 0x00000002
  FSMC_PCR4_PWAITEN* = FSMC_PCR4_PWAITEN_Msk
  FSMC_PCR4_PBKEN_Pos* = (2)
  FSMC_PCR4_PBKEN_Msk* = (0x00000001 shl FSMC_PCR4_PBKEN_Pos) ## !< 0x00000004
  FSMC_PCR4_PBKEN* = FSMC_PCR4_PBKEN_Msk
  FSMC_PCR4_PTYP_Pos* = (3)
  FSMC_PCR4_PTYP_Msk* = (0x00000001 shl FSMC_PCR4_PTYP_Pos) ## !< 0x00000008
  FSMC_PCR4_PTYP* = FSMC_PCR4_PTYP_Msk
  FSMC_PCR4_PWID_Pos* = (4)
  FSMC_PCR4_PWID_Msk* = (0x00000003 shl FSMC_PCR4_PWID_Pos) ## !< 0x00000030
  FSMC_PCR4_PWID* = FSMC_PCR4_PWID_Msk
  FSMC_PCR4_PWID_Bit0* = (0x00000001 shl FSMC_PCR4_PWID_Pos) ## !< 0x00000010
  FSMC_PCR4_PWID_Bit1* = (0x00000002 shl FSMC_PCR4_PWID_Pos) ## !< 0x00000020
  FSMC_PCR4_ECCEN_Pos* = (6)
  FSMC_PCR4_ECCEN_Msk* = (0x00000001 shl FSMC_PCR4_ECCEN_Pos) ## !< 0x00000040
  FSMC_PCR4_ECCEN* = FSMC_PCR4_ECCEN_Msk
  FSMC_PCR4_TCLR_Pos* = (9)
  FSMC_PCR4_TCLR_Msk* = (0x0000000F shl FSMC_PCR4_TCLR_Pos) ## !< 0x00001E00
  FSMC_PCR4_TCLR* = FSMC_PCR4_TCLR_Msk
  FSMC_PCR4_TCLR_Bit0* = (0x00000001 shl FSMC_PCR4_TCLR_Pos) ## !< 0x00000200
  FSMC_PCR4_TCLR_Bit1* = (0x00000002 shl FSMC_PCR4_TCLR_Pos) ## !< 0x00000400
  FSMC_PCR4_TCLR_Bit2* = (0x00000004 shl FSMC_PCR4_TCLR_Pos) ## !< 0x00000800
  FSMC_PCR4_TCLR_Bit3* = (0x00000008 shl FSMC_PCR4_TCLR_Pos) ## !< 0x00001000
  FSMC_PCR4_TAR_Pos* = (13)
  FSMC_PCR4_TAR_Msk* = (0x0000000F shl FSMC_PCR4_TAR_Pos) ## !< 0x0001E000
  FSMC_PCR4_TAR* = FSMC_PCR4_TAR_Msk
  FSMC_PCR4_TAR_Bit0* = (0x00000001 shl FSMC_PCR4_TAR_Pos) ## !< 0x00002000
  FSMC_PCR4_TAR_Bit1* = (0x00000002 shl FSMC_PCR4_TAR_Pos) ## !< 0x00004000
  FSMC_PCR4_TAR_Bit2* = (0x00000004 shl FSMC_PCR4_TAR_Pos) ## !< 0x00008000
  FSMC_PCR4_TAR_Bit3* = (0x00000008 shl FSMC_PCR4_TAR_Pos) ## !< 0x00010000
  FSMC_PCR4_ECCPS_Pos* = (17)
  FSMC_PCR4_ECCPS_Msk* = (0x00000007 shl FSMC_PCR4_ECCPS_Pos) ## !< 0x000E0000
  FSMC_PCR4_ECCPS* = FSMC_PCR4_ECCPS_Msk
  FSMC_PCR4_ECCPS_Bit0* = (0x00000001 shl FSMC_PCR4_ECCPS_Pos) ## !< 0x00020000
  FSMC_PCR4_ECCPS_Bit1* = (0x00000002 shl FSMC_PCR4_ECCPS_Pos) ## !< 0x00040000
  FSMC_PCR4_ECCPS_Bit2* = (0x00000004 shl FSMC_PCR4_ECCPS_Pos) ## !< 0x00080000

## ******************  Bit definition for FSMC_SR2 register  ******************

const
  FSMC_SR2_IRS_Pos* = (0)
  FSMC_SR2_IRS_Msk* = (0x00000001 shl FSMC_SR2_IRS_Pos) ## !< 0x00000001
  FSMC_SR2_IRS* = FSMC_SR2_IRS_Msk
  FSMC_SR2_ILS_Pos* = (1)
  FSMC_SR2_ILS_Msk* = (0x00000001 shl FSMC_SR2_ILS_Pos) ## !< 0x00000002
  FSMC_SR2_ILS* = FSMC_SR2_ILS_Msk
  FSMC_SR2_IFS_Pos* = (2)
  FSMC_SR2_IFS_Msk* = (0x00000001 shl FSMC_SR2_IFS_Pos) ## !< 0x00000004
  FSMC_SR2_IFS* = FSMC_SR2_IFS_Msk
  FSMC_SR2_IREN_Pos* = (3)
  FSMC_SR2_IREN_Msk* = (0x00000001 shl FSMC_SR2_IREN_Pos) ## !< 0x00000008
  FSMC_SR2_IREN* = FSMC_SR2_IREN_Msk
  FSMC_SR2_ILEN_Pos* = (4)
  FSMC_SR2_ILEN_Msk* = (0x00000001 shl FSMC_SR2_ILEN_Pos) ## !< 0x00000010
  FSMC_SR2_ILEN* = FSMC_SR2_ILEN_Msk
  FSMC_SR2_IFEN_Pos* = (5)
  FSMC_SR2_IFEN_Msk* = (0x00000001 shl FSMC_SR2_IFEN_Pos) ## !< 0x00000020
  FSMC_SR2_IFEN* = FSMC_SR2_IFEN_Msk
  FSMC_SR2_FEMPT_Pos* = (6)
  FSMC_SR2_FEMPT_Msk* = (0x00000001 shl FSMC_SR2_FEMPT_Pos) ## !< 0x00000040
  FSMC_SR2_FEMPT* = FSMC_SR2_FEMPT_Msk

## ******************  Bit definition for FSMC_SR3 register  ******************

const
  FSMC_SR3_IRS_Pos* = (0)
  FSMC_SR3_IRS_Msk* = (0x00000001 shl FSMC_SR3_IRS_Pos) ## !< 0x00000001
  FSMC_SR3_IRS* = FSMC_SR3_IRS_Msk
  FSMC_SR3_ILS_Pos* = (1)
  FSMC_SR3_ILS_Msk* = (0x00000001 shl FSMC_SR3_ILS_Pos) ## !< 0x00000002
  FSMC_SR3_ILS* = FSMC_SR3_ILS_Msk
  FSMC_SR3_IFS_Pos* = (2)
  FSMC_SR3_IFS_Msk* = (0x00000001 shl FSMC_SR3_IFS_Pos) ## !< 0x00000004
  FSMC_SR3_IFS* = FSMC_SR3_IFS_Msk
  FSMC_SR3_IREN_Pos* = (3)
  FSMC_SR3_IREN_Msk* = (0x00000001 shl FSMC_SR3_IREN_Pos) ## !< 0x00000008
  FSMC_SR3_IREN* = FSMC_SR3_IREN_Msk
  FSMC_SR3_ILEN_Pos* = (4)
  FSMC_SR3_ILEN_Msk* = (0x00000001 shl FSMC_SR3_ILEN_Pos) ## !< 0x00000010
  FSMC_SR3_ILEN* = FSMC_SR3_ILEN_Msk
  FSMC_SR3_IFEN_Pos* = (5)
  FSMC_SR3_IFEN_Msk* = (0x00000001 shl FSMC_SR3_IFEN_Pos) ## !< 0x00000020
  FSMC_SR3_IFEN* = FSMC_SR3_IFEN_Msk
  FSMC_SR3_FEMPT_Pos* = (6)
  FSMC_SR3_FEMPT_Msk* = (0x00000001 shl FSMC_SR3_FEMPT_Pos) ## !< 0x00000040
  FSMC_SR3_FEMPT* = FSMC_SR3_FEMPT_Msk

## ******************  Bit definition for FSMC_SR4 register  ******************

const
  FSMC_SR4_IRS_Pos* = (0)
  FSMC_SR4_IRS_Msk* = (0x00000001 shl FSMC_SR4_IRS_Pos) ## !< 0x00000001
  FSMC_SR4_IRS* = FSMC_SR4_IRS_Msk
  FSMC_SR4_ILS_Pos* = (1)
  FSMC_SR4_ILS_Msk* = (0x00000001 shl FSMC_SR4_ILS_Pos) ## !< 0x00000002
  FSMC_SR4_ILS* = FSMC_SR4_ILS_Msk
  FSMC_SR4_IFS_Pos* = (2)
  FSMC_SR4_IFS_Msk* = (0x00000001 shl FSMC_SR4_IFS_Pos) ## !< 0x00000004
  FSMC_SR4_IFS* = FSMC_SR4_IFS_Msk
  FSMC_SR4_IREN_Pos* = (3)
  FSMC_SR4_IREN_Msk* = (0x00000001 shl FSMC_SR4_IREN_Pos) ## !< 0x00000008
  FSMC_SR4_IREN* = FSMC_SR4_IREN_Msk
  FSMC_SR4_ILEN_Pos* = (4)
  FSMC_SR4_ILEN_Msk* = (0x00000001 shl FSMC_SR4_ILEN_Pos) ## !< 0x00000010
  FSMC_SR4_ILEN* = FSMC_SR4_ILEN_Msk
  FSMC_SR4_IFEN_Pos* = (5)
  FSMC_SR4_IFEN_Msk* = (0x00000001 shl FSMC_SR4_IFEN_Pos) ## !< 0x00000020
  FSMC_SR4_IFEN* = FSMC_SR4_IFEN_Msk
  FSMC_SR4_FEMPT_Pos* = (6)
  FSMC_SR4_FEMPT_Msk* = (0x00000001 shl FSMC_SR4_FEMPT_Pos) ## !< 0x00000040
  FSMC_SR4_FEMPT* = FSMC_SR4_FEMPT_Msk

## *****************  Bit definition for FSMC_PMEM2 register  *****************

const
  FSMC_PMEM2_MEMSET2_Pos* = (0)
  FSMC_PMEM2_MEMSET2_Msk* = (0x000000FF shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x000000FF
  FSMC_PMEM2_MEMSET2* = FSMC_PMEM2_MEMSET2_Msk
  FSMC_PMEM2_MEMSET2_Bit0* = (0x00000001 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000001
  FSMC_PMEM2_MEMSET2_Bit1* = (0x00000002 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000002
  FSMC_PMEM2_MEMSET2_Bit2* = (0x00000004 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000004
  FSMC_PMEM2_MEMSET2_Bit3* = (0x00000008 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000008
  FSMC_PMEM2_MEMSET2_Bit4* = (0x00000010 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000010
  FSMC_PMEM2_MEMSET2_Bit5* = (0x00000020 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000020
  FSMC_PMEM2_MEMSET2_Bit6* = (0x00000040 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000040
  FSMC_PMEM2_MEMSET2_Bit7* = (0x00000080 shl FSMC_PMEM2_MEMSET2_Pos) ## !< 0x00000080
  FSMC_PMEM2_MEMWAIT2_Pos* = (8)
  FSMC_PMEM2_MEMWAIT2_Msk* = (0x000000FF shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x0000FF00
  FSMC_PMEM2_MEMWAIT2* = FSMC_PMEM2_MEMWAIT2_Msk
  FSMC_PMEM2_MEMWAIT2_Bit0* = (0x00000001 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00000100
  FSMC_PMEM2_MEMWAIT2_Bit1* = (0x00000002 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00000200
  FSMC_PMEM2_MEMWAIT2_Bit2* = (0x00000004 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00000400
  FSMC_PMEM2_MEMWAIT2_Bit3* = (0x00000008 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00000800
  FSMC_PMEM2_MEMWAIT2_Bit4* = (0x00000010 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00001000
  FSMC_PMEM2_MEMWAIT2_Bit5* = (0x00000020 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00002000
  FSMC_PMEM2_MEMWAIT2_Bit6* = (0x00000040 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00004000
  FSMC_PMEM2_MEMWAIT2_Bit7* = (0x00000080 shl FSMC_PMEM2_MEMWAIT2_Pos) ## !< 0x00008000
  FSMC_PMEM2_MEMHOLD2_Pos* = (16)
  FSMC_PMEM2_MEMHOLD2_Msk* = (0x000000FF shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00FF0000
  FSMC_PMEM2_MEMHOLD2* = FSMC_PMEM2_MEMHOLD2_Msk
  FSMC_PMEM2_MEMHOLD2_Bit0* = (0x00000001 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00010000
  FSMC_PMEM2_MEMHOLD2_Bit1* = (0x00000002 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00020000
  FSMC_PMEM2_MEMHOLD2_Bit2* = (0x00000004 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00040000
  FSMC_PMEM2_MEMHOLD2_Bit3* = (0x00000008 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00080000
  FSMC_PMEM2_MEMHOLD2_Bit4* = (0x00000010 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00100000
  FSMC_PMEM2_MEMHOLD2_Bit5* = (0x00000020 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00200000
  FSMC_PMEM2_MEMHOLD2_Bit6* = (0x00000040 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00400000
  FSMC_PMEM2_MEMHOLD2_Bit7* = (0x00000080 shl FSMC_PMEM2_MEMHOLD2_Pos) ## !< 0x00800000
  FSMC_PMEM2_MEMHIZ2_Pos* = (24)
  FSMC_PMEM2_MEMHIZ2_Msk* = (0x000000FF shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0xFF000000
  FSMC_PMEM2_MEMHIZ2* = FSMC_PMEM2_MEMHIZ2_Msk
  FSMC_PMEM2_MEMHIZ2_Bit0* = (0x00000001 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x01000000
  FSMC_PMEM2_MEMHIZ2_Bit1* = (0x00000002 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x02000000
  FSMC_PMEM2_MEMHIZ2_Bit2* = (0x00000004 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x04000000
  FSMC_PMEM2_MEMHIZ2_Bit3* = (0x00000008 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x08000000
  FSMC_PMEM2_MEMHIZ2_Bit4* = (0x00000010 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x10000000
  FSMC_PMEM2_MEMHIZ2_Bit5* = (0x00000020 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x20000000
  FSMC_PMEM2_MEMHIZ2_Bit6* = (0x00000040 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x40000000
  FSMC_PMEM2_MEMHIZ2_Bit7* = (0x00000080 shl FSMC_PMEM2_MEMHIZ2_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_PMEM3 register  *****************

const
  FSMC_PMEM3_MEMSET3_Pos* = (0)
  FSMC_PMEM3_MEMSET3_Msk* = (0x000000FF shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x000000FF
  FSMC_PMEM3_MEMSET3* = FSMC_PMEM3_MEMSET3_Msk
  FSMC_PMEM3_MEMSET3_Bit0* = (0x00000001 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000001
  FSMC_PMEM3_MEMSET3_Bit1* = (0x00000002 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000002
  FSMC_PMEM3_MEMSET3_Bit2* = (0x00000004 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000004
  FSMC_PMEM3_MEMSET3_Bit3* = (0x00000008 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000008
  FSMC_PMEM3_MEMSET3_Bit4* = (0x00000010 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000010
  FSMC_PMEM3_MEMSET3_Bit5* = (0x00000020 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000020
  FSMC_PMEM3_MEMSET3_Bit6* = (0x00000040 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000040
  FSMC_PMEM3_MEMSET3_Bit7* = (0x00000080 shl FSMC_PMEM3_MEMSET3_Pos) ## !< 0x00000080
  FSMC_PMEM3_MEMWAIT3_Pos* = (8)
  FSMC_PMEM3_MEMWAIT3_Msk* = (0x000000FF shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x0000FF00
  FSMC_PMEM3_MEMWAIT3* = FSMC_PMEM3_MEMWAIT3_Msk
  FSMC_PMEM3_MEMWAIT3_Bit0* = (0x00000001 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00000100
  FSMC_PMEM3_MEMWAIT3_Bit1* = (0x00000002 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00000200
  FSMC_PMEM3_MEMWAIT3_Bit2* = (0x00000004 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00000400
  FSMC_PMEM3_MEMWAIT3_Bit3* = (0x00000008 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00000800
  FSMC_PMEM3_MEMWAIT3_Bit4* = (0x00000010 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00001000
  FSMC_PMEM3_MEMWAIT3_Bit5* = (0x00000020 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00002000
  FSMC_PMEM3_MEMWAIT3_Bit6* = (0x00000040 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00004000
  FSMC_PMEM3_MEMWAIT3_Bit7* = (0x00000080 shl FSMC_PMEM3_MEMWAIT3_Pos) ## !< 0x00008000
  FSMC_PMEM3_MEMHOLD3_Pos* = (16)
  FSMC_PMEM3_MEMHOLD3_Msk* = (0x000000FF shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00FF0000
  FSMC_PMEM3_MEMHOLD3* = FSMC_PMEM3_MEMHOLD3_Msk
  FSMC_PMEM3_MEMHOLD3_Bit0* = (0x00000001 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00010000
  FSMC_PMEM3_MEMHOLD3_Bit1* = (0x00000002 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00020000
  FSMC_PMEM3_MEMHOLD3_Bit2* = (0x00000004 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00040000
  FSMC_PMEM3_MEMHOLD3_Bit3* = (0x00000008 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00080000
  FSMC_PMEM3_MEMHOLD3_Bit4* = (0x00000010 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00100000
  FSMC_PMEM3_MEMHOLD3_Bit5* = (0x00000020 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00200000
  FSMC_PMEM3_MEMHOLD3_Bit6* = (0x00000040 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00400000
  FSMC_PMEM3_MEMHOLD3_Bit7* = (0x00000080 shl FSMC_PMEM3_MEMHOLD3_Pos) ## !< 0x00800000
  FSMC_PMEM3_MEMHIZ3_Pos* = (24)
  FSMC_PMEM3_MEMHIZ3_Msk* = (0x000000FF shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0xFF000000
  FSMC_PMEM3_MEMHIZ3* = FSMC_PMEM3_MEMHIZ3_Msk
  FSMC_PMEM3_MEMHIZ3_Bit0* = (0x00000001 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x01000000
  FSMC_PMEM3_MEMHIZ3_Bit1* = (0x00000002 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x02000000
  FSMC_PMEM3_MEMHIZ3_Bit2* = (0x00000004 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x04000000
  FSMC_PMEM3_MEMHIZ3_Bit3* = (0x00000008 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x08000000
  FSMC_PMEM3_MEMHIZ3_Bit4* = (0x00000010 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x10000000
  FSMC_PMEM3_MEMHIZ3_Bit5* = (0x00000020 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x20000000
  FSMC_PMEM3_MEMHIZ3_Bit6* = (0x00000040 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x40000000
  FSMC_PMEM3_MEMHIZ3_Bit7* = (0x00000080 shl FSMC_PMEM3_MEMHIZ3_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_PMEM4 register  *****************

const
  FSMC_PMEM4_MEMSET4_Pos* = (0)
  FSMC_PMEM4_MEMSET4_Msk* = (0x000000FF shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x000000FF
  FSMC_PMEM4_MEMSET4* = FSMC_PMEM4_MEMSET4_Msk
  FSMC_PMEM4_MEMSET4_Bit0* = (0x00000001 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000001
  FSMC_PMEM4_MEMSET4_Bit1* = (0x00000002 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000002
  FSMC_PMEM4_MEMSET4_Bit2* = (0x00000004 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000004
  FSMC_PMEM4_MEMSET4_Bit3* = (0x00000008 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000008
  FSMC_PMEM4_MEMSET4_Bit4* = (0x00000010 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000010
  FSMC_PMEM4_MEMSET4_Bit5* = (0x00000020 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000020
  FSMC_PMEM4_MEMSET4_Bit6* = (0x00000040 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000040
  FSMC_PMEM4_MEMSET4_Bit7* = (0x00000080 shl FSMC_PMEM4_MEMSET4_Pos) ## !< 0x00000080
  FSMC_PMEM4_MEMWAIT4_Pos* = (8)
  FSMC_PMEM4_MEMWAIT4_Msk* = (0x000000FF shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x0000FF00
  FSMC_PMEM4_MEMWAIT4* = FSMC_PMEM4_MEMWAIT4_Msk
  FSMC_PMEM4_MEMWAIT4_Bit0* = (0x00000001 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00000100
  FSMC_PMEM4_MEMWAIT4_Bit1* = (0x00000002 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00000200
  FSMC_PMEM4_MEMWAIT4_Bit2* = (0x00000004 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00000400
  FSMC_PMEM4_MEMWAIT4_Bit3* = (0x00000008 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00000800
  FSMC_PMEM4_MEMWAIT4_Bit4* = (0x00000010 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00001000
  FSMC_PMEM4_MEMWAIT4_Bit5* = (0x00000020 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00002000
  FSMC_PMEM4_MEMWAIT4_Bit6* = (0x00000040 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00004000
  FSMC_PMEM4_MEMWAIT4_Bit7* = (0x00000080 shl FSMC_PMEM4_MEMWAIT4_Pos) ## !< 0x00008000
  FSMC_PMEM4_MEMHOLD4_Pos* = (16)
  FSMC_PMEM4_MEMHOLD4_Msk* = (0x000000FF shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00FF0000
  FSMC_PMEM4_MEMHOLD4* = FSMC_PMEM4_MEMHOLD4_Msk
  FSMC_PMEM4_MEMHOLD4_Bit0* = (0x00000001 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00010000
  FSMC_PMEM4_MEMHOLD4_Bit1* = (0x00000002 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00020000
  FSMC_PMEM4_MEMHOLD4_Bit2* = (0x00000004 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00040000
  FSMC_PMEM4_MEMHOLD4_Bit3* = (0x00000008 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00080000
  FSMC_PMEM4_MEMHOLD4_Bit4* = (0x00000010 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00100000
  FSMC_PMEM4_MEMHOLD4_Bit5* = (0x00000020 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00200000
  FSMC_PMEM4_MEMHOLD4_Bit6* = (0x00000040 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00400000
  FSMC_PMEM4_MEMHOLD4_Bit7* = (0x00000080 shl FSMC_PMEM4_MEMHOLD4_Pos) ## !< 0x00800000
  FSMC_PMEM4_MEMHIZ4_Pos* = (24)
  FSMC_PMEM4_MEMHIZ4_Msk* = (0x000000FF shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0xFF000000
  FSMC_PMEM4_MEMHIZ4* = FSMC_PMEM4_MEMHIZ4_Msk
  FSMC_PMEM4_MEMHIZ4_Bit0* = (0x00000001 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x01000000
  FSMC_PMEM4_MEMHIZ4_Bit1* = (0x00000002 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x02000000
  FSMC_PMEM4_MEMHIZ4_Bit2* = (0x00000004 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x04000000
  FSMC_PMEM4_MEMHIZ4_Bit3* = (0x00000008 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x08000000
  FSMC_PMEM4_MEMHIZ4_Bit4* = (0x00000010 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x10000000
  FSMC_PMEM4_MEMHIZ4_Bit5* = (0x00000020 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x20000000
  FSMC_PMEM4_MEMHIZ4_Bit6* = (0x00000040 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x40000000
  FSMC_PMEM4_MEMHIZ4_Bit7* = (0x00000080 shl FSMC_PMEM4_MEMHIZ4_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_PATT2 register  *****************

const
  FSMC_PATT2_ATTSET2_Pos* = (0)
  FSMC_PATT2_ATTSET2_Msk* = (0x000000FF shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x000000FF
  FSMC_PATT2_ATTSET2* = FSMC_PATT2_ATTSET2_Msk
  FSMC_PATT2_ATTSET2_Bit0* = (0x00000001 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000001
  FSMC_PATT2_ATTSET2_Bit1* = (0x00000002 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000002
  FSMC_PATT2_ATTSET2_Bit2* = (0x00000004 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000004
  FSMC_PATT2_ATTSET2_Bit3* = (0x00000008 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000008
  FSMC_PATT2_ATTSET2_Bit4* = (0x00000010 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000010
  FSMC_PATT2_ATTSET2_Bit5* = (0x00000020 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000020
  FSMC_PATT2_ATTSET2_Bit6* = (0x00000040 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000040
  FSMC_PATT2_ATTSET2_Bit7* = (0x00000080 shl FSMC_PATT2_ATTSET2_Pos) ## !< 0x00000080
  FSMC_PATT2_ATTWAIT2_Pos* = (8)
  FSMC_PATT2_ATTWAIT2_Msk* = (0x000000FF shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x0000FF00
  FSMC_PATT2_ATTWAIT2* = FSMC_PATT2_ATTWAIT2_Msk
  FSMC_PATT2_ATTWAIT2_Bit0* = (0x00000001 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00000100
  FSMC_PATT2_ATTWAIT2_Bit1* = (0x00000002 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00000200
  FSMC_PATT2_ATTWAIT2_Bit2* = (0x00000004 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00000400
  FSMC_PATT2_ATTWAIT2_Bit3* = (0x00000008 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00000800
  FSMC_PATT2_ATTWAIT2_Bit4* = (0x00000010 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00001000
  FSMC_PATT2_ATTWAIT2_Bit5* = (0x00000020 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00002000
  FSMC_PATT2_ATTWAIT2_Bit6* = (0x00000040 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00004000
  FSMC_PATT2_ATTWAIT2_Bit7* = (0x00000080 shl FSMC_PATT2_ATTWAIT2_Pos) ## !< 0x00008000
  FSMC_PATT2_ATTHOLD2_Pos* = (16)
  FSMC_PATT2_ATTHOLD2_Msk* = (0x000000FF shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00FF0000
  FSMC_PATT2_ATTHOLD2* = FSMC_PATT2_ATTHOLD2_Msk
  FSMC_PATT2_ATTHOLD2_Bit0* = (0x00000001 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00010000
  FSMC_PATT2_ATTHOLD2_Bit1* = (0x00000002 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00020000
  FSMC_PATT2_ATTHOLD2_Bit2* = (0x00000004 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00040000
  FSMC_PATT2_ATTHOLD2_Bit3* = (0x00000008 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00080000
  FSMC_PATT2_ATTHOLD2_Bit4* = (0x00000010 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00100000
  FSMC_PATT2_ATTHOLD2_Bit5* = (0x00000020 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00200000
  FSMC_PATT2_ATTHOLD2_Bit6* = (0x00000040 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00400000
  FSMC_PATT2_ATTHOLD2_Bit7* = (0x00000080 shl FSMC_PATT2_ATTHOLD2_Pos) ## !< 0x00800000
  FSMC_PATT2_ATTHIZ2_Pos* = (24)
  FSMC_PATT2_ATTHIZ2_Msk* = (0x000000FF shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0xFF000000
  FSMC_PATT2_ATTHIZ2* = FSMC_PATT2_ATTHIZ2_Msk
  FSMC_PATT2_ATTHIZ2_Bit0* = (0x00000001 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x01000000
  FSMC_PATT2_ATTHIZ2_Bit1* = (0x00000002 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x02000000
  FSMC_PATT2_ATTHIZ2_Bit2* = (0x00000004 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x04000000
  FSMC_PATT2_ATTHIZ2_Bit3* = (0x00000008 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x08000000
  FSMC_PATT2_ATTHIZ2_Bit4* = (0x00000010 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x10000000
  FSMC_PATT2_ATTHIZ2_Bit5* = (0x00000020 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x20000000
  FSMC_PATT2_ATTHIZ2_Bit6* = (0x00000040 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x40000000
  FSMC_PATT2_ATTHIZ2_Bit7* = (0x00000080 shl FSMC_PATT2_ATTHIZ2_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_PATT3 register  *****************

const
  FSMC_PATT3_ATTSET3_Pos* = (0)
  FSMC_PATT3_ATTSET3_Msk* = (0x000000FF shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x000000FF
  FSMC_PATT3_ATTSET3* = FSMC_PATT3_ATTSET3_Msk
  FSMC_PATT3_ATTSET3_Bit0* = (0x00000001 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000001
  FSMC_PATT3_ATTSET3_Bit1* = (0x00000002 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000002
  FSMC_PATT3_ATTSET3_Bit2* = (0x00000004 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000004
  FSMC_PATT3_ATTSET3_Bit3* = (0x00000008 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000008
  FSMC_PATT3_ATTSET3_Bit4* = (0x00000010 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000010
  FSMC_PATT3_ATTSET3_Bit5* = (0x00000020 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000020
  FSMC_PATT3_ATTSET3_Bit6* = (0x00000040 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000040
  FSMC_PATT3_ATTSET3_Bit7* = (0x00000080 shl FSMC_PATT3_ATTSET3_Pos) ## !< 0x00000080
  FSMC_PATT3_ATTWAIT3_Pos* = (8)
  FSMC_PATT3_ATTWAIT3_Msk* = (0x000000FF shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x0000FF00
  FSMC_PATT3_ATTWAIT3* = FSMC_PATT3_ATTWAIT3_Msk
  FSMC_PATT3_ATTWAIT3_Bit0* = (0x00000001 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00000100
  FSMC_PATT3_ATTWAIT3_Bit1* = (0x00000002 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00000200
  FSMC_PATT3_ATTWAIT3_Bit2* = (0x00000004 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00000400
  FSMC_PATT3_ATTWAIT3_Bit3* = (0x00000008 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00000800
  FSMC_PATT3_ATTWAIT3_Bit4* = (0x00000010 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00001000
  FSMC_PATT3_ATTWAIT3_Bit5* = (0x00000020 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00002000
  FSMC_PATT3_ATTWAIT3_Bit6* = (0x00000040 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00004000
  FSMC_PATT3_ATTWAIT3_Bit7* = (0x00000080 shl FSMC_PATT3_ATTWAIT3_Pos) ## !< 0x00008000
  FSMC_PATT3_ATTHOLD3_Pos* = (16)
  FSMC_PATT3_ATTHOLD3_Msk* = (0x000000FF shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00FF0000
  FSMC_PATT3_ATTHOLD3* = FSMC_PATT3_ATTHOLD3_Msk
  FSMC_PATT3_ATTHOLD3_Bit0* = (0x00000001 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00010000
  FSMC_PATT3_ATTHOLD3_Bit1* = (0x00000002 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00020000
  FSMC_PATT3_ATTHOLD3_Bit2* = (0x00000004 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00040000
  FSMC_PATT3_ATTHOLD3_Bit3* = (0x00000008 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00080000
  FSMC_PATT3_ATTHOLD3_Bit4* = (0x00000010 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00100000
  FSMC_PATT3_ATTHOLD3_Bit5* = (0x00000020 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00200000
  FSMC_PATT3_ATTHOLD3_Bit6* = (0x00000040 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00400000
  FSMC_PATT3_ATTHOLD3_Bit7* = (0x00000080 shl FSMC_PATT3_ATTHOLD3_Pos) ## !< 0x00800000
  FSMC_PATT3_ATTHIZ3_Pos* = (24)
  FSMC_PATT3_ATTHIZ3_Msk* = (0x000000FF shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0xFF000000
  FSMC_PATT3_ATTHIZ3* = FSMC_PATT3_ATTHIZ3_Msk
  FSMC_PATT3_ATTHIZ3_Bit0* = (0x00000001 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x01000000
  FSMC_PATT3_ATTHIZ3_Bit1* = (0x00000002 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x02000000
  FSMC_PATT3_ATTHIZ3_Bit2* = (0x00000004 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x04000000
  FSMC_PATT3_ATTHIZ3_Bit3* = (0x00000008 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x08000000
  FSMC_PATT3_ATTHIZ3_Bit4* = (0x00000010 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x10000000
  FSMC_PATT3_ATTHIZ3_Bit5* = (0x00000020 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x20000000
  FSMC_PATT3_ATTHIZ3_Bit6* = (0x00000040 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x40000000
  FSMC_PATT3_ATTHIZ3_Bit7* = (0x00000080 shl FSMC_PATT3_ATTHIZ3_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_PATT4 register  *****************

const
  FSMC_PATT4_ATTSET4_Pos* = (0)
  FSMC_PATT4_ATTSET4_Msk* = (0x000000FF shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x000000FF
  FSMC_PATT4_ATTSET4* = FSMC_PATT4_ATTSET4_Msk
  FSMC_PATT4_ATTSET4_Bit0* = (0x00000001 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000001
  FSMC_PATT4_ATTSET4_Bit1* = (0x00000002 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000002
  FSMC_PATT4_ATTSET4_Bit2* = (0x00000004 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000004
  FSMC_PATT4_ATTSET4_Bit3* = (0x00000008 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000008
  FSMC_PATT4_ATTSET4_Bit4* = (0x00000010 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000010
  FSMC_PATT4_ATTSET4_Bit5* = (0x00000020 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000020
  FSMC_PATT4_ATTSET4_Bit6* = (0x00000040 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000040
  FSMC_PATT4_ATTSET4_Bit7* = (0x00000080 shl FSMC_PATT4_ATTSET4_Pos) ## !< 0x00000080
  FSMC_PATT4_ATTWAIT4_Pos* = (8)
  FSMC_PATT4_ATTWAIT4_Msk* = (0x000000FF shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x0000FF00
  FSMC_PATT4_ATTWAIT4* = FSMC_PATT4_ATTWAIT4_Msk
  FSMC_PATT4_ATTWAIT4_Bit0* = (0x00000001 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00000100
  FSMC_PATT4_ATTWAIT4_Bit1* = (0x00000002 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00000200
  FSMC_PATT4_ATTWAIT4_Bit2* = (0x00000004 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00000400
  FSMC_PATT4_ATTWAIT4_Bit3* = (0x00000008 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00000800
  FSMC_PATT4_ATTWAIT4_Bit4* = (0x00000010 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00001000
  FSMC_PATT4_ATTWAIT4_Bit5* = (0x00000020 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00002000
  FSMC_PATT4_ATTWAIT4_Bit6* = (0x00000040 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00004000
  FSMC_PATT4_ATTWAIT4_Bit7* = (0x00000080 shl FSMC_PATT4_ATTWAIT4_Pos) ## !< 0x00008000
  FSMC_PATT4_ATTHOLD4_Pos* = (16)
  FSMC_PATT4_ATTHOLD4_Msk* = (0x000000FF shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00FF0000
  FSMC_PATT4_ATTHOLD4* = FSMC_PATT4_ATTHOLD4_Msk
  FSMC_PATT4_ATTHOLD4_Bit0* = (0x00000001 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00010000
  FSMC_PATT4_ATTHOLD4_Bit1* = (0x00000002 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00020000
  FSMC_PATT4_ATTHOLD4_Bit2* = (0x00000004 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00040000
  FSMC_PATT4_ATTHOLD4_Bit3* = (0x00000008 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00080000
  FSMC_PATT4_ATTHOLD4_Bit4* = (0x00000010 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00100000
  FSMC_PATT4_ATTHOLD4_Bit5* = (0x00000020 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00200000
  FSMC_PATT4_ATTHOLD4_Bit6* = (0x00000040 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00400000
  FSMC_PATT4_ATTHOLD4_Bit7* = (0x00000080 shl FSMC_PATT4_ATTHOLD4_Pos) ## !< 0x00800000
  FSMC_PATT4_ATTHIZ4_Pos* = (24)
  FSMC_PATT4_ATTHIZ4_Msk* = (0x000000FF shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0xFF000000
  FSMC_PATT4_ATTHIZ4* = FSMC_PATT4_ATTHIZ4_Msk
  FSMC_PATT4_ATTHIZ4_Bit0* = (0x00000001 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x01000000
  FSMC_PATT4_ATTHIZ4_Bit1* = (0x00000002 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x02000000
  FSMC_PATT4_ATTHIZ4_Bit2* = (0x00000004 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x04000000
  FSMC_PATT4_ATTHIZ4_Bit3* = (0x00000008 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x08000000
  FSMC_PATT4_ATTHIZ4_Bit4* = (0x00000010 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x10000000
  FSMC_PATT4_ATTHIZ4_Bit5* = (0x00000020 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x20000000
  FSMC_PATT4_ATTHIZ4_Bit6* = (0x00000040 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x40000000
  FSMC_PATT4_ATTHIZ4_Bit7* = (0x00000080 shl FSMC_PATT4_ATTHIZ4_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_PIO4 register  ******************

const
  FSMC_PIO4_IOSET4_Pos* = (0)
  FSMC_PIO4_IOSET4_Msk* = (0x000000FF shl FSMC_PIO4_IOSET4_Pos) ## !< 0x000000FF
  FSMC_PIO4_IOSET4* = FSMC_PIO4_IOSET4_Msk
  FSMC_PIO4_IOSET4_Bit0* = (0x00000001 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000001
  FSMC_PIO4_IOSET4_Bit1* = (0x00000002 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000002
  FSMC_PIO4_IOSET4_Bit2* = (0x00000004 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000004
  FSMC_PIO4_IOSET4_Bit3* = (0x00000008 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000008
  FSMC_PIO4_IOSET4_Bit4* = (0x00000010 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000010
  FSMC_PIO4_IOSET4_Bit5* = (0x00000020 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000020
  FSMC_PIO4_IOSET4_Bit6* = (0x00000040 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000040
  FSMC_PIO4_IOSET4_Bit7* = (0x00000080 shl FSMC_PIO4_IOSET4_Pos) ## !< 0x00000080
  FSMC_PIO4_IOWAIT4_Pos* = (8)
  FSMC_PIO4_IOWAIT4_Msk* = (0x000000FF shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x0000FF00
  FSMC_PIO4_IOWAIT4* = FSMC_PIO4_IOWAIT4_Msk
  FSMC_PIO4_IOWAIT4_Bit0* = (0x00000001 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00000100
  FSMC_PIO4_IOWAIT4_Bit1* = (0x00000002 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00000200
  FSMC_PIO4_IOWAIT4_Bit2* = (0x00000004 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00000400
  FSMC_PIO4_IOWAIT4_Bit3* = (0x00000008 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00000800
  FSMC_PIO4_IOWAIT4_Bit4* = (0x00000010 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00001000
  FSMC_PIO4_IOWAIT4_Bit5* = (0x00000020 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00002000
  FSMC_PIO4_IOWAIT4_Bit6* = (0x00000040 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00004000
  FSMC_PIO4_IOWAIT4_Bit7* = (0x00000080 shl FSMC_PIO4_IOWAIT4_Pos) ## !< 0x00008000
  FSMC_PIO4_IOHOLD4_Pos* = (16)
  FSMC_PIO4_IOHOLD4_Msk* = (0x000000FF shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00FF0000
  FSMC_PIO4_IOHOLD4* = FSMC_PIO4_IOHOLD4_Msk
  FSMC_PIO4_IOHOLD4_Bit0* = (0x00000001 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00010000
  FSMC_PIO4_IOHOLD4_Bit1* = (0x00000002 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00020000
  FSMC_PIO4_IOHOLD4_Bit2* = (0x00000004 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00040000
  FSMC_PIO4_IOHOLD4_Bit3* = (0x00000008 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00080000
  FSMC_PIO4_IOHOLD4_Bit4* = (0x00000010 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00100000
  FSMC_PIO4_IOHOLD4_Bit5* = (0x00000020 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00200000
  FSMC_PIO4_IOHOLD4_Bit6* = (0x00000040 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00400000
  FSMC_PIO4_IOHOLD4_Bit7* = (0x00000080 shl FSMC_PIO4_IOHOLD4_Pos) ## !< 0x00800000
  FSMC_PIO4_IOHIZ4_Pos* = (24)
  FSMC_PIO4_IOHIZ4_Msk* = (0x000000FF shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0xFF000000
  FSMC_PIO4_IOHIZ4* = FSMC_PIO4_IOHIZ4_Msk
  FSMC_PIO4_IOHIZ4_Bit0* = (0x00000001 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x01000000
  FSMC_PIO4_IOHIZ4_Bit1* = (0x00000002 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x02000000
  FSMC_PIO4_IOHIZ4_Bit2* = (0x00000004 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x04000000
  FSMC_PIO4_IOHIZ4_Bit3* = (0x00000008 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x08000000
  FSMC_PIO4_IOHIZ4_Bit4* = (0x00000010 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x10000000
  FSMC_PIO4_IOHIZ4_Bit5* = (0x00000020 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x20000000
  FSMC_PIO4_IOHIZ4_Bit6* = (0x00000040 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x40000000
  FSMC_PIO4_IOHIZ4_Bit7* = (0x00000080 shl FSMC_PIO4_IOHIZ4_Pos) ## !< 0x80000000

## *****************  Bit definition for FSMC_ECCR2 register  *****************

const
  FSMC_ECCR2_ECC2_Pos* = (0)
  FSMC_ECCR2_ECC2_Msk* = (0xFFFFFFFF shl FSMC_ECCR2_ECC2_Pos) ## !< 0xFFFFFFFF
  FSMC_ECCR2_ECC2* = FSMC_ECCR2_ECC2_Msk

## *****************  Bit definition for FSMC_ECCR3 register  *****************

const
  FSMC_ECCR3_ECC3_Pos* = (0)
  FSMC_ECCR3_ECC3_Msk* = (0xFFFFFFFF shl FSMC_ECCR3_ECC3_Pos) ## !< 0xFFFFFFFF
  FSMC_ECCR3_ECC3* = FSMC_ECCR3_ECC3_Msk

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
