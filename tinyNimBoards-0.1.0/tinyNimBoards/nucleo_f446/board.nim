###############################################################################

###############################################################################
import stm32f446xx

const CPU_FREQUENCY = 180000000 # 180 MHz


type
  Pin* =
    uint32

template definePin(port: uint32, pinIndex: uint32): uint32 =
  cast[uint32](uint32(port) or uint32(pinIndex))

const
  PA0*   :Pin =   definePin(GPIOA_BASE, 0x00)
  PA1*   :Pin =   definePin(GPIOA_BASE, 0x01)
  PA2*   :Pin =   definePin(GPIOA_BASE, 0x02)
  PA3*   :Pin =   definePin(GPIOA_BASE, 0x03)
  PA4*   :Pin =   definePin(GPIOA_BASE, 0x04)
  PA5*   :Pin =   definePin(GPIOA_BASE, 0x05)
  PA6*   :Pin =   definePin(GPIOA_BASE, 0x06)
  PA7*   :Pin =   definePin(GPIOA_BASE, 0x07)
  PA8*   :Pin =   definePin(GPIOA_BASE, 0x08)
  PA9*   :Pin =   definePin(GPIOA_BASE, 0x09)
  PA10*  :Pin =   definePin(GPIOA_BASE, 0x0A)
  PA11*  :Pin =   definePin(GPIOA_BASE, 0x0B)
  PA12*  :Pin =   definePin(GPIOA_BASE, 0x0C)
  PA13*  :Pin =   definePin(GPIOA_BASE, 0x0D)
  PA14*  :Pin =   definePin(GPIOA_BASE, 0x0E)
  PA15*  :Pin =   definePin(GPIOA_BASE, 0x0F)

const
  PB0*    :Pin =   definePin(GPIOB_BASE, 0x00)
  PB1*    :Pin =   definePin(GPIOB_BASE, 0x01)
  PB2*    :Pin =   definePin(GPIOB_BASE, 0x02)
  PB3*    :Pin =   definePin(GPIOB_BASE, 0x03)
  PB4*    :Pin =   definePin(GPIOB_BASE, 0x04)
  PB5*    :Pin =   definePin(GPIOB_BASE, 0x05)
  PB6*    :Pin =   definePin(GPIOB_BASE, 0x06)
  PB7*    :Pin =   definePin(GPIOB_BASE, 0x07)
  PB8*    :Pin =   definePin(GPIOB_BASE, 0x08)
  PB9*    :Pin =   definePin(GPIOB_BASE, 0x09)
  PB10*   :Pin =   definePin(GPIOB_BASE, 0x0A)
  PB11*   :Pin =   definePin(GPIOB_BASE, 0x0B)
  PB12*   :Pin =   definePin(GPIOB_BASE, 0x0C)
  PB13*   :Pin =   definePin(GPIOB_BASE, 0x0D)
  PB14*   :Pin =   definePin(GPIOB_BASE, 0x0E)
  PB15*   :Pin =   definePin(GPIOB_BASE, 0x0F)

const
  PC0*  =   definePin(GPIOC_BASE, 0x00)
  PC1*  =   definePin(GPIOC_BASE, 0x01)
  PC2*  =   definePin(GPIOC_BASE, 0x02)
  PC3*  =   definePin(GPIOC_BASE, 0x03)
  PC4*  =   definePin(GPIOC_BASE, 0x04)
  PC5*  =   definePin(GPIOC_BASE, 0x05)
  PC6*  =   definePin(GPIOC_BASE, 0x06)
  PC7*  =   definePin(GPIOC_BASE, 0x07)
  PC8*  =   definePin(GPIOC_BASE, 0x08)
  PC9*  =   definePin(GPIOC_BASE, 0x09)
  PC10* =   definePin(GPIOC_BASE, 0x0A)
  PC11* =   definePin(GPIOC_BASE, 0x0B)
  PC12* =   definePin(GPIOC_BASE, 0x0C)
  PC13* =   definePin(GPIOC_BASE, 0x0D)
  PC14* =   definePin(GPIOC_BASE, 0x0E)
  PC15* =   definePin(GPIOC_BASE, 0x0F)

const
#  PD0*  =   definePin(GPIOD_BASE, 0x00)
#  PD1*  =   definePin(GPIOD_BASE, 0x01)
  PD2*  =   definePin(GPIOD_BASE, 0x02)
#  PD3*  =   definePin(GPIOD_BASE, 0x03)
#  PD4*  =   definePin(GPIOD_BASE, 0x04)
#  PD5*  =   definePin(GPIOD_BASE, 0x05)
#  PD6*  =   definePin(GPIOD_BASE, 0x06)
#  PD7*  =   definePin(GPIOD_BASE, 0x07)
#  PD8*  =   definePin(GPIOD_BASE, 0x08)
#  PD9*  =   definePin(GPIOD_BASE, 0x09)
#  PD10* =   definePin(GPIOD_BASE, 0x0A)
#  PD11* =   definePin(GPIOD_BASE, 0x0B)
#  PD12* =   definePin(GPIOD_BASE, 0x0C)
#  PD13* =   definePin(GPIOD_BASE, 0x0D)
#  PD14* =   definePin(GPIOD_BASE, 0x0E)
#  PD15* =   definePin(GPIOD_BASE, 0x0F)

#const
#  PE0*  =   definePin(GPIOE_BASE, 0x00)
#  PE1*  =   definePin(GPIOE_BASE, 0x01)
#  PE2*  =   definePin(GPIOE_BASE, 0x02)
#  PE3*  =   definePin(GPIOE_BASE, 0x03)
#  PE4*  =   definePin(GPIOE_BASE, 0x04)
#  PE5*  =   definePin(GPIOE_BASE, 0x05)
#  PE6*  =   definePin(GPIOE_BASE, 0x06)
#  PE7*  =   definePin(GPIOE_BASE, 0x07)
#  PE8*  =   definePin(GPIOE_BASE, 0x08)
#  PE9*  =   definePin(GPIOE_BASE, 0x09)
#  PE10* =   definePin(GPIOE_BASE, 0x0A)
#  PE11* =   definePin(GPIOE_BASE, 0x0B)
#  PE12* =   definePin(GPIOE_BASE, 0x0C)
#  PE13* =   definePin(GPIOE_BASE, 0x0D)
#  PE14* =   definePin(GPIOE_BASE, 0x0E)
#  PE15* =   definePin(GPIOE_BASE, 0x0F)
#
#const
#  PF0*  =   definePin(GPIOF_BASE, 0x00)
#  PF1*  =   definePin(GPIOF_BASE, 0x01)
#  PF2*  =   definePin(GPIOF_BASE, 0x02)
#  PF3*  =   definePin(GPIOF_BASE, 0x03)
#  PF4*  =   definePin(GPIOF_BASE, 0x04)
#  PF5*  =   definePin(GPIOF_BASE, 0x05)
#  PF6*  =   definePin(GPIOF_BASE, 0x06)
#  PF7*  =   definePin(GPIOF_BASE, 0x07)
#  PF8*  =   definePin(GPIOF_BASE, 0x08)
#  PF9*  =   definePin(GPIOF_BASE, 0x09)
#  PF10* =   definePin(GPIOF_BASE, 0x0A)
#  PF11* =   definePin(GPIOF_BASE, 0x0B)
#  PF12* =   definePin(GPIOF_BASE, 0x0C)
#  PF13* =   definePin(GPIOF_BASE, 0x0D)
#  PF14* =   definePin(GPIOF_BASE, 0x0E)
#  PF15* =   definePin(GPIOF_BASE, 0x0F)
#
#const
#  PG0*  =   definePin(GPIOG_BASE, 0x00)
#  PG1*  =   definePin(GPIOG_BASE, 0x01)
#  PG2*  =   definePin(GPIOG_BASE, 0x02)
#  PG3*  =   definePin(GPIOG_BASE, 0x03)
#  PG4*  =   definePin(GPIOG_BASE, 0x04)
#  PG5*  =   definePin(GPIOG_BASE, 0x05)
#  PG6*  =   definePin(GPIOG_BASE, 0x06)
#  PG7*  =   definePin(GPIOG_BASE, 0x07)
#  PG8*  =   definePin(GPIOG_BASE, 0x08)
#  PG9*  =   definePin(GPIOG_BASE, 0x09)
#  PG10* =   definePin(GPIOG_BASE, 0x0A)
#  PG11* =   definePin(GPIOG_BASE, 0x0B)
#  PG12* =   definePin(GPIOG_BASE, 0x0C)
#  PG13* =   definePin(GPIOG_BASE, 0x0D)
#  PG14* =   definePin(GPIOG_BASE, 0x0E)
#  PG15* =   definePin(GPIOG_BASE, 0x0F)
#
const
  PH0*  =   definePin(GPIOH_BASE, 0x00)
  PH1*  =   definePin(GPIOH_BASE, 0x01)
#  PH2*  =   definePin(GPIOH_BASE, 0x02)
#  PH3*  =   definePin(GPIOH_BASE, 0x03)
#  PH4*  =   definePin(GPIOH_BASE, 0x04)
#  PH5*  =   definePin(GPIOH_BASE, 0x05)
#  PH6*  =   definePin(GPIOH_BASE, 0x06)
#  PH7*  =   definePin(GPIOH_BASE, 0x07)
#  PH8*  =   definePin(GPIOH_BASE, 0x08)
#  PH9*  =   definePin(GPIOH_BASE, 0x09)
#  PH10* =   definePin(GPIOH_BASE, 0x0A)
#  PH11* =   definePin(GPIOH_BASE, 0x0B)
#  PH12* =   definePin(GPIOH_BASE, 0x0C)
#  PH13* =   definePin(GPIOH_BASE, 0x0D)
#  PH14* =   definePin(GPIOH_BASE, 0x0E)
#  PH15* =   definePin(GPIOH_BASE, 0x0F)

#const
#  PI0*  =   definePin(GPIOI_BASE, 0x00)
#  PI1*  =   definePin(GPIOI_BASE, 0x01)
#  PI2*  =   definePin(GPIOI_BASE, 0x02)
#  PI3*  =   definePin(GPIOI_BASE, 0x03)
#  PI4*  =   definePin(GPIOI_BASE, 0x04)
#  PI5*  =   definePin(GPIOI_BASE, 0x05)
#  PI6*  =   definePin(GPIOI_BASE, 0x06)
#  PI7*  =   definePin(GPIOI_BASE, 0x07)
#  PI8*  =   definePin(GPIOI_BASE, 0x08)
#  PI9*  =   definePin(GPIOI_BASE, 0x09)
#  PI10* =   definePin(GPIOI_BASE, 0x0A)
#  PI11* =   definePin(GPIOI_BASE, 0x0B)
#  PI12* =   definePin(GPIOI_BASE, 0x0C)
#  PI13* =   definePin(GPIOI_BASE, 0x0D)
#  PI14* =   definePin(GPIOI_BASE, 0x0E)
#  PI15* =   definePin(GPIOI_BASE, 0x0F)

const
  LED  = PA5
  TX   = PA2
  RX   = PA3