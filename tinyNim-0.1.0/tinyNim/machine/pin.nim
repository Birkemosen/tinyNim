###############################################################################
# TinyNim Machine Pin module
###############################################################################
#import tinyNimDevices
import tinyNimBoards
import ../registerMacros

type 
  PinMode* =
    uint8

when not declared(Pin):
  type
    Pin* = uint32

const
  # Mode flags
  PinOutput*        : PinMode = 0
  PinInputFloating* : PinMode = 1
  PinInput*         : PinMode = PinInputFloating
  PinInputPullDown* : PinMode = 2
  PinInputPullUp*   : PinMode = 3

  # For UART
  PinModeUartTx*    : PinMode = 4
  PinModeUartRx*    : PinMode = 5

  # GPIOx_MODER
  GPIO_MODE_INPUT    {.used.}         = 0
  GPIO_MODE_OUTPUT   {.used.}         = 1
  GPIO_MODE_ALTERNATE  {.used.}       = 2
  GPIO_MODE_ANALOG   {.used.}         = 3

  # GPIOx_OTYPER
  GPIO_OUTPUT_MODE_PUSH_PULL {.used.}  = 0
  GPIO_OUTPUT_MODE_OPEN_DRAIN {.used.} = 1

  # GPIOx_OSPEEDR
  GPIO_SPEED_LOW {.used.}             = 0
  GPIO_SPEED_MEDIUM {.used.}          = 1
  GPIO_SPEED_HIGH {.used.}            = 2
  GPIO_SPEED_VERY_HIGH {.used.}       = 3

  # GPIOx_PUPDR
  GPIO_FLOATING  {.used.}             = 0
  GPIO_PULL_UP  {.used.}              = 1
  GPIO_PULL_DOWN  {.used.}            = 2

proc PinInit*() =
  when declared(GPIOA):
    RCC.AHB1ENR.SetBits(RCC_AHB1ENR_GPIOAEN)
  when declared(GPIOB):
    RCC.AHB1ENR.SetBits(RCC_AHB1ENR_GPIOBEN)
  when declared(GPIOC):
    RCC.AHB1ENR.SetBits(RCC_AHB1ENR_GPIOCEN)
  when declared(GPIOD):
    RCC.AHB1ENR.SetBits(RCC_AHB1ENR_GPIODEN)
  when declared(GPIOE):
    RCC.AHB1ENR.SetBits(RCC_AHB1ENR_GPIOEEN)
  when declared(GPIOF):
    RCC.AHB1ENR.SetBits(RCC_AHB1ENR_GPIOFEN)
  when declared(GPIOG):
    RCC.AHB1ENR.SetBits(RCC_AHB1ENR_GPIOGEN)
  when declared(GPIOH):
    RCC.AHB1ENR.SetBits(RCC_AHB1ENR_GPIOHEN)

template getPort*(p: Pin): ptr GPIO_TypeDef =
  cast[ptr GPIO_TypeDef](uint32(p) and uint32(0xFFFFFFF0))

template getPinIndex*(p: Pin): int8 =
  cast[int8]((uint32(p) and uint32(0x0000000F)))

proc SetAltFunc*(p: Pin, af: uint32) =
  var port = p.getPort()
  var pin = p.getPinIndex()
  var pos = pin * 4
  if pin >= (int8)8:
    port.AFR[1].SetBits((af and 0xF) shl pos)
  else:
    port.AFR[0].SetBits((af and 0xf) shl pos)

proc Configure*(p: Pin, config: PinMode) =
  # Configure the GPIO pin
  var port = p.getPort()
  var pin = p.getPinIndex()
  var pos = pin * 2

  if config == PinInputFloating:
    port.MODER.SetBits((uint32)GPIO_MODE_INPUT shl pos)
    port.PUPDR.SetBits((uint32)GPIO_FLOATING shl pos)
  elif config == PinInputPullDown:
    port.MODER.SetBits((uint32)GPIO_MODE_INPUT shl pos)
    port.PUPDR.SetBits((uint32)GPIO_PULL_DOWN shl pos)
  elif config == PinInputPullUp:
    port.MODER.SetBits((uint32)GPIO_MODE_INPUT shl pos)
    port.PUPDR.SetBits((uint32)GPIO_PULL_UP shl pos)
  elif config == PinOutput:
    port.MODER.SetBits((uint32)GPIO_MODE_OUTPUT shl pos)
    port.OSPEEDR.SetBits((uint32)GPIO_SPEED_HIGH shl pos)
  elif config == PinModeUartTx:
    port.MODER.SetBits((uint32)GPIO_MODE_ALTERNATE shl pos)
    port.OSPEEDR.SetBits((uint32)GPIO_SPEED_HIGH shl pos)
    port.PUPDR.SetBits((uint32)GPIO_PULL_UP shl pos)
    p.setAltFunc(0x7)
  elif config == PinModeUartRx:
    port.MODER.SetBits((uint32)GPIO_MODE_ALTERNATE shl pos)
    port.PUPDR.SetBits((uint32)GPIO_FLOATING shl pos)
    p.setAltFunc(0x7)

proc Set*(p: Pin, on: bool) =
  var port = p.getPort()
  var pin = p.getPinIndex()
  if on:
    port.BSRR.Set(1 shl uint8(pin))
  else:
    port.BSRR.Set(1 shl uint8(pin + 16))


###############################################################################
# Public Procedures
###############################################################################

proc On*(p: Pin) =
  p.Set(true)

proc Off*(p: Pin) =
  p.Set(false)

proc Toggle*(p: Pin) =
  var port = p.getPort()
  var pin = p.getPinIndex()
  port.ODR.FlipBits(1 shl uint8(pin))