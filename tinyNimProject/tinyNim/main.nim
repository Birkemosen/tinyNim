###############################################################################
# Main
###############################################################################
import tinyNim

proc SystemInit() {.exportc.} =
  RCC.CR.SetBits(0x00000001) # HSI ON
  RCC.CFGR.SetBits(0x00000000) # Reset
  RCC.CR.Set(0xFEF6FFFF) #Reset HSEON, CSSON and PLLON bits
  RCC.PLLCFGR.SetBits(0x24003010)
  RCC.CR.SetBits(0xFFFBFFFF)
  RCC.CIR.SetBits(0x00000000)
  
proc main() = 
  PinInit()

  PA5.Configure(PinOutput)
  PA5.On()
  while true:

    var x = 0
    while x in 0..2000000:
      x = x + 1
      discard
    PA5.Toggle()

main()