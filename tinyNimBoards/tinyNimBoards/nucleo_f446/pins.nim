###############################################################################

###############################################################################
type
  Pin* =
    int8
    
const
  PORTA* :Pin = (Pin)0
  PA0*   :Pin = PORTA + 0
  PA1*   :Pin = PORTA + 1
  PA2*   :Pin = PORTA + 2
  PA3*   :Pin = PORTA + 3
  PA4*   :Pin = PORTA + 4
  PA5*   :Pin = PORTA + 5
  PA6*   :Pin = PORTA + 6
  PA7*   :Pin = PORTA + 7
  PA8*   :Pin = PORTA + 8
  PA9*   :Pin = PORTA + 9
  PA10*  :Pin = PORTA + 10
  PA11*  :Pin = PORTA + 11
  PA12*  :Pin = PORTA + 12
  PA13*  :Pin = PORTA + 13
  PA14*  :Pin = PORTA + 14
  PA15*  :Pin = PORTA + 15

const
  PORTB*  :Pin = (Pin)16
  PB0*    :Pin = PORTB + 0
  PB1*    :Pin = PORTB + 1
  PB2*    :Pin = PORTB + 2
  PB3*    :Pin = PORTB + 3
  PB4*    :Pin = PORTB + 4
  PB5*    :Pin = PORTB + 5
  PB6*    :Pin = PORTB + 6
  PB7*    :Pin = PORTB + 7
  PB8*    :Pin = PORTB + 8
  PB9*    :Pin = PORTB + 9
  PB10*   :Pin = PORTB + 10
  PB11*   :Pin = PORTB + 11
  PB12*   :Pin = PORTB + 12
  PB13*   :Pin = PORTB + 13
  PB14*   :Pin = PORTB + 14
  PB15*   :Pin = PORTB + 15

const
  PORTC*  :Pin= (Pin)32
  PC0*  = PORTC + 0
  PC1*  = PORTC + 1
  PC2*  = PORTC + 2
  PC3*  = PORTC + 3
  PC4*  = PORTC + 4
  PC5*  = PORTC + 5
  PC6*  = PORTC + 6
  PC7*  = PORTC + 7
  PC8*  = PORTC + 8
  PC9*  = PORTC + 9
  PC10* = PORTC + 10
  PC11* = PORTC + 11
  PC12* = PORTC + 12
  PC13* = PORTC + 13
  PC14* = PORTC + 14
  PC15* = PORTC + 15

const
  PORTD*  :Pin= (Pin)48
  PD0*  = PORTD + 0
  PD1*  = PORTD + 1
  PD2*  = PORTD + 2
  PD3*  = PORTD + 3
  PD4*  = PORTD + 4
  PD5*  = PORTD + 5
  PD6*  = PORTD + 6
  PD7*  = PORTD + 7
  PD8*  = PORTD + 8
  PD9*  = PORTD + 9
  PD10* = PORTD + 10
  PD11* = PORTD + 11
  PD12* = PORTD + 12
  PD13* = PORTD + 13
  PD14* = PORTD + 14
  PD15* = PORTD + 15

const
  PORTE*  :Pin= (Pin)64
  PE0*  = PORTE + 0
  PE1*  = PORTE + 1
  PE2*  = PORTE + 2
  PE3*  = PORTE + 3
  PE4*  = PORTE + 4
  PE5*  = PORTE + 5
  PE6*  = PORTE + 6
  PE7*  = PORTE + 7
  PE8*  = PORTE + 8
  PE9*  = PORTE + 9
  PE10* = PORTE + 10
  PE11* = PORTE + 11
  PE12* = PORTE + 12
  PE13* = PORTE + 13
  PE14* = PORTE + 14
  PE15* = PORTE + 15

const
  PORTF*  :Pin= (Pin)80
  PF0*  = PORTF + 0
  PF1*  = PORTF + 1
  PF2*  = PORTF + 2
  PF3*  = PORTF + 3
  PF4*  = PORTF + 4
  PF5*  = PORTF + 5
  PF6*  = PORTF + 6
  PF7*  = PORTF + 7
  PF8*  = PORTF + 8
  PF9*  = PORTF + 9
  PF10* = PORTF + 10
  PF11* = PORTF + 11
  PF12* = PORTF + 12
  PF13* = PORTF + 13
  PF14* = PORTF + 14
  PF15* = PORTF + 15

const
  PORTG*  :Pin= (Pin)96
  PG0*  = PORTG + 0
  PG1*  = PORTG + 1
  PG2*  = PORTG + 2
  PG3*  = PORTG + 3
  PG4*  = PORTG + 4
  PG5*  = PORTG + 5
  PG6*  = PORTG + 6
  PG7*  = PORTG + 7
  PG8*  = PORTG + 8
  PG9*  = PORTG + 9
  PG10* = PORTG + 10
  PG11* = PORTG + 11
  PG12* = PORTG + 12
  PG13* = PORTG + 13
  PG14* = PORTG + 14
  PG15* = PORTG + 15
  
const
  PORTH*  :Pin= (Pin)112
  PH0*  = PORTH + 0
  PH1*  = PORTH + 1
  PH2*  = PORTH + 2
  PH3*  = PORTH + 3
  PH4*  = PORTH + 4
  PH5*  = PORTH + 5
  PH6*  = PORTH + 6
  PH7*  = PORTH + 7
  PH8*  = PORTH + 8
  PH9*  = PORTH + 9
  PH10* = PORTH + 10
  PH11* = PORTH + 11
  PH12* = PORTH + 12
  PH13* = PORTH + 13
  PH14* = PORTH + 14
  PH15* = PORTH + 15