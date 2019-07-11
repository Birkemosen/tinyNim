###############################################################################

###############################################################################
import volatile

template store*[T: SomeInteger, U: SomeInteger](reg: T, val: U) =
  volatileStore(reg.addr, cast[T](val))

template load*[T: SomeInteger](reg: T): T =
  volatileLoad(reg.addr)

template Bit*[T: SomeInteger](n: varargs[T]): T =
  var ret: T = 0;
  for i in n:
    ret = ret or (1 shl i)
  ret

# Get returns the value in the register.
# It is the volatile equivalent of: *reg
template Get*[T: SomeInteger](reg: T): uint32 =
  uint32(reg.load)

# Set updates the value in the register.
# It is the volatile equivalent of: *reg = value
template Set*[T, U: SomeInteger](reg: T, val: U) =
  reg.store cast[T](val)

# SetBits reads the register, sets the given bits, and writes it back. 
# It is the volatile equivalent of: *reg |= value
template SetBits*[T, U: SomeInteger](reg: T, val: U) =
  reg.store reg.load or cast[T](val)

# ClearBits reads the register, clears the given bits, and writes it back. 
# It is the volatile equivalent of: *reg &^= value
template ClearBits*[T, U: SomeInteger](reg: T, val: U) =
  reg.store reg.load and not cast[T](val)

# FlipBits reads the register, inverts the given bits and writes it back
# It is the volatile equivalent of: *reg ^= value
template FlipBits*[T, U: SomeInteger](reg: T, val: U) =
  reg.store reg.load xor cast[T](val)

# HasBits reads the register and then checks to see if the passed bits are set.
# It is the volatile equivalent of: (*reg & value) > 0
template HasBits*[T, U: SomeInteger](reg: T, val: U): bool =
  if (reg.Get() & value) > 0:
    true
  else:
    false

