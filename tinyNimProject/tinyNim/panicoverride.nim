###############################################################################
#
# panicoverride.nim
#
# Provides required functions for Nim in standalone mode.
#
###############################################################################
proc printf(formatstr: cstring) {.importc, varargs, header: "<stdio.h>", cdecl.}
proc exit(code: int) {.importc, header: "<stdlib.h>", cdecl.}

{.push stack_trace: off, profiler:off.}

proc rawoutput(s: string) =
  printf("%s\n", s)

proc panic(s: string) =
  rawoutput(s)
  exit(1)
  
{.pop.}