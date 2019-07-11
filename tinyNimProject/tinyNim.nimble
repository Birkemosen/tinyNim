# Package

version     = "0.1.0"
author      = "Julian Birkemose"
description = "tinyNim - Nim on embedded devices"
license     = "MIT"

#srcDir      = "src"
#skipDirs    = @[]

# Deps


# Define board to use
var board = "nucleo_f446"

###############################################################################
# Build
###############################################################################
import os
mode = ScriptMode.Verbose

var cfg = "/.nimble/pkgs/tinyNimBoards-0.1.0/tinyNimBoards/" & board & "/" & board & ".cfg"
# Variables for customization
#var board = "nucleo_f446"
var target = "tinyNim"
var builddir = "build/"
var srcdir = "tinyNim/"
var nimMain = srcdir & "main.nim"  # Main nim file
var compiler = "gcc"
var includes = "-Inimcache"

# Compiler Flags
var nimflags = " cc -c --cpu=arm --d:release --d:useMalloc --gc:stack --os:standalone --deadCodeElim:on --nimcache:./nimcache"
nimflags &= " --d:" & board

var ToolchainSettings = " -mthumb -fmessage-length=0"
var cFlags = ToolchainSettings
var ldFlags = ToolchainSettings

# C Compiler -- Warnings
cFlags &= " -O3"
#cFlags &= " -fno-common"
cFlags &= " -Wall" # turn on warnings
#cFlags &= " -pedantic" # more warnings
#cFlags &= " -Wsign-compare"
#cFlags &= " -Wcast-align"
#cFlags &= " -Wconversion" # neg int const implicitly converted to uint
cFlags &= " -fsingle-precision-constant"
#cFlags &= " -fomit-frame-pointer" # do not use fp if not needed
cFlags &= " -ffunction-sections -fdata-sections"
cFlags &= " -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable"
cFlags &= includes
# Linker
#ldFlags &= " -nostartfiles" # No start files are used
ldFlags &= " --specs=nano.specs"
ldFlags &= " -Wl,--gc-sections" # Linker garbage collector
ldFlags &= " -Wl,-Map=" & board & ".map,--cref" #generate map file
ldFlags &= " -lc -lnosys"

import configParser

# make ########################################################################
task make, "Build everything":
  exec "nimble clean"
  exec "nimble makeNim"
  exec "nimble makeC"

# makeNim #####################################################################
task makeNim, "Compile Nim files to C":
  echo("Compiling Nim files")
  if not system.dirExists("./nimcache"):
    mkDir("./nimcache")
  cpFile("c:/nim/lib/nimbase.h", "./nimcache/nimbase.h")
  exec "nim $# $#" % [nimflags, nimMain]

# makeC #######################################################################
task makeC, "Compile c files":
  var home = os.getHomeDir()
  var configFile: string = ""
  var linkerscript: string = ""
  var startupFile: string = ""
  var dStartupFile: string
  var oStartupFile: string
  # Find linkerscript and startup file
  for file in os.walkDirRec(home & "/.nimble/pkgs/"):
    var (dir, name, ext) = splitFile(file)
    if dir.contains(board):
      if ext == ".ld" and linkerscript == "":
        linkerscript = file
        echo("Linkerscript: " & name & ext)
      if ext == ".s" and startupFile == "":
        startupFile = file
        dStartupFile = "./" & builddir & name & ".d"
        oStartupFile = "./" & builddir & name & ".o"
        echo("Startup file: " & name & ext)
      if name & ext == board & ".cfg" and configFile == "":
        configFile = file
        echo("Config file: " & name & ext)
    else:
      continue

  var cont = readFile(configFile)
  var d = parseIni(cont)
  cFlags &= " " & d.getProperty("build","cFlags")
  ldFlags &= " " & d.getProperty("build","ldFlags")

  echo("Compiling C files")

  if not system.dirExists(builddir):
    mkDir(builddir)

  var cfiles: seq[string] = @[]
  var srcs = ""
  var objs = ""
  var asms = ""
  for file in os.walkDirRec("./nimcache"):
    var (dir, name, ext) = splitFile(file)
    if ext == ".c":
      cfiles.add(file)

  # Create srcs and object strings
  for file in cfiles:
    var (dir, name, ext) = splitFile(file)
    var dFile = "./" & builddir & name & ".d"
    var oFile = "./" & builddir & name & ".o"
    srcs &= " " & file
    objs &= " " & oFile
    try:
      exec "arm-none-eabi-gcc.exe $# -c -MMD -MP -MF\"$#\" -o $# $#" % [cflags, dFile, oFile, file]
    except:
      echo "Error while compiling, see stack trace above"

  # Build S file
  asms = " " & oStartupFile
  try:
    exec "arm-none-eabi-gcc.exe $# -x assembler-with-cpp -c -g -MMD -MP -MF\"$#\" -o $# $#" % [cFlags, dStartupFile, oStartupFile, startupFile]
  except:
    echo "Error while compiling, see stack trace above"

  # Merge c objs and s objs
  objs &= asms
  try:
    exec "arm-none-eabi-gcc.exe $# $# -T$# -o ./$#$#.elf" % [objs, ldFlags, linkerscript, builddir, board]
  except:
    echo "Error while linking, see stack trace above"
  # Create bin and hex files
  try:
    exec "arm-none-eabi-objcopy.exe -O ihex ./$#$#.elf ./$#$#.hex" % [builddir, board, builddir, board]
    exec "arm-none-eabi-objcopy.exe -O binary ./$#$#.elf ./$#$#.bin" % [builddir, board, builddir, board]
    echo("Finished building")
  except:
    echo "Error during ihex and binary creation"

# flash #######################################################################
task flash, "Flash the device":
  try:
    exec "st-flash write ./$#$#.bin 0x8000000" % [builddir, board]
  except:
    echo "Error during flashing"

# clean #######################################################################
task clean, "Clean build dir":
  try:
    if system.dirExists("./" & builddir):
      rmDir("./" & builddir)
    if system.dirExists("./nimcache"):
      rmDir("./nimcache")
  except:
    echo "Error: couldn't clean build directory: ./$#" % builddir