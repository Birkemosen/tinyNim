# tinyNim
Nim on STM32F446 uisng nimble for building

# Dependencies
- nim-configparser ``nimble install configparser``
- arm-none-eabi-gcc compile
- st-link utility (unix/linux == texane/stlink)

# Howto
Copy the two following folders to your ".nimble/pkgs/" directory: 
  - tinyNim-0.1.0
  - tinyNimBoards-0.1.0 

Open the tinyNimProject in vSCode or other editor.

In the tinyNim.nimble script, make sure that the following section regarding nimbase.h fits your environment:
```
# makeNim #####################################################################
task makeNim, "Compile Nim files to C":
  echo("Compiling Nim files")
  if not system.dirExists("./nimcache"):
    mkDir("./nimcache")
  cpFile("c:/nim/lib/nimbase.h", "./nimcache/nimbase.h")
  exec "nim $# $#" % [nimflags, nimMain]
```

If on Linux/Unix make sure to remove .exe from arm-none-eabi-objcopy and arm-none-eabi-gcc in the nimble script

## Building the project
You need to have arm-none-eabi-gcc installed and on your PATH.
The nimble script defines three tasks:
- ``nimble make``       # Compiles both nim and .c files
- ``nimble makeNim``   # Compiles only the nim files
- ``nimble makeC``     # Compiles all the .c files
- ``nimble flash``      # Uploads the compilede binary to the board.
