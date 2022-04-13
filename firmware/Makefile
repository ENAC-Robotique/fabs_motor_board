##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
# -Wdouble-promotion -fno-omit-frame-pointer
GCCVERSIONGTEQ7 := $(shell expr `arm-none-eabi-gcc -dumpversion | cut -f1 -d.` \>= 7)
GCC_DIAG =  -Wno-error=unused-variable -Wno-error=format \
            -Wno-error=unused-function \
            -Wunused -Wpointer-arith \
            -Werror=sign-compare \
            -Wshadow -Wparentheses -fmax-errors=5 \
            -ftrack-macro-expansion=2 -Wno-error=strict-overflow -Wstrict-overflow=5 

ifeq "$(GCCVERSIONGTEQ7)" "1"
    GCC_DIAG += -Wvla-larger-than=128 -Wduplicated-branches -Wdangling-else \
                -Wformat-overflow=2 -Wformat-truncation=2
endif


ifeq ($(USE_OPT),)
  USE_OPT =  -O0  -ggdb3  -Wall -Wextra \
            -falign-functions=16 -fomit-frame-pointer \
            $(GCC_DIAG)
endif

ifeq ($(USE_OPT),)
  USE_OPT =  -Ofast  -flto  -Wall -Wextra \
            -falign-functions=16 -fomit-frame-pointer \
             $(GCC_DIAG)
endif



# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = -std=gnu11
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -std=c++17 -fno-rtti -fno-exceptions -lstdc++
endif


# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif







# protobuf
PROTOC = protoc
PROTO_DIR = protoduck
PROTO_GEN_DIR = build/generated
EMBEDDED_PROTO_DIR = $(shell pwd)/EmbeddedProto

PROTO_FILES = $(wildcard $(PROTO_DIR)/*.proto)
PROTO_HDR := $(PROTO_FILES:%.proto=$(PROTO_GEN_DIR)/%.h) 
EMBEDDED_PROTO_SRC := $(wildcard ./EmbeddedProto/src/*.cpp)

EMBEDDED_PROTO_OBJS := $(EMBEDDED_PROTO_SRC:%.cpp=$(OBJECT_DIR)/%.o)



#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU on Cortex-M4 (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = no
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = ch
BOARD = MOTOR_BOARD_TRD

# Imported source files and paths
MY_DIRNAME=../../../ChibiOS_stable
ifneq "$(wildcard $(MY_DIRNAME) )" ""
   RELATIVE=../../..
else
  RELATIVE=../..
endif
CHIBIOS = chibios_svn_mirror
STMSRC = $(RELATIVE)/COMMON/stm
VARIOUS = chibios_enac_various_common
USBD_LIB = $(VARIOUS)/Chibios-USB-Devices
BOARDGEN = $(VARIOUS)/TOOLS/boardGen.pl
BMPFLASH = $(VARIOUS)/TOOLS/bmpflash



# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f7xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F7xx/platform.mk
include local/$(BOARD)/board.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
# Other files (optional).


# Define linker script file here
LDSCRIPT= ${STARTUPLD}/STM32F76xxI.ld


# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(CHIBIOS)/os/various/syscalls.c \
       $(VARIOUS)/stdutil.c \
       $(VARIOUS)/printf.c \
       $(VARIOUS)/microrl/microrlShell.c \
       $(VARIOUS)/microrl/microrl.c \
       $(VARIOUS)/rtcAccess.c \
       $(VARIOUS)/usb_serial.c \
       src/ttyConsole.c \
       src/globalVar.c

# files extracted from ARM CMSIS DSP
DSPBASE = cmsis_arm
include $(DSPBASE)/files.mk

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =  src/ducklink/communication.cpp \
          src/ducklink/BytesReadBuffer.cpp \
          src/ducklink/BytesWriteBuffer.cpp \
          src/power_check.cpp \
          src/motors.cpp \
          src/OdometryHolo.cpp \
          src/speed_control.cpp \
          src/HolonomicControl.cpp \
          src/DifferentialControl.cpp \
          src/encoders.cpp \
          src/utils.cpp \
          src/OdometryDiff.cpp \
          src/main.cpp

CPPSRC += $(EMBEDDED_PROTO_SRC)

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMXSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR = $(CHIBIOS)/os/license $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) $(TESTINC) \
         $(CHIBIOS)/os/various $(VARIOUS)

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m7

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
#LD   = $(TRGT)gcc
LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DTRACE

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR = src/ src/ducklink \
	  $(EMBEDDED_PROTO_DIR)/src \
    $(BUILDDIR)/generated \
    cmsis_arm

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm

#
# End of user defines
##############################################################################


RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk
$(OBJS): local/$(BOARD)/board.h

local/$(BOARD)/board.h: local/$(BOARD)/board.cfg
	$(BOARDGEN)  --no-pp-line   $<  $@


stflash:        all
	@echo write $(BUILDDIR)/$(PROJECT).bin to flash memory
	/usr/local/bin/st-flash write  $(BUILDDIR)/$(PROJECT).bin 0x08000000
	@echo Done

flash: all
	@echo write $(BUILDDIR)/$(PROJECT).bin to flash memory
	$(BMPFLASH)  $(BUILDDIR)/$(PROJECT).elf
	@echo Done


generate: $(PROTO_HDR)
	$(info Done generating source files based on *.proto files.)

$(PROTO_GEN_DIR)/%.h: %.proto
	$(shell mkdir -p $(dir $@))
	cd $(EMBEDDED_PROTO_DIR) && $(PROTOC) --plugin=protoc-gen-eams=protoc-gen-eams -I../$(PROTO_DIR) --eams_out=../$(PROTO_GEN_DIR) ../$<


