##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O0 -ggdb -fomit-frame-pointer -falign-functions=16
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = -std=gnu11
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  GCC_DIAG =  -Werror -Wno-error=unused-variable -Wno-error=format \
            -Wno-error=cpp \
            -Wno-error=unused-function \
            -Wunused -Wpointer-arith \
            -Werror=sign-compare \
            -Wshadow -Wparentheses -fmax-errors=5 \
            -ftrack-macro-expansion=2 -Wno-error=strict-overflow -Wstrict-overflow=2

  G++_DIAG =   -Wnon-virtual-dtor -Woverloaded-virtual   \
	     -Wnull-dereference

  USE_CPPOPT = -std=c++17 -fno-rtti -fno-exceptions
endif


# Enable this if you want the linker to remove unused code and data.
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Enable this if you want link time optimizations (LTO).
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
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

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# FPU-related options.
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv5-d16
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, target, sources and paths
#

# Define project name here
PROJECT = ch

# Target settings.
MCU  = cortex-m7

# Imported source files and paths.
CHIBIOS  := chibios_svn_mirror
CONFDIR  := ./cfg
BUILDDIR := ./build
DEPDIR   := ./.dep
VARIOUS = chibios_enac_various_common
USBD_LIB = $(VARIOUS)/Chibios-USB-Devices
BOARDGEN = $(VARIOUS)/TOOLS/boardGen.pl
BMPFLASH = $(VARIOUS)/TOOLS/bmpflash


# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f7xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F7xx/platform.mk
include $(CONFDIR)/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk
# Auto-build files in ./source recursively.
include $(CHIBIOS)/tools/mk/autobuild.mk
# Other files (optional).
include $(CHIBIOS)/os/test/test.mk
include $(CHIBIOS)/test/rt/rt_test.mk
include $(CHIBIOS)/test/oslib/oslib_test.mk

# Define linker script file here
LDSCRIPT= $(STARTUPLD)/STM32F76xxI.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       $(TESTSRC) \
       $(CHIBIOS)/os/various/syscalls.c \
       $(VARIOUS)/stdutil.c \
       $(VARIOUS)/printf.c \
       $(VARIOUS)/microrl/microrlShell.c \
       $(VARIOUS)/microrl/microrl.c \
       $(VARIOUS)/usb_serial.c \
       src/ttyConsole.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC) \
          src/ducklink/communication.cpp \
          src/ducklink/BytesReadBuffer.cpp \
          src/ducklink/BytesWriteBuffer.cpp \
          src/power_check.cpp \
          src/motors.cpp \
          src/OdometryHolo.cpp \
          src/state_estimation.cpp \
          src/HolonomicControl.cpp \
          src/encoders.cpp \
          src/utils.cpp \
          src/main.cpp \
          src/high_gain_filter.cpp \
          src/globalVar.cpp

# src/OdometryDiff.cpp \
# src/DifferentialControl.cpp \

CPPSRC += $(EMBEDDED_PROTO_SRC)

# List ASM source files here.
ASMSRC = $(ALLASMSRC)

# List ASM with preprocessor source files here.
ASMXSRC = $(ALLXASMSRC)

# Inclusion directories.
INCDIR = $(CONFDIR) $(ALLINC) $(TESTINC) \
         $(CHIBIOS)/os/various $(VARIOUS)

# Define C warning options here.
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here.
CPPWARN = -Wall -Wextra -Wundef

#
# Project, target, sources and paths
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DTRACE -DEIGEN_NO_MALLOC -DEIGEN_NO_AUTOMATIC_RESIZING -DEIGEN_NO_DEBUG

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR = src/ src/ducklink \
	  $(EMBEDDED_PROTO_DIR)/src \
    $(BUILDDIR)/generated \
    eigen

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm

#
# End of user section
##############################################################################

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD=$(TRGT)g++

#
# Common rules
##############################################################################

##############################################################################
# Custom rules
#

#
# Custom rules
##############################################################################
$(OBJS): $(CONFDIR)/board.h

$(CONFDIR)/board.h: $(CONFDIR)/board.cfg
	$(BOARDGEN)  --no-pp-line   $<  $@


generate: $(PROTO_HDR)
	$(info Done generating source files based on *.proto files.)

$(PROTO_GEN_DIR)/%.h: %.proto
	$(shell mkdir -p $(dir $@))
	cd $(EMBEDDED_PROTO_DIR) && $(PROTOC) --plugin=protoc-gen-eams=protoc-gen-eams -I../$(PROTO_DIR) --eams_out=../$(PROTO_GEN_DIR) ../$<

