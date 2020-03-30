#
#             LUFA Library
#     Copyright (C) Dean Camera, 2014.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Run "make help" for target help.


# Set the MCU accordingly to your device
# the REAL_BOARD is the name you want xxx.hex
# there must be a define of BOARD in LUFA
REAL_BOARD   = Teensy2
BOARD        = TEENSY2
MCU          = atmega32u4
MEM_SIZE     = 900
LEDMASK_TX   = LEDS_LED1
LEDMASK_RX   = LEDS_LED1
ARCH         = AVR8
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = ./$(REAL_BOARD)/$(REAL_BOARD)
SRC          = Joystick.c Descriptors.c $(LUFA_SRC_USB)
LUFA_PATH    = ./lufa/LUFA
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -DMEM_SIZE=$(MEM_SIZE) -DBOARD=$(BOARD) -DLEDMASK_TX=$(LEDMASK_TX) -DLEDMASK_RX=$(LEDMASK_RX) -IConfig/ 
LD_FLAGS     =

$(shell mkdir ./$(REAL_BOARD))

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk

# Target for LED/buzzer to alert when print is done
with-alert: all
with-alert: CC_FLAGS += -DALERT_WHEN_DONE
