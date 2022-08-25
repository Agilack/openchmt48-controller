##
 # @file  Makefile
 # @brief Script to compile sources and create flat binary using "make" program
 # 
 # @author Saint-Genest Gwenael <gwen@agilack.fr>
 # @copyright Agilack (c) 2021
 # 
 # @page License
 # This firmware is free software: you can redistribute it and/or modify it
 # under the terms of the GNU Lesser General Public License version 3 as
 # published by the Free Software Foundation. You should have received a
 # copy of the GNU Lesser General Public License along with this program,
 # see LICENSE.md file for more details.
 # This program is distributed WITHOUT ANY WARRANTY.
##
TARGET = fw-pnp

ASRC = stm32f746xx.s
SRC  = main.c console.c hardware.c time.c uart.c motor.c

SRC_RTOS =  heap_1.c list.c queue.c tasks.c timers.c
SRC_RTOS += stm32f7_port.c

CROSS = arm-none-eabi-
BUILDDIR = build/
CC = $(CROSS)gcc
OBJCOPY = $(CROSS)objcopy
OBJDUMP = $(CROSS)objdump
RM = rm
AFLAGS = -mcpu=cortex-m7 -mthumb
CFLAGS = -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I. -Isrc
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -nostdlib -Os -ffunction-sections
CFLAGS += -Wall -Wimplicit -Wcast-align -Wpointer-arith -Wswitch
CFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref,--gc-sections
LDFLAGS += -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -Tsrc/target.ld

# Debug symbols
CFLAGS += -g
LDFLAGS += -g

AOBJ = $(patsubst %.s,build/%.o,$(ASRC))
COBJ = $(patsubst %.c,build/%.o,$(SRC))
HTML_OBJ = $(patsubst %.html,build/%.o,$(HTML))
ICO_OBJ  = $(patsubst %.ico,build/%.o,$(ICO))
CFLAGS += -Irtos -Irtos/include -Irtos/cmsis
CFLAGS_OS = $(CFLAGS)
OS_OBJ = $(patsubst %.c,build/rtos/%.o,$(SRC_RTOS))


all: $(BUILDDIR) $(AOBJ) $(COBJ) $(OS_OBJ)
	@echo "  [LD] $(TARGET)"
	@$(CC) $(LDFLAGS) -o $(TARGET).elf $(AOBJ) $(COBJ) $(OS_OBJ) -lm
	@echo "  [OC] $(TARGET).bin"
	@$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin
	@echo "  [OD] $(TARGET).dis"
	@$(OBJDUMP) -D $(TARGET).elf > $(TARGET).dis

clean:
	@echo "  [RM] $(TARGET)"
	@$(RM) -f $(TARGET).*
	@echo "  [RM] $(BUILDDIR)*.o $(BUILDDIR)rtos/*.o"
	@$(RM) -f $(AOBJ) $(COBJ) $(OS_OBJ)
	@echo "  [RM] Temporary files"
	@$(RM) -f *~ src/*~
	@$(RM) -f rtos/*~ rtos/include/*~
	@$(RM) -rf build

$(BUILDDIR):
	@echo "  [MKDIR] $@"
	@mkdir build
	@echo "  [MKDIR] $@/rtos"
	@mkdir build/rtos

build/%.o: src/%.s
	@echo "  [CC] $<"
	@$(CC) $(AFLAGS) -c $< -o $@

build/%.o: src/%.c
	@echo "  [CC] $<"
	@$(CC) $(CFLAGS) -c $< -o $@

build/rtos/%.o: rtos/%.c
	@echo "  [CC] $<"
	@$(CC) $(CFLAGS_OS) -c $< -o $@

debug:
	arm-none-eabi-gdb --command=scripts/gdb.cfg $(TARGET).elf

load:
	arm-none-eabi-gdb --command=scripts/gdb_load.cfg $(TARGET).elf

openocd:
	openocd -f scripts/openocd.cfg
