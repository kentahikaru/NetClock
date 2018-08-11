# Makefile for STM32F7-Discovery-Blinky

PROJECT = NetClock
OBJDIR = obj
RESULT = $(OBJDIR)/$(PROJECT)


################
# Sources


#SOURCES_S = lib/Driver/CMSIS/Device/ST/STM32F7xx/Source/Templates/gcc/startup_stm32f746xx.s
#SOURCES_S = startup/startup_stm32.s
SOURCES_S = startup/startup_stm32f746xx.s

SOURCES_C = src/main.c
#SOURCES_C += lib/Driver/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.c
#SOURCES_C += startup/sysmem.c
SOURCES_C += startup/system_stm32f7xx.c
#SOURCES_C += lib/Driver/BSP/STM32746G-Discovery/stm32746g_discovery.c
#SOURCES_C += lib/STM32F7xx_StdPeriph_Driver/src/stm32f7xx_gpio.c

#FreeRTOS^M
#SOURCES_C += FreeRTOS/CMSIS_RTOS/cmsis_os.c
SOURCES_C += FreeRTOS/portable/GCC/ARM_CM4F/port.c
SOURCES_C += FreeRTOS/portable/MemMang/heap_4.c
SOURCES_C += FreeRTOS/croutine.c
SOURCES_C += FreeRTOS/list.c
SOURCES_C += FreeRTOS/queue.c
SOURCES_C += FreeRTOS/tasks.c
SOURCES_C += FreeRTOS/timers.c
#SOURCES_C +=/FreeRTOS/event_groups.c





SOURCES_CPP =

SOURCES = $(SOURCES_S) $(SOURCES_C) $(SOURCES_CPP)
#OBJS = $(SOURCES_S:.s=.o) $(SOURCES_C:.c=.o) $(SOURCES_CPP:.cpp=.o)
OBJS = $(addprefix $(OBJDIR)/, $(addsuffix .o, $(basename $(SOURCES)) ))

################
# Includes and Defines

INCLUDES += -I . -I inc -I sys
INCLUDES += -I src
INCLUDES += -I lib/Driver/CMSIS/Include
INCLUDES += -I lib/Driver/CMSIS/Device/ST/STM32F7xx/Include
INCLUDES += -I lib/Driver/BSP/STM32746G-Discovery_SPL/
INCLUDES += -I lib/STM32F7xx_StdPeriph_Driver/inc

#INCLUDES += -IFreeRTOS/CMSIS_RTOS
INCLUDES += -IFreeRTOS/include
INCLUDES += -IFreeRTOS/portable/GCC/ARM_CM4F


DEFINES = -DSTM32 -DSTM32F7 -DSTM32F746xx -DSTM32F746NGHx -DSTM32F746G_DISCO 

################
# Compiler/Assembler/Linker/etc

PREFIX = arm-none-eabi

CC = $(PREFIX)-gcc
AS = $(PREFIX)-as
AR = $(PREFIX)-ar
LD = $(PREFIX)-gcc
NM = $(PREFIX)-nm
OBJCOPY = $(PREFIX)-objcopy
OBJDUMP = $(PREFIX)-objdump
READELF = $(PREFIX)-readelf
SIZE = $(PREFIX)-size
GDB = $(PREFIX)-gdb
RM = rm -f

################
# Compiler options

MCUFLAGS = -mcpu=cortex-m7 -mlittle-endian
MCUFLAGS += -mfloat-abi=hard -mfpu=fpv5-sp-d16
MCUFLAGS += -mthumb

DEBUGFLAGS = -Og -g3 -gdwarf-2 -ggdb
#DEBUGFLAGS = -O2

DEPFLAGS += -MMD -MP -MF $(OBJDIR)/$(*D).d

CFLAGS_EXTRA = -nostartfiles -fdata-sections -ffunction-sections
CFLAGS_EXTRA += -Wl,--gc-sections -Wl,-Map=$(PROJECT).map

#CFLAGS = -std=c11
CFLAGS = -std=gnu99
CFLAGS += -Wall -Wextra --pedantic
CFLAGS += $(DEFINES) $(MCUFLAGS) $(DEBUG_FLAGS) $(CFLAGS_EXTRA) $(INCLUDES) $(DEPFLAGS)

LDFLAGS = -static $(MCUFLAGS)
LDFLAGS += -Wl,--start-group -lgcc -lm -lc -lg -lstdc++ -lsupc++ -Wl,--end-group
LDFLAGS += -Wl,--gc-sections
#LDFLAGS += -Tstm32f7-discovery.ld -L. -Lldscripts
#LDFLAGS += -TLinkerScript.ld -L. -Lldscripts
LDFLAGS += -TLinkerScript.ld 
LDFLAGS += -Xlinker -Map -Xlinker $(RESULT).map


	
################
# Build rules

all: $(RESULT).hex bin dump


$(RESULT).hex: $(RESULT).elf
	$(OBJCOPY) -O ihex $(RESULT).elf $(RESULT).hex

$(RESULT).elf: $(OBJS)
	@echo "..... Linking ....."
	@echo "..... Linking $<"
	$(LD) $(OBJS) $(LDFLAGS) -o $(RESULT).elf
	$(SIZE) -A $(RESULT).elf

$(OBJDIR)/%.o : %.c
	@echo "..... Making source (.c) files ....."
	@echo "..... Compiling $<"
	$(CC) -c $(CFLAGS) $(DEBUGFLAGS) $< -o $@

$(OBJDIR)/%.o : %.s
	@echo "..... Making assembler (.s) files ....." 
	@echo "..... Compiling $<"
	$(AS) -c $(MCUFLAGS) $< -o $@

bin: $(RESULT).elf
	$(OBJCOPY) $(RESULT).elf -O binary $(RESULT).bin

dump: $(RESULT).elf
	$(OBJDUMP) -D $(RESULT).elf > $(RESULT).list

$(OBJS): | $(OBJDIR)  # order-only-prerequisite, make directories before making objects

$(OBJDIR) :
	mkdir $(OBJDIR)
	mkdir $(OBJDIR)/startup
	mkdir $(OBJDIR)/src/
	mkdir $(OBJDIR)/FreeRTOS/
	mkdir $(OBJDIR)/FreeRTOS/portable/
	mkdir $(OBJDIR)/FreeRTOS/portable/GCC/
	mkdir $(OBJDIR)/FreeRTOS/portable/GCC/ARM_CM4F
	mkdir $(OBJDIR)/FreeRTOS/portable/MemMang

clean:
	#$(RM) $(OBJS) $(PROJECT).elf $(PROJECT).hex $(PROJECT).map
	$(RM) -r $(OBJDIR)

.PHONY: all clean

# EOF
