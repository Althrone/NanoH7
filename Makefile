#用于编译FWLIB
#路径都是相对于/d/STM32Project的

TARGET=RcF_NanoH7_RTT
COMPILE_TIME=$(shell date +%Y_%m_%d)

#路径处理
TOP_DIR=$(shell pwd)#PATH表示路径，指向具体文件，DIR表示目录
ALL_DIR=$(shell find -type d)#遍历整个工程文件夹的路径

# c编译生成文件夹
BUILD_DIR=./build

# SRCS_C+=$(foreach dir,$(ALL_DIR),$(wildcard $(dir)/*.c))
# OBJS_C+=$(addprefix $(BUILD_DIR)/,$(notdir $(SRCS_C:.c=.o)))
# # SRCS_s=$(foreach dir,$(ALL_DIR),$(wildcard $(dir)/*.s))
# SRCS_S=$(foreach dir,$(ALL_DIR),$(wildcard $(dir)/*.s))
# #由于SRCS_S中含有.s和.S所以只能用下面这种极其麻烦的方法
# # OBJS_s=$(foreach src_s,$(SRCS_s),$(addsuffix .o,$(basename $(addprefix $(BUILD_DIR)/,$(notdir $(src_s))))))
# OBJS_S=$(foreach src_S,$(SRCS_S),$(addsuffix .o,$(basename $(addprefix $(BUILD_DIR)/,$(notdir $(src_S))))))

# INC_DIRS+=$(foreach dir,$(ALL_DIR),$(dir $(wildcard $(dir)/*.h)))
# INCS_C=$(patsubst ./%,-I%,$(sort $(INC_DIRS)))

# SRCS_D=$(OBJS_C:.o=.d)

SRCS_C=$(shell find ./rt-thread -type f -name *.c)
OBJS_C=$(addprefix $(BUILD_DIR)/,$(notdir $(SRCS_C:.c=.o)))
SRCS_s=$(shell find ./rt-thread -type f -name *.s)
OBJS_s=$(addprefix $(BUILD_DIR)/,$(notdir $(SRCS_s:.s=.o)))
SRCS_S=$(shell find ./rt-thread -type f -name *.S)
OBJS_S=$(addprefix $(BUILD_DIR)/,$(notdir $(SRCS_S:.S=.o)))

INC_DIRS=$(sort $(dir $(shell find ./rt-thread -type f -name *.h)))
INCS_C=$(patsubst ./%,-I%,$(INC_DIRS))

echo:
	@echo  $(sort $(dir $(SRCS_S)))

######################################
# building variables
######################################
# optimization
OPT = -O0

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
CP = $(PREFIX)objcopy
AS = $(PREFIX)gcc -x assembler-with-cpp
SZ = $(PREFIX)size

#######################################
# CFLAGS
#######################################
# 这几个值请查阅arm-none-eabi-gcc的readme.txt
# 路径在你安装的文件夹的share\doc\gcc-arm-none-eabi
# cpu
CPU = -mcpu=cortex-m7
# ARCH = -march=armv7e-m

# fpu
# USE_HARD_FP
# ifdef USE_HARD_FP
FLOAT-ABI = -mfloat-abi=hard
# USE_DP_FP#二选一
# USE_SP_FP
# ifdef USE_DP_FP
FPU = -mfpu=auto#默认开启双精度浮点，也就是fpv5-d16
# FPU = -mfpu=fpv5-d16#自行指定也是可以的
# else
# FPU = -mfpu=fpv5-sp-d16
# endif
# else
# FLOAT-ABI = -mfloat-abi=soft
# # FLOAT-ABI = -mfloat-abi=softfp
# endif

# mcu
MCU = -mthumb $(CPU) $(FLOAT-ABI) $(FPU)#加不加-mthumb生成的代码是一样的
#不支持-marm，不知道为什么

# macros for gcc
# AS defines
AS_DEFS =\

# C defines
# USE_STDPERIPH_DRIVER用于使用标准外设库，包含stm32f4xx_conf.h文件
# __VFP_FP__用于启动407的浮点核，这个是编译器自动生成的，所以不要写在这里
C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32H750xx \
-DHSE_VALUE=8000000UL \
-DUSE_FULL_LL_DRIVER \
-D__need_time_t \
-D__need_timespec \
-DBSP_SCB_ENABLE_I_CACHE \
-DBSP_SCB_ENABLE_D_CACHE \
-D__ARM_ARCH_7EM__ \
-DCONFIG_USB_DWC2_PORT=FS_PORT
# -DSDIO_MAX_FREQ=20000000
# -D__FPU_PRESENT\#在stm32h750xx.h中定义了
# -DARM_MATH_CM4 \#数学库以后再说
# -DARM_MATH_MATRIX_CHECK

# compile gcc flags
ASFLAGS = $(MCU) $(OPT) -Wa,-mimplicit-it=thumb
# ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -fdata-sections -ffunction-sections
CFLAGS = $(MCU) $(C_DEFS) $(INCS_C) $(OPT)
# CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -fdata-sections -ffunction-sections

#######################################
# LDFLAGS
#######################################

# LIBS = -lc -lm
# LIBDIR = \
# -Lxxxxxx

# link script  链接配置文件
#如果出现多个ld文件 应该给个警告，然后自动选第一个
# LDSCRIPT=$(patsubst ./%,%,$(sort $(foreach dir,$(ALL_DIR),$(wildcard $(dir)/*750*.ld))))
LDSCRIPT=$(shell find ./rt-thread -type f -name *.ld)
ifneq ($(words $(LDSCRIPT)),1)
$(error more than one ld file was found)
endif

# LDSCRIPT = source/CORE/STM32F417IG_FLASH.ld

LDGROUP = -Wl,--start-group -lc -lm -Wl,--end-group

LDFLAGS = $(MCU) -T$(LDSCRIPT) $(LIBDIR) $(LIBS)

#######################################
# SZFLAGS
#######################################


#######################################
#OpenOCD
#######################################
OCD_LINK_FILE = cmsis-dap.cfg#烧录器配置文件，用于普通买到的烧录器
# OCD_LINK_FILE = stlink-v2-1.cfg	#烧录器配置文件，用于stm32f4discovery
# OCD_CHIP_FILE = stm32h7x_reset.cfg	#芯片配置文件
OCD_CHIP_FILE = stm32h7x_2MB.cfg	#芯片配置文件


# all:
# 	@echo $(sort $(dir $(SRCS_S)))

.PHONY: all burn size link clean macro reset#这里顺序随意

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin##########请nm的绝对保证all放在PHONY第一个
	@echo build at $(COMPILE_TIME)

OBJECTS=$(OBJS_C)
vpath %.c $(sort $(dir $(SRCS_C)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -g -Wall -ffunction-sections -fdata-sections $< -o $@
# $(CC) -c $(CFLAGS) -g -Wall $< -o $@
# $(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

OBJECTS+=$(OBJS_s)
# # OBJECTS+=./build/startup_stm32h750xx.o
# vpath %.s $(sort $(dir $(SRCS_s)))

# $(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
# 	$(CC) -c $(ASFLAGS) -g -Wa,--warn $< -o $@
# # $(AS) -c $(ASFLAGS) $< -o $@

OBJECTS+=$(OBJS_S)
# vpath %.S $(sort $(dir $(SRCS_S)))

# $(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
# 	$(CC) -c $(ASFLAGS) -g -Wa,--warn $< -o $@
# $(AS) -c $(ASFLAGS) $< -o $@

$(OBJS_s):$(SRCS_s)
	@echo build s file
	$(CC) -c $(ASFLAGS) -g -Wa,--warn $< -o $@

$(OBJS_S):$(SRCS_S)
	@echo build S file
	$(CC) -c $(ASFLAGS) -g -Wa,--warn $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map \
-Wl,--start-group -lc -lm -Wl,--end-group -Wl,-gc-sections \
-lnosys \
-L ./rt-thread/bsp/stm32/libraries/STM32H7xx_HAL/CMSIS/DSP/Lib/GCC -larm_cortexM7lfdp_math \
-o $@
	$(SZ) $@


$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo creat hex file
	$(CP) -O ihex $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	@echo creat bin file
	$(CP) -O binary $< $@	
	
$(BUILD_DIR):
	mkdir $@	

burn:
	openocd \
	-f interface/$(OCD_LINK_FILE) \
	-f target/$(OCD_CHIP_FILE) \
	-c init \
	-c "reset halt" \
	-c "flash write_image erase $(BUILD_DIR)/$(TARGET).elf" \
	-c "reset run" \
	-c exit

link:
	openocd \
	-f interface/$(OCD_LINK_FILE) \
	-f target/$(OCD_CHIP_FILE) \
	-c init \
	-c "rtt server start 8765 0" \
	-c "rtt setup 0x20000000 0x20000 \"SEGGER RTT\"" \
	-c "rtt start"

size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) $@

macro:
	touch gnuc_macro.c
	-rm gnuc_macro.h
	$(CC) -E -dM $(CFLAGS) -c gnuc_macro.c >> gnuc_macro.h

reset:
	openocd \
	-f interface/$(OCD_LINK_FILE) \
	-f target/$(OCD_CHIP_FILE) \
	-c init \
	-c halt \
	-c reset \
	-c exit

clean:
	@echo build at $(COMPILE_TIME)

comman=:
empty=
space=$(empty) $(empty)

# export C_INCLUDE_PATH=/d/NXP/S32DS.3.4/S32DS/build_tools/gcc_v10.2/gcc-10.2-arm32-eabi/include

# $(warning $(sort $(dir $(C_INCLUDE_PATH))))
# $(warning $(sort $(dir $(INC_DIRS))))