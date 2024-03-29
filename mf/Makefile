#用于编译FWLIB
#路径都是相对于/d/STM32Project的

NAME = Rcf
COMPILE_TIME = $(shell date +%Y_%m_%d)



#路径处理
POWER_SHELL
#GIT_BASH
ifdef POWER_SHELL
TOP_DIR=$(pwd)#PATH表示路径，指向具体文件，DIR表示目录
ALL_DIR=$(ls -dir -r)#遍历整个工程文件夹的路径

# c编译生成文件夹
BUILD_DIR = $(TOP_DIR)\build

SRCS_C+=$(foreach dir,$(ALL_DIR),$(wildcard $(dir)/*.c))
OBJS_C+=$(addprefix $(BUILD_DIR)/,$(notdir $(SRCS_C:%.c=%.o)))
SRCS_S+=$(foreach dir,$(ALL_DIR),$(wildcard $(dir)/*.s))
SRCS_S+=$(foreach dir,$(ALL_DIR),$(wildcard $(dir)/*.S))#要做预编译操作的
OBJS_S+=$(addprefix $(BUILD_DIR)/,$(notdir $(SRCS_C:%.s=%.o)))
OBJS_S+=$(addprefix $(BUILD_DIR)/,$(notdir $(SRCS_C:%.S=%.o)))
# INC_DIR+=#查找ALL_DIR下包含.h的目录
else
endif



######################################
# building variables
######################################
# optimization
OPT = -Og

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
USE_HARD_FP
ifdef USE_HARD_FP
FLOAT-ABI = -mfloat-abi=hard
USE_DP_FP#二选一
# USE_SP_FP
ifdef USE_DP_FP
FPU = -mfpu=auto#默认开启双精度浮点，也就是fpv5-d16
# FPU = -mfpu=fpv5-d16#自行指定也是可以的
else
FPU = -mfpu=fpv5-sp-d16
endif
else
FLOAT-ABI = -mfloat-abi=soft
# FLOAT-ABI = -mfloat-abi=softfp
endif

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
# -D__FPU_PRESENT\#在stm32h750xx.h中定义了
# -DARM_MATH_CM4 \#数学库以后再说
# -DARM_MATH_MATRIX_CHECK
 
# AS includes
AS_INCLUDES =\
 
# C includes
C_INCLUDES = \
-Isource/USER \
-Isource/CORE \
-Isource/FWLIB/inc \
-Isource/DRIVER/inc \
-Isource/DEVICE/inc \
-Isource/CONTROL/inc \
-Isource/MATH/inc \
-Isource/DSPLIB\
-Isource/STRUCTURE/inc


# compile gcc flags
ASFLAGS = $(MCU) $(OPT)
# ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -fdata-sections -ffunction-sections
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT)
# CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -fdata-sections -ffunction-sections

#######################################
# LDFLAGS
#######################################

# LIBS = -lc -lm
# LIBDIR = \
# -Lxxxxxx

# link script  链接配置文件
LDSCRIPT = source/CORE/STM32F417IG_FLASH.ld

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
OCD_CHIP_FILE = stm32h7x.cfg	#芯片配置文件

#######################################
# 开始编译
# 先编译startup文件
#######################################

ASM_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
STDPERIPH_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(STDPERIPH_C_SOURCES:.c=.o)))
DEVICE_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(DEVICE_C_SOURCES:.c=.o)))
DRIVER_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(DRIVER_C_SOURCES:.c=.o)))
USER_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(USER_C_SOURCES:.c=.o)))

MATH_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(MATH_C_SOURCES:.c=.o)))
CONTROL_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CONTROL_C_SOURCES:.c=.o)))
STRUCTURE_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(STRUCTURE_C_SOURCES:.c=.o)))

OBJECTS = \
$(ASM_OBJECTS) $(STDPERIPH_OBJECTS) $(DEVICE_OBJECTS) $(DRIVER_OBJECTS) $(USER_OBJECTS) \
$(MATH_OBJECTS) $(CONTROL_OBJECTS) $(STRUCTURE_OBJECTS)

.PHONY: \
all \
startup stdperiph device driver user \
math control structure \
burn link reset \
clean cleano \
macro pwd

all: startup stdperiph device driver user math control structure\
$(BUILD_DIR)/$(NAME)_$(COMPILE_TIME).elf $(BUILD_DIR)/$(NAME)_$(COMPILE_TIME).bin

$(BUILD_DIR)/$(NAME)_$(COMPILE_TIME).bin: $(BUILD_DIR)/$(NAME)_$(COMPILE_TIME).elf
	rm -rf $(BUILD_DIR)/*.bin
	$(CP) $^ $@
# -O binary 
# $(BUILD_DIR)/$(NAME)_$(DATE).hex: $(BUILD_DIR)/$(NAME)_$(DATE).elf
# 	$(CP) -O ihex $^ $@

$(BUILD_DIR)/$(NAME)_$(COMPILE_TIME).elf: $(OBJECTS)
	rm -rf $(BUILD_DIR)/*.elf
	rm -rf $(BUILD_DIR)/*.map
	$(CC) $(LDFLAGS) \
-specs=nosys.specs -static \
-Wl,-cref,-u,Reset_Handler \
-Wl,-Map=$(BUILD_DIR)/$(NAME)_$(COMPILE_TIME).map \
-Wl,--gc-sections \
-Wl,--defsym=malloc_getpagesize_P=0x80 \
$^ \
-Wl,--start-group -lm -lc -Wl,--end-group \
-Lsource/DSPLIB -larm_cortexM4lf_math \
-o $@

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# -lm -lc要在放在.o后面，不然.o里面调用到的库函数找不到地方，我服了
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

$(BUILD_DIR)/%.o: %.s
	$(CC) -c $(ASFLAGS) -g -Wa,--warn $< -o $@

$(BUILD_DIR)/%.o: %.c
	$(CC) -c $(CFLAGS) -g -Wall $< -o $@

# build/spl06.s: source/DEVICE/src/spl06.c
# $(CC) -S $(CFLAGS) -g -Wall $< -o $@

vpath %.s $(sort $(dir $(ASM_SOURCES)))

startup: $(ASM_OBJECTS)
	@echo \<\<\<\<\<Startup File Compile Completely\>\>\>\>\>

vpath %.c $(sort $(dir $(STDPERIPH_C_SOURCES)))

stdperiph: $(STDPERIPH_OBJECTS)
	@echo \<\<\<\<\<Standard Peripheral Library Compile Completely\>\>\>\>\>

vpath %.c $(sort $(dir $(DEVICE_C_SOURCES)))

device: $(DEVICE_OBJECTS)
	@echo \<\<\<\<\<Device Compile Completely\>\>\>\>\>

vpath %.c $(sort $(dir $(DRIVER_C_SOURCES)))

driver: $(DRIVER_OBJECTS)
	@echo \<\<\<\<\<Driver Compile Completely\>\>\>\>\>

vpath %.c $(sort $(dir $(USER_C_SOURCES)))

user: $(USER_OBJECTS)
	@echo \<\<\<\<\<User File Compile Completely\>\>\>\>\>

vpath %.c $(sort $(dir $(MATH_C_SOURCES)))

math: $(MATH_OBJECTS)
	@echo \<\<\<\<\<Math File Compile Completely\>\>\>\>\>

vpath %.c $(sort $(dir $(CONTROL_C_SOURCES)))

control: $(CONTROL_OBJECTS)
	@echo \<\<\<\<\<Control File Compile Completely\>\>\>\>\>

vpath %.c $(sort $(dir $(STRUCTURE_C_SOURCES)))

structure: $(STRUCTURE_OBJECTS)
	@echo \<\<\<\<\<Structure File Compile Completely\>\>\>\>\>

burn:
	openocd \
	-f interface/$(OCD_LINK_FILE) \
	-f target/$(OCD_CHIP_FILE) \
	-c init \
	-c "reset halt" \
	-c "flash write_image erase $(BUILD_DIR)/$(NAME)_$(COMPILE_TIME).bin" \
	-c "reset run" \
	-c exit

link:
	openocd \
	-f interface/$(OCD_LINK_FILE) \
	-f target/$(OCD_CHIP_FILE)

reset:
	openocd \
	-f interface/$(OCD_LINK_FILE) \
	-f target/$(OCD_CHIP_FILE) \
	-c init \
	-c halt \
	-c reset \
	-c exit
clean:
	rm -rf $(BUILD_DIR)/*

cleano:
	rm -rf $(BUILD_DIR)/*.o

macro:
	touch gnuc_macro.c
	-rm gnuc_macro.h
	$(CC) -E -dM $(FPU) $(FLOAT-ABI) -c gnuc_macro.c >> gnuc_macro.h

pwd:
	@echo $(BUILD_DIR)/$(NAME)_$(DATE)
#@echo $(words $(wildcard $(BUILD_DIR)/*.bin))
#@echo $(NAME)_$(DATE).bin
#@echo $(BUILD_DIR)/$(NAME)_$(DATE).hex: $(BUILD_DIR)/$(NAME)_$(DATE).elf
#@echo $(STDPERIPH_C_SOURCES:.c=.o)
#@echo $(wildcard source/FWLIB/src/*.c)