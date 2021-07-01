TARGET:=FreeRTOS
TOOLCHAIN_ROOT:=/usr
TOOLCHAIN_PATH:=$(TOOLCHAIN_ROOT)/bin
TOOLCHAIN_PREFIX:=arm-none-eabi

OPTLVL:=0

CORE:=$(CURDIR)/Core
STARTUP=$(CORE)/Startup
FREERTOS:=$(CURDIR)/FreeRTOS-Kernel
DRIVERS:=$(CURDIR)/Drivers
LINKER_SCRIPT:=$(CURDIR)/STM32F407VETX_FLASH.ld


INCLUDE=-I$(CORE)/Inc
INCLUDE+=-I$(DRIVERS)/CMSIS/Device/ST/STM32F4xx/Include
INCLUDE+=-I$(DRIVERS)/CMSIS/Include
INCLUDE+=-I$(DRIVERS)/STM32_DMP_Driver/Include/MPU6050
INCLUDE+=-I$(DRIVERS)/STM32F4xx_HAL_Driver/Inc
INCLUDE+=-I$(FREERTOS)/portable/GCC/ARM_CM4F
INCLUDE+=-I$(FREERTOS)/include

BUILD_DIR = $(CURDIR)/build
BIN_DIR = $(CURDIR)/binary

vpath %.c   $(CORE)/Src \
			$(CORE)/Algorithm \
			$(CORE)/anoPro \
			$(CORE)/Startup \
			$(DRIVERS)/STM32_DMP_Driver/Source/MPU6050 \
			$(DRIVERS)/STM32F4xx_HAL_Driver/Src \
			$(FREERTOS) \
			$(FREERTOS)/portable/MemMang \
			$(FREERTOS)/portable/GCC/ARM_CM4F

# 開機程式
asrc=startup_stm32f407vetx.s

# 主程式
src=main.c
src+=init.c
src+=uart.c
src+=motor.c
src+=sysmem.c
src+=freertos_vapp.c
src+=stm32f4xx_hal_msp.c
src+=syscalls.c
src+=system_stm32f4xx.c

# 地面站
src+=ano.c

# 演算法
src+=pid.c


# HAL
src+=stm32f4xx_hal.c         
src+=stm32f4xx_hal_flash.c          
src+=stm32f4xx_hal_pwr_ex.c
src+=stm32f4xx_hal_cortex.c  
src+=stm32f4xx_hal_flash_ex.c       
src+=stm32f4xx_hal_rcc.c
src+=stm32f4xx_hal_dma.c     
src+=stm32f4xx_hal_flash_ramfunc.c  
src+=stm32f4xx_hal_rcc_ex.c
src+=stm32f4xx_hal_dma_ex.c  
src+=stm32f4xx_hal_gpio.c           
src+=stm32f4xx_hal_tim.c
src+=stm32f4xx_hal_exti.c    
src+=stm32f4xx_hal_pwr.c            
src+=stm32f4xx_hal_tim_ex.c
src+=stm32f4xx_hal_uart.c
src+=stm32f4xx_hal_i2c.c
src+=stm32f4xx_hal_i2c_ex.c

# MPU6050 驅動程式
src+=I2C.c
src+=inv_mpu.c
src+=inv_mpu_dmp_motion_driver.c
src+=mpu6050.c

# FreeRtos
src+=tasks.c
src+=queue.c
src+=port.c
src+=list.c
src+=croutine.c
src+=event_groups.c
src+=timers.c
src+=heap_4.c


CDEFS=-DUSE_HAL_DRIVER
CDEFS+=-DSTM32F407xx
CDEFS+=-DDEBUG
COMMONFLAGS=-O$(OPTLVL) -std=gnu99 -g3 -Wall -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP
MCUFLAGS=-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb 
CFLAGS=$(COMMONFLAGS) $(MCUFLAGS) $(INCLUDE) $(CDEFS)

#  -fstack-usage -MMD -MP -MF"Core/Src/main.d" -MT"Core/Src/main.o" --specs=nano.specs
LDLIBS=-lm -lc -lgcc
LDFLAGS=$(MCUFLAGS) -u _scanf_float -u _printf_float -fno-exceptions -Wl,--gc-sections,-T$(LINKER_SCRIPT),-Map,$(BIN_DIR)/$(TARGET).map

CC=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gcc
LD=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gcc
OBJCOPY=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-objcopy
AS=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-as
AR=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-ar
GDB=$(TOOLCHAIN_PATH)/$(TOOLCHAIN_PREFIX)-gdb

OBJ = $(src:%.c=$(BUILD_DIR)/%.o)

$(BUILD_DIR)/%.o: %.c
	@echo [CC] $(notdir $<)
	@$(CC) $(CFLAGS) $< -c -o $@

all: $(OBJ)
	@echo [AS] $(asrc)
	@$(AS) -g3 -o $(asrc:%.s=$(BUILD_DIR)/%.o) $(STARTUP)/$(asrc)
	@echo [LD] $(TARGET).elf
	@$(CC) -o $(BIN_DIR)/$(TARGET).elf $(LDFLAGS) $(OBJ) $(asrc:%.s=$(BUILD_DIR)/%.o) $(LDLIBS)
	@echo [HEX] $(TARGET).hex
	@$(OBJCOPY) -O ihex $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).hex
	@echo [BIN] $(TARGET).bin
	@$(OBJCOPY) -O binary $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).bin

.PHONY: clean

clean:
	@echo [RM] OBJ
	@rm -f $(OBJ)
	@rm -f $(asrc:%.s=$(BUILD_DIR)/%.o)
	@echo [RM] BIN
	@rm -f $(BIN_DIR)/$(TARGET).elf
	@rm -f $(BIN_DIR)/$(TARGET).hex
	@rm -f $(BIN_DIR)/$(TARGET).bin

flash:
	@st-flash write $(BIN_DIR)/$(TARGET).bin 0x8000000
