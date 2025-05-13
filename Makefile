# Toolchain setup
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
SZ = arm-none-eabi-size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

# Target configuration
TARGET = TinyML_STM32F746NG
DEVICE = STM32F746NGHx
CPU = cortex-m7
FPU = fpv5-sp-d16
FLOAT-ABI = hard

# Optimization and debug flags
OPT = -O3
DEBUG = -g
CSTANDARD = -std=c99

# MCU flags
MCU = -mcpu=$(CPU) -mthumb -mfpu=$(FPU) -mfloat-abi=$(FLOAT-ABI)

# Compiler flags
CFLAGS = $(MCU) $(CSTANDARD) $(OPT) $(DEBUG)
CFLAGS += -ffunction-sections -fdata-sections -Wall
CFLAGS += -DUSE_HAL_DRIVER -DSTM32F746xx -DARM_MATH_CM7 -D__FPU_PRESENT=1
# Linker script
LDSCRIPT = STM32F746NGHx_FLASH.ld
# Assembler flags
ASFLAGS = $(MCU) $(OPT) $(DEBUG) -Wa,--no-warn

# Linker flags
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(DEBUG) $(OPT)
LDFLAGS += -Wl,--gc-sections -static
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref

# Include paths
INCLUDES = -I./Inc
INCLUDES += -I./Drivers/STM32F7xx_HAL_Driver/Inc
INCLUDES += -I./Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
INCLUDES += -I./Drivers/CMSIS/Device/ST/STM32F7xx/Include
INCLUDES += -I./Drivers/CMSIS/Include
INCLUDES += -I./Drivers/CMSIS/DSP/Include
INCLUDES += -I./Middlewares/ST/AI/Inc
INCLUDES += -I./Middlewares/ST/Application/Validation/Inc
INCLUDES += -I./Drivers/BSP/STM32746G-Discovery



# Build directory
BUILD_DIR = build

# Source files
## Application files
SRC_FILES = \
  ./Src/network.c \
  ./Src/network_data.c \
  ./Src/main.c \
  ./Src/app_x-cube-ai.c \
  ./Src/stm32f7xx_it.c \
  ./Src/stm32f7xx_hal_msp.c \
  ./Src/stm32f7xx_hal_timebase_tim.c \
  ./Src/system_stm32f7xx.c \
  ./Utilities/Fonts/font57.c

## HAL Drivers
SRC_FILES += \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_crc.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_crc_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dcmi.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dcmi_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma2d.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_fmc.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sdram.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_ltdc.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_ltdc_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dsi.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_qspi.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rtc.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rtc_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sai.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sai_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_sdmmc.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_sd.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spdifrx.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart_ex.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_hcd.c \
  ./Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c

## BSP Components
SRC_FILES += \
  ./Drivers/BSP/STM32746G-Discovery/stm32746g_discovery.c \
  ./Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_lcd.c \
  ./Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_sdram.c \
  ./Drivers/BSP/STM32746G-Discovery/stm32746g_discovery_ts.c \
  ./Drivers/BSP/Components/ft5336/ft5336.c

## CMSIS DSP
SRC_FILES += \
  ./Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_dot_prod_f32.c \
  ./Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_shift_q15.c \
  ./Drivers/CMSIS/DSP/Source/BasicMathFunctions/arm_shift_q7.c \
  ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.c \
  ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.c \
  ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.c \
  ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.c \
  ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.c \
  ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.c \
  ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.c \
  ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.c \
  ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.c \
  ./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.c \
  ./Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c

## X-CUBE-AI Validation
SRC_FILES += \
  ./Middlewares/ST/Application/Validation/Src/aiPbMgr.c \
  ./Middlewares/ST/Application/Validation/Src/aiTestUtility.c \
  ./Middlewares/ST/Application/Validation/Src/aiValidation.c \
  ./Middlewares/ST/Application/Validation/Src/pb_common.c \
  ./Middlewares/ST/Application/Validation/Src/pb_decode.c \
  ./Middlewares/ST/Application/Validation/Src/pb_encode.c \
  ./Middlewares/ST/Application/Validation/Src/stm32msg.pb.c

## Startup file
ASM_SRC = startup_stm32f746xx.s

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(SRC_FILES:.c=.o)))
vpath %.c $(sort $(dir $(SRC_FILES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SRC:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SRC)))

# Libraries
LIBS = -lc -lm -lnosys 
LIBS += -L./Middlewares/ST/AI/Lib -l:NetworkRuntime410_CM7_GCC.a

# Default target
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) $(INCLUDES) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

clean:
	-rm -fR $(BUILD_DIR)

# Flash the device using OpenOCD
flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash write $(BUILD_DIR)/$(TARGET).bin  0x08000000

.PHONY: all clean flash