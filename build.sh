#!/bin/bash
#===============================================================================
# Keil μVision 命令行编译/烧录脚本
# 用途: UV4 在 CLI 环境崩溃时的替代方案
# 用法: ./build.sh [flash|build|clean]
#   build  — 编译 + 链接 + 生成 .hex (默认)
#   flash  — 编译 + 链接 + 生成 .hex + 烧录
#   clean  — 清理 .o 和 .axf
#===============================================================================

set -e
cd "$(dirname "$0")/project"

# ── Pre-flight: 检查 DFP 包 ───────────────────────────────────────────────────
LOCAL_PACKS="$LOCALAPPDATA/Arm/Packs"
KEIL_PACKS="$(dirname "$(dirname "$0")")/ARM/PACK" 2>/dev/null || true
KEIL_PACKS="C:/Keil_v5/ARM/PACK"
if [ ! -f "$KEIL_PACKS/Keil/STM32F4xx_DFP/3.1.1/Keil.STM32F4xx_DFP.pdsc" ]; then
  if [ -d "$LOCAL_PACKS/Keil/STM32F4xx_DFP" ]; then
    echo "⚠ DFP pack missing from Keil dir, copying from AppData..."
    mkdir -p "$KEIL_PACKS/Keil/STM32F4xx_DFP"
    cp -r "$LOCAL_PACKS/Keil/STM32F4xx_DFP/"* "$KEIL_PACKS/Keil/STM32F4xx_DFP/"
    cp "$LOCAL_PACKS/.Web/Keil.STM32F4xx_DFP.pdsc" "$KEIL_PACKS/.Web/" 2>/dev/null || true
  fi
fi
if [ ! -d "$KEIL_PACKS/ARM/CMSIS/6.1.0" ]; then
  if [ -d "$LOCAL_PACKS/ARM/CMSIS" ]; then
    echo "⚠ CMSIS pack missing from Keil dir, copying from AppData..."
    mkdir -p "$KEIL_PACKS/ARM/CMSIS"
    cp -r "$LOCAL_PACKS/ARM/CMSIS/"* "$KEIL_PACKS/ARM/CMSIS/"
    cp "$LOCAL_PACKS/.Web/ARM.CMSIS.pdsc" "$KEIL_PACKS/.Web/" 2>/dev/null || true
  fi
fi

# ── Toolchain ─────────────────────────────────────────────────────────────────
CC="C:/Keil_v5/ARM/ARMCLANG/bin/armclang"
ASM="C:/Keil_v5/ARM/ARMCLANG/bin/armasm"
LD="C:/Keil_v5/ARM/ARMCLANG/bin/armlink"
HEX="C:/Keil_v5/ARM/ARMCLANG/bin/fromelf"
UV4="C:/Keil_v5/UV4/UV4.exe"
OBJ_DIR="MDK-ARM/WheelRobot"
OUT_DIR="MDK-ARM"
PROJ="WheelRobot"

# ── Compiler flags (必须保持 -fshort-wchar -fshort-enums 以匹配 UV4 项目设置) ──
CFLAGS_COMMON=(
  --target=arm-arm-none-eabi
  -mcpu=cortex-m4
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
  -fshort-wchar
  -fshort-enums
  -DUSE_HAL_DRIVER
  -DSTM32F407xx
  -c -O2 -g
)

INCLUDES=(
  -ICore/Inc
  -IDrivers/STM32F4xx_HAL_Driver/Inc
  -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy
  -IDrivers/CMSIS/Device/ST/STM32F4xx/Include
  -IDrivers/CMSIS/Include
  -IMiddlewares/Third_Party/FreeRTOS/Source/include
  -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
  -IMiddlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F
  -IMotor_Drivers/M0601C_DRIVE/Core/Inc
  -IMotor_Drivers/EL05_MOTOR_DRIVE/Core/Inc
)

# ── Source files ──────────────────────────────────────────────────────────────
C_SOURCES=(
  "Core/Src/main.c"
  "Core/Src/freertos.c"
  "Core/Src/gpio.c"
  "Core/Src/spi.c"
  "Core/Src/can.c"
  "Core/Src/stm32f4xx_it.c"
  "Core/Src/stm32f4xx_hal_msp.c"
  "Core/Src/stm32f4xx_hal_timebase_tim.c"
  "Core/Src/icm42688.c"
  "Core/Src/nrf24l01_rx.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c"
  "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c"
  "Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c"
  "Middlewares/Third_Party/FreeRTOS/Source/croutine.c"
  "Middlewares/Third_Party/FreeRTOS/Source/event_groups.c"
  "Middlewares/Third_Party/FreeRTOS/Source/list.c"
  "Middlewares/Third_Party/FreeRTOS/Source/queue.c"
  "Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c"
  "Middlewares/Third_Party/FreeRTOS/Source/tasks.c"
  "Middlewares/Third_Party/FreeRTOS/Source/timers.c"
  "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c"
  "Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c"
  "Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
  "Motor_Drivers/M0601C_DRIVE/Core/Src/motor_driver.c"
  "Motor_Drivers/EL05_MOTOR_DRIVE/Core/Src/el05_motor.c"
  "Motor_Drivers/EL05_MOTOR_DRIVE/Core/Src/can_rx_handler.c"
)

S_SOURCES=(
  "MDK-ARM/startup_stm32f407xx.s"
)

# ── Functions ─────────────────────────────────────────────────────────────────

clean() {
  echo "=== Cleaning ==="
  rm -f "$OBJ_DIR"/*.o "$OBJ_DIR"/*.axf "$OBJ_DIR"/*.hex "$OBJ_DIR"/*.bin "$OBJ_DIR"/*.map
  echo "Done"
}

build() {
  echo "=== Building $PROJ ==="

  mkdir -p "$OBJ_DIR"

  # 1. Assemble .s files
  for s in "${S_SOURCES[@]}"; do
    base=$(basename "$s" .s)
    echo "  ASM $base"
    "$ASM" --cpu=Cortex-M4.fp.sp \
      --pd="__UVISION_VERSION SETA 543" \
      --pd="STM32F407xx SETA 1" \
      --pd="_RTE_ SETA 1" \
      -o "$OBJ_DIR/${base}.o" "$s"
  done

  # 2. Compile .c files
  for c in "${C_SOURCES[@]}"; do
    base=$(basename "$c" .c)
    # Check if source is newer than object
    if [ -f "$OBJ_DIR/${base}.o" ] && [ "$OBJ_DIR/${base}.o" -nt "$c" ]; then
      continue  # skip up-to-date files
    fi
    echo "  CC  $base"
    "$CC" "${CFLAGS_COMMON[@]}" "${INCLUDES[@]}" -o "$OBJ_DIR/${base}.o" "$c"
  done

  # 3. Link
  echo "  LINK"
  "$LD" --cpu=Cortex-M4.fp.sp --entry=Reset_Handler \
    --scatter="$OBJ_DIR/$PROJ.sct" --library_type=standardlib \
    --output="$OBJ_DIR/$PROJ.axf" \
    "$OBJ_DIR"/*.o

  # 4. Generate .hex
  echo "  HEX"
  "$HEX" --i32combined --output="$OBJ_DIR/$PROJ.hex" "$OBJ_DIR/$PROJ.axf"

  echo "=== Build OK ==="
  echo "  AXF: $OBJ_DIR/$PROJ.axf"
  echo "  HEX: $OBJ_DIR/$PROJ.hex"
}

flash() {
  build
  echo "=== Flashing ==="
  # Kill any lingering UV4 debugger
  taskkill //F //IM UV4.exe 2>/dev/null || true
  sleep 1
  "$UV4" -f "$OUT_DIR/$PROJ.uvprojx"
  echo "=== Flash done ==="
}

case "${1:-build}" in
  clean)  clean ;;
  build)  build ;;
  flash)  flash ;;
  *)
    echo "Usage: $0 [build|flash|clean]"
    exit 1
    ;;
esac
