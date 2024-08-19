cmd_src/utils/src/tdoa/tdoaEngine.o := arm-none-eabi-gcc -Wp,-MD,src/utils/src/tdoa/.tdoaEngine.o.d    -I/home/bitcraze/projects/crazyflie-firmware/src/utils/src -Isrc/utils/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/bitcraze/projects/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/bitcraze/projects/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/bitcraze/projects/crazyflie-firmware/vendor/libdw1000/inc   -I/home/bitcraze/projects/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/bitcraze/projects/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/bitcraze/projects/crazyflie-firmware/src/config   -I/home/bitcraze/projects/crazyflie-firmware/src/platform/interface   -I/home/bitcraze/projects/crazyflie-firmware/src/deck/interface   -I/home/bitcraze/projects/crazyflie-firmware/src/deck/drivers/interface   -I/home/bitcraze/projects/crazyflie-firmware/src/drivers/interface   -I/home/bitcraze/projects/crazyflie-firmware/src/drivers/bosch/interface   -I/home/bitcraze/projects/crazyflie-firmware/src/drivers/esp32/interface   -I/home/bitcraze/projects/crazyflie-firmware/src/hal/interface   -I/home/bitcraze/projects/crazyflie-firmware/src/modules/interface   -I/home/bitcraze/projects/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/bitcraze/projects/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/bitcraze/projects/crazyflie-firmware/src/utils/interface   -I/home/bitcraze/projects/crazyflie-firmware/src/utils/interface/kve   -I/home/bitcraze/projects/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/bitcraze/projects/crazyflie-firmware/src/utils/interface/tdoa   -I/home/bitcraze/projects/crazyflie-firmware/src/lib/FatFS   -I/home/bitcraze/projects/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/bitcraze/projects/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/bitcraze/projects/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/bitcraze/projects/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/bitcraze/projects/crazyflie-firmware/src/lib/vl53l1   -I/home/bitcraze/projects/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/bitcraze/projects/crazyflie-firmware/examples/demos/demos/daizy_chain/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -c -o src/utils/src/tdoa/tdoaEngine.o /home/bitcraze/projects/crazyflie-firmware/src/utils/src/tdoa/tdoaEngine.c

source_src/utils/src/tdoa/tdoaEngine.o := /home/bitcraze/projects/crazyflie-firmware/src/utils/src/tdoa/tdoaEngine.c

deps_src/utils/src/tdoa/tdoaEngine.o := \
  /usr/include/newlib/string.h \
  /usr/include/newlib/_ansi.h \
  /usr/include/newlib/newlib.h \
  /usr/include/newlib/_newlib_version.h \
  /usr/include/newlib/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/include/newlib/machine/ieeefp.h \
  /usr/include/newlib/sys/features.h \
  /usr/include/newlib/sys/reent.h \
  /usr/include/newlib/_ansi.h \
  /usr/lib/gcc/arm-none-eabi/9.2.1/include/stddef.h \
  /usr/include/newlib/sys/_types.h \
  /usr/include/newlib/machine/_types.h \
  /usr/include/newlib/machine/_default_types.h \
  /usr/include/newlib/sys/lock.h \
  /usr/include/newlib/sys/cdefs.h \
  /usr/include/newlib/sys/_locale.h \
  /usr/include/newlib/strings.h \
  /usr/include/newlib/sys/string.h \
  /home/bitcraze/projects/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /home/bitcraze/projects/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/bitcraze/projects/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /usr/lib/gcc/arm-none-eabi/9.2.1/include/stdbool.h \
  /home/bitcraze/projects/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/bitcraze/projects/crazyflie-firmware/src/config/trace.h \
  /home/bitcraze/projects/crazyflie-firmware/src/hal/interface/usec_time.h \
  /usr/lib/gcc/arm-none-eabi/9.2.1/include/stdint.h \
  /home/bitcraze/projects/crazyflie-firmware/src/modules/interface/console.h \
  /home/bitcraze/projects/crazyflie-firmware/src/utils/interface/eprintf.h \
  /usr/lib/gcc/arm-none-eabi/9.2.1/include/stdarg.h \
  /home/bitcraze/projects/crazyflie-firmware/src/utils/interface/tdoa/tdoaEngine.h \
  /home/bitcraze/projects/crazyflie-firmware/src/utils/interface/tdoa/tdoaStorage.h \
  /home/bitcraze/projects/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/bitcraze/projects/crazyflie-firmware/src/hal/interface/imu_types.h \
  /home/bitcraze/projects/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /home/bitcraze/projects/crazyflie-firmware/src/utils/interface/clockCorrectionEngine.h \
  /home/bitcraze/projects/crazyflie-firmware/src/utils/interface/tdoa/tdoaStats.h \
  /usr/include/newlib/inttypes.h \
  /usr/include/newlib/sys/_intsup.h \
  /home/bitcraze/projects/crazyflie-firmware/src/utils/interface/statsCnt.h \
  /home/bitcraze/projects/crazyflie-firmware/src/modules/interface/log.h \
  /home/bitcraze/projects/crazyflie-firmware/src/utils/interface/tdoa/tdoaStats.h \
  /home/bitcraze/projects/crazyflie-firmware/src/utils/interface/physicalConstants.h \

src/utils/src/tdoa/tdoaEngine.o: $(deps_src/utils/src/tdoa/tdoaEngine.o)

$(deps_src/utils/src/tdoa/tdoaEngine.o):
