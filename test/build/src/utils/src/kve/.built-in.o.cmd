cmd_src/utils/src/kve/built-in.o :=  arm-none-eabi-gcc --specs=nosys.specs --specs=nano.specs -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -nostdlib   -r -o src/utils/src/kve/built-in.o src/utils/src/kve/kve.o src/utils/src/kve/kve_storage.o
