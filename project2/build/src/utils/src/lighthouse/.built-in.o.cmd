cmd_src/utils/src/lighthouse/built-in.o :=  arm-none-eabi-gcc --specs=nosys.specs --specs=nano.specs -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -nostdlib   -r -o src/utils/src/lighthouse/built-in.o src/utils/src/lighthouse/lighthouse_calibration.o src/utils/src/lighthouse/lighthouse_geometry.o src/utils/src/lighthouse/ootx_decoder.o src/utils/src/lighthouse/pulse_processor.o src/utils/src/lighthouse/pulse_processor_v1.o src/utils/src/lighthouse/pulse_processor_v2.o
