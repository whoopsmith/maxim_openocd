TARGET=max32xxx_write
ENTRY=algo_write

BIN2C = ../../../../src/helper/bin2char.sh

all: max32xxx_write.inc

%.o: %.c
	arm-none-eabi-gcc -mthumb -mcpu=cortex-m4 -Wa,-mimplicit-it=thumb -Os -Wall -c ${<} -o ${@}

%.elf: %.o
	arm-none-eabi-ld -nostdlib -nostartfiles --entry $(ENTRY) ${<} -o ${@}
	arm-none-eabi-size ${@}

%.bin: %.elf
	arm-none-eabi-objcopy -O binary ${<} ${@}

%.inc: %.bin
	$(BIN2C) < $< > $@

%.dasm: %.o
	arm-none-eabi-objdump -d -s ${<} > ${TARGET}.dasm

clean:
	rm -rf $(TARGET).bin $(TARGET).elf $(TARGET).o $(TARGET).dasm $(TARGET).inc

