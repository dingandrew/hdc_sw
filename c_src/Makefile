# Copyright TU Wien
# Licensed under the Solderpad Hardware License v2.1, see LICENSE.txt for details
# SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1


# Generic Makefile for RV32IMV applications
# Specify program name with PROG, object files with OBJ

SW_DIR      := ../vicuna/sw
PROG        := test-vicuna
OBJ         := test_main.o encoder.o onlinehd.o spatial.o fasthd.o

RISCV_CC    := riscv32-unknown-elf-gcc
RISCV_DUMP  := riscv32-unknown-elf-objdump
RISCV_OBCP  := riscv32-unknown-elf-objcopy
ARCH        := rv32imv
LD_SCRIPT   := $(SW_DIR)/link.ld

RISCV_FLAGS := -march=$(ARCH) -mabi=ilp32 -static -mcmodel=medany             \
               -fvisibility=hidden -nostartfiles -Wall                        \
               $(CC_SPECIFIC_OPTIONS)

CFLAGS := $(CFLAGS) -I$(SW_DIR)/lib/ -I/opt/riscv/riscv32-unknown-elf/include -lm

# add crt0 to object files
RISCV_OBJ := $(SW_DIR)/crt0.o $(OBJ) $(SW_DIR)/lib/uart.o

all: $(PROG).vmem

encoder.o: src/encoder.c
	$(RISCV_CC) $(RICSV_FLAGS) $(CFLAGS) -c -o $@ $<
fasthd.o: src/fasthd.c
	$(RISCV_CC) $(RICSV_FLAGS) $(CFLAGS) -c -o $@ $<

onlinehd.o: src/onlinehd.c
	$(RISCV_CC) $(RICSV_FLAGS) $(CFLAGS) -c -o $@ $<

spatial.o: src/spatial.c
	$(RISCV_CC) $(RICSV_FLAGS) $(CFLAGS) -c -o $@ $<

test_main.o: src/test_main.c
	$(RISCV_CC) $(RICSV_FLAGS) $(CFLAGS) -c -o $@ $<

dump: $(PROG).elf
	$(RISCV_DUMP) -D $<

$(PROG).elf: $(RISCV_OBJ) $(LD_SCRIPT)
	$(RISCV_CC) $(RISCV_FLAGS) -T $(LD_SCRIPT) $(LDFLAGS) $(RISCV_OBJ) -o $@ -lm

# currently unusable due to problems with byte order
# (see https://github.com/riscv/riscv-tools/issues/168#issuecomment-554973539)
#%.vmem: %.elf
#	$(OBJCOPY) -O verilog --verilog-data-width 4 $^ $@

# workaround (requires srecord):
# note: start address must be reset manually because it is lost in bin file
%.vmem: %.bin
	srec_cat $^ -binary -offset 0x0000 -byte-swap 4 -o $@ -vmem
%.bin: %.elf
	$(RISCV_OBCP) -O binary $^ $@

clean:
	rm -f *.o *.elf *.bin *.vmem

include $(SW_DIR)/toolchain.mk

sim: $(PROG).vmem
	echo $(CURDIR)/$(PROG).vmem > progs.txt
	mkdir -p $$PWD/sim.out
	make -f $(CURDIR)/../vicuna/sim/Makefile \
		questa \
		PROJ_DIR=$$PWD/sim.out
		PROG_PATHS=progs.txt \
		VPROC_CONFIG=dual \
	    VREG_W=512 \
