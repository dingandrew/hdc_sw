
.PHONY: run
run: run_hdc.c model_bin tokenizer_bin
	gcc -O3 -o run run_hdc.c model_data.o tokenizer_data.o -T linker.ld -lm -DTOK512

.PHONY: run289
run289: run_hdc.c model_bin289 tokenizer_bin289
	gcc -O3 -o run run_hdc.c model_data.o tokenizer_data.o -T linker.ld -lm -DTOK289


.PHONY: riscv
riscv: run_hdc.c model_bin_riscv tokenizer_bin_riscv
	clang -O -c run_hdc.c --target=riscv32 -march=rv32gcv -mabi=ilp32f -O3 \
		--sysroot=/home/joeybalardeta/Research/riscv/_install/riscv64-unknown-elf -DTOK512
	riscv64-unknown-elf-gcc -o run run_hdc.o model_data.o tokenizer_data.o \
		-T linker_riscv.ld -lm -march=rv32gcv -mabi=ilp32f

.PHONY: riscv289
riscv289: run_hdc.c model_bin_riscv289 tokenizer_bin_riscv289
	clang -O -c run_hdc.c --target=riscv32 -march=rv32gcv -mabi=ilp32f -O3 \
		--sysroot=/home/joeybalardeta/Research/riscv/_install/riscv64-unknown-elf -DTOK289
	riscv64-unknown-elf-gcc -o run run_hdc.o model_data.o tokenizer_data.o \
		-T linker_riscv.ld -lm -march=rv32gcv -mabi=ilp32f


.PHONY: stable
stable: run.c
	gcc -O3 -o run run.c -lm


.PHONY: model_bin
model_bin: stories260K.bin
	objcopy -I binary -O elf64-x86-64 -B i386 stories260K.bin model_data.o \
    	--rename-section .data=.model_data

.PHONY: tokenizer_bin
tokenizer_bin: tok512.bin
	objcopy -I binary -O elf64-x86-64 -B i386 tok512.bin tokenizer_data.o \
        --rename-section .data=.tokenizer_data

.PHONY: model_bin289
model_bin289: stories289v.bin
	objcopy -I binary -O elf64-x86-64 -B i386 stories289v.bin model_data.o \
    	--rename-section .data=.model_data

.PHONY: tokenizer_bin
tokenizer_bin289: tok289.bin
	objcopy -I binary -O elf64-x86-64 -B i386 tok289.bin tokenizer_data.o \
        --rename-section .data=.tokenizer_data


.PHONY: model_bin_riscv
model_bin_riscv: stories260K.bin
	riscv64-unknown-elf-objcopy -I binary -O elf32-littleriscv -B riscv stories260K.bin model_data.o \
    	--rename-section .data=.model_data

.PHONY: tokenizer_bin_riscv
tokenizer_bin_riscv: tok512.bin
	riscv64-unknown-elf-objcopy -I binary -O elf32-littleriscv -B riscv tok512.bin tokenizer_data.o \
        --rename-section .data=.tokenizer_data


.PHONY: model_bin_riscv289
model_bin_riscv289: stories260K.bin
	riscv64-unknown-elf-objcopy -I binary -O elf32-littleriscv -B riscv stories289v.bin model_data.o \
    	--rename-section .data=.model_data

.PHONY: tokenizer_bin_riscv289
tokenizer_bin_riscv289: tok512.bin
	riscv64-unknown-elf-objcopy -I binary -O elf32-littleriscv -B riscv tok289.bin tokenizer_data.o \
        --rename-section .data=.tokenizer_data


.PHONY: clean
clean:
	rm -f run
	rm -f *.o
