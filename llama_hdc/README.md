# HDC llama2.c - Binary Build Guide

---

## Building the executable

Several build targets are supported:
- run
- run289
- riscv
- riscv289
- stable


### "HDC" Targets

The `run` build targets are for building x86 compatible executables, while the `riscv` build targets are for building riscv32 compatible executables. Both of these use the `run_hdc.c` file, which is written to be able to run on baremetal hardware.

The default model used is ~1MB with a 512 vocab size tokenizer.

The `289` targets use a ~350KB model with a 289 vocab size tokenizer.


### Extra Targets

Lastly, the `stable` target is to build the original run executable. This is mostly for testing or comparison, uses the ~1MB model  with a 512 vocab size tokenizer.


---

## Other Notes

`linker_riscv.ld` is subject to change.