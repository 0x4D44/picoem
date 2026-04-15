// OneROM PIO differential — C shim public interface.
//
// The Rust trace generator binary calls these functions. Implementation in
// trace_gen_core.c.
//
// See wrk_docs/2026.04.14 - HLD - OneROM PIO Differential.md §6.4.

#ifndef EPIO_SYS_TRACE_GEN_CORE_H
#define EPIO_SYS_TRACE_GEN_CORE_H

#include <stdint.h>

typedef struct trace_gen_ctx trace_gen_ctx;

// Build apio state from setup_onerom, call epio_from_apio, configure the
// OneROM DMA chain, pre-populate SRAM with a deterministic pattern
// (sram[i] = i & 0xFF), reset the cycle counter. Returns NULL on allocation
// failure.
trace_gen_ctx *trace_gen_init(void);

// Free the epio instance and the context. Safe to call with NULL.
void trace_gen_free(trace_gen_ctx *ctx);

// Copy up to `buf_len` instructions from the given PIO block's instr_mem
// into `buf`. Returns the number of instructions written (<= 32).
uint32_t trace_gen_dump_instr_mem(
    trace_gen_ctx *ctx,
    uint8_t block,
    uint16_t *buf,
    uint32_t buf_len
);

// Read back an SM's four config registers.
void trace_gen_dump_sm_reg(
    trace_gen_ctx *ctx,
    uint8_t block,
    uint8_t sm,
    uint32_t *clkdiv,
    uint32_t *execctrl,
    uint32_t *shiftctrl,
    uint32_t *pinctrl
);

// Drive the input-pin state (`input_drive` = which pins the host is
// driving; `input_level` = their levels), step exactly one cycle, read
// back the pin state. `out_drive` returns which pins are currently being
// driven by PIO / host; `out_level` returns their levels. Both are
// truncated to the low 32 bits — our diff side's PIO uses u32 pin masks.
void trace_gen_step(
    trace_gen_ctx *ctx,
    uint32_t input_drive,
    uint32_t input_level,
    uint32_t *out_drive,
    uint32_t *out_level
);

#endif // EPIO_SYS_TRACE_GEN_CORE_H
