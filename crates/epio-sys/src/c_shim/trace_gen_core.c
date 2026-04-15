// OneROM PIO differential — C shim implementation.
//
// Bridges Rust to apio + epio. setup_onerom() is lifted verbatim from
// epio/test/onerom_programs.h (just the `#include "test.h"` removed —
// that file drags in cmocka which we don't need). APIO_LOG_ENABLE is
// not defined, so APIO_LOG / APIO_LOG_SM expand to no-ops.
//
// See wrk_docs/2026.04.14 - HLD - OneROM PIO Differential.md §6.4.

#include <stdint.h>
#include <stdlib.h>

#include <apio.h>
#include <epio.h>

#include "trace_gen_core.h"

// --- setup_onerom (verbatim from third_party/epio/test/onerom_programs.h) ---
//
// The only change is the removal of `#include "test.h"`. The `void **state`
// parameter is kept so the function body is byte-identical; it is unused.
// `APIO_ASM_WFI()` in emulation mode (apio.h:828) expands to `return 0`,
// so the trailing `while (1) { APIO_ASM_WFI(); }` exits on the first
// iteration.

static int setup_onerom(void **state) {
    (void)state;

    APIO_GPIO_INIT();
    for (int ii = 0; ii < 8; ii++) {
        APIO_GPIO_OUTPUT(ii, 0);  // Data GPIOs 0-7 controlled by PIO block 0
    }

    APIO_ASM_INIT();
    APIO_CLEAR_ALL_IRQS();
    APIO_SET_BLOCK(0);

    // SM0 - CS handler
    APIO_SET_SM(0);

    APIO_ADD_INSTR(APIO_MOV_PINDIRS_NULL);
    APIO_LABEL_NEW(load_cs);
    APIO_ADD_INSTR(APIO_MOV_X_PINS);
    APIO_ADD_INSTR(APIO_JMP_X_DEC(APIO_LABEL(load_cs)));
    APIO_ADD_INSTR(APIO_MOV_PINDIRS_NOT_NULL);
    APIO_LABEL_NEW(check_cs_gone_inactive);
    APIO_ADD_INSTR(APIO_MOV_X_PINS);
    APIO_WRAP_TOP();
    APIO_ADD_INSTR(APIO_JMP_NOT_X(APIO_LABEL(check_cs_gone_inactive)));

    APIO_SM_CLKDIV_SET(1, 0);
    APIO_SM_EXECCTRL_SET(0);
    APIO_SM_SHIFTCTRL_SET(
        APIO_IN_COUNT(1) |
        APIO_IN_SHIFTDIR_L
    );
    APIO_SM_PINCTRL_SET(
        APIO_OUT_COUNT(8) |
        APIO_OUT_BASE(0) |
        APIO_IN_BASE(8)
    );

    APIO_SM_JMP_TO_START();
    APIO_LOG_SM("CS Handler");

    // SM1 - Address reader
    APIO_SET_SM(1);

    APIO_ADD_INSTR(APIO_ADD_DELAY(APIO_IN_X(16), 2));
    APIO_WRAP_TOP();
    APIO_ADD_INSTR(APIO_IN_PINS(16));

    APIO_SM_CLKDIV_SET(1, 0);
    APIO_SM_EXECCTRL_SET(0);
    APIO_SM_SHIFTCTRL_SET(
        APIO_IN_COUNT(16) |
        APIO_AUTOPUSH |
        APIO_PUSH_THRESH(32) |
        APIO_IN_SHIFTDIR_L |
        APIO_OUT_SHIFTDIR_L
    );
    APIO_SM_PINCTRL_SET(
        APIO_IN_BASE(8)
    );

    APIO_TXF = 0x00002000;
    APIO_SM_EXEC_INSTR(APIO_PULL_BLOCK);
    APIO_SM_EXEC_INSTR(APIO_MOV_X_OSR);

    APIO_SM_JMP_TO_START();
    APIO_LOG_SM("Address Reader");

    // SM2 - Data byte output
    APIO_SET_SM(2);
    APIO_ADD_INSTR(APIO_OUT_PINS(8));

    APIO_SM_CLKDIV_SET(1, 0);
    APIO_SM_EXECCTRL_SET(0);
    APIO_SM_SHIFTCTRL_SET(
        APIO_OUT_SHIFTDIR_R |
        APIO_AUTOPULL |
        APIO_PULL_THRESH(8)
    );
    APIO_SM_PINCTRL_SET(
        APIO_OUT_BASE(0) |
        APIO_OUT_COUNT(8)
    );

    APIO_SM_JMP_TO_START();
    APIO_LOG_SM("Data Byte Output");

    APIO_END_BLOCK();

    APIO_ENABLE_SMS(0, (1 << 0) | (1 << 1) | (1 << 2));

    while (1) {
        APIO_ASM_WFI();
    }
}

// --- shim implementation ---------------------------------------------------

struct trace_gen_ctx {
    epio_t *epio;
};

trace_gen_ctx *trace_gen_init(void) {
    trace_gen_ctx *ctx = (trace_gen_ctx *)calloc(1, sizeof(*ctx));
    if (!ctx) return NULL;

    // Populate apio globals from the verbatim OneROM setup.
    (void)setup_onerom(NULL);

    // Transfer apio state into a fresh epio instance. This also replays
    // any APIO_SM_EXEC_INSTR pre-init instructions (PULL BLOCK + MOV X, OSR
    // in setup_onerom) via epio's internal exec path.
    ctx->epio = epio_from_apio();
    if (!ctx->epio) {
        free(ctx);
        return NULL;
    }

    // OneROM's DMA chain — parameters taken directly from
    // epio/test/onerom.c::test_onerom_program:
    //   dma_chan=0, read_block=0, read_sm=1, read_cycles=4,
    //   write_block=0, write_sm=2, write_cycles=4, bit_mode=8
    epio_dma_setup_read_pio_chain(ctx->epio, 0, 0, 1, 4, 0, 2, 4, 8);

    // SRAM is zero-initialised by `epio_init`. Any DMA-driven read hits
    // zero bytes, which is acceptable for the first trace — the diff
    // side will match the same zeros from our zero-initialised fake DMA.

    epio_reset_cycle_count(ctx->epio);
    return ctx;
}

void trace_gen_free(trace_gen_ctx *ctx) {
    if (!ctx) return;
    if (ctx->epio) epio_free(ctx->epio);
    free(ctx);
}

uint32_t trace_gen_dump_instr_mem(
    trace_gen_ctx *ctx,
    uint8_t block,
    uint16_t *buf,
    uint32_t buf_len
) {
    uint32_t n = buf_len < 32 ? buf_len : 32;
    for (uint32_t i = 0; i < n; i++) {
        buf[i] = epio_get_instr(ctx->epio, block, (uint8_t)i);
    }
    return n;
}

void trace_gen_dump_sm_reg(
    trace_gen_ctx *ctx,
    uint8_t block,
    uint8_t sm,
    uint32_t *clkdiv,
    uint32_t *execctrl,
    uint32_t *shiftctrl,
    uint32_t *pinctrl
) {
    epio_sm_reg_t reg;
    epio_get_sm_reg(ctx->epio, block, sm, &reg);
    *clkdiv = reg.clkdiv;
    *execctrl = reg.execctrl;
    *shiftctrl = reg.shiftctrl;
    *pinctrl = reg.pinctrl;
}

void trace_gen_step(
    trace_gen_ctx *ctx,
    uint32_t input_drive,
    uint32_t input_level,
    uint32_t *out_drive,
    uint32_t *out_level
) {
    // Drive the input pins (widen to u64 for the epio API — we use only
    // the low 32 bits).
    epio_drive_gpios_ext(ctx->epio, (uint64_t)input_drive, (uint64_t)input_level);

    // One cycle.
    epio_step_cycles(ctx->epio, 1);

    // Read back.
    *out_drive = (uint32_t)epio_read_driven_pins(ctx->epio);
    *out_level = (uint32_t)epio_read_pin_states(ctx->epio);
}
