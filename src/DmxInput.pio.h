// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// -------- //
// DmxInput //
// -------- //

#define DmxInput_wrap_target 4
#define DmxInput_wrap 9

static const uint16_t DmxInput_program_instructions[] = {
    0xe03d, //  0: set    x, 29                      
    0x00c0, //  1: jmp    pin, 0                     
    0x0141, //  2: jmp    x--, 1                 [1] 
    0x20a0, //  3: wait   1 pin, 0                   
            //     .wrap_target
    0x2020, //  4: wait   0 pin, 0                   
    0xe427, //  5: set    x, 7                   [4] 
    0x4001, //  6: in     pins, 1                    
    0x0246, //  7: jmp    x--, 6                 [2] 
    0x20a0, //  8: wait   1 pin, 0                   
    0x8060, //  9: push   iffull block               
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program DmxInput_program = {
    .instructions = DmxInput_program_instructions,
    .length = 10,
    .origin = -1,
};

static inline pio_sm_config DmxInput_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + DmxInput_wrap_target, offset + DmxInput_wrap);
    return c;
}
#endif

