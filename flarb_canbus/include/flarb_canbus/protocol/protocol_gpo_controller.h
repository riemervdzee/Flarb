#ifndef _GPO_CONTROLLER_H
#define _GPO_CONTROLLER_H

#include "protocol.h"

struct gpo_opcodes
{
        enum {
                OP_SET_OUTPUTS = crisis_header::END_RESERVED + 1,
        };
};

struct set_outputs
{
        uint8_t bits;
} PACKED;

#endif /* gpo_controller.h */
