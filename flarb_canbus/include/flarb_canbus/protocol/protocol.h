/*
 * protocol.h
 *
 *  Created on: May 20, 2013
 *      Author: leon
 */

#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include <cstdint>

#define PACKED __attribute__((packed))

/*
 * C.R.I.S.I.S. CAN protocol
 *
 * - Byte order: Little Endian
 *
 *
 * Each frame consists of a maximum of 8 bytes.
 * The minimum frame size is 1 byte (sizeof(crisis_header)).
 * All opcodes < 64 are reserved for protocol use,
 * all opcodes > 64 are device specific opcodes.
 *
 *
 * The first action a device should take when it comes online
 * is to send a hello frame, indicating its own identifier
 * and device type.
 *
 */


struct crisis_header
{
        uint8_t opcode;

        enum opcode {
                OP_CRISIS_INVALID_FRAME = 1,
                OP_HELLO,

                END_RESERVED = 64
        };
} PACKED;

/*
 * General frame layouts:
 *
 *  ------------------------
 * | opcode == CRISIS_HELLO | 1 byte
 * |------------------------|
 * |   struct crisis_hello  | 1 byte
 *  ------------------------
 *
 *
 *
 *
 *  ------------------------
 * |       opcode > 64      | 1 byte
 * |------------------------
 * | Device specific frame  | n bytes
 * |                        |
 *
 */

struct crisis_hello
{
        uint8_t device_type;

        enum device_type
        {
                DUAL_DC_MOTOR_DRIVER = 1,
        };
} PACKED;



#endif /* _PROTOCOL_H */
