#ifndef _PROTOCOL_DUAL_DC_MOTOR_DRIVER_H
#define _PROTOCOL_DUAL_DC_MOTOR_DRIVER_H

#include "protocol.h"

struct dual_motor_driver_opcodes
{
        enum {
                OP_SET_SPEED = crisis_header::END_RESERVED + 1,
                OP_SET_ENCODER,
        };
};

/*
 * The speed field is the setpoint of the PID controller in pulses
 * per second.
 * A negative speed means backward rotation.
 *
 * The flags field is a bitset.
 */

struct dual_motor_driver_speed
{
        int16_t speed[2];
        uint8_t flags;

        enum {
                FLAG_BRAKE_0 = (1 << 0),
                FLAG_BRAKE_1 = (1 << 1),
        };
} PACKED;

/*
 * The speed field is the most recent measured frequency of the encoder.
 */

struct dual_motor_driver_encoder
{
        int16_t speed[2];
} PACKED;

#endif /* protocol_dual_dc_motor_driver.h */
