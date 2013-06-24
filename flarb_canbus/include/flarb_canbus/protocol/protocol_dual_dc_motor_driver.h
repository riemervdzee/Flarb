#ifndef _PROTOCOL_DUAL_DC_MOTOR_DRIVER_H
#define _PROTOCOL_DUAL_DC_MOTOR_DRIVER_H

#include "protocol.h"

struct dual_motor_driver_opcodes
{
        enum {
                OP_SET_SPEED = crisis_header::END_RESERVED + 1,
                OP_STATUS,
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
                FLAG_BRAKE_LEFT = (1 << 0),
                FLAG_BRAKE_RIGHT = (1 << 1),
        };
} PACKED;

struct dual_motor_driver_status
{
        int16_t speed[2];
        uint8_t error;
} PACKED;

#endif /* protocol_dual_dc_motor_driver.h */
