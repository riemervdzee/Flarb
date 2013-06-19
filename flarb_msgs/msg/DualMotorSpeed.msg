##
# The speed field is the setpoint of the PID controller in pulses
# per second.
# A negative speed means backward rotation.
#
# The flags field is a bitset following
#	enum {
#        FLAG_BRAKE_0 = (1 << 0),
#        FLAG_BRAKE_1 = (1 << 1),
#	};
##

int16 speed_left
int16 speed_right

uint8 flags
