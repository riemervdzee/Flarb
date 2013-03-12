#include "meas.h"

#include <iostream>
#include <string>
#include <cstring>

#include <fcntl.h>
#include <iomanip>

#include <ros/ros.h>



#define abs(a) \
	(((a) < 0) ? -(a) : (a))

meas::meas(const std::string& sfile): sfile(sfile)
{
	set_event_mask(UFILE_EVENT_READ);
	
	int fd = ::open(sfile.c_str(), O_RDONLY | O_NOCTTY);
	set_fd(fd);

	buflen = 0;
	bufpos = 0;
}

void meas::data_available()
{
	size_t avail = bytes_available();

	int16_t* axisp;
	float floatreader;
	
	char buffer[avail+1];
	read(buffer, avail);
	buffer[avail] = 0;

	push_data(buffer, avail);

//	std::cout << "Read '" << buffer << "'" << std::endl;

	while(buflen >= 9) {
		char axis_ind;
		char sign;

		pop_data(&axis_ind, 1);

		if(axis_ind == 'X') {
			axisp = &axis[0];
		} else if(axis_ind == 'Y') {
			axisp = &axis[1];
		} else {
			tidy_buffer();
			continue;
		}

		pop_data(&sign, 1); // '='
		pop_data(&sign, 1);

		char digits[8];
		for(int i = 0; i < 7; i++) {
			pop_data(&digits[i], 1);
			if(!isdigit(digits[i]) && digits[i] != '.') {
				digits[i] = '\0';
				break;
			}
		}

		std::stringstream conv;
		conv.write(digits, strlen(digits));
		conv >> floatreader;

		floatreader *= ((sign == '-') ? -1000.0f : 1000.0f);

		tidy_buffer();

//		std::cout << "Delta: " << abs(*axisp - floatreader) << std::endl;
		if(abs(*axisp - floatreader) > MEAS_THRESHOLD) {
			*axisp = floatreader;
			std::cout << "Gyro: " << axis[0] << ", " << axis[1] << std::endl;
			client::incoming_gyro_event(0, axis);
		}
	}
}




void meas::push_data(char* data, size_t len)
{
	if(buflen + len > MEAS_BUFFER_SIZE) {
		std::cout << "Buffer overflow while writing" << std::endl;
		return;
	}

	memcpy(&buffer[buflen], data, len);
	buflen += len;
}

void meas::pop_data(char* data, size_t len)
{
	if(bufpos + len > MEAS_BUFFER_SIZE) {
		std::cout << "Buffer overflow while reading" << std::endl;
		return;
	}

	memcpy(data, &buffer[bufpos], len);
	bufpos += len;
	buflen -= len;
}

void meas::tidy_buffer()
{
	memcpy(buffer, &buffer[bufpos], buflen);
	bufpos = 0;
}
