#ifndef _MEAS_H
#define _MEAS_H

#include <ufile.h>
#include <string>
#include <sstream>

#include <stdint.h>

#define MEAS_BUFFER_SIZE 32
#define MEAS_THRESHOLD 20

class meas: public ufile
{
	public:
		meas(const std::string& sfile);

	public:
		void data_available();

		std::string sfile;

		void push_data(char* data, size_t len);
		void pop_data(char* data, size_t len);
		void tidy_buffer();
		char buffer[MEAS_BUFFER_SIZE];
		int buflen;
		int bufpos;

		int16_t axis[2];
};

#endif /* meas.h */

