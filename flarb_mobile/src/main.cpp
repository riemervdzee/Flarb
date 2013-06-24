#include "finch/net/event_loop.h"
#include "app_server.h"

#include <iostream>

class my_server: public server
{
public:
	void tour(int first_direction)
	{
		std::cout << "Tour, first = " << first_direction << std::endl;
	}

	void navigate(const std::string& directions)
	{
		std::cout << "Navigate, directions = " << directions << std::endl;
	}

	void find_damaged(int first_direction)
	{
		std::cout << "Find damaged, first = " << first_direction << std::endl;
	}
};

int main(int argc, char* argv[])
{
    my_server serv;

    finch::net::event_loop ev;
    ev.run();
}

