#ifndef _APP_SERVER_H
#define _APP_SERVER_H

#include "finch/net/server.h"

#include <memory>
#include <list>
#include <algorithm>

#include "app_client.h"

struct Direction
{
    enum {
        Left,
        Right
    };
};


class server: public finch::net::server
{
    public:
        server()
        {
            bind(1337);
        }
        virtual ~server() {}

        void client_accepted(finch::net::socket* c)
        {
            clients.push_back(static_cast<client*>(c));
        }

        finch::net::socket* new_client_handler()
        {
            return new client(this);
        }

	void drop_client(client* c)
	{
		clients.erase(std::find_if(clients.begin(), clients.end(), [c] (client* c_l) -> bool {return c_l == c;}));
	}

        virtual void tour(int first_direction) = 0;
        virtual void navigate(const std::string& directions) = 0;
        virtual void find_damaged(int first_direction) = 0;

    private:
        std::list<client*> clients;
};
#endif /* app_server.h */ 
