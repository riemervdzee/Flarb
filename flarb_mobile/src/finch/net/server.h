#ifndef _FINCH_NET_SERVER_H
#define _FINCH_NET_SERVER_H

#include "../exception.h"

#include "socket.h"

namespace finch
{
    namespace net
    {
        class server: public finch::net::socket
        {
            public:
                server();
                virtual ~server();

                void bind(const int port) throw(finch::exception);

                virtual void client_accepted(finch::net::socket* client) = 0;
                virtual finch::net::socket* new_client_handler() = 0;

            protected:
                void read_event();

                void accept();

            private:
                int _port;
        };
    };
};

#endif /* finch/net/server.h */

