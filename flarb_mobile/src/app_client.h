#ifndef _APP_CLIENT_H
#define _APP_CLIENT_H

#include "finch/net/socket.h"

#include <string>

#include "commands.pb.h"

class server;

class client: public finch::net::socket
{
    public:
        client(server* serv): _serv(serv) {}
        virtual ~client() {}

        void read_event();

        void parse_command(const nl::flarb::crisis::communication::Command& cmd);
        void parse_program(const nl::flarb::crisis::communication::Program& pgm);

    private:
        server* _serv;
};

#endif /* app_client.h */
