#include "app_server.h"

#include "app_client.h"
#include <iostream>

#include <cstdio>
using namespace nl::flarb::crisis::communication;

void client::read_event()
{
	try {
	    char hdr_data[5];
	    read(hdr_data, 5);
	    Header hdr;
	    hdr.ParseFromArray(hdr_data, 5);
	    size_t sz = hdr.size();

	    char msg_data[sz];
	    read(msg_data, sz);
	    Command cmd;
	    cmd.ParseFromArray(msg_data, sz);

	    parse_command(cmd);
	} catch(finch::net::socket::disconnected_exception& e) {
		_serv->drop_client(this);
	}
}

void client::parse_command(const Command& cmd)
{
    switch(cmd.type()) {
        case Command::StartProgram:
            parse_program(cmd.program());
            break;
    }
}

void client::parse_program(const Program& pgm)
{
    switch(pgm.type()) {
        case Program::Tour:
            _serv->tour((pgm.first_turn() == Program::Left) ? Direction::Left : Direction::Right);
            break;
        case Program::Navigate:
            _serv->navigate(pgm.directions());
            break;
        case Program::FindDamaged:
            _serv->find_damaged((pgm.first_turn() == Program::Left) ? Direction::Left : Direction::Right);
            break;
    }
}
