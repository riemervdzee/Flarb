#include "ServerSocket.h"
#include "SocketException.h"
#include <string>
#include "flarb_mobile/cServer.h"

int main ( int argc, int argv[] )
{
  	std::cout << "Startup....\n";
	
	Startup(1337); 
	return 0;
}

void Cserver::Startup(int port){
	std::cout<<"
	try
    {
      // Create the socket
      server ( port );

      while ( true )
	{

	  ServerSocket new_sock;
	  server.accept ( new_sock );

	  try
	    {
	      while ( true )
		{
		  std::string data;
		  new_sock >> data;
		  new_sock << data;
		}
	    }
	  catch ( SocketException& ) {}

	}
    }
  catch ( SocketException& e )
    {
      std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }

}

