#include <stdio.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <strings.h>
#include "flarb_mobile/cServer.h"
#include <unistd.h>


/*	
 *	Create Socket listen for one clients
 * 	0 == oke everything else is BAD 
 */
int cServer::createSocket(int port)
{
	Port = port;
    struct sockaddr_in serv_addr, cli_addr;

    // First call to socket() function 
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
    {
        perror("ERROR opening socket");
		return -1;        
		
    }
    // Initialize socket structure 
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(port);
 
    // Now bind the host address using bind() call.
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
                          sizeof(serv_addr)) < 0)
    {
     	perror("ERROR on binding");
     	return -1;        
	
    }
	
	// Now start listening for the clients, here process will
    // go in sleep mode and will wait for the incoming connection
    
    listen(sockfd,5);
    clilen = sizeof(cli_addr);

    // Accept actual connection from the client 
    newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, (socklen_t*)&clilen);
    if (newsockfd < 0) 
    {
        perror("ERROR on accept");
        return -1;        	
    }
	return 0;
}

/*
 *	Close Socket
 */
int cServer::closeSocket()
{
	close(newsockfd);
	return 0;
}

/*
 * 	Read data from socket
 * 	0 == oke everything else is BAD 
 */
int cServer::readData(char* D )
{
    int n = read( newsockfd,D,255 );
    if (n < 0)    {	perror("ERROR reading from socket"); return n;	}
  	return 0;
}

/*
 * 	Send data to client
 * 	0 == oke everything else is bad
 */
int cServer::sendData(char* D )
{
    // Write a response to the client
    int n = write(newsockfd,D,sizeof(D));
    if (n < 0)    {	perror("ERROR writing to socket");	return n;	}
    return 0; 
}


