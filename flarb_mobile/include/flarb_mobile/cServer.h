#ifndef CLASS_SERVER_H
#define CLASS_SERVER_H


class cServer{
	public:
		cServer() {};
		int sendData(char* D);
		int Port;
		int createSocket(int port);
		int readData(char* D);
		int closeSocket();
	private:
		int sockfd, newsockfd, clilen;

		
};
#endif
