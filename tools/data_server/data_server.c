//Don't use this. 
// Use the following:
//   ./data_recorder | socat - tcp-listen:5555,fork > /dev/null
// SOCAT is better.

//Blatantly ripped off of https://www.cs.cmu.edu/afs/cs/academic/class/15213-f99/www/class26/tcpserver.c

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <os_generic.h>

#define MAX_CONNS 32

int SocketList[MAX_CONNS];
int droppedct[MAX_CONNS];

void error(char *msg) {
	perror(msg);
	exit(1);
}

void * SendThread( void * v )
{
	while(1)
	{
		int i;
		char buff[1024];
		int rd = read( STDIN_FILENO, buff, sizeof( buff ) );
		if( rd <= 0 )
		{
			fprintf( stderr, "Error: Can't read data\n" );
			exit( -1 );
		}
		for( i = 0; i < MAX_CONNS; i++ )
		{
			int sockc = SocketList[i];
			if( sockc == 0 ) continue;
			int ss = send( sockc, buff, rd, MSG_DONTWAIT | MSG_NOSIGNAL );
			if( ss < rd )
			{
                if( droppedct[i]++ > 20 )
                {
				    fprintf( stderr, "Dropped %d\n", i );
				    close( SocketList[i] );
				    SocketList[i] = 0;
                }
			}
            else
            {
                droppedct[i] = 0;
            }
		}
	}
}

int main( int argc, char ** argv )
{
	int parentfd; /* parent socket */
	int childfd; /* child socket */
	int portno; /* port to listen on */
	int clientlen; /* byte size of client's address */
	struct sockaddr_in serveraddr; /* server's addr */
	struct sockaddr_in clientaddr; /* client addr */
	struct hostent *hostp; /* client host info */
	char *hostaddrp; /* dotted decimal host addr string */
	int optval; /* flag value for setsockopt */
	int n; /* message byte size */

	if (argc != 2) {
		fprintf(stderr, "usage: %s <port>\n", argv[0]);
		exit(1);
	}
	portno = atoi(argv[1]);

	parentfd = socket(AF_INET, SOCK_STREAM, 0);
	if (parentfd < 0) 
		error("ERROR opening socket");

	
	/* setsockopt: Handy debugging trick that lets 
	* us rerun the server immediately after we kill it; 
	* otherwise we have to wait about 20 secs. 
	* Eliminates "ERROR on binding: Address already in use" error. 
	*/
	optval = 1;
	setsockopt(parentfd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval , sizeof(int));


	/*
	* build the server's Internet address
	*/
	bzero((char *) &serveraddr, sizeof(serveraddr));

	/* this is an Internet address */
	serveraddr.sin_family = AF_INET;

	/* let the system figure out our IP address */
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);

	/* this is the port we will listen on */
	serveraddr.sin_port = htons((unsigned short)portno);

	/* 
	* bind: associate the parent socket with a port 
	*/
	if (bind(parentfd, (struct sockaddr *) &serveraddr, 
	   sizeof(serveraddr)) < 0) 
	error("ERROR on binding");

	/* 
	* listen: make this socket ready to accept connection requests 
	*/
	if (listen(parentfd, 5) < 0) /* allow 5 requests to queue up */ 
	error("ERROR on listen");

	/* 
	* main loop: wait for a connection request, echo input line, 
	* then close connection.
	*/
	clientlen = sizeof(clientaddr);

	OGCreateThread( SendThread, 0 );
	while (1) {
		/* 
		* accept: wait for a connection request 
		*/
		childfd = accept(parentfd, (struct sockaddr *) &clientaddr, &clientlen);
		if (childfd < 0) 
			error("ERROR on accept");

//		hostp = gethostbyaddr((const char *)&clientaddr.sin_addr.s_addr, 
//			  sizeof(clientaddr.sin_addr.s_addr), AF_INET);
//		if (hostp == NULL)			
//			error("ERROR on gethostbyaddr");
		hostaddrp = inet_ntoa(clientaddr.sin_addr);
		//if (hostaddrp == NULL)
		//error("ERROR on inet_ntoa\n");


		int il;
		for( il = 0; il < MAX_CONNS; il++ )
		{
			if( SocketList[il] == 0 )
			{
				SocketList[il] = childfd;
                droppedct[il] = 0;
				printf("Conn %s At %d\n", 
					hostaddrp, il);
				break;
			}
		}
		if( il == MAX_CONNS )
		{
			close( childfd );
		}

		/* 
		* read: read input string from the client
		*/
/*		bzero(buf, BUFSIZE);
		n = read(childfd, buf, BUFSIZE);
		if (n < 0) 
		error("ERROR reading from socket");
		printf("server received %d bytes: %s", n, buf);
*/
		/* 
		* write: echo the input string back to the client 
		*/
/*		n = write(childfd, buf, strlen(buf));
		if (n < 0) 
		error("ERROR writing to socket");

		close(childfd);*/
	}
}

