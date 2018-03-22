#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

//HTTP stuff blatantly ripped off of https://www.cs.cmu.edu/afs/cs/academic/class/15213-f99/www/class26/tcpserver.c

#define BUFSIZE 1200000

#define WIDTH 2160
#define HEIGHT 1200
#define COMPS 4
#define PORTNO 8080

char buf[BUFSIZE]; /* message buffer */
int jpegsize = 0;

void myjpegwrite(void *context, void *data, int size)
{
	memcpy( buf + jpegsize, data, size );
	jpegsize += size;
}


int main()
{
	uint32_t * copiedrx = malloc( WIDTH * HEIGHT * COMPS );
	//MMap the fb device.
	int fd = open("/dev/fb0", O_RDWR | O_CREAT | O_TRUNC, (mode_t)0600);
	uint32_t *map;
	int FILESIZE = WIDTH*HEIGHT*COMPS;
    if (fd == -1)
	{
		fprintf( stderr,"Error opening file for writing");
		exit(EXIT_FAILURE);
    }
    map = mmap(0, FILESIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (map == MAP_FAILED) {
		close(fd);
		fprintf( stderr,"Error mmapping the file");
		exit(EXIT_FAILURE);
    }


	//An awful single-thread TCP server.
	int parentfd; /* parent socket */
	int childfd; /* child socket */
	int clientlen; /* byte size of client's address */
	struct sockaddr_in serveraddr; /* server's addr */
	struct sockaddr_in clientaddr; /* client addr */
	struct hostent *hostp; /* client host info */
	char *hostaddrp; /* dotted decimal host addr string */
	int optval; /* flag value for setsockopt */
	int n; /* message byte size */
	parentfd = socket(AF_INET, SOCK_STREAM, 0);
	optval = 1;
	setsockopt(parentfd, SOL_SOCKET, SO_REUSEADDR, 
	     (const void *)&optval , sizeof(int));
	bzero((char *) &serveraddr, sizeof(serveraddr));

	serveraddr.sin_family = AF_INET;
	serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serveraddr.sin_port = htons((unsigned short)PORTNO);

	if (bind(parentfd, (struct sockaddr *) &serveraddr, 
	   sizeof(serveraddr)) < 0) 
	{
		fprintf( stderr,"ERROR on binding");
		exit( -1 );
	}

	if (listen(parentfd, 5) < 0) /* allow 5 requests to queue up */ 
	{
		fprintf( stderr,"ERROR on listen");
		exit( -1 );
	}

	clientlen = sizeof(clientaddr);
	while (1) {
		int childfd = accept(parentfd, (struct sockaddr *) &clientaddr, &clientlen);
    	if (childfd < 0) 
		{
			fprintf( stderr,"Error on accept\n" );
			exit(-1);
		}

		int reads = recv( childfd, buf, BUFSIZE, 0 );
		char header[1024];
		int ts = sprintf( header, "HTTP/1.1 200 Ok\r\nContent-type: multipart/x-mixed-replace;boundary=ipcam264\r\n\r\n" );
		send( childfd, header, ts, MSG_NOSIGNAL );
		while(1)
		{
			jpegsize = 0;
			memcpy( copiedrx, map, WIDTH * HEIGHT * COMPS );
			int k = 0;
			int x, y;
			for( y = 0; y < HEIGHT/2; y++ )
			for( x = 0; x < WIDTH/2; x++ )
			{
				//uint32_t c1 = map[x*2+y*2*WIDTH]&0xfcfcfcfc;
				//uint32_t c2 = map[x*2+y*2*WIDTH+1]&0xfcfcfcfc;
				//uint32_t c3 = map[x*2+(y*2+1)*WIDTH]&0xfcfcfcfc;
				//uint32_t c4 = map[x*2+(y*2+1)*WIDTH+1]&0xfcfcfcfc;
				//uint32_t c = ((c1+c2+c3+c4))/4;
				uint32_t c = map[x*2+y*2*WIDTH];
				copiedrx[k++] = ((c & 0xff)<<16) | ((c & 0xff0000)>>16) | 0xff000000 | (c & 0xff00);
			}
			int sr = stbi_write_jpg_to_func( myjpegwrite, 0, WIDTH/2, HEIGHT/2, COMPS, ((uint8_t*)copiedrx), 50);
			char header[1024];
			int ts = sprintf( header, "--ipcam264\r\nContent-type:image/jpeg\r\nContent-Length:%d\r\n\r\n", jpegsize );
			send( childfd, header, ts, MSG_NOSIGNAL );
			if( send( childfd, buf, jpegsize, MSG_NOSIGNAL ) <= 0 ) break;
			printf( "!\n" );
		}
	}

//	printf( "%p\n", map );
//	printf( "%p\n", map[0] );
//	stbi_write_jpg("/tmp/test.jpg", WIDTH, HEIGHT, 4, map, 3);
//	printf( "%p\n", map[0] );
}

