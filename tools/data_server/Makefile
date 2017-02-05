all : data_server

data_server : data_server.c os_generic.c
	gcc -o $@ $^ -I. -lpthread

clean :
	rm -rf data_server *.o *~
