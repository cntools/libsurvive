all : data_server

data_server : data_server.c ../../redist/os_generic.c
	gcc -o $@ $^ -I. -lpthread -I../../redist

clean :
	rm -rf data_server *.o *~
