#ifndef _SURVIVE_H
#define _SURVIVE_H

struct SurviveContext;

struct SurviveContext * survive_init( void(*faultfunction)( struct SurviveContext * ctx, const char * fault ),
	void(*notefunction)( struct SurviveContext * ctx, const char * note )  );

void survive_close( struct SurviveContext * ctx );
int survive_poll();


int survive_simple_inflate( struct SurviveContext * ctx, const char * input, int inlen, char * output, int outlen );
int survive_usb_send_magic(struct SurviveContext * ctx, int on );

#endif

