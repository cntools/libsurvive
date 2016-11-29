#ifndef _SURVIVE_H
#define _SURVIVE_H

struct SurviveContext;

struct SurviveContext * survive_init( );
void survive_close( struct SurviveContext * ctx );
int survive_poll();

#endif

