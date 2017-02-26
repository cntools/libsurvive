#ifndef SURVIVE_DRIVERMAN_H
#define SURVIVE_DRIVERMAN_H

//Driver registration
#define MAX_DRIVERS 32

void   RegisterDriver( const char * name, void * data );
void * GetDriver( const char * name );
const char * GetDriverNameMatching( const char * prefix, int place );
void   ListDrivers();

#define REGISTER_LINKTIME( func ) \
	void __attribute__((constructor)) Register##func() { RegisterDriver( #func, &func ); }

struct SurviveContext;

typedef int (*DeviceDriver)( struct SurviveContext * ctx );
typedef int (*DeviceDriverCb)( struct SurviveContext * ctx, void * driver );
typedef int (*DeviceDriverMagicCb)( struct SurviveContext * ctx, void * driver, int magic_code, void * data, int datalen );

#endif

