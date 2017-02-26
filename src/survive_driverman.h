// (C) 2017 <>< C. N. Lohr, Under MIT/x11 License.
//
// This file is intended to be used for self-registering functions.  By using
// this it means that you do not need to have complicated switch statements or
// #defines for dfferent inclusion of drivers/other code.  You can simply
// register your function and it will be put into a list.
//
//

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


//
// Specific types of drivers.
//

struct SurviveContext;

//Device drivers (prefix your drivers with "DriverReg") i.e.
//		REGISTER_LINKTIME( DriverRegHTCVive );
typedef int (*DeviceDriver)( struct SurviveContext * ctx );

//more driver types here? i.e. posefinders, etc.

#endif

