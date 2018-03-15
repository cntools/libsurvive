//Copyright 2016 <>< C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include <survive.h>
#include "survive_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "survive_config.h"
#include "os_generic.h"

#ifdef __APPLE__
#define z_const const
#endif

#ifdef RUNTIME_SYMNUM
#include <symbol_enumerator.h>
static int did_runtime_symnum;
int SymnumCheck( const char * path, const char * name, void * location, long size )
{
	if( strncmp( name, "REGISTER", 8 ) == 0 )
	{
		typedef void (*sf)();
		sf fn = (sf)location;
		fn();
	}
	return 0;
}

#endif

static void survivefault( struct SurviveContext * ctx, const char * fault )
{
	fprintf( stderr, "Error: %s\n", fault );
	exit( -1 );
}

static void survivenote( struct SurviveContext * ctx, const char * fault )
{
	fprintf( stderr, "Info: %s\n", fault );
}

static void *button_servicer(void * context)
{
	SurviveContext *ctx = (SurviveContext*)context;

	while (1)
	{
		OGLockSema(ctx->buttonQueue.buttonservicesem);

		if (ctx->isClosing)
		{
			// we're shutting down.  Close.
			return NULL;
		}

		ButtonQueueEntry *entry = &(ctx->buttonQueue.entry[ctx->buttonQueue.nextReadIndex]);
		if (entry->isPopulated == 0)
		{
			// should never happen.  indicates failure of code pushing stuff onto
			// the buttonQueue
			// if it does happen, it will kill all future button input
			printf("ERROR: Unpopulated ButtonQueueEntry! NextReadIndex=%d\n", ctx->buttonQueue.nextReadIndex);
			return NULL;
		}

		//printf("ButtonEntry: eventType:%x, buttonId:%d, axis1:%d, axis1Val:%8.8x, axis2:%d, axis2Val:%8.8x\n",
		//	entry->eventType,
		//	entry->buttonId,
		//	entry->axis1Id,
		//	entry->axis1Val,
		//	entry->axis2Id,
		//	entry->axis2Val);

		button_process_func butt_func = ctx->buttonproc;
		if (butt_func)
		{
			butt_func(entry->so,
				entry->eventType,
				entry->buttonId,
				entry->axis1Id,
				entry->axis1Val,
				entry->axis2Id,
				entry->axis2Val);
		}

		ctx->buttonQueue.nextReadIndex++;
		if (ctx->buttonQueue.nextReadIndex >= BUTTON_QUEUE_MAX_LEN)
		{
			ctx->buttonQueue.nextReadIndex = 0;
		}
	};
	return NULL;
}

void survive_verify_FLT_size(uint32_t user_size) {
  if(sizeof(FLT) != user_size) {
    fprintf(stderr, "FLT type incompatible; the shared library libsurvive has FLT size %lu vs user program %u\n", (unsigned long)sizeof(FLT), user_size);
    fprintf(stderr, "Add '#define FLT %s' before including survive.h or recompile the shared library with the appropriate flag. \n",
	    sizeof(FLT) == sizeof(double) ? "double" : "float");
    exit(-1);
  }
}

SurviveContext * survive_init_internal( int headless )
{
#ifdef RUNTIME_SYMNUM
	if( !did_runtime_symnum )
	{
		EnumerateSymbols( SymnumCheck );
		did_runtime_symnum = 1;
	}
#endif
#ifdef MANUAL_REGISTRATION
	// note: this manual registration is currently only in use on builds using Visual Studio.

#define MANUAL_DRIVER_REGISTRATION(func) int func( SurviveObject * so, PoserData * pd ); RegisterDriver( #func, &func);

	MANUAL_DRIVER_REGISTRATION(PoserCharlesSlow)
	MANUAL_DRIVER_REGISTRATION(PoserDaveOrtho)
	MANUAL_DRIVER_REGISTRATION(PoserDummy)
	MANUAL_DRIVER_REGISTRATION(DriverRegHTCVive)

#endif

	int r = 0;
	int i = 0;
	SurviveContext * ctx = calloc( 1, sizeof( SurviveContext ) );

	ctx->isClosing = 0;

	ctx->global_config_values = malloc( sizeof(config_group) );
	ctx->lh_config = malloc( sizeof(config_group) * NUM_LIGHTHOUSES);

	init_config_group(ctx->global_config_values,10);
	init_config_group(&ctx->lh_config[0],10);
	init_config_group(&ctx->lh_config[1],10);

	config_read(ctx, "config.json");
	ctx->activeLighthouses = config_read_uint32(ctx->global_config_values, "LighthouseCount", 2);
	config_read_lighthouse(ctx->lh_config, &(ctx->bsd[0]), 0);
	config_read_lighthouse(ctx->lh_config, &(ctx->bsd[1]), 1);

	ctx->faultfunction = survivefault;
	ctx->notefunction = survivenote;

	ctx->lightproc = survive_default_light_process;
	ctx->imuproc = survive_default_imu_process;
	ctx->angleproc = survive_default_angle_process;
	ctx->lighthouseposeproc = survive_default_lighthouse_pose_process;

	// initialize the button queue
	memset(&(ctx->buttonQueue), 0, sizeof(ctx->buttonQueue));
	ctx->buttonQueue.buttonservicesem = OGCreateSema();	

	// start the thread to process button data
	ctx->buttonservicethread = OGCreateThread(button_servicer, ctx);
	survive_install_button_fn(ctx, NULL);
	survive_install_raw_pose_fn(ctx, NULL);

	i = 0;
	const char * DriverName;

	//const char * PreferredPoser = config_read_str(ctx->global_config_values, "DefaultPoser", "PoserDummy");
	const char * PreferredPoser = config_read_str(ctx->global_config_values, "DefaultPoser", "PoserTurveyTori");
	PoserCB PreferredPoserCB = 0;
	const char * FirstPoser = 0;
	printf( "Available posers:\n" );
	while( ( DriverName = GetDriverNameMatching( "Poser", i++ ) ) )
	{
		PoserCB p = GetDriver( DriverName );
		if( !PreferredPoserCB ) PreferredPoserCB = p;
		int ThisPoser = strcmp( DriverName, PreferredPoser ) == 0;
		printf( "\t%c%s\n", ThisPoser?'*':' ', DriverName );
		if( ThisPoser ) PreferredPoserCB = p;
	}
	printf( "Totals %d posers.  Using selected poser (or first!).\n", i-1 );
	if( !PreferredPoserCB )
	{
		SV_ERROR( "Error.  Cannot find any valid poser." );
	}

	i = 0;
	while( ( DriverName = GetDriverNameMatching( "DriverReg", i++ ) ) )
	{
		DeviceDriver dd = GetDriver( DriverName );
		printf( "Loading driver %s (%p) (%d)\n", DriverName, dd, i );
		r = dd( ctx );
		printf( "Driver %s reports status %d\n", DriverName, r );
	}
printf( "REGISTERING DRIVERS\n" );

	//Apply poser to objects.
	for( i = 0; i < ctx->objs_ct; i++ )
	{
		ctx->objs[i]->PoserFn = PreferredPoserCB;
	}

	// saving the config extra to make sure that the user has a config file they can change.
	config_save(ctx, "config.json");


	return ctx;
}

void survive_install_info_fn( SurviveContext * ctx,  text_feedback_func fbp )
{
	if( fbp )
		ctx->notefunction = fbp;
	else
		ctx->notefunction = survivenote;
}

void survive_install_error_fn( SurviveContext * ctx,  text_feedback_func fbp )
{
	if( fbp )
		ctx->faultfunction = fbp;
	else
		ctx->faultfunction = survivefault;
}

void survive_install_light_fn( SurviveContext * ctx, light_process_func fbp )
{
	if( fbp )
		ctx->lightproc = fbp;
	else
		ctx->lightproc = survive_default_light_process;
}

void survive_install_imu_fn( SurviveContext * ctx,  imu_process_func fbp )
{
	if( fbp )
		ctx->imuproc = fbp;
	else
		ctx->imuproc = survive_default_imu_process;
}


void survive_install_angle_fn( SurviveContext * ctx,  angle_process_func fbp )
{
	if( fbp )
		ctx->angleproc = fbp;
	else
		ctx->angleproc = survive_default_angle_process;
}

void survive_install_button_fn(SurviveContext * ctx, button_process_func fbp)
{
	if (fbp)
		ctx->buttonproc = fbp;
	else
		ctx->buttonproc = survive_default_button_process;
}

void survive_install_raw_pose_fn(SurviveContext * ctx, raw_pose_func fbp)
{
	if (fbp)
		ctx->rawposeproc = fbp;
	else
		ctx->rawposeproc = survive_default_raw_pose_process;
}

void survive_install_lighthouse_pose_fn(SurviveContext *ctx, lighthouse_pose_func fbp) {
	if (fbp)
		ctx->lighthouseposeproc = fbp;
	else
		ctx->lighthouseposeproc = survive_default_lighthouse_pose_process;
}

int survive_add_object( SurviveContext * ctx, SurviveObject * obj )
{
	int oldct = ctx->objs_ct;
	ctx->objs = realloc( ctx->objs, sizeof( SurviveObject * ) * (oldct+1) );
	ctx->objs[oldct] = obj;
	ctx->objs_ct = oldct+1;
	return 0;
}

void survive_add_driver( SurviveContext * ctx, void * payload, DeviceDriverCb poll, DeviceDriverCb close, DeviceDriverMagicCb magic )
{
	int oldct = ctx->driver_ct;
	ctx->drivers = realloc( ctx->drivers, sizeof( void * ) * (oldct+1) );
	ctx->driverpolls = realloc( ctx->driverpolls, sizeof( DeviceDriverCb * ) * (oldct+1) );
	ctx->drivercloses = realloc( ctx->drivercloses, sizeof( DeviceDriverCb * ) * (oldct+1) );
	ctx->drivermagics = realloc( ctx->drivermagics, sizeof( DeviceDriverMagicCb * ) * (oldct+1) );
	ctx->drivers[oldct] = payload;
	ctx->driverpolls[oldct] = poll;
	ctx->drivercloses[oldct] = close;
	ctx->drivermagics[oldct] = magic;
	ctx->driver_ct = oldct+1;
}

int survive_send_magic( SurviveContext * ctx, int magic_code, void * data, int datalen )
{
	int oldct = ctx->driver_ct;
	int i;
	for( i = 0; i < oldct; i++ )
	{
		if (ctx->drivermagics[i]) {
			ctx->drivermagics[i](ctx, ctx->drivers[i], magic_code, data, datalen);
		}
	}
	return 0;
}

int survive_haptic(SurviveObject * so, uint8_t reserved, uint16_t pulseHigh, uint16_t pulseLow, uint16_t repeatCount)
{
	if (NULL == so || NULL == so->haptic)
	{
		return -404;
	}

	return so->haptic(so, reserved, pulseHigh, pulseLow, repeatCount);
}


void survive_close( SurviveContext * ctx )
{
	const char * DriverName;
	int r = 0;

	ctx->isClosing = 1;

	// unlock/ post to button service semaphore so the thread can kill itself
	OGUnlockSema(ctx->buttonQueue.buttonservicesem);

	while( ( DriverName = GetDriverNameMatching( "DriverUnreg", r++ ) ) )
	{
		DeviceDriver dd = GetDriver( DriverName );
		SV_INFO( "De-registering driver %s (%p)", DriverName, dd );
		r = dd( ctx );
		SV_INFO( "Driver %s reports status %d", DriverName, r );
	}

	int oldct = ctx->driver_ct;
	int i;

	for( i = 0; i < ctx->objs_ct; i++ )
	{
		PoserData pd;
		pd.pt = POSERDATA_DISASSOCIATE;
		if( ctx->objs[i]->PoserFn ) ctx->objs[i]->PoserFn( ctx->objs[i], &pd );
	}

	for( i = 0; i < oldct; i++ )
	{
		ctx->drivercloses[i]( ctx, ctx->drivers[i] );
	}


	config_save(ctx, "config.json");

	destroy_config_group(ctx->global_config_values);
	destroy_config_group(ctx->lh_config);

	free( ctx->objs );
	free( ctx->drivers );
	free( ctx->driverpolls );
	free( ctx->drivermagics );
	free( ctx->drivercloses );
	free( ctx->global_config_values );
	free( ctx->lh_config );

	free( ctx );
}

int survive_poll( struct SurviveContext * ctx )
{
	int oldct = ctx->driver_ct;
	int i, r;

	for( i = 0; i < oldct; i++ )
	{
		r = ctx->driverpolls[i]( ctx, ctx->drivers[i] );
		if( r ) return r;
	}

	return 0;
}


struct SurviveObject * survive_get_so_by_name( struct SurviveContext * ctx, const char * name )
{
	int i;
	for( i = 0; i < ctx->objs_ct; i++ )
	{
		if( strcmp( ctx->objs[i]->codename, name ) == 0 )
			return ctx->objs[i];
	}
	return 0;
}

#ifdef NOZLIB

#include <puff.h>

		
int survive_simple_inflate( struct SurviveContext * ctx, const char * input, int inlen, char * output, int outlen )
{
	//Tricky: we actually get 2 bytes of data on the front.  I don't know what it's for. 0x78 0x9c - puff doesn't deal with it well.
	unsigned long ol = outlen;
	unsigned long il = inlen-2;
	int ret = puff( output, &ol, input+2, &il );
	if( ret == 0 )
		return ol;
	else
	{
		SV_INFO( "puff returned error code %d\n", ret );
		return -5;
	}
}
 
#else
	
#include <zlib.h>

int survive_simple_inflate( struct SurviveContext * ctx, const char * input, int inlen, char * output, int outlen )
{
	z_stream zs; //Zlib stream.  May only be used by configuration at beginning and by USB thread periodically.
	memset( &zs, 0, sizeof( zs ) );
	inflateInit( &zs ); ///Consider checking error

	//XXX: Todo: If we find that this is not useful past the beginning (nix this here and move into the configuration getter)
    zs.avail_in = inlen;
    zs.next_in = (z_const Bytef *)input;
    zs.avail_out = outlen;
	zs.next_out = output;

    if( inflate( &zs, Z_FINISH) != Z_STREAM_END )
	{
        SV_INFO("survive_simple_inflate could not inflate." );
        return -1;
	}
	int len = zs.total_out;
	inflateEnd( &zs );
	return len;
}

#endif
