// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL
// or LGPL licenses.

#ifdef _WIN32
#include "winsock2.h"
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

#include "os_generic.h"
#include "survive_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>
#include <sys/types.h>
#include <time.h>

#define EXAMPLE_PORT 2333
#define EXAMPLE_GROUP "226.5.1.32"

struct __attribute__((packed)) sendbuf_t {
	uint32_t header;
	uint8_t dev_no;
	uint16_t sensor_no;
	uint16_t length_event;
	uint32_t ccount_struc;
} sendbuf;

struct SurviveDriverUDP {
	SurviveContext *ctx;
	SurviveObject *so;
	struct sockaddr_in addr;
	int sock;
	socklen_t addrlen;
	struct ip_mreq mreq;
};
typedef struct SurviveDriverUDP SurviveDriverUDP;

static int UDP_poll(struct SurviveContext *ctx, void *_driver) {
	SurviveDriverUDP *driver = _driver;
	int cnt;
	for (;;) {
		cnt = recvfrom(driver->sock, (char *)&sendbuf, sizeof(sendbuf), MSG_DONTWAIT | MSG_NOSIGNAL,
					   (struct sockaddr *)&driver->addr, &driver->addrlen);
		if (cnt < 0) {
			// perror("recvfrom");
			//  exit(1);
			return 0;
		} else if (cnt == 0) {
			return 0;
			// break;
		}
		// printf("On %s: Header - %s | Device_Number - %d | Sensor_no - %d | Event_length - %5d | Ccount - %u \n",
		// inet_ntoa(driver->addr.sin_addr), (char *) &sendbuf.header, sendbuf.dev_no, sendbuf.sensor_no,
		// sendbuf.length_event, sendbuf.ccount_struc);

		LightcapElement le;
		le.sensor_id = 0;								// 8 bits
		le.length = sendbuf.length_event * 48 / 160;	// 16 bits
		le.timestamp = sendbuf.ccount_struc * 48 / 160; // 32 bits
		handle_lightcap(driver->so, &le);
	}

	/*
		To emit an IMU event, send this:
			driver->ctx->imuproc(so, mask, accelgyro, timecode, id);

		To emit light data, send this:
			LightcapElement le;
			le.sensor_id = X		//8 bits
			le.length = Z			//16 bits
			le.timestamp = Y		//32 bits
			handle_lightcap(so, &le);
	*/

	return 0;
}

static int UDP_close(struct SurviveContext *ctx, void *_driver) {
	SurviveDriverUDP *driver = _driver;

	/*
		If you need to handle any cleanup here, like closing handles, etc.
		you can perform it here.
	*/

	return 0;
}

int UDP_haptic(SurviveObject *so, uint8_t reserved, uint16_t pulseHigh, uint16_t pulseLow, uint16_t repeatCount) {
	/*
		If your device has haptics, you can add the control for them here.
	*/
	return 0;
}

int DriverRegUDP(SurviveContext *ctx) {

	int enable_UDP_driver = survive_configi(ctx, "UDP_driver_enable", SC_GET, 0);

	if (!enable_UDP_driver)
		return 0;

	SurviveDriverUDP *sp = calloc(1, sizeof(SurviveDriverUDP));
	sp->ctx = ctx;

	SV_INFO("Setting up UDP driver.");

	// Create a new SurviveObject...
	SurviveObject *device = calloc(1, sizeof(SurviveObject));
	device->ctx = ctx;
	device->driver = sp;
	memcpy(device->codename, "UD0", 4);
	memcpy(device->drivername, "UDP", 4);
	device->sensor_ct = 1;
	device->sensor_locations = malloc(sizeof(FLT) * 3);
	device->sensor_normals = malloc(sizeof(FLT) * 3);
	device->sensor_locations[0] = 0;
	device->sensor_locations[1] = 0;
	device->sensor_locations[2] = 0;
	device->sensor_normals[0] = 0;
	device->sensor_normals[1] = 0;
	device->sensor_normals[2] = 1;

	device->timebase_hz = 48000000;
	device->imu_freq = 1000.0f;
	device->haptic = UDP_haptic;

	sp->so = device;
	survive_add_object(ctx, device);
	survive_add_driver(ctx, sp, UDP_poll, UDP_close, 0);

	sp->sock = socket(AF_INET, SOCK_DGRAM, 0);

	if (sp->sock < 0) {
		perror("socket");
		exit(1);
	}
	bzero((char *)&sp->addr, sizeof(sp->addr));
	sp->addr.sin_family = AF_INET;
	sp->addr.sin_addr.s_addr = htonl(INADDR_ANY);
	sp->addr.sin_port = htons(EXAMPLE_PORT);
	sp->addrlen = sizeof(sp->addr);

	if (bind(sp->sock, (struct sockaddr *)&sp->addr, sizeof(sp->addr)) < 0) {
		perror("bind");
		exit(1);
	}
	sp->mreq.imr_multiaddr.s_addr = inet_addr(EXAMPLE_GROUP);
	sp->mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	if (setsockopt(sp->sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &sp->mreq, sizeof(sp->mreq)) < 0) {
		perror("setsockopt mreq");
		exit(1);
	}

	return 0;
}

REGISTER_LINKTIME(DriverRegUDP);
