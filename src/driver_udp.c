// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL
// or LGPL licenses.

#ifdef _WIN32
#include "winsock2.h"
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#endif

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#include "os_generic.h"
#include "survive_config.h"
#include "survive_default_devices.h"
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <survive.h>
#include <sys/types.h>
#include <time.h>

#define EXAMPLE_PORT 2333
#define EXAMPLE_GROUP "224.0.2.122"

struct SurviveDriverUDP {
	SurviveContext *ctx;
	SurviveObject *so;
	struct sockaddr_in addr;
	int sock;
	socklen_t addrlen;
	struct ip_mreq mreq;
};
typedef struct SurviveDriverUDP SurviveDriverUDP;

static void *UDP_poll(void *_driver) {
	SurviveDriverUDP *driver = _driver;
	struct SurviveContext *ctx = driver->so->ctx;

#ifdef __APPLE__
	int opt = 1;
	setsockopt(driver->sock, SOL_SOCKET, SO_NOSIGPIPE, &opt, sizeof(opt));
#endif

	int cnt;
	for (;;) {
		uint32_t buffer[1000] = {0};
		cnt = recvfrom(driver->sock, (char *)buffer, sizeof(buffer), MSG_NOSIGNAL, (struct sockaddr *)&driver->addr,
					   &driver->addrlen);
		if (cnt < 0) {
			// perror("recvfrom");
			//  exit(1);
			break;
		} else if (cnt == 0) {
			break;
		}

		survive_get_ctx_lock(ctx);
		switch (buffer[0]) {
		case 1: {
			SURVIVE_INVOKE_HOOK_SO(config, driver->so, (char *)&buffer[1], cnt - 4);
			break;
		}
		case 2: {
			SURVIVE_INVOKE_HOOK_SO(pose, driver->so, OGRelativeTime() * 48000000., (const SurvivePose *)&buffer[1]);
			break;
		}
		}
		survive_release_ctx_lock(ctx);
		// printf("On %s: Header - %s | Device_Number - %d | Sensor_no - %d | Event_length - %5d | Ccount - %u \n",
		// inet_ntoa(driver->addr.sin_addr), (char *) &sendbuf.header, sendbuf.dev_no, sendbuf.sensor_no,
		// sendbuf.length_event, sendbuf.ccount_struc);
	}

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

int DriverRegUDP(SurviveContext *ctx) {
	SurviveDriverUDP *sp = SV_CALLOC(sizeof(SurviveDriverUDP));
	sp->ctx = ctx;

	SV_INFO("Setting up UDP driver.");

	// Create a new SurviveObject...
	SurviveObject *device = survive_create_device(ctx, "UDP", sp, "UP0", 0);

	sp->so = device;
	sp->sock = socket(AF_INET, SOCK_DGRAM, 0);

	if (sp->sock < 0) {
		perror("socket");
		exit(1);
	}
	memset((char *)&sp->addr, 0, sizeof(sp->addr));
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

	survive_add_threaded_driver(ctx, sp, "UDP", UDP_poll, UDP_close);
	return 0;
}

REGISTER_LINKTIME(DriverRegUDP)
