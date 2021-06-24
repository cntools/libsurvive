#include "survive_default_devices.h"
#include <chrono>
#include <map>
#include <survive.h>

#include "crazyflieLinkCpp/Connection.h"

std::string cf0_conf = R"(
{
	"device_class": "drone",
	"head": {
		"plus_x": [
			1,
			0,
			0
		],
		"plus_z": [
			0,
			0,
			1
		],
		"position": [
			0,
			0,
			0
		]
	},
	"imu": {
		"plus_x": [
		    0,
		    1,
		    0
		],
		"plus_z": [
			0,
			0,
			1
		],
		"position": [
		    0,
		    0,
		    0
		]
	},
	"lighthouse_config": {
		"channelMap": [
			0,
			1,
			2,
			3
		],
		"modelNormals": [
			[
				0,
				0,
				1
			],
			[
				0,
				0,
				1
			],
			[
				0,
				0,
				1
			],
			[
				0,
				0,
				1
			]
		],
		"modelPoints": [
			[
				-0.015,
				0.0075,
				0
			],
			[
				-0.015,
				-0.0075,
				0
			],
			[
				0.015,
				0.0075,
				0
			],
			[
				0.015,
				-0.0075,
				0
			]
		]
	},
	"manufacturer": "bitcraze",
	"type": "drone"
}
)";

using namespace bitcraze::crazyflieLinkCpp;

struct clock_info {
	uint8_t channel = 0;

	int64_t offset = -1;
	uint32_t period = 0;

	SurviveContext *ctx = nullptr;

	uint32_t last_mod_sync = 0;
	survive_long_timecode last_updates[12] = {};
	uint32_t last_updates_idx = 0;

	bool is_valid() const {
		if (last_updates_idx <= 12)
			return false;
		int64_t difference = last_updates[(last_updates_idx - 1) % 12] - last_updates[last_updates_idx % 12];
		if (difference >= 48000000)
			SV_VERBOSE(200, "Difference %d: %ld %lu %lu", last_updates_idx, difference,
					   last_updates[(last_updates_idx - 1) % 12], last_updates[last_updates_idx % 12]);
		return difference < (5 * 48000000) && confidence > 5;
	}

	int confidence = 0;

	bool update(survive_long_timecode t) {
		if (period == 0) {
			period = 960000;
		}
		last_updates[last_updates_idx++ % 12] = t;
		if (last_updates_idx < 12)
			return false;

		survive_long_timecode t_1 = last_updates[(last_updates_idx - 2) % 12];
		if (t_1 > t) {
			t_1 -= (1 << 24);

			if (t_1 > t)
				return false;
		}

		int error = 0;
		int32_t t_diff = t - t_1;
		if (t_diff < 100) {
			error = t_diff;
			offset = t_1 + t_diff / 2;
			confidence++;
		} else if (t_diff > (923077 * .9) && t_diff < (923077 * 1.1)) {
			offset = t;
			error = ::abs((int32_t)period - (int32_t)t_diff);
			confidence++;
		} else {
			confidence--;
		}

		SV_VERBOSE(1000, "Time: %12lu %2d Offset: %ld Period: %u Error: %3d Freq: %7.3fhz", t, channel, offset, period,
				   error, 48e6 / period);
		return false;
	}

	bool should_sync(survive_long_timecode t) {
		auto m = find_m(t);
		if (m != 0 && last_mod_sync < m) {
			last_mod_sync = m;
			return true;
		}
		return false;
	}

	inline int32_t find_m(survive_long_timecode t) const {
		if (offset == -1)
			return 0;

		auto t1 = t - (offset % period);
		return t1 / period;
	}
};

static FLT freq_per_channel[NUM_GEN2_LIGHTHOUSES] = {
	50.0521, 50.1567, 50.3673, 50.5796, 50.6864, 50.9014, 51.0096, 51.1182,
	51.2273, 51.6685, 52.2307, 52.6894, 52.9217, 53.2741, 53.7514, 54.1150,
};

struct DriverCrazyFlie {
	DriverCrazyFlie(Connection *conn, SurviveContext *ctx) : connection(conn), ctx(ctx) {
		for (int i = 0; i < 16; i++) {
			clocks[i].channel = i;
			clocks[i].ctx = ctx;
			clocks[i].period = roundf(48000000.0 / freq_per_channel[i]);
		}
	}
	std::unique_ptr<Connection> connection;
	SurviveContext *ctx = nullptr;
	SurviveObject *so = nullptr;
	uint32_t last_timestamp = 0;
	survive_long_timecode extension = 0;
	bool running = true;
	clock_info clocks[NUM_GEN2_LIGHTHOUSES];

	uint32_t sync_timecodes[NUM_GEN2_LIGHTHOUSES] = {0};
};

static void process_frame(DriverCrazyFlie *driver, const uint8_t *frame) {
	SurviveContext *ctx = driver->ctx;

	uint32_t channel_ootx_sync = 0;
	memcpy(&channel_ootx_sync, frame, 3);
	uint32_t ref_packet_timecode = 0;
	memcpy(&ref_packet_timecode, frame + 3, 3);
	survive_long_timecode ref_timecode = ref_packet_timecode;
	if (driver->last_timestamp > ref_timecode) {
		auto extended = driver->last_timestamp | driver->extension;
		driver->extension += 1 << 24;
		// SV_VERBOSE(100, "Extend %f %lx %x %lx %lx %f", extended / 24000000., extended, driver->last_timestamp,
		// ref_timecode | driver->extension, ref_timecode, ((ref_timecode | driver->extension) - extended) / 24000000.);
	}
	driver->last_timestamp = ref_timecode;
	ref_timecode = (driver->extension | ref_timecode) * 2 + 1;

	uint32_t sync = channel_ootx_sync & 0x1FFFF;
	if (sync == 0)
		return;

	for (int i = 0; i < 14; i++) {
		printf("%02x ", frame[i]);
	}
	printf("\n");

	SV_VERBOSE(100, "Sync %x Time %x", sync, ref_packet_timecode);

	uint8_t channel = channel_ootx_sync >> 20;
	int bsd_idx = ctx->bsd_map[channel];
	if (bsd_idx == -1)
		return;

	bool ootx = (channel_ootx_sync & 0x20000) > 0;
	uint32_t sync_in_24mhz = (((uint32_t)sync * 8) + 4);
	uint32_t timecode = 0;
	if (sync_in_24mhz < ref_timecode) {
		timecode = ref_timecode - sync_in_24mhz;
	} else {
		SurviveContext *ctx = driver->ctx;
		SV_VERBOSE(100, "ROLLOVER");
		timecode = ref_timecode + 0xFFFFFF - sync_in_24mhz;
	}
	driver->clocks[channel].update(timecode);
	bool doSync = driver->clocks[channel].should_sync(ref_timecode);

	uint32_t timecodes[4] = {0};
	uint8_t sensor_flags = 0;
	for (int i = 0; i < 4; i++) {
		int16_t delta = 0;
		memcpy(&delta, frame + 6 + i * 2, 2);
		if (delta != 0x7FFF) {
			timecodes[i] = ref_timecode + ((int32_t)delta * 2 + 1);
			SV_VERBOSE(100, "%d(%d) %ld = %ld + (%x * 2 + 1)", i, timecodes[i],
					   timecodes[i] - driver->clocks[channel].offset, ref_timecode - driver->clocks[channel].offset,
					   delta);
			sensor_flags |= 1 << i;
		}
	}

	if (doSync) {
		SURVIVE_INVOKE_HOOK_SO(sync, driver->so, channel, timecode, ootx, false);
		driver->sync_timecodes[channel] = timecode;
	}

	SV_VERBOSE(
		100,
		"Time %2d %f tc: %8lu %8d  sync24: %8d sync: %5x offset: %ld m: %d ootx: %d should_sync: %d calc_off: %d %ld",
		channel, ref_timecode / 48000000., ref_timecode, timecode, sync_in_24mhz, sync,
		driver->clocks[channel].offset % driver->clocks[channel].period, driver->clocks[channel].find_m(ref_timecode),
		ootx, doSync, timecode % driver->clocks[channel].period, ref_timecode - driver->clocks[channel].offset);

	for (int i = 0; i < 4; i++) {
		if (sensor_flags & (1 << i) && driver->clocks[channel].offset != -1) {
			SURVIVE_INVOKE_HOOK_SO(sweep, driver->so, channel, i, timecodes[i], false);
		}
	}
}

static void *CFL_poll(void *_driver) {
	auto driver = static_cast<DriverCrazyFlie *>(_driver);

	int32_t packets = 0;
	int32_t frames = 0;
	SurviveContext *ctx = driver->ctx;
	auto start = std::chrono::steady_clock::now();
	while (driver->running) {
		Packet p = driver->connection->recv(0);
		if (p.port() == 0) {
			char buffer[32] = {0};
			memcpy(buffer, p.payload(), p.payloadSize());
			fprintf(stderr, "%s", buffer);

			SV_VERBOSE(10, "CF Log: %s", buffer);
		} else if (p.port() == 6 && p.channel() == 3) {
			packets++;
			frames += p.payloadSize() / 14;

			survive_get_ctx_lock(driver->ctx);
			for (int i = 0; i < p.payloadSize() / 14; i++) {
				process_frame(driver, p.payload() + i * 14);
			}
			survive_release_ctx_lock(driver->ctx);

			std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start;
			double s = diff.count();
			if (s > 5) {
				SV_VERBOSE(100, "Packet hz %f frame hz %f (%f)", packets / s, frames / s, s);
				start = std::chrono::steady_clock::now();
				packets = frames = 0;
			}
		} else if (p.port() == 15) {

		} else {
			SV_VERBOSE(1000, "Recv %d %d", p.port(), p.channel());
		}
	}
	return 0;
}

extern "C" int DriverRegCrazyFlie(SurviveContext *ctx) {
	auto conn = new Connection("radio://0/80/2M/E7E7E7E7E7");

	Packet packet;
	packet.setPort(2);
	packet.setChannel(3);
	std::string groupName = "lighthouse";
	std::string paramName = "enLhRawStream";
	int offset = 3 + groupName.length() + paramName.length();
	packet.setPayloadSize(offset + 2);
	memset(packet.payload(), 0, 28);
	strcpy(reinterpret_cast<char *>(packet.payload() + 1), groupName.c_str());
	strcpy(reinterpret_cast<char *>(packet.payload() + groupName.length() + 2), paramName.c_str());
	packet.payload()[offset] = 0x08;  // uint8_t
	packet.payload()[offset + 1] = 1; // enable;
	conn->send(packet);

	auto *sp = new DriverCrazyFlie(conn, ctx);
	SurviveObject *device = survive_create_device(ctx, "CFL", sp, "CF0", 0);
	sp->so = device;

	// Create a new SurviveObject...

	survive_add_object(ctx, device);

	char *cf0_conf_copy = static_cast<char *>(malloc(cf0_conf.length() + 1));
	memcpy(cf0_conf_copy, cf0_conf.c_str(), cf0_conf.length());
	SURVIVE_INVOKE_HOOK_SO(config, device, cf0_conf_copy, cf0_conf.length());
	sp->so = device;

	survive_add_threaded_driver(ctx, sp, "CFL", CFL_poll, 0);
	return 0;
}

REGISTER_LINKTIME(DriverRegCrazyFlie)
