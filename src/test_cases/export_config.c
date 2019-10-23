#include "test_case.h"
#include "../survive_str.h"
#include "string.h"

TEST(Survive, ExportConfig) {
    SurviveContext *ctx = SV_CALLOC(1, sizeof(SurviveContext));
#define SURVIVE_HOOK_PROCESS_DEF(hook) survive_install_##hook##_fn(ctx, 0);
#define SURVIVE_HOOK_FEEDBACK_DEF(hook) survive_install_##hook##_fn(ctx, 0);
#include "survive_hooks.h"

    ctx->log_target = stderr;

    SurviveObject *device = (SurviveObject *)calloc(1, sizeof(SurviveObject));
    device->ctx = ctx;

    device->drivername[0] = device->codename[0] = 'T';

    device->sensor_ct = 32;
    device->sensor_locations = calloc(sizeof(FLT) * 3, device->sensor_ct);
    device->sensor_normals = calloc(sizeof(FLT) * 3, device->sensor_ct);

    char* cfg = survive_export_config(device);

    free(device->sensor_normals);
    free(device->sensor_locations);
    device->sensor_normals = device->sensor_locations = 0;

    survive_default_config_process(device, cfg, strlen(cfg));

    FILE *fw = fopen("test_config.json", "w");
    fwrite(cfg, 1, strlen(cfg), fw);
    free(cfg);
    fclose(fw);

    return 0;
}