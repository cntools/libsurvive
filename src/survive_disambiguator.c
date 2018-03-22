#include "survive.h"

void handle_lightcap(SurviveObject *so, LightcapElement *le) { so->ctx->lightcapfunction(so, le); }
