#include "survive.h"

#define USE_TURVEYBIGUATOR

void handle_lightcap_charlesbiguator(SurviveObject *so, LightcapElement *le);
void handle_lightcap_turveybiguator(SurviveObject *so, LightcapElement *le);

void handle_lightcap(SurviveObject *so, LightcapElement *le) {
#ifdef USE_TURVEYBIGUATOR
	handle_lightcap_turveybiguator(so, le);
#else
	handle_lightcap_charlesbiguator(so, le);
#endif
}
