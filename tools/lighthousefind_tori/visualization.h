
#ifndef __VISUALIZATION_H
#define __VISUALIZATION_H

#include <stdio.h>
#include "tori_includes.h"

extern int pointsWritten;

void writePoint(FILE *file, double x, double y, double z, unsigned int rgb);

void updateHeader(FILE * file);

void writeAxes(FILE * file);

void drawLineBetweenPoints(FILE *file, Point a, Point b, unsigned int color);

void writePcdHeader(FILE * file);

void writePointCloud(FILE *f, Point *pointCloud, unsigned int Color);

void markPointWithStar(FILE *file, Point point, unsigned int color);

#define MAX_COLORS 18
static unsigned int COLORS[] = {
	0x00FFFF,
	0xFF00FF,
	0xFFFF00,
	0xFF0000,
	0x00FF00,
	0x0000FF,
	0x0080FF,
	0x8000FF,
	0x80FF00,
	0x00FF80,
	0xFF0080,
	0xFF8000,
	0x008080,
	0x800080,
	0x808000,
	0x000080,
	0x008000,
	0x800000
};
#endif


