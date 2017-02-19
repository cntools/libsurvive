#include "visualization.h"
#include "tori_includes.h"

int pointsWritten = 0;


void writePoint(FILE *file, double x, double y, double z, unsigned int rgb)
{
	fprintf(file, "%f %f %f %u\n", x, y, z, rgb);
	pointsWritten++;
}

void updateHeader(FILE * file)
{
	fseek(file, 0x4C, SEEK_SET);
	fprintf(file, "%d", pointsWritten);
	fseek(file, 0x7C, SEEK_SET);
	fprintf(file, "%d", pointsWritten);
}
void writeAxes(FILE * file)
{
	double scale = 5;
	for (double i = 0; i < scale; i = i + scale / 1000)
	{
		writePoint(file, i, 0, 0, 255);
	}
	for (double i = 0; i < scale; i = i + scale / 1000)
	{
		if ((int)(i / (scale / 5)) % 2 == 1)
		{
			writePoint(file, 0, i, 0, 255 << 8);
		}
	}
	for (double i = 0; i < scale; i = i + scale / 10001)
	{
		if ((int)(i / (scale / 10)) % 2 == 1)
		{
			writePoint(file, 0, 0, i, 255 << 16);
		}
	}
}

void drawLineBetweenPoints(FILE *file, Point a, Point b, unsigned int color)
{
	int max = 50;
	for (int i = 0; i < max; i++)
	{
		writePoint(file,
			(a.x*i + b.x*(max - i)) / max,
			(a.y*i + b.y*(max - i)) / max,
			(a.z*i + b.z*(max - i)) / max,
			color);
	}
}

void writePcdHeader(FILE * file)
{
	fprintf(file, "VERSION 0.7\n");
	fprintf(file, "FIELDS  x y z rgb\n");
	fprintf(file, "SIZE 4 4 4 4\n");
	fprintf(file, "TYPE F F F U\n");
	fprintf(file, "COUNT 1 1 1 1\n");
	fprintf(file, "WIDTH        \n");
	fprintf(file, "HEIGHT 1\n");
	fprintf(file, "VIEWPOINT 0 0 0 1 0 0 0\n");
	fprintf(file, "POINTS        \n");
	fprintf(file, "DATA ascii\n");

	//fprintf(file, "100000.0, 100000.0, 100000\n");

}

void writePointCloud(FILE *f, Point *pointCloud, unsigned int Color)
{
	Point *currentPoint = pointCloud;

	while (currentPoint->x != -1000 || currentPoint->y != -1000 || currentPoint->z != -1000)
	{
		writePoint(f, currentPoint->x, currentPoint->y, currentPoint->z, Color);
		currentPoint++;
	}
}

void markPointWithStar(FILE *file, Point point, unsigned int color)
{
	double i;
	for (i = -0.8; i <= 0.8; i = i + 0.0025)
	{
		writePoint(file, point.x + i, point.y, point.z, color);
		writePoint(file, point.x, point.y + i, point.z, color);
		writePoint(file, point.x, point.y, point.z + i, color);
	}

}
