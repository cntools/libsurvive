#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define NUM_DEVICES 3
#define MAX_SENSORS 32
#define MAX_SWEEP_INFO 4

#define MAX_DATAPOINTS_PER_SECTION 16384

//Takes 300MB of ram to boot.

const char * Devices[] = { "HMD", "WM0", "WM1" };
const char * DevMap[] = { "LX", "LY", "RX", "RY" };

uint32_t    DATASWEEP[NUM_DEVICES][MAX_SENSORS][MAX_SWEEP_INFO][MAX_DATAPOINTS_PER_SECTION];
uint16_t    DATALENGTH[NUM_DEVICES][MAX_SENSORS][MAX_SWEEP_INFO][MAX_DATAPOINTS_PER_SECTION];
int    DPNUM[NUM_DEVICES][MAX_SENSORS][MAX_SWEEP_INFO];


int main( int argc, char ** argv )
{
	if( argc < 2 )
	{
		fprintf( stderr, "Error: usage process_to_points [raw data.csv]\n" );
		exit( -5 );
	}

	FILE * f = fopen( argv[1], "r" );

	int lineno = 0;
	while(!feof(f) && !ferror(f) )
	{
		int i;
		lineno++;

		//Expecting; L X WM0 632359048 2 2 190890 319
		//Pulsecode is ignored, L/R X/Y is what is used.

		char  * line;
		size_t n = 0;
		ssize_t r = getline( &line, &n, f );
		if( r <= 0 ) break;

		char lhn[10];
		char axn[10];
		char dev[10];
		int unusedtime = 0;
		int sensor = 0;
		int unusedcode = 0;
		int timeinsweep = 0;
		int pulselength = 0;

		int rr = sscanf(line,"%8s %8s %8s %d %d %d %d %d\n", lhn, axn, dev, &unusedtime, &sensor, &unusedcode, &timeinsweep, &pulselength );
		free( line );
		if( rr != 8 )
		{
			fprintf( stderr, "Warning:  On line %d, only %d values read\n", lineno, rr );
			continue;
		}


		int devnum = -1;
		for( i = 0; i < NUM_DEVICES; i++ )
		{
			if( strcmp( dev, Devices[i] ) == 0 ) break;
		}
		if( i == NUM_DEVICES )
		{
			fprintf( stderr, "Error: unrecognized device %s on line %d\n",dev, lineno );
			continue;
		}
		devnum = i;
		if( sensor < 0 || sensor > MAX_SENSORS )
		{
			fprintf( stderr, "Error: sensor # too high on dev at line %d\n",lineno );
			continue;
		}
		char lcom[3];
		lcom[0] = lhn[0];
		lcom[1] = axn[0];
		lcom[2] = 0;

		for( i = 0; i < 4; i++ )
		{
			if( strcmp( lcom, DevMap[i] ) == 0 ) break;
		}
		if( i == 4 )
		{
			fprintf( stderr, "Error: entry type %s confusing on line %d\n", lcom, lineno );
			continue;
		}
		int swe = i;

		int num = DPNUM[devnum][sensor][swe]++;
		if( num >= MAX_DATAPOINTS_PER_SECTION )
		{
			fprintf( stderr, "Error: Too many datapoints on line %d - %d found, %d max\n", lineno, num, MAX_DATAPOINTS_PER_SECTION );
			continue;
		}
		DATASWEEP[devnum][sensor][swe][num] = timeinsweep;
		DATALENGTH[devnum][sensor][swe][num] = pulselength;
	}
	printf( "Read %d lines of data.\n", lineno );

	//Now, process the data.

	FILE * hists = fopen( "histograms.csv", "w" );
	int dev, sen, swe;
	for( dev = 0; dev < NUM_DEVICES; dev++ )
	for( sen = 0; sen < MAX_SENSORS; sen++ )
	for( swe = 0; swe < 4; swe++ )
	{
		int dpmax = DPNUM[dev][sen][swe];
		if( !dpmax ) continue;
		int i;

		double sumsweeptime = 0;
		double sumlentime = 0;
		int count = 0;

		for( i = 0; i < dpmax; i++ )
		{
			int sweeptime = DATASWEEP[dev][sen][swe][i];
			int datalen = DATALENGTH[dev][sen][swe][i];

			sumsweeptime += sweeptime;
			sumlentime += datalen;
			count++;
		}

		double avgsweep = sumsweeptime / count;
		double avglen = sumlentime / count;

		double stddevtim = 0;
		double stddevlen = 0;

		#define HISTOGRAMSIZE 65

		int histo[HISTOGRAMSIZE];
		memset( histo, 0, sizeof( histo ) );

		for( i = 0; i < dpmax; i++ )
		{
			int sweeptime = DATASWEEP[dev][sen][swe][i];
			int datalen = DATALENGTH[dev][sen][swe][i];

			double Sdiff = sweeptime - avgsweep;
			double Ldiff = datalen - avglen;

			stddevtim += Sdiff * Sdiff;
			stddevlen += Ldiff * Ldiff;

			Sdiff/=4;

			int llm = Sdiff + (HISTOGRAMSIZE/2.0);
			if( llm < 0 ) llm = 0;
			if( llm >= HISTOGRAMSIZE ) llm = HISTOGRAMSIZE-1;

			histo[llm]++;
		}
		stddevtim /= count;
		stddevlen /= count;

		fprintf( hists, "%s%s%02d, ", Devices[dev], DevMap[swe], sen );

		for( i = 0; i < HISTOGRAMSIZE; i++ )
		{
			fprintf( hists, "%d, ", histo[i] );
		}
		fprintf( hists, "\n" );

		printf( "%s %s %d %d %f %f %f %f\n", Devices[dev], DevMap[swe], sen, count, avgsweep, avglen, stddevtim, stddevlen );
	}


}
