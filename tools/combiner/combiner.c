#include <stdint.h>
#include <stdio.h>

uint8_t rd[131072];
uint8_t ld[131072];
uint8_t lo[131072*4];

int main()
{
	FILE * l = fopen( "L_lighthouse.dat", "rb" );
	FILE * r = fopen( "R_lighthouse.dat", "rb" );
	FILE * o = fopen( "dual_lighthouse.dat", "wb" );
	fread( rd, 131072, 1, r );
	fread( ld, 131072, 1, l );
	int i;
	for( i = 0; i < 131072; i++ )
	{
		int rr = rd[i];
		int lr = ld[i];
		int alpha = (rr>lr)?rr:lr;
		fprintf( o, "%c%c%c%c", rr, lr, 0, alpha );
	}
	fclose( o );

}

