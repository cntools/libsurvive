//Don't call this file yourself.  It is intended to be included in any drivers which want to support the rasterizer plugin.

#ifdef RASTERIZER
#include "CNFGFunctions.h"
#include <stdlib.h>
#include <stdint.h>

static uint32_t * buffer = 0;
static short bufferx;
static short buffery;


void CNFGInternalResize( short x, short y )
{
	bufferx = x;
	buffery = y;
	if( buffer ) free( buffer );
	buffer = malloc( bufferx * buffery * 4 );
}

static uint32_t SWAPS( uint32_t r )
{
	uint32_t ret = (r&0xFF)<<16;
	r>>=8;
	ret |= (r&0xff)<<8;
	r>>=8;
	ret |= r;
	return ret;
}

uint32_t CNFGColor( uint32_t RGB )
{
	CNFGLastColor = SWAPS(RGB);
	return CNFGLastColor;
}

void CNFGTackSegment( short x1, short y1, short x2, short y2 )
{
	short tx, ty;
	float slope, lp;

	short dx = x2 - x1;
	short dy = y2 - y1;

	if( !buffer ) return;

	if( dx < 0 ) dx = -dx;
	if( dy < 0 ) dy = -dy;

	if( dx > dy )
	{
		short minx = (x1 < x2)?x1:x2;
		short maxx = (x1 < x2)?x2:x1;
		short miny = (x1 < x2)?y1:y2;
		short maxy = (x1 < x2)?y2:y1;
		float thisy = miny;
		slope = (float)(maxy-miny) / (float)(maxx-minx);

		for( tx = minx; tx <= maxx; tx++ )
		{
			ty = thisy;
			if( tx < 0 || ty < 0 || ty >= buffery ) continue;
			if( tx >= bufferx ) break;
			buffer[ty * bufferx + tx] = CNFGLastColor;
			thisy += slope;
		}
	}
	else
	{
		short minx = (y1 < y2)?x1:x2;
		short maxx = (y1 < y2)?x2:x1;
		short miny = (y1 < y2)?y1:y2;
		short maxy = (y1 < y2)?y2:y1;
		float thisx = minx;
		slope = (float)(maxx-minx) / (float)(maxy-miny);

		for( ty = miny; ty <= maxy; ty++ )
		{
			tx = thisx;
			if( ty < 0 || tx < 0 || tx >= bufferx ) continue;
			if( ty >= buffery ) break;
			buffer[ty * bufferx + tx] = CNFGLastColor;
			thisx += slope;
		}
	}
}
void CNFGTackRectangle( short x1, short y1, short x2, short y2 )
{
	short minx = (x1<x2)?x1:x2;
	short miny = (y1<y2)?y1:y2;
	short maxx = (x1>=x2)?x1:x2;
	short maxy = (y1>=y2)?y1:y2;

	short x, y;

	if( minx < 0 ) minx = 0;
	if( miny < 0 ) miny = 0;
	if( maxx >= bufferx ) maxx = bufferx-1;
	if( maxy >= buffery ) maxy = buffery-1;

	for( y = miny; y <= maxy; y++ )
	{
		uint32_t * bufferstart = &buffer[y * bufferx + minx];
		for( x = minx; x <= maxx; x++ )
		{
			(*bufferstart++) = CNFGLastColor;
		}
	}
}

void CNFGTackPoly( RDPoint * points, int verts )
{
	short minx = 10000, miny = 10000;
	short maxx =-10000, maxy =-10000;
	short i, x, y;

	//Just in case...
	if( verts > 32767 ) return;

	for( i = 0; i < verts; i++ )
	{
		RDPoint * p = &points[i];
		if( p->x < minx ) minx = p->x;
		if( p->y < miny ) miny = p->y;
		if( p->x > maxx ) maxx = p->x;
		if( p->y > maxy ) maxy = p->y;
	}

	if( miny < 0 ) miny = 0;
	if( maxy >= buffery ) maxy = buffery-1;

	for( y = miny; y <= maxy; y++ )
	{
		short startfillx = maxx;
		short endfillx = minx;

		//Figure out what line segments intersect this line.
		for( i = 0; i < verts; i++ )
		{
			short pl = i + 1;
			if( pl == verts ) pl = 0;

			RDPoint ptop;
			RDPoint pbot;

			ptop.x = points[i].x;
			ptop.y = points[i].y;
			pbot.x = points[pl].x;
			pbot.y = points[pl].y;
//printf( "Poly: %d %d\n", pbot.y, ptop.y );

			if( pbot.y < ptop.y )
			{
				RDPoint ptmp;
				ptmp.x = pbot.x;
				ptmp.y = pbot.y;
				pbot.x = ptop.x;
				pbot.y = ptop.y;
				ptop.x = ptmp.x;
				ptop.y = ptmp.y;
			}

			//Make sure this line segment is within our range.
//printf( "PT: %d %d %d\n", y, ptop.y, pbot.y );
			if( ptop.y <= y && pbot.y >= y )
			{
				short diffy = pbot.y - ptop.y;
				uint32_t placey = (uint32_t)(y - ptop.y)<<16;  //Scale by 16 so we can do integer math.
				short diffx = pbot.x - ptop.x;
				short isectx;

				if( diffy == 0 )
				{
					if( pbot.x < ptop.x )
					{
						if( startfillx > pbot.x ) startfillx = pbot.x;
						if( endfillx < ptop.x ) endfillx = ptop.x;
					}
					else
					{
						if( startfillx > ptop.x ) startfillx = ptop.x;
						if( endfillx < pbot.x ) endfillx = pbot.x;
					}
				}
				else
				{
					//Inner part is scaled by 65536, outer part must be scaled back.
					isectx = (( (placey / diffy) * diffx + 32768 )>>16) + ptop.x;
					if( isectx < startfillx ) startfillx = isectx;
					if( isectx > endfillx ) endfillx = isectx;
				}
//printf( "R: %d %d %d\n", pbot.x, ptop.x, isectx );
			}
		}

//printf( "%d %d %d\n", y, startfillx, endfillx );

		if( endfillx >= bufferx ) endfillx = bufferx - 1;
		if( endfillx >= bufferx ) endfillx = buffery - 1;
		if( startfillx < 0 ) startfillx = 0;
		if( startfillx < 0 ) startfillx = 0;

		unsigned int * bufferstart = &buffer[y * bufferx + startfillx];
		for( x = startfillx; x <= endfillx; x++ )
		{
			(*bufferstart++) = CNFGLastColor;
		}
	}
//exit(1);
}


void CNFGClearFrame()
{
	int i, m;
	uint32_t col = 0;
	short x, y;
	CNFGGetDimensions( &x, &y );
	if( x != bufferx || y != buffery || !buffer )
	{
		bufferx = x;
		buffery = y;
		buffer = malloc( x * y * 8 );
	}

	m = x * y;
	col = CNFGColor( CNFGBGColor );
	for( i = 0; i < m; i++ )
	{
//printf( "Got: %d %p %d\n", m, buffer, i );
		buffer[i] = col;
	}
}

void CNFGTackPixel( short x, short y )
{
	if( x < 0 || y < 0 || x >= bufferx || y >= buffery ) return;
	buffer[x+bufferx*y] = CNFGLastColor;
}

void CNFGSwapBuffers()
{
	CNFGUpdateScreenWithBitmap( (long unsigned int*)buffer, bufferx, buffery );
}


#endif
