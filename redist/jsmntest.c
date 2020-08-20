#include "jsmn.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
	if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}

int main()
{
	int i;
	FILE * f = fopen( "test.config.json", "r" );
	fseek( f, 0, SEEK_END );
	int len = ftell( f );
	fseek( f, 0, SEEK_SET );
	char * JSON_STRING = malloc( len );
	fread( JSON_STRING, len, 1, f );
	fclose( f );

	jsmn_parser p;
    jsmn_init(&p);

	int r = jsmn_parse(&p, JSON_STRING, len);
    jsmntok_t* t = p.token_pool;

	if (r < 0) {
		printf("Failed to parse JSON: %d\n", r);
		return 1;
	}
	if (r < 1 || t[0].type != JSMN_OBJECT) {
		printf("Object expected\n");
		return 1;
	}

	/* Loop over all keys of the root object */
	for (i = 1; i < r; i++) {
//		if (jsoneq(JSON_STRING, &t[i], "acc_bias") == 0) {
			/* We may use strndup() to fetch string value */
//			printf("ACC_BIAS: %.*s\n", t[i+1].end-t[i+1].start,	JSON_STRING + t[i+1].start);
//		}
		if (jsoneq(JSON_STRING, &t[i], "modelPoints") == 0) {
			int k;
			jsmntok_t * tk = &t[i+1];
			printf( "%d / %d / %d / %d\n", tk->type, tk->start, tk->end, tk->size );
			int pts = tk->size;
			for( k = 0; k < pts; k++ )
			{
				tk = &t[i+2+k*4];
				printf( "++%d / %d / %d / %d\n", tk->type, tk->start, tk->end, tk->size );
				
				float vals[3];
				int m;
				for( m = 0; m < 3; m++ )
				{
					tk++;
					if( tk->type != 4 )
					{
						fprintf( stderr, "Parse error.\n" );
						break;
					}
					printf( "%d\n", tk->type );
				} 
			}
			/* We may additionally check if the value is either "true" or "false" */
//			printf("- CMap: %.*s\n", t[i+1].end-t[i+1].start,
//					JSON_STRING + t[i+1].start);
		}
	}
    jsmn_free(&p);
	return EXIT_SUCCESS;


}
