#define FLOAT float
#define ORDER 50
#include "dclapack.h"

#include <stdio.h>
#include <stdlib.h>

int main()
{
    float A[ORDER][ORDER];
    float Ainv[ORDER][ORDER];
    float Prod[ORDER][ORDER];

    int i, j, n = 12;
    srand(7779);

    for(i=0; i<n; i++) {
        for(j=0; j<n; j++) {
            A[i][j] = (float)rand() / RAND_MAX;
        }
    }

    for (i=0; i<10000; i++) {
        INV(A,Ainv,n);
    }

    PRINT(Ainv,n,n);
    MUL(A,Ainv,Prod,n,n,n);
    PRINT(Prod,n,n);

    return 0;
}

