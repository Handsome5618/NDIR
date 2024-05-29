#ifndef _COEFFICIENT_H_
#define _COEFFICIENT_H_

#include "main.h"
#include "math.h"

typedef struct tagPOINT_2 {
    double x;
    double y;
} POINT_2, *PPOINT_2;

int LineInfo(POINT_2 *pt_input, int ptNumbers, double *k, double *b);

#endif // !_COEFFICIENT_H_
