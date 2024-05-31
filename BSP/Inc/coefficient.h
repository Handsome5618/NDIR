#ifndef _COEFFICIENT_H_
#define _COEFFICIENT_H_

#include "main.h"
#include "math.h"

#define WL_CO_5000_01_100PPM  4.0
#define WL_CO_5000_01_5000PPM 181.0

#define WL_CO_5000_02_100PPM  3.0
#define WL_CO_5000_02_5000PPM 175.0

typedef struct tagPOINT_2 {
    double x;
    double y;
} POINT_2, *PPOINT_2;

int LineInfo(POINT_2 *pt_input, int ptNumbers, double *k, double *b);

#endif // !_COEFFICIENT_H_
