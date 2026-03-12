#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <ucdr/microcdr.h>

extern double getPreciseTime();
extern bool deserializeJointStates(ucdrBuffer *ub);
#endif