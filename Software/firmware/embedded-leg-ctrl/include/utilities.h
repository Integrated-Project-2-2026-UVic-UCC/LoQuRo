#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <ucdr/microcdr.h>
#include "IMUMPU9250.h"

extern double getPreciseTime();
extern bool deserializeJointStates(ucdrBuffer *ub);
extern void saveConfigCallback();
extern void beginNetwork();
extern bool shouldSafeConfig;
uint32_t serializeImu(uint8_t *buffer, uint32_t size, const IMUdata &data, double timestamp);
uint32_t serializeMag(uint8_t *buffer, uint32_t size, const IMUdata &data, double timestamp);
#endif