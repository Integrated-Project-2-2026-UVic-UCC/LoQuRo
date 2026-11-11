#ifndef _UTILITIES_H_
#define _UTILITIES_H_

#include <ucdr/microcdr.h>
#include "IMUMPU9250.h"

extern double getPreciseTime();
bool deserializeJointStates(ucdrBuffer *ub, float target_joints[4][3], double *timestamp);
extern bool findRouter(char *locator, size_t locator_size);
extern void beginNetwork();
uint32_t serializeImu(uint8_t *buffer, uint32_t size, const IMUdata &data, double timestamp);
uint32_t serializeByte(uint8_t *buffer, uint8_t value);
bool deserializeTime(const uint8_t *buffer, size_t len, double *time);
uint32_t serializeMag(uint8_t *buffer, uint32_t size, const IMUdata &data, double timestamp);
#endif