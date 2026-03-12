#ifndef _ZENOH_H_
#define _ZENOH_H_
#include <zenoh-pico.h>

extern void initZenoh();
extern z_owned_session_t s;      // session object
extern z_owned_subscriber_t sub; // subsciber object

#endif