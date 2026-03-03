#include "utilities.h"
#include <sys/time.h>
// TODO: Maybe this time is not paralel to ROS2 time
double getPreciseTime() // standard function to get epoch time via NTP
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}
