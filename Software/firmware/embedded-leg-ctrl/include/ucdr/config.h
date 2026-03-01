#ifndef _MICROCDR_CONFIG_H_
#define _MICROCDR_CONFIG_H_

// Versión de la librería (es un estándar que pide el código interno)
#define UCDR_VERSION_MAJOR 2
#define UCDR_VERSION_MINOR 0
#define UCDR_VERSION_MICRO 1

// Define el "Endianness" (El orden de los bytes en memoria).
// 0 = Little Endian (ESP32, Arduino, ARM Cortex)
// 1 = Big Endian
#define UCDR_MACHINE_ENDIANNESS 0
#define MACHINE_ENDIANNESS 0 // Mantenemos ambas por retrocompatibilidad con versiones viejas

#endif // _MICROCDR_CONFIG_H_