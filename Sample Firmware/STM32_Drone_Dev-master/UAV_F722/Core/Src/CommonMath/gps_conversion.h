/*
 * gps_conversion.h
 *
 *  Created on: Oct 22, 2024
 *      Author: Vincent
 */

#ifndef SRC_COMMONMATH_GPS_CONVERSION_H_
#define SRC_COMMONMATH_GPS_CONVERSION_H_

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

uint32_t GPS_coord_to_degrees(const char* coordinateString);

#endif /* SRC_COMMONMATH_GPS_CONVERSION_H_ */
