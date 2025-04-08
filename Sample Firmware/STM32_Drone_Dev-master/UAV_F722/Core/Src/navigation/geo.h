/*
 * geo.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Vincent
 */

#ifndef SRC_NAVIGATION_GEO_H_
#define SRC_NAVIGATION_GEO_H_

#include "../CommonMath/maths.h"
#include "../flight/position.h"
#include "navigationDefine.h"

typedef enum {
    GEO_ORIGIN_SET,
    GEO_ORIGIN_RESET_ALTITUDE
} geoOriginResetMode_e;

#endif /* SRC_NAVIGATION_GEO_H_ */
