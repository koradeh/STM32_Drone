/*
 * geo.c
 *
 *  Created on: Nov 21, 2024
 *      Author: Vincent
 */

#include "geo.h"

void geoSetOrigin(position_t *position , geoOriginResetMode_e resetMode)
{
    if (resetMode == GEO_ORIGIN_SET)
    {
        //origin->valid = true;

    	for (int i=0 ;i<3 ;i++)
    	{
    		position->origin.v[i] = position->llh.v[i];
    	}

    	//using latitude to calculate scale (x is latitude)
        position->originScale = constrainf(cos_approx((ABS(position->origin.x) / 10000000.0f) * 0.0174532925f), 0.01f, 1.0f);
    }
    else if (position->originValid && (resetMode == GEO_ORIGIN_RESET_ALTITUDE))
    {
        position->origin.z = position->llh.z;
    }
}

//bool geoConvertGeodeticToLocal(fpVector3_t *pos, const gpsOrigin_t *origin, const gpsLocation_t *llh, geoAltitudeConversionMode_e altConv)
//{
//    if (origin->valid) {
//        pos->x = (llh->lat - origin->lat) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR;
//        pos->y = (llh->lon - origin->lon) * (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * origin->scale);
//
//        // If flag GEO_ALT_RELATIVE, than llh altitude is already relative to origin
//        if (altConv == GEO_ALT_RELATIVE) {
//            pos->z = llh->alt;
//        } else {
//            pos->z = llh->alt - origin->alt;
//        }
//        return true;
//    }
//
//    pos->x = 0.0f;
//    pos->y = 0.0f;
//    pos->z = 0.0f;
//    return false;
//}
//
//bool geoConvertGeodeticToLocalOrigin(fpVector3_t * pos, const gpsLocation_t *llh, geoAltitudeConversionMode_e altConv)
//{
//    return geoConvertGeodeticToLocal(pos, &posControl.gpsOrigin, llh, altConv);
//}
