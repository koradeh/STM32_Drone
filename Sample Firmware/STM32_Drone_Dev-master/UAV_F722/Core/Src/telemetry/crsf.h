/*
 * crsf.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Vincent
 */

#ifndef SRC_TELEMETRY_CRSF_H_
#define SRC_TELEMETRY_CRSF_H_










/*
CRSF frame has the structure:
<Device address> <Frame length> <Type> <Payload> <CRC>
Device address: (uint8_t)
Frame length:   length in  bytes including Type (uint8_t)
Type:           (uint8_t)
CRC:            (uint8_t), crc of <Type> and <Payload>
*/

/*
0x02 GPS
Payload:
int32_t     Latitude ( degree / 10`000`000 )
int32_t     Longitude (degree / 10`000`000 )
uint16_t    Groundspeed ( km/h / 10 )
uint16_t    GPS heading ( degree / 100 )
uint16      Altitude ( meter ­1000m offset )
uint8_t     Satellites in use ( counter )
*/

#endif /* SRC_TELEMETRY_CRSF_H_ */
