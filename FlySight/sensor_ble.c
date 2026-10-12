/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2026 Bionic Avionics Inc.                                   **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
**  This program is distributed in the hope that it will be useful,       **
**  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
**  GNU General Public License for more details.                          **
**                                                                        **
**  You should have received a copy of the GNU General Public License     **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>. **
**                                                                        **
****************************************************************************
**  Contact: Bionic Avionics Inc.                                         **
**  Website: http://flysight.ca/                                          **
****************************************************************************/

#include <string.h>

#include "sensor_ble.h"

/* Fields each stream can carry */
static const uint8_t s_supported[SENSOR_BLE_COUNT] =
{
    [SENSOR_BLE_BARO]  = SENSOR_BLE_DEFAULT_MASK | SENSOR_BLE_BIT_BARO_REF,
    [SENSOR_BLE_HUM]   = SENSOR_BLE_DEFAULT_MASK,
    [SENSOR_BLE_ACCEL] = SENSOR_BLE_DEFAULT_MASK,
    [SENSOR_BLE_GYRO]  = SENSOR_BLE_DEFAULT_MASK,
    [SENSOR_BLE_MAG]   = SENSOR_BLE_DEFAULT_MASK
};

static uint8_t  s_mask[SENSOR_BLE_COUNT];
static uint16_t s_divider[SENSOR_BLE_COUNT];

void Sensor_BLE_Init(void)
{
    uint8_t i;

    for (i = 0; i < SENSOR_BLE_COUNT; ++i)
    {
        s_mask[i] = SENSOR_BLE_DEFAULT_MASK;
        s_divider[i] = 1;
    }

    s_mask[SENSOR_BLE_BARO] |= SENSOR_BLE_BIT_BARO_REF;
}

uint8_t Sensor_BLE_GetMask(Sensor_BLE_Stream_t stream)
{
    return s_mask[stream];
}

void Sensor_BLE_SetMask(Sensor_BLE_Stream_t stream, uint8_t mask)
{
    /* Drop bits this stream doesn't support, so the mask byte at the start
       of each packet always describes what follows */
    s_mask[stream] = mask & s_supported[stream];
}

uint16_t Sensor_BLE_GetDivider(Sensor_BLE_Stream_t stream)
{
    return s_divider[stream];
}

void Sensor_BLE_SetDivider(Sensor_BLE_Stream_t stream, uint16_t divider)
{
    s_divider[stream] = divider ? divider : 1;
}

static uint8_t *Sensor_BLE_BuildHeader(Sensor_BLE_Stream_t stream,
        uint64_t time, uint8_t *dst)
{
    uint8_t *p = dst;
    uint32_t time_ms;

    *p++ = s_mask[stream];                            /* byte 0 : mask        */

    if (s_mask[stream] & SENSOR_BLE_BIT_TIME) {       /* time (ms)            */
        time_ms = (uint32_t) (time / 1000);
        memcpy(p, &time_ms, sizeof(time_ms));       p += 4;
    }

    return p;
}

uint8_t Sensor_BLE_BuildBaro(const FS_Baro_Data_t *src, int32_t ref_pressure, uint8_t *dst)
{
    const uint8_t mask = s_mask[SENSOR_BLE_BARO];
    uint8_t *p = Sensor_BLE_BuildHeader(SENSOR_BLE_BARO, src->time, dst);

    if (mask & SENSOR_BLE_BIT_DATA) {                 /* pressure (Pa * 100)  */
        memcpy(p, &src->pressure, sizeof(src->pressure));       p += 4;
    }

    if (mask & SENSOR_BLE_BIT_TEMPERATURE) {          /* temperature (C * 100) */
        memcpy(p, &src->temperature, sizeof(src->temperature)); p += 2;
    }

    if (mask & SENSOR_BLE_BIT_BARO_REF) {             /* reference (Pa * 100) */
        memcpy(p, &ref_pressure, sizeof(ref_pressure));         p += 4;
    }

    return (uint8_t)(p - dst);                        /* total payload length */
}

uint8_t Sensor_BLE_BuildHum(const FS_Hum_Data_t *src, uint8_t *dst)
{
    const uint8_t mask = s_mask[SENSOR_BLE_HUM];
    uint8_t *p = Sensor_BLE_BuildHeader(SENSOR_BLE_HUM, src->time, dst);

    if (mask & SENSOR_BLE_BIT_DATA) {                 /* humidity (% * 10)    */
        memcpy(p, &src->humidity, sizeof(src->humidity));       p += 2;
    }

    if (mask & SENSOR_BLE_BIT_TEMPERATURE) {          /* temperature (C * 10) */
        memcpy(p, &src->temperature, sizeof(src->temperature)); p += 2;
    }

    return (uint8_t)(p - dst);
}

uint8_t Sensor_BLE_BuildAccel(const FS_IMU_Data_t *src, uint8_t *dst)
{
    const uint8_t mask = s_mask[SENSOR_BLE_ACCEL];
    uint8_t *p = Sensor_BLE_BuildHeader(SENSOR_BLE_ACCEL, src->time, dst);

    if (mask & SENSOR_BLE_BIT_DATA) {                 /* accel (g * 100000)   */
        memcpy(p, &src->ax, sizeof(src->ax));                   p += 4;
        memcpy(p, &src->ay, sizeof(src->ay));                   p += 4;
        memcpy(p, &src->az, sizeof(src->az));                   p += 4;
    }

    if (mask & SENSOR_BLE_BIT_TEMPERATURE) {          /* temperature (C * 100) */
        memcpy(p, &src->temperature, sizeof(src->temperature)); p += 2;
    }

    return (uint8_t)(p - dst);
}

uint8_t Sensor_BLE_BuildGyro(const FS_IMU_Data_t *src, uint8_t *dst)
{
    const uint8_t mask = s_mask[SENSOR_BLE_GYRO];
    uint8_t *p = Sensor_BLE_BuildHeader(SENSOR_BLE_GYRO, src->time, dst);

    if (mask & SENSOR_BLE_BIT_DATA) {                 /* gyro (deg/s * 1000)  */
        memcpy(p, &src->wx, sizeof(src->wx));                   p += 4;
        memcpy(p, &src->wy, sizeof(src->wy));                   p += 4;
        memcpy(p, &src->wz, sizeof(src->wz));                   p += 4;
    }

    if (mask & SENSOR_BLE_BIT_TEMPERATURE) {          /* temperature (C * 100) */
        memcpy(p, &src->temperature, sizeof(src->temperature)); p += 2;
    }

    return (uint8_t)(p - dst);
}

uint8_t Sensor_BLE_BuildMag(const FS_Mag_Data_t *src, uint8_t *dst)
{
    const uint8_t mask = s_mask[SENSOR_BLE_MAG];
    uint8_t *p = Sensor_BLE_BuildHeader(SENSOR_BLE_MAG, src->time, dst);

    if (mask & SENSOR_BLE_BIT_DATA) {                 /* mag (gauss * 1000)   */
        memcpy(p, &src->x, sizeof(src->x));                     p += 2;
        memcpy(p, &src->y, sizeof(src->y));                     p += 2;
        memcpy(p, &src->z, sizeof(src->z));                     p += 2;
    }

    if (mask & SENSOR_BLE_BIT_TEMPERATURE) {          /* temperature (C * 10) */
        memcpy(p, &src->temperature, sizeof(src->temperature)); p += 2;
    }

    return (uint8_t)(p - dst);
}
