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

#ifndef SENSOR_BLE_H_
#define SENSOR_BLE_H_

#include <stdint.h>
#include "stm32wbxx_hal.h"
#include "baro.h"
#include "hum.h"
#include "imu.h"
#include "mag.h"

/* ------------------------------------------------------------------ */
/* Streams (values are the sensor_id used on the SD control point)    */
/* ------------------------------------------------------------------ */
typedef enum
{
    SENSOR_BLE_BARO = 0,
    SENSOR_BLE_HUM,
    SENSOR_BLE_ACCEL,
    SENSOR_BLE_GYRO,
    SENSOR_BLE_MAG,
    SENSOR_BLE_COUNT
} Sensor_BLE_Stream_t;

/* ------------------------------------------------------------------ */
/* Packet layout control                                              */
/* ------------------------------------------------------------------ */
#define SENSOR_BLE_MAX_LEN          19u

/* Bit-layout of mask byte (MSB first), shared by all streams */
#define SENSOR_BLE_BIT_TIME         0x80u   /* sample time (ms)                       */
#define SENSOR_BLE_BIT_DATA         0x40u   /* pressure, humidity, accel, gyro or mag */
#define SENSOR_BLE_BIT_TEMPERATURE  0x20u   /* sensor temperature                     */
#define SENSOR_BLE_BIT_BARO_REF     0x10u   /* barometer only: reference pressure     */

#define SENSOR_BLE_DEFAULT_MASK     0xE0u

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */
void     Sensor_BLE_Init(void);

uint8_t  Sensor_BLE_GetMask(Sensor_BLE_Stream_t stream);
void     Sensor_BLE_SetMask(Sensor_BLE_Stream_t stream, uint8_t mask);

/* Send every Nth sample (1 = every sample) */
uint16_t Sensor_BLE_GetDivider(Sensor_BLE_Stream_t stream);
void     Sensor_BLE_SetDivider(Sensor_BLE_Stream_t stream, uint16_t divider);

uint8_t  Sensor_BLE_BuildBaro(const FS_Baro_Data_t *src, int32_t ref_pressure, uint8_t *dst);
uint8_t  Sensor_BLE_BuildHum(const FS_Hum_Data_t *src, uint8_t *dst);
uint8_t  Sensor_BLE_BuildAccel(const FS_IMU_Data_t *src, uint8_t *dst);
uint8_t  Sensor_BLE_BuildGyro(const FS_IMU_Data_t *src, uint8_t *dst);
uint8_t  Sensor_BLE_BuildMag(const FS_Mag_Data_t *src, uint8_t *dst);

#endif /* SENSOR_BLE_H_ */
