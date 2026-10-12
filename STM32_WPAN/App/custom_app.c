/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "crs.h"
#include "start_control.h"
#include "gnss_ble.h"
#include "sensor_ble.h"
#include "mode.h"
#include "ble_tx_queue.h"
#include "sensor_data.h"
#include "device_state.h"
#include "control_point_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* File_Transfer */
  uint8_t               Ft_packet_out_Notification_Status;
  /* Sensor_Data */
  uint8_t               Sd_gnss_measurement_Notification_Status;
  uint8_t               Sd_control_point_Indication_Status;
  uint8_t               Sd_baro_measurement_Notification_Status;
  uint8_t               Sd_hum_measurement_Notification_Status;
  uint8_t               Sd_accel_measurement_Notification_Status;
  uint8_t               Sd_gyro_measurement_Notification_Status;
  uint8_t               Sd_mag_measurement_Notification_Status;
  /* Starter_Pistol */
  uint8_t               Sp_control_point_Indication_Status;
  uint8_t               Sp_result_Indication_Status;
  /* Device_State */
  uint8_t               Ds_mode_Indication_Status;
  uint8_t               Ds_control_point_Indication_Status;
  /* Battery */
  uint8_t               Battery_level_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */
typedef struct
{
  Custom_STM_Char_Opcode_t opcode;
  uint8_t                 *size_ptr;
  uint8_t                 *notification_status;
  BLE_TX_Queue_callback_t  tx_complete;
  uint16_t                 sample_count;  // Samples since the last packet
  volatile uint8_t         tx_pending;    // Packet waiting in the TX queue
} Custom_Sensor_Stream_t;
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Sensor packets are only queued while this many TX queue slots remain free,
// so GNSS, control point responses and mode updates always have room
#define SENSOR_TX_RESERVE 3
/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TIMEOUT_MSEC  30000
#define TIMEOUT_TICKS (TIMEOUT_MSEC*1000/CFG_TS_TICK_VAL)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */
static Custom_CRS_Packet_t rx_buffer[FS_CRS_WINDOW_LENGTH+1];
static uint32_t rx_read_index, rx_write_index;

static uint8_t gnss_pv_packet[GNSS_BLE_MAX_LEN];

static Custom_Sensor_Stream_t sensor_streams[SENSOR_BLE_COUNT];

// Latest samples, written from the sensor (I2C DMA) interrupts
static FS_Baro_Data_t baro_sample;
static FS_Hum_Data_t hum_sample;
static FS_IMU_Data_t imu_sample;
static FS_Mag_Data_t mag_sample;
static int32_t baro_ref_pressure;
static uint8_t sensor_valid;
static uint8_t sensor_ready;

static uint8_t start_result_packet[9];

static uint8_t connected_flag = 0;

extern uint8_t SizeSd_Gnss_Measurement;
extern uint8_t SizeSd_Control_Point;

static uint8_t timeout_timer_id;

static uint8_t current_battery_level_percent = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* File_Transfer */
static void Custom_Ft_packet_out_Update_Char(void);
static void Custom_Ft_packet_out_Send_Notification(void);
/* Sensor_Data */
static void Custom_Sd_gnss_measurement_Update_Char(void);
static void Custom_Sd_gnss_measurement_Send_Notification(void);
static void Custom_Sd_control_point_Update_Char(void);
static void Custom_Sd_control_point_Send_Indication(void);
static void Custom_Sd_baro_measurement_Update_Char(void);
static void Custom_Sd_baro_measurement_Send_Notification(void);
static void Custom_Sd_hum_measurement_Update_Char(void);
static void Custom_Sd_hum_measurement_Send_Notification(void);
static void Custom_Sd_accel_measurement_Update_Char(void);
static void Custom_Sd_accel_measurement_Send_Notification(void);
static void Custom_Sd_gyro_measurement_Update_Char(void);
static void Custom_Sd_gyro_measurement_Send_Notification(void);
static void Custom_Sd_mag_measurement_Update_Char(void);
static void Custom_Sd_mag_measurement_Send_Notification(void);
/* Starter_Pistol */
static void Custom_Sp_control_point_Update_Char(void);
static void Custom_Sp_control_point_Send_Indication(void);
static void Custom_Sp_result_Update_Char(void);
static void Custom_Sp_result_Send_Indication(void);
/* Device_State */
static void Custom_Ds_mode_Update_Char(void);
static void Custom_Ds_mode_Send_Indication(void);
static void Custom_Ds_control_point_Update_Char(void);
static void Custom_Ds_control_point_Send_Indication(void);
/* Battery */
static void Custom_Battery_level_Update_Char(void);
static void Custom_Battery_level_Send_Notification(void);

/* USER CODE BEGIN PFP */
static void Custom_CRS_OnConnect(Custom_App_ConnHandle_Not_evt_t *pNotification);
static void Custom_CRS_OnDisconnect(void);
static void Custom_CRS_OnRxWrite(Custom_STM_App_Notification_evt_t *pNotification);
static void Custom_App_Timeout(void);
static void Custom_Sensor_Init(void);
static void Custom_Sensor_Sampled(Sensor_BLE_Stream_t stream);
static uint8_t Custom_Sensor_Build(Sensor_BLE_Stream_t stream, uint8_t *dst);
static void Custom_Sensor_Read(Sensor_BLE_Stream_t stream);
static void Custom_Sensor_NotifyEnabled(Sensor_BLE_Stream_t stream);
static void Custom_Sensor_Transmit(void);
static void Custom_Baro_TxComplete(void);
static void Custom_Hum_TxComplete(void);
static void Custom_Accel_TxComplete(void);
static void Custom_Gyro_TxComplete(void);
static void Custom_Mag_TxComplete(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* File_Transfer */
    case CUSTOM_STM_FT_PACKET_OUT_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_FT_PACKET_OUT_NOTIFY_ENABLED_EVT */
      Custom_App_Context.Ft_packet_out_Notification_Status = 1;
      /* USER CODE END CUSTOM_STM_FT_PACKET_OUT_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_FT_PACKET_OUT_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_FT_PACKET_OUT_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Ft_packet_out_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_FT_PACKET_OUT_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_FT_PACKET_IN_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_FT_PACKET_IN_READ_EVT */

      /* USER CODE END CUSTOM_STM_FT_PACKET_IN_READ_EVT */
      break;

    case CUSTOM_STM_FT_PACKET_IN_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_FT_PACKET_IN_WRITE_NO_RESP_EVT */
      Custom_CRS_OnRxWrite(pNotification);
      /* USER CODE END CUSTOM_STM_FT_PACKET_IN_WRITE_NO_RESP_EVT */
      break;

    /* Sensor_Data */
    case CUSTOM_STM_SD_GNSS_MEASUREMENT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_GNSS_MEASUREMENT_READ_EVT */
      if (!Custom_App_Context.Sd_gnss_measurement_Notification_Status)
      {
        Custom_STM_App_Update_Char(CUSTOM_STM_SD_GNSS_MEASUREMENT, gnss_pv_packet);
      }
      /* USER CODE END CUSTOM_STM_SD_GNSS_MEASUREMENT_READ_EVT */
      break;

    case CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_ENABLED_EVT */
      Custom_App_Context.Sd_gnss_measurement_Notification_Status = 1;
      /* USER CODE END CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Sd_gnss_measurement_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_SD_GNSS_MEASUREMENT_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_SD_CONTROL_POINT_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_CONTROL_POINT_WRITE_EVT */
      SensorData_Handle_SD_ControlPointWrite(
          pNotification->DataTransfered.pPayload,
          pNotification->DataTransfered.Length,
          pNotification->ConnectionHandle,
          Custom_App_Context.Sd_control_point_Indication_Status);
      /* USER CODE END CUSTOM_STM_SD_CONTROL_POINT_WRITE_EVT */
      break;

    case CUSTOM_STM_SD_CONTROL_POINT_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_CONTROL_POINT_INDICATE_ENABLED_EVT */
      Custom_App_Context.Sd_control_point_Indication_Status = 1;
      /* USER CODE END CUSTOM_STM_SD_CONTROL_POINT_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_SD_CONTROL_POINT_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_CONTROL_POINT_INDICATE_DISABLED_EVT */
      Custom_App_Context.Sd_control_point_Indication_Status = 0;
      /* USER CODE END CUSTOM_STM_SD_CONTROL_POINT_INDICATE_DISABLED_EVT */
      break;

    case CUSTOM_STM_SD_BARO_MEASUREMENT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_BARO_MEASUREMENT_READ_EVT */
      Custom_Sensor_Read(SENSOR_BLE_BARO);
      /* USER CODE END CUSTOM_STM_SD_BARO_MEASUREMENT_READ_EVT */
      break;

    case CUSTOM_STM_SD_BARO_MEASUREMENT_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_BARO_MEASUREMENT_NOTIFY_ENABLED_EVT */
      Custom_Sensor_NotifyEnabled(SENSOR_BLE_BARO);
      /* USER CODE END CUSTOM_STM_SD_BARO_MEASUREMENT_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SD_BARO_MEASUREMENT_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_BARO_MEASUREMENT_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Sd_baro_measurement_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_SD_BARO_MEASUREMENT_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_SD_HUM_MEASUREMENT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_HUM_MEASUREMENT_READ_EVT */
      Custom_Sensor_Read(SENSOR_BLE_HUM);
      /* USER CODE END CUSTOM_STM_SD_HUM_MEASUREMENT_READ_EVT */
      break;

    case CUSTOM_STM_SD_HUM_MEASUREMENT_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_HUM_MEASUREMENT_NOTIFY_ENABLED_EVT */
      Custom_Sensor_NotifyEnabled(SENSOR_BLE_HUM);
      /* USER CODE END CUSTOM_STM_SD_HUM_MEASUREMENT_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SD_HUM_MEASUREMENT_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_HUM_MEASUREMENT_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Sd_hum_measurement_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_SD_HUM_MEASUREMENT_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_SD_ACCEL_MEASUREMENT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_ACCEL_MEASUREMENT_READ_EVT */
      Custom_Sensor_Read(SENSOR_BLE_ACCEL);
      /* USER CODE END CUSTOM_STM_SD_ACCEL_MEASUREMENT_READ_EVT */
      break;

    case CUSTOM_STM_SD_ACCEL_MEASUREMENT_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_ACCEL_MEASUREMENT_NOTIFY_ENABLED_EVT */
      Custom_Sensor_NotifyEnabled(SENSOR_BLE_ACCEL);
      /* USER CODE END CUSTOM_STM_SD_ACCEL_MEASUREMENT_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SD_ACCEL_MEASUREMENT_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_ACCEL_MEASUREMENT_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Sd_accel_measurement_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_SD_ACCEL_MEASUREMENT_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_SD_GYRO_MEASUREMENT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_GYRO_MEASUREMENT_READ_EVT */
      Custom_Sensor_Read(SENSOR_BLE_GYRO);
      /* USER CODE END CUSTOM_STM_SD_GYRO_MEASUREMENT_READ_EVT */
      break;

    case CUSTOM_STM_SD_GYRO_MEASUREMENT_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_GYRO_MEASUREMENT_NOTIFY_ENABLED_EVT */
      Custom_Sensor_NotifyEnabled(SENSOR_BLE_GYRO);
      /* USER CODE END CUSTOM_STM_SD_GYRO_MEASUREMENT_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SD_GYRO_MEASUREMENT_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_GYRO_MEASUREMENT_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Sd_gyro_measurement_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_SD_GYRO_MEASUREMENT_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_SD_MAG_MEASUREMENT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_MAG_MEASUREMENT_READ_EVT */
      Custom_Sensor_Read(SENSOR_BLE_MAG);
      /* USER CODE END CUSTOM_STM_SD_MAG_MEASUREMENT_READ_EVT */
      break;

    case CUSTOM_STM_SD_MAG_MEASUREMENT_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_MAG_MEASUREMENT_NOTIFY_ENABLED_EVT */
      Custom_Sensor_NotifyEnabled(SENSOR_BLE_MAG);
      /* USER CODE END CUSTOM_STM_SD_MAG_MEASUREMENT_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SD_MAG_MEASUREMENT_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SD_MAG_MEASUREMENT_NOTIFY_DISABLED_EVT */
      Custom_App_Context.Sd_mag_measurement_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_SD_MAG_MEASUREMENT_NOTIFY_DISABLED_EVT */
      break;

    /* Starter_Pistol */
    case CUSTOM_STM_SP_CONTROL_POINT_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SP_CONTROL_POINT_WRITE_EVT */
      FS_StartControl_Handle_SP_ControlPointWrite(
          pNotification->DataTransfered.pPayload,
          pNotification->DataTransfered.Length,
          pNotification->ConnectionHandle,
          Custom_App_Context.Sp_control_point_Indication_Status);
      /* USER CODE END CUSTOM_STM_SP_CONTROL_POINT_WRITE_EVT */
      break;

    case CUSTOM_STM_SP_CONTROL_POINT_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SP_CONTROL_POINT_INDICATE_ENABLED_EVT */
      Custom_App_Context.Sp_control_point_Indication_Status = 1;
      /* USER CODE END CUSTOM_STM_SP_CONTROL_POINT_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_SP_CONTROL_POINT_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SP_CONTROL_POINT_INDICATE_DISABLED_EVT */
      Custom_App_Context.Sp_control_point_Indication_Status = 0;
      /* USER CODE END CUSTOM_STM_SP_CONTROL_POINT_INDICATE_DISABLED_EVT */
      break;

    case CUSTOM_STM_SP_RESULT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SP_RESULT_READ_EVT */
      if (!Custom_App_Context.Sp_result_Indication_Status)
      {
        Custom_STM_App_Update_Char(CUSTOM_STM_SP_RESULT, start_result_packet);
      }
      /* USER CODE END CUSTOM_STM_SP_RESULT_READ_EVT */
      break;

    case CUSTOM_STM_SP_RESULT_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SP_RESULT_INDICATE_ENABLED_EVT */
      Custom_App_Context.Sp_result_Indication_Status = 1;
      /* USER CODE END CUSTOM_STM_SP_RESULT_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_SP_RESULT_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SP_RESULT_INDICATE_DISABLED_EVT */
      Custom_App_Context.Sp_result_Indication_Status = 0;
      /* USER CODE END CUSTOM_STM_SP_RESULT_INDICATE_DISABLED_EVT */
      break;

    /* Device_State */
    case CUSTOM_STM_DS_MODE_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DS_MODE_READ_EVT */
      uint8_t modeVal = (uint8_t) FS_Mode_State();  // Get current state from mode module
      Custom_STM_App_Update_Char(CUSTOM_STM_DS_MODE, &modeVal);
      /* USER CODE END CUSTOM_STM_DS_MODE_READ_EVT */
      break;

    case CUSTOM_STM_DS_MODE_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DS_MODE_INDICATE_ENABLED_EVT */
      Custom_App_Context.Ds_mode_Indication_Status = 1;
      /* USER CODE END CUSTOM_STM_DS_MODE_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_DS_MODE_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DS_MODE_INDICATE_DISABLED_EVT */
      Custom_App_Context.Ds_mode_Indication_Status = 0;
      /* USER CODE END CUSTOM_STM_DS_MODE_INDICATE_DISABLED_EVT */
      break;

    case CUSTOM_STM_DS_CONTROL_POINT_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DS_CONTROL_POINT_WRITE_EVT */
      DeviceState_Handle_DS_ControlPointWrite(
          pNotification->DataTransfered.pPayload,
          pNotification->DataTransfered.Length,
          pNotification->ConnectionHandle,
          Custom_App_Context.Ds_control_point_Indication_Status);
      /* USER CODE END CUSTOM_STM_DS_CONTROL_POINT_WRITE_EVT */
      break;

    case CUSTOM_STM_DS_CONTROL_POINT_INDICATE_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DS_CONTROL_POINT_INDICATE_ENABLED_EVT */
      Custom_App_Context.Ds_control_point_Indication_Status = 1;
      /* USER CODE END CUSTOM_STM_DS_CONTROL_POINT_INDICATE_ENABLED_EVT */
      break;

    case CUSTOM_STM_DS_CONTROL_POINT_INDICATE_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DS_CONTROL_POINT_INDICATE_DISABLED_EVT */
      Custom_App_Context.Ds_control_point_Indication_Status = 0;
      /* USER CODE END CUSTOM_STM_DS_CONTROL_POINT_INDICATE_DISABLED_EVT */
      break;

    /* Battery */
    case CUSTOM_STM_BATTERY_LEVEL_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BATTERY_LEVEL_READ_EVT */
      APP_DBG_MSG("CUSTOM_STM_BATTERY_LEVEL_READ_EVT received\n");
      // The actual calculation and update should happen when FS_VBAT_ValueReady_Callback runs,
      // or we need a way to trigger FS_VBAT_ValueReady_Callback or get the latest value here.
      // For simplicity, let's assume current_battery_level_percent is up-to-date.
      Custom_STM_App_Update_Char(CUSTOM_STM_BATTERY_LEVEL, &current_battery_level_percent);
      /* USER CODE END CUSTOM_STM_BATTERY_LEVEL_READ_EVT */
      break;

    case CUSTOM_STM_BATTERY_LEVEL_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BATTERY_LEVEL_NOTIFY_ENABLED_EVT */
      APP_DBG_MSG("CUSTOM_STM_BATTERY_LEVEL_NOTIFY_ENABLED_EVT received\n");
      Custom_App_Context.Battery_level_Notification_Status = 1;
      // Optionally, send the current level immediately upon enabling notifications
      Custom_STM_App_Update_Char(CUSTOM_STM_BATTERY_LEVEL, &current_battery_level_percent);
      /* USER CODE END CUSTOM_STM_BATTERY_LEVEL_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_BATTERY_LEVEL_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BATTERY_LEVEL_NOTIFY_DISABLED_EVT */
      APP_DBG_MSG("CUSTOM_STM_BATTERY_LEVEL_NOTIFY_DISABLED_EVT received\n");
      Custom_App_Context.Battery_level_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_BATTERY_LEVEL_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      DeviceState_Handle_DS_ControlPointNotificationComplete(pNotification->AttrHandle);
      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */
      Custom_CRS_OnConnect(pNotification);
      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
      Custom_CRS_OnDisconnect();
      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
  BLE_TX_Queue_Init();

  Custom_Sensor_Init();

  SensorData_Init();
  DeviceState_Init();
  // Starter Pistol is initialized by the main mode logic when entering that mode.

  /* File_Transfer */
  Custom_App_Context.Ft_packet_out_Notification_Status = 0;

  /* Sensor_Data */
  Custom_App_Context.Sd_gnss_measurement_Notification_Status = 0;
  Custom_App_Context.Sd_control_point_Indication_Status = 0;
  Custom_App_Context.Sd_baro_measurement_Notification_Status = 0;
  Custom_App_Context.Sd_hum_measurement_Notification_Status = 0;
  Custom_App_Context.Sd_accel_measurement_Notification_Status = 0;
  Custom_App_Context.Sd_gyro_measurement_Notification_Status = 0;
  Custom_App_Context.Sd_mag_measurement_Notification_Status = 0;

  /* Starter_Pistol */
  Custom_App_Context.Sp_control_point_Indication_Status = 0;
  Custom_App_Context.Sp_result_Indication_Status = 0;

  /* Device_State */
  Custom_App_Context.Ds_mode_Indication_Status = 0;
  Custom_App_Context.Ds_control_point_Indication_Status = 0;

  /* Battery Service */
  Custom_App_Context.Battery_level_Notification_Status = 0;
  current_battery_level_percent = 0;

  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &timeout_timer_id, hw_ts_SingleShot, Custom_App_Timeout);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void Custom_APP_TxPoolAvailableNotification(void)
{
  BLE_TX_Queue_TxPoolAvailableNotification();
}

uint8_t Custom_APP_IsConnected(void)
{
  return connected_flag;
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* File_Transfer */
void Custom_Ft_packet_out_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ft_packet_out_UC_1*/

  /* USER CODE END Ft_packet_out_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_FT_PACKET_OUT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Ft_packet_out_UC_Last*/

  /* USER CODE END Ft_packet_out_UC_Last*/
  return;
}

void Custom_Ft_packet_out_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ft_packet_out_NS_1*/

  /* USER CODE END Ft_packet_out_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_FT_PACKET_OUT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Ft_packet_out_NS_Last*/

  /* USER CODE END Ft_packet_out_NS_Last*/

  return;
}

/* Sensor_Data */
void Custom_Sd_gnss_measurement_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_gnss_measurement_UC_1*/

  /* USER CODE END Sd_gnss_measurement_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_GNSS_MEASUREMENT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sd_gnss_measurement_UC_Last*/

  /* USER CODE END Sd_gnss_measurement_UC_Last*/
  return;
}

void Custom_Sd_gnss_measurement_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_gnss_measurement_NS_1*/

  /* USER CODE END Sd_gnss_measurement_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_GNSS_MEASUREMENT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sd_gnss_measurement_NS_Last*/

  /* USER CODE END Sd_gnss_measurement_NS_Last*/

  return;
}

void Custom_Sd_control_point_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_control_point_UC_1*/

  /* USER CODE END Sd_control_point_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_CONTROL_POINT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sd_control_point_UC_Last*/

  /* USER CODE END Sd_control_point_UC_Last*/
  return;
}

void Custom_Sd_control_point_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_control_point_IS_1*/

  /* USER CODE END Sd_control_point_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_CONTROL_POINT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sd_control_point_IS_Last*/

  /* USER CODE END Sd_control_point_IS_Last*/

  return;
}

void Custom_Sd_baro_measurement_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_baro_measurement_UC_1*/

  /* USER CODE END Sd_baro_measurement_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_BARO_MEASUREMENT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sd_baro_measurement_UC_Last*/

  /* USER CODE END Sd_baro_measurement_UC_Last*/
  return;
}

void Custom_Sd_baro_measurement_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_baro_measurement_NS_1*/

  /* USER CODE END Sd_baro_measurement_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_BARO_MEASUREMENT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sd_baro_measurement_NS_Last*/

  /* USER CODE END Sd_baro_measurement_NS_Last*/

  return;
}

void Custom_Sd_hum_measurement_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_hum_measurement_UC_1*/

  /* USER CODE END Sd_hum_measurement_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_HUM_MEASUREMENT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sd_hum_measurement_UC_Last*/

  /* USER CODE END Sd_hum_measurement_UC_Last*/
  return;
}

void Custom_Sd_hum_measurement_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_hum_measurement_NS_1*/

  /* USER CODE END Sd_hum_measurement_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_HUM_MEASUREMENT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sd_hum_measurement_NS_Last*/

  /* USER CODE END Sd_hum_measurement_NS_Last*/

  return;
}

void Custom_Sd_accel_measurement_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_accel_measurement_UC_1*/

  /* USER CODE END Sd_accel_measurement_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_ACCEL_MEASUREMENT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sd_accel_measurement_UC_Last*/

  /* USER CODE END Sd_accel_measurement_UC_Last*/
  return;
}

void Custom_Sd_accel_measurement_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_accel_measurement_NS_1*/

  /* USER CODE END Sd_accel_measurement_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_ACCEL_MEASUREMENT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sd_accel_measurement_NS_Last*/

  /* USER CODE END Sd_accel_measurement_NS_Last*/

  return;
}

void Custom_Sd_gyro_measurement_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_gyro_measurement_UC_1*/

  /* USER CODE END Sd_gyro_measurement_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_GYRO_MEASUREMENT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sd_gyro_measurement_UC_Last*/

  /* USER CODE END Sd_gyro_measurement_UC_Last*/
  return;
}

void Custom_Sd_gyro_measurement_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_gyro_measurement_NS_1*/

  /* USER CODE END Sd_gyro_measurement_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_GYRO_MEASUREMENT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sd_gyro_measurement_NS_Last*/

  /* USER CODE END Sd_gyro_measurement_NS_Last*/

  return;
}

void Custom_Sd_mag_measurement_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_mag_measurement_UC_1*/

  /* USER CODE END Sd_mag_measurement_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_MAG_MEASUREMENT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sd_mag_measurement_UC_Last*/

  /* USER CODE END Sd_mag_measurement_UC_Last*/
  return;
}

void Custom_Sd_mag_measurement_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sd_mag_measurement_NS_1*/

  /* USER CODE END Sd_mag_measurement_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SD_MAG_MEASUREMENT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sd_mag_measurement_NS_Last*/

  /* USER CODE END Sd_mag_measurement_NS_Last*/

  return;
}

/* Starter_Pistol */
void Custom_Sp_control_point_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sp_control_point_UC_1*/

  /* USER CODE END Sp_control_point_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SP_CONTROL_POINT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sp_control_point_UC_Last*/

  /* USER CODE END Sp_control_point_UC_Last*/
  return;
}

void Custom_Sp_control_point_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sp_control_point_IS_1*/

  /* USER CODE END Sp_control_point_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SP_CONTROL_POINT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sp_control_point_IS_Last*/

  /* USER CODE END Sp_control_point_IS_Last*/

  return;
}

void Custom_Sp_result_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sp_result_UC_1*/

  /* USER CODE END Sp_result_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SP_RESULT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Sp_result_UC_Last*/

  /* USER CODE END Sp_result_UC_Last*/
  return;
}

void Custom_Sp_result_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Sp_result_IS_1*/

  /* USER CODE END Sp_result_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SP_RESULT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Sp_result_IS_Last*/

  /* USER CODE END Sp_result_IS_Last*/

  return;
}

/* Device_State */
void Custom_Ds_mode_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ds_mode_UC_1*/

  /* USER CODE END Ds_mode_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DS_MODE, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Ds_mode_UC_Last*/

  /* USER CODE END Ds_mode_UC_Last*/
  return;
}

void Custom_Ds_mode_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ds_mode_IS_1*/

  /* USER CODE END Ds_mode_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DS_MODE, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Ds_mode_IS_Last*/

  /* USER CODE END Ds_mode_IS_Last*/

  return;
}

void Custom_Ds_control_point_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ds_control_point_UC_1*/

  /* USER CODE END Ds_control_point_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DS_CONTROL_POINT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Ds_control_point_UC_Last*/

  /* USER CODE END Ds_control_point_UC_Last*/
  return;
}

void Custom_Ds_control_point_Send_Indication(void) /* Property Indication */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Ds_control_point_IS_1*/

  /* USER CODE END Ds_control_point_IS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DS_CONTROL_POINT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Ds_control_point_IS_Last*/

  /* USER CODE END Ds_control_point_IS_Last*/

  return;
}

/* Battery */
void Custom_Battery_level_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Battery_level_UC_1*/

  /* USER CODE END Battery_level_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_BATTERY_LEVEL, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Battery_level_UC_Last*/

  /* USER CODE END Battery_level_UC_Last*/
  return;
}

void Custom_Battery_level_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Battery_level_NS_1*/

  /* USER CODE END Battery_level_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_BATTERY_LEVEL, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Battery_level_NS_Last*/

  /* USER CODE END Battery_level_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
static void Custom_CRS_OnConnect(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  // Reset buffer indices
  rx_read_index = 0;
  rx_write_index = 0;

  // Update state
  connected_flag = 1;

  // Remember connection handle
  Custom_App_Context.ConnectionHandle = pNotification->ConnectionHandle;

  // Start timeout timer
  APP_DBG_MSG("Start connection timeout\n");
  HW_TS_Start(timeout_timer_id, TIMEOUT_TICKS);
}

static void Custom_CRS_OnDisconnect(void)
{
  // Stop timeout timer
  APP_DBG_MSG("Stop connection timeout\n");
  HW_TS_Stop(timeout_timer_id);

  // Update state
  connected_flag = 0;

  // Reset per-connection CCCD mirrors so producers stop sending
  Custom_App_Context.Ft_packet_out_Notification_Status = 0;

  Custom_App_Context.Sd_gnss_measurement_Notification_Status = 0;
  Custom_App_Context.Sd_control_point_Indication_Status = 0;
  Custom_App_Context.Sd_baro_measurement_Notification_Status = 0;
  Custom_App_Context.Sd_hum_measurement_Notification_Status = 0;
  Custom_App_Context.Sd_accel_measurement_Notification_Status = 0;
  Custom_App_Context.Sd_gyro_measurement_Notification_Status = 0;
  Custom_App_Context.Sd_mag_measurement_Notification_Status = 0;

  Custom_App_Context.Sp_control_point_Indication_Status = 0;
  Custom_App_Context.Sp_result_Indication_Status = 0;

  Custom_App_Context.Ds_mode_Indication_Status = 0;
  Custom_App_Context.Ds_control_point_Indication_Status = 0;

  Custom_App_Context.Battery_level_Notification_Status = 0;

  // Call update task
  UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
}

static void Custom_CRS_OnRxWrite(Custom_STM_App_Notification_evt_t *pNotification)
{
  Custom_CRS_Packet_t *packet;

  if (rx_write_index < rx_read_index + FS_CRS_WINDOW_LENGTH)
  {
    APP_DBG_MSG("Received data on CRS_RX\n");

    packet = &rx_buffer[(rx_write_index++) % FS_CRS_WINDOW_LENGTH];
	packet->length = pNotification->DataTransfered.Length;
    memcpy(packet->data, pNotification->DataTransfered.pPayload, packet->length);

	// Call update task
	UTIL_SEQ_SetTask(1<<CFG_TASK_FS_CRS_UPDATE_ID, CFG_SCH_PRIO_1);
  }
  else
  {
    APP_DBG_MSG("Custom_CRS_OnRxWrite: buffer overflow\n");
  }

  // Refresh timeout timer
  HW_TS_Start(timeout_timer_id, TIMEOUT_TICKS);
}

Custom_CRS_Packet_t *Custom_CRS_GetNextRxPacket(void)
{
  Custom_CRS_Packet_t *ret = 0;

  if (rx_read_index < rx_write_index)
  {
	ret = &rx_buffer[(rx_read_index++) % FS_CRS_WINDOW_LENGTH];
  }

  return ret;
}

void Custom_GNSS_Update(const FS_GNSS_Data_t *current)
{
  uint8_t length = GNSS_BLE_Build(current, gnss_pv_packet);

  if (Custom_App_Context.Sd_gnss_measurement_Notification_Status)
  {
    BLE_TX_Queue_SendTxPacket(CUSTOM_STM_SD_GNSS_MEASUREMENT,
        gnss_pv_packet, length, &SizeSd_Gnss_Measurement, 0);
  }
}

static void Custom_Sensor_Init(void)
{
  static const struct
  {
    Custom_STM_Char_Opcode_t opcode;
    uint8_t *size_ptr;
    uint8_t *notification_status;
    BLE_TX_Queue_callback_t tx_complete;
  } init[SENSOR_BLE_COUNT] =
  {
    [SENSOR_BLE_BARO]  = { CUSTOM_STM_SD_BARO_MEASUREMENT, &SizeSd_Baro_Measurement,
        &Custom_App_Context.Sd_baro_measurement_Notification_Status, Custom_Baro_TxComplete },
    [SENSOR_BLE_HUM]   = { CUSTOM_STM_SD_HUM_MEASUREMENT, &SizeSd_Hum_Measurement,
        &Custom_App_Context.Sd_hum_measurement_Notification_Status, Custom_Hum_TxComplete },
    [SENSOR_BLE_ACCEL] = { CUSTOM_STM_SD_ACCEL_MEASUREMENT, &SizeSd_Accel_Measurement,
        &Custom_App_Context.Sd_accel_measurement_Notification_Status, Custom_Accel_TxComplete },
    [SENSOR_BLE_GYRO]  = { CUSTOM_STM_SD_GYRO_MEASUREMENT, &SizeSd_Gyro_Measurement,
        &Custom_App_Context.Sd_gyro_measurement_Notification_Status, Custom_Gyro_TxComplete },
    [SENSOR_BLE_MAG]   = { CUSTOM_STM_SD_MAG_MEASUREMENT, &SizeSd_Mag_Measurement,
        &Custom_App_Context.Sd_mag_measurement_Notification_Status, Custom_Mag_TxComplete }
  };
  uint8_t i;

  for (i = 0; i < SENSOR_BLE_COUNT; ++i)
  {
    sensor_streams[i].opcode = init[i].opcode;
    sensor_streams[i].size_ptr = init[i].size_ptr;
    sensor_streams[i].notification_status = init[i].notification_status;
    sensor_streams[i].tx_complete = init[i].tx_complete;
    sensor_streams[i].sample_count = 0;
    sensor_streams[i].tx_pending = 0;
  }

  sensor_valid = 0;
  sensor_ready = 0;

  UTIL_SEQ_RegTask(1<<CFG_TASK_FS_SENSOR_BLE_UPDATE_ID, UTIL_SEQ_RFU, Custom_Sensor_Transmit);
}

void Custom_Sensor_Reset(void)
{
  uint32_t primask_bit;
  uint8_t i;

  // Sensor time restarts with each session, so samples from an earlier
  // session must not be sent, and the barometer reference is taken again
  primask_bit = __get_PRIMASK();
  __disable_irq();
  for (i = 0; i < SENSOR_BLE_COUNT; ++i)
  {
    sensor_streams[i].sample_count = 0;
  }
  sensor_valid = 0;
  sensor_ready = 0;
  __set_PRIMASK(primask_bit);
}

void Custom_Baro_Update(const FS_Baro_Data_t *current)
{
  uint32_t primask_bit = __get_PRIMASK();
  __disable_irq();

  memcpy(&baro_sample, current, sizeof(FS_Baro_Data_t));

  // Remember the first sample of the session, so clients can use it as a
  // ground reference no matter when they connect
  if (!(sensor_valid & (1 << SENSOR_BLE_BARO)))
  {
    baro_ref_pressure = current->pressure;
  }

  sensor_valid |= (1 << SENSOR_BLE_BARO);
  Custom_Sensor_Sampled(SENSOR_BLE_BARO);

  __set_PRIMASK(primask_bit);
}

void Custom_Hum_Update(const FS_Hum_Data_t *current)
{
  uint32_t primask_bit = __get_PRIMASK();
  __disable_irq();

  memcpy(&hum_sample, current, sizeof(FS_Hum_Data_t));
  sensor_valid |= (1 << SENSOR_BLE_HUM);
  Custom_Sensor_Sampled(SENSOR_BLE_HUM);

  __set_PRIMASK(primask_bit);
}

void Custom_IMU_Update(const FS_IMU_Data_t *current)
{
  uint32_t primask_bit = __get_PRIMASK();
  __disable_irq();

  memcpy(&imu_sample, current, sizeof(FS_IMU_Data_t));
  sensor_valid |= (1 << SENSOR_BLE_ACCEL) | (1 << SENSOR_BLE_GYRO);
  Custom_Sensor_Sampled(SENSOR_BLE_ACCEL);
  Custom_Sensor_Sampled(SENSOR_BLE_GYRO);

  __set_PRIMASK(primask_bit);
}

void Custom_Mag_Update(const FS_Mag_Data_t *current)
{
  uint32_t primask_bit = __get_PRIMASK();
  __disable_irq();

  memcpy(&mag_sample, current, sizeof(FS_Mag_Data_t));
  sensor_valid |= (1 << SENSOR_BLE_MAG);
  Custom_Sensor_Sampled(SENSOR_BLE_MAG);

  __set_PRIMASK(primask_bit);
}

static void Custom_Sensor_Sampled(Sensor_BLE_Stream_t stream)
{
  Custom_Sensor_Stream_t *s = &sensor_streams[stream];

  // Called with interrupts disabled. Only wake the transmit task when a
  // client is listening, so streaming costs nothing otherwise.
  if (!*s->notification_status) return;

  if (++s->sample_count >= Sensor_BLE_GetDivider(stream))
  {
    s->sample_count = 0;
    sensor_ready |= (1 << stream);
    UTIL_SEQ_SetTask(1<<CFG_TASK_FS_SENSOR_BLE_UPDATE_ID, CFG_SCH_PRIO_1);
  }
}

static uint8_t Custom_Sensor_Build(Sensor_BLE_Stream_t stream, uint8_t *dst)
{
  union
  {
    FS_Baro_Data_t baro;
    FS_Hum_Data_t hum;
    FS_IMU_Data_t imu;
    FS_Mag_Data_t mag;
  } sample;
  int32_t ref_pressure = 0;
  uint32_t primask_bit;

  // Take a consistent copy of the latest sample
  primask_bit = __get_PRIMASK();
  __disable_irq();
  switch (stream)
  {
  case SENSOR_BLE_BARO:
    memcpy(&sample.baro, &baro_sample, sizeof(FS_Baro_Data_t));
    ref_pressure = baro_ref_pressure;
    break;
  case SENSOR_BLE_HUM:
    memcpy(&sample.hum, &hum_sample, sizeof(FS_Hum_Data_t));
    break;
  case SENSOR_BLE_ACCEL:
  case SENSOR_BLE_GYRO:
    memcpy(&sample.imu, &imu_sample, sizeof(FS_IMU_Data_t));
    break;
  case SENSOR_BLE_MAG:
  default:
    memcpy(&sample.mag, &mag_sample, sizeof(FS_Mag_Data_t));
    break;
  }
  __set_PRIMASK(primask_bit);

  switch (stream)
  {
  case SENSOR_BLE_BARO:
    return Sensor_BLE_BuildBaro(&sample.baro, ref_pressure, dst);
  case SENSOR_BLE_HUM:
    return Sensor_BLE_BuildHum(&sample.hum, dst);
  case SENSOR_BLE_ACCEL:
    return Sensor_BLE_BuildAccel(&sample.imu, dst);
  case SENSOR_BLE_GYRO:
    return Sensor_BLE_BuildGyro(&sample.imu, dst);
  case SENSOR_BLE_MAG:
  default:
    return Sensor_BLE_BuildMag(&sample.mag, dst);
  }
}

static void Custom_Sensor_Read(Sensor_BLE_Stream_t stream)
{
  Custom_Sensor_Stream_t *s = &sensor_streams[stream];
  uint8_t packet[SENSOR_BLE_MAX_LEN];

  // While notifications are enabled, the value is kept up to date by the
  // notifications themselves. Before the first sample, the value is empty.
  if (!*s->notification_status && (sensor_valid & (1 << stream)))
  {
    *s->size_ptr = Custom_Sensor_Build(stream, packet);
    Custom_STM_App_Update_Char(s->opcode, packet);
  }
}

static void Custom_Sensor_NotifyEnabled(Sensor_BLE_Stream_t stream)
{
  Custom_Sensor_Stream_t *s = &sensor_streams[stream];

  s->tx_pending = 0;
  *s->notification_status = 1;
}

static void Custom_Sensor_Transmit(void)
{
  Custom_Sensor_Stream_t *s;
  uint32_t primask_bit;
  uint8_t ready;
  uint8_t stream;
  uint8_t *tx_packet;

  primask_bit = __get_PRIMASK();
  __disable_irq();
  ready = sensor_ready;
  sensor_ready = 0;
  __set_PRIMASK(primask_bit);

  for (stream = 0; stream < SENSOR_BLE_COUNT; ++stream)
  {
    s = &sensor_streams[stream];

    // Keep at most one packet per stream in the TX queue. If the link can't
    // keep up with the sensor rate, samples are skipped instead of crowding
    // out GNSS packets.
    if ((ready & (1 << stream))
        && *s->notification_status
        && !s->tx_pending
        && (BLE_TX_Queue_GetFreeCount() > SENSOR_TX_RESERVE)
        && (tx_packet = BLE_TX_Queue_GetNextTxPacket()))
    {
      s->tx_pending = 1;
      BLE_TX_Queue_SendNextTxPacket(s->opcode,
          Custom_Sensor_Build(stream, tx_packet), s->size_ptr, s->tx_complete);
    }
  }
}

static void Custom_Baro_TxComplete(void)
{
  sensor_streams[SENSOR_BLE_BARO].tx_pending = 0;
}

static void Custom_Hum_TxComplete(void)
{
  sensor_streams[SENSOR_BLE_HUM].tx_pending = 0;
}

static void Custom_Accel_TxComplete(void)
{
  sensor_streams[SENSOR_BLE_ACCEL].tx_pending = 0;
}

static void Custom_Gyro_TxComplete(void)
{
  sensor_streams[SENSOR_BLE_GYRO].tx_pending = 0;
}

static void Custom_Mag_TxComplete(void)
{
  sensor_streams[SENSOR_BLE_MAG].tx_pending = 0;
}

void Custom_Start_Update(uint16_t year, uint8_t month, uint8_t day,
                         uint8_t hour, uint8_t min, uint8_t sec, uint16_t ms)
{
  // Copy to packet
  memcpy(&start_result_packet[0], &year, sizeof(year));
  memcpy(&start_result_packet[2], &month, sizeof(month));
  memcpy(&start_result_packet[3], &day, sizeof(day));
  memcpy(&start_result_packet[4], &hour, sizeof(hour));
  memcpy(&start_result_packet[5], &min, sizeof(min));
  memcpy(&start_result_packet[6], &sec, sizeof(sec));
  memcpy(&start_result_packet[7], &ms, sizeof(ms));

  if (Custom_App_Context.Sp_result_Indication_Status)
  {
    BLE_TX_Queue_SendTxPacket(CUSTOM_STM_SP_RESULT,
        start_result_packet, SizeSp_Result, 0, 0);
  }
}

static void Custom_App_Timeout(void)
{
  APP_DBG_MSG("Connection timeout triggered\n");
  aci_gap_terminate(Custom_App_Context.ConnectionHandle,
		  HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE);
}

void Custom_Mode_Update(uint8_t newMode)
{
  /*
   * Even if Mode_Notification_Status == 0 (notifications off),
   * we still call BLE_TX_Queue_SendTxPacket so that the
   * characteristic value is always up-to-date when read.
   */
  BLE_TX_Queue_SendTxPacket(CUSTOM_STM_DS_MODE,
      &newMode, sizeof(newMode), &SizeDs_Mode, 0);
}

static uint8_t calculate_battery_percentage(uint16_t voltage_mv) {
  const uint16_t MIN_VOLTAGE_MV = 3300; // Example for LiPo empty
  const uint16_t MAX_VOLTAGE_MV = 4200; // Example for LiPo full
  int32_t percentage;

  if (voltage_mv <= MIN_VOLTAGE_MV) {
    percentage = 0;
  } else if (voltage_mv >= MAX_VOLTAGE_MV) {
    percentage = 100;
  } else {
    percentage = ((int32_t)voltage_mv - MIN_VOLTAGE_MV) * 100 / (MAX_VOLTAGE_MV - MIN_VOLTAGE_MV);
  }
  return (uint8_t)percentage;
}

void Custom_VBAT_Update(const FS_VBAT_Data_t *current)
{
  uint8_t new_battery_level = calculate_battery_percentage(current->voltage);

  // Update the global/context variable
  // This ensures that read requests get the most recent value
  current_battery_level_percent = new_battery_level;

  // If notifications are enabled for the Battery Level characteristic, send an update
  if (Custom_App_Context.Battery_level_Notification_Status)
  {
    // APP_DBG_MSG("Sending battery level notification: %d%%\n", new_battery_level);
    BLE_TX_Queue_SendTxPacket(CUSTOM_STM_BATTERY_LEVEL,
        &new_battery_level, sizeof(new_battery_level), &SizeBattery_Level, 0);
  }
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
