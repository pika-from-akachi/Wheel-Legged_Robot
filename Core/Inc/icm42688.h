/**
  ******************************************************************************
  * @file    icm42688.h
  * @brief   ICM-42688-P 6-axis IMU Driver for Wheel-Legged Robot
  * @note    SPI Interface: PA5=SCLK, PA6=MISO, PA7=MOSI, PA4=CS
  *
  *          Hardware Connection (STM32F407):
  *          ICM42688P Module    STM32F407
  *          ─────────────────────────────
  *          VCC         ───►   3.3V
  *          GND         ───►   GND
  *          CS          ───►   PA4 (GPIO)
  *          SCLK        ───►   PA5 (SPI1_SCK)
  *          MISO        ───►   PA6 (SPI1_MISO)
  *          MOSI        ───►   PA7 (SPI1_MOSI)
  *          INT1        ───►   Optional (data ready interrupt)
  *
  * @author  Mingyue Class 2026
  * @date    2026-04-27
  ******************************************************************************
  */

#ifndef __ICM42688_H__
#define __ICM42688_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 *                          HARDWARE PIN DEFINITIONS
 * ============================================================================ */

#define ICM42688_SPI                    hspi1
extern SPI_HandleTypeDef hspi1;

#define ICM42688_CS_PORT                GPIOA
#define ICM42688_CS_PIN                 GPIO_PIN_4

/* ============================================================================
 *                          ICM42688 CONFIGURATION
 * ============================================================================ */

#define ICM42688_SPI_TIMEOUT            100

/* Device Identification */
#define ICM42688_WHO_AM_I               0x47

/* Register Bank Selection */
#define ICM42688_REG_BANK_SEL           0x76

/* Bank 0 Registers */
#define ICM42688_REG_DEVICE_CONFIG      0x11
#define ICM42688_REG_INT_CONFIG         0x14
#define ICM42688_REG_FIFO_CONFIG        0x16
#define ICM42688_REG_TEMP_DATA1         0x1D
#define ICM42688_REG_TEMP_DATA0         0x1E
#define ICM42688_REG_ACCEL_DATA_X1      0x1F
#define ICM42688_REG_ACCEL_DATA_X0      0x20
#define ICM42688_REG_ACCEL_DATA_Y1      0x21
#define ICM42688_REG_ACCEL_DATA_Y0      0x22
#define ICM42688_REG_ACCEL_DATA_Z1      0x23
#define ICM42688_REG_ACCEL_DATA_Z0      0x24
#define ICM42688_REG_GYRO_DATA_X1       0x25
#define ICM42688_REG_GYRO_DATA_X0       0x26
#define ICM42688_REG_GYRO_DATA_Y1       0x27
#define ICM42688_REG_GYRO_DATA_Y0       0x28
#define ICM42688_REG_GYRO_DATA_Z1       0x29
#define ICM42688_REG_GYRO_DATA_Z0       0x2A
#define ICM42688_REG_FIFO_DATA          0x30
#define ICM42688_REG_INTF_CONFIG0       0x4C
#define ICM42688_REG_INTF_CONFIG6       0x52
#define ICM42688_REG_PWR_MGMT0          0x4E
#define ICM42688_REG_GYRO_CONFIG0       0x4F
#define ICM42688_REG_ACCEL_CONFIG0      0x50
#define ICM42688_REG_INT_CONFIG1        0x54
#define ICM42688_REG_INT_SOURCE0        0x55
#define ICM42688_REG_INT_SOURCE1        0x56
#define ICM42688_REG_INT_SOURCE3        0x58
#define ICM42688_REG_INT_SOURCE4        0x59
#define ICM42688_REG_FIFO_LOST_PKT0     0x5C
#define ICM42688_REG_FIFO_LOST_PKT1     0x5D
#define ICM42688_REG_FIFO_CONFIG1       0x5F
#define ICM42688_REG_FIFO_WM1           0x60
#define ICM42688_REG_FIFO_WM2           0x61
#define ICM42688_REG_MCLK_RDY           0x64
#define ICM42688_REG_WHO_AM_I           0x75

/* Bank 1 Registers (Filter Configuration) */
#define ICM42688_REG_GYRO_UI_FILT_ORD   0x28
#define ICM42688_REG_GYRO_UI_FILT_BW    0x29
#define ICM42688_REG_ACCEL_UI_FILT_ORD  0x2A
#define ICM42688_REG_ACCEL_UI_FILT_BW   0x2B

/* Bank 4 Registers (APEX / Offset) */
#define ICM42688_REG_APEX_CONFIG0       0x05
#define ICM42688_REG_APEX_CONFIG1       0x06
#define ICM42688_REG_OFFSET_USER0       0x77
#define ICM42688_REG_OFFSET_USER1       0x78
#define ICM42688_REG_OFFSET_USER2       0x79
#define ICM42688_REG_OFFSET_USER3       0x7A
#define ICM42688_REG_OFFSET_USER4       0x7B
#define ICM42688_REG_OFFSET_USER5       0x7C
#define ICM42688_REG_OFFSET_USER6       0x7D
#define ICM42688_REG_OFFSET_USER7       0x7E
#define ICM42688_REG_OFFSET_USER8       0x7F

/* ============================================================================
 *                          GYRO FULL SCALE SETTINGS
 * ============================================================================ */

typedef enum {
    ICM42688_GYRO_FS_2000DPS = 0,   /* ±2000 dps */
    ICM42688_GYRO_FS_1000DPS = 1,   /* ±1000 dps */
    ICM42688_GYRO_FS_500DPS  = 2,   /* ±500 dps */
    ICM42688_GYRO_FS_250DPS  = 3,   /* ±250 dps */
    ICM42688_GYRO_FS_125DPS  = 4,   /* ±125 dps */
    ICM42688_GYRO_FS_62_5DPS = 5,   /* ±62.5 dps */
    ICM42688_GYRO_FS_31_25DPS = 6,  /* ±31.25 dps */
    ICM42688_GYRO_FS_15_625DPS = 7  /* ±15.625 dps */
} ICM42688_GyroFS_t;

/* ============================================================================
 *                          ACCEL FULL SCALE SETTINGS
 * ============================================================================ */

typedef enum {
    ICM42688_ACCEL_FS_16G = 0,      /* ±16 g */
    ICM42688_ACCEL_FS_8G  = 1,      /* ±8 g */
    ICM42688_ACCEL_FS_4G  = 2,      /* ±4 g */
    ICM42688_ACCEL_FS_2G  = 3       /* ±2 g */
} ICM42688_AccelFS_t;

/* ============================================================================
 *                          ODR (Output Data Rate) SETTINGS
 * ============================================================================ */

typedef enum {
    ICM42688_ODR_32KHZ    = 1,      /* 32 kHz */
    ICM42688_ODR_16KHZ    = 2,      /* 16 kHz */
    ICM42688_ODR_8KHZ     = 3,      /* 8 kHz */
    ICM42688_ODR_4KHZ     = 4,      /* 4 kHz */
    ICM42688_ODR_2KHZ     = 5,      /* 2 kHz */
    ICM42688_ODR_1KHZ     = 6,      /* 1 kHz (default) */
    ICM42688_ODR_200HZ    = 7,      /* 200 Hz */
    ICM42688_ODR_100HZ    = 8,      /* 100 Hz */
    ICM42688_ODR_50HZ     = 9,      /* 50 Hz */
    ICM42688_ODR_25HZ     = 10,     /* 25 Hz */
    ICM42688_ODR_12_5HZ   = 11,     /* 12.5 Hz */
    ICM42688_ODR_6_25HZ   = 12,     /* 6.25 Hz (LN mode) */
    ICM42688_ODR_3_125HZ  = 13,     /* 3.125 Hz (LN mode) */
    ICM42688_ODR_1_5625HZ = 14,     /* 1.5625 Hz (LN mode) */
    ICM42688_ODR_500HZ    = 15      /* 500 Hz */
} ICM42688_ODR_t;

/* ============================================================================
 *                          POWER MODE SETTINGS
 * ============================================================================ */

typedef enum {
    ICM42688_GYRO_MODE_OFF      = 0,    /* Gyro Off */
    ICM42688_GYRO_MODE_STANDBY  = 1,    /* Gyro Standby */
    ICM42688_GYRO_MODE_LN       = 3     /* Gyro Low-Noise Mode */
} ICM42688_GyroMode_t;

typedef enum {
    ICM42688_ACCEL_MODE_OFF     = 0,    /* Accel Off */
    ICM42688_ACCEL_MODE_LP      = 2,    /* Accel Low-Power Mode */
    ICM42688_ACCEL_MODE_LN      = 3     /* Accel Low-Noise Mode */
} ICM42688_AccelMode_t;

/* ============================================================================
 *                          DATA STRUCTURES
 * ============================================================================ */

/**
  * @brief Raw sensor data structure (16-bit signed integers)
  */
typedef struct {
    int16_t accel_x;        /* Acceleration X (raw) */
    int16_t accel_y;        /* Acceleration Y (raw) */
    int16_t accel_z;        /* Acceleration Z (raw) */
    int16_t gyro_x;         /* Angular rate X (raw) */
    int16_t gyro_y;         /* Angular rate Y (raw) */
    int16_t gyro_z;         /* Angular rate Z (raw) */
    int16_t temperature;    /* Temperature (raw) */
} ICM42688_RawData_t;

/**
  * @brief Scaled sensor data structure (floating point, physical units)
  */
typedef struct {
    float accel_x_g;        /* Acceleration X (g) */
    float accel_y_g;        /* Acceleration Y (g) */
    float accel_z_g;        /* Acceleration Z (g) */
    float gyro_x_dps;       /* Angular rate X (deg/s) */
    float gyro_y_dps;       /* Angular rate Y (deg/s) */
    float gyro_z_dps;       /* Angular rate Z (deg/s) */
    float temperature_c;    /* Temperature (°C) */
} ICM42688_ScaledData_t;

/**
  * @brief ICM42688 device handle
  */
typedef struct {
    SPI_HandleTypeDef *hspi;            /* SPI handle */
    ICM42688_GyroFS_t gyro_fs;          /* Gyro full scale setting */
    ICM42688_AccelFS_t accel_fs;        /* Accel full scale setting */
    float gyro_sensitivity;             /* Gyro sensitivity (dps/LSB) */
    float accel_sensitivity;            /* Accel sensitivity (g/LSB) */
    uint8_t is_initialized;             /* Initialization flag */
    ICM42688_RawData_t raw_data;        /* Latest raw data */
    ICM42688_ScaledData_t scaled_data;  /* Latest scaled data */
} ICM42688_Handle_t;

/* ============================================================================
 *                          GLOBAL HANDLE
 * ============================================================================ */

extern ICM42688_Handle_t icm42688_handle;

/* ============================================================================
 *                          DEBUG VARIABLES (Keil Watch Window)
 * ============================================================================ */

extern volatile float g_imu_accel_x_g;
extern volatile float g_imu_accel_y_g;
extern volatile float g_imu_accel_z_g;
extern volatile float g_imu_gyro_x_dps;
extern volatile float g_imu_gyro_y_dps;
extern volatile float g_imu_gyro_z_dps;
extern volatile float g_imu_temperature_c;
extern volatile int16_t g_imu_accel_x_raw;
extern volatile int16_t g_imu_accel_y_raw;
extern volatile int16_t g_imu_accel_z_raw;
extern volatile int16_t g_imu_gyro_x_raw;
extern volatile int16_t g_imu_gyro_y_raw;
extern volatile int16_t g_imu_gyro_z_raw;
extern volatile uint8_t g_imu_who_am_i;
extern volatile uint8_t g_imu_init_status;

/* ============================================================================
 *                          FUNCTION PROTOTYPES
 * ============================================================================ */

/* Initialization */
void ICM42688_GPIO_Init(void);
uint8_t ICM42688_Init(void);

/* Configuration */
void ICM42688_SetGyroFS(ICM42688_GyroFS_t fs);
void ICM42688_SetAccelFS(ICM42688_AccelFS_t fs);
void ICM42688_SetGyroODR(ICM42688_ODR_t odr);
void ICM42688_SetAccelODR(ICM42688_ODR_t odr);
void ICM42688_SetPowerMode(ICM42688_GyroMode_t gyro_mode, ICM42688_AccelMode_t accel_mode);

/* Data Reading */
void ICM42688_ReadRawData(ICM42688_RawData_t *data);
void ICM42688_ReadScaledData(ICM42688_ScaledData_t *data);
void ICM42688_Update(void);

/* Low-Level Functions */
uint8_t ICM42688_ReadReg(uint8_t bank, uint8_t reg);
void ICM42688_WriteReg(uint8_t bank, uint8_t reg, uint8_t value);
void ICM42688_ReadRegs(uint8_t bank, uint8_t reg, uint8_t *buf, uint8_t len);

/* Utility Functions */
float ICM42688_GetTemperature(void);
uint8_t ICM42688_CheckWhoAmI(void);

/* Pin Control Macros */
#define ICM42688_CS_HIGH()      HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET)
#define ICM42688_CS_LOW()       HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET)

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688_H__ */
