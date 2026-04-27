/**
  ******************************************************************************
  * @file    icm42688.c
  * @brief   ICM-42688-P 6-axis IMU Driver Implementation
  ******************************************************************************
  */

#include "icm42688.h"

/* Global device handle */
ICM42688_Handle_t icm42688_handle = {0};

/* Debug variables for Keil Watch Window */
volatile float g_imu_accel_x_g = 0.0f;
volatile float g_imu_accel_y_g = 0.0f;
volatile float g_imu_accel_z_g = 0.0f;
volatile float g_imu_gyro_x_dps = 0.0f;
volatile float g_imu_gyro_y_dps = 0.0f;
volatile float g_imu_gyro_z_dps = 0.0f;
volatile float g_imu_temperature_c = 0.0f;
volatile int16_t g_imu_accel_x_raw = 0;
volatile int16_t g_imu_accel_y_raw = 0;
volatile int16_t g_imu_accel_z_raw = 0;
volatile int16_t g_imu_gyro_x_raw = 0;
volatile int16_t g_imu_gyro_y_raw = 0;
volatile int16_t g_imu_gyro_z_raw = 0;
volatile uint8_t g_imu_who_am_i = 0;
volatile uint8_t g_imu_init_status = 0;

/* ============================================================================
 *                          SPI LOW-LEVEL FUNCTIONS
 * ============================================================================ */

static uint8_t SPI_TxRx(uint8_t data)
{
    uint8_t rx_data = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx_data, 1, ICM42688_SPI_TIMEOUT);
    return rx_data;
}

/* ============================================================================
 *                          REGISTER ACCESS FUNCTIONS
 * ============================================================================ */

static void ICM42688_SelectBank(uint8_t bank)
{
    ICM42688_CS_LOW();
    SPI_TxRx(ICM42688_REG_BANK_SEL);
    SPI_TxRx(bank);
    ICM42688_CS_HIGH();
}

uint8_t ICM42688_ReadReg(uint8_t bank, uint8_t reg)
{
    uint8_t value;

    ICM42688_SelectBank(bank);

    ICM42688_CS_LOW();
    SPI_TxRx(reg | 0x80);
    value = SPI_TxRx(0xFF);
    ICM42688_CS_HIGH();

    return value;
}

void ICM42688_WriteReg(uint8_t bank, uint8_t reg, uint8_t value)
{
    ICM42688_SelectBank(bank);

    ICM42688_CS_LOW();
    SPI_TxRx(reg);
    SPI_TxRx(value);
    ICM42688_CS_HIGH();
}

void ICM42688_ReadRegs(uint8_t bank, uint8_t reg, uint8_t *buf, uint8_t len)
{
    ICM42688_SelectBank(bank);

    ICM42688_CS_LOW();
    SPI_TxRx(reg | 0x80);
    while (len--) {
        *buf++ = SPI_TxRx(0xFF);
    }
    ICM42688_CS_HIGH();
}

/* ============================================================================
 *                          GPIO INITIALIZATION
 * ============================================================================ */

void ICM42688_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* CS Pin: PA4 - Output Push-Pull */
    GPIO_InitStruct.Pin = ICM42688_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(ICM42688_CS_PORT, &GPIO_InitStruct);

    /* Set CS high (deselect) */
    ICM42688_CS_HIGH();
}

/* ============================================================================
 *                          INITIALIZATION
 * ============================================================================ */

uint8_t ICM42688_Init(void)
{
    uint8_t who_am_i;
    uint8_t data;

    /* Initialize GPIO */
    ICM42688_GPIO_Init();

    /* Initialize handle */
    icm42688_handle.hspi = &hspi1;
    icm42688_handle.gyro_fs = ICM42688_GYRO_FS_2000DPS;
    icm42688_handle.accel_fs = ICM42688_ACCEL_FS_16G;
    icm42688_handle.gyro_sensitivity = 16.4f;   /* 2000 dps / 32768 */
    icm42688_handle.accel_sensitivity = 0.488f; /* 16 g / 32768 * 1000 mg/g */

    g_imu_init_status = 1; /* GPIO initialized */

    /* Wait for power-up */
    HAL_Delay(10);

    /* Software Reset */
    ICM42688_WriteReg(0, ICM42688_REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(1);
    g_imu_init_status = 2; /* Reset done */

    /* Read WHO_AM_I */
    who_am_i = ICM42688_ReadReg(0, ICM42688_REG_WHO_AM_I);
    g_imu_who_am_i = who_am_i;

    if (who_am_i != ICM42688_WHO_AM_I) {
        g_imu_init_status = 0xFF; /* WHO_AM_I failed */
        return 0;
    }
    g_imu_init_status = 3; /* WHO_AM_I OK */

    /* Configure SPI interface (Bank 0) */
    /* INTF_CONFIG0: FIFO count in big-endian, SPI mode */
    ICM42688_WriteReg(0, ICM42688_REG_INTF_CONFIG0, 0x10);
    g_imu_init_status = 4; /* Interface configured */

    /* Configure power management (Bank 0) */
    /* PWR_MGMT0: CLKSEL=01 (PLL), then enable sensors */
    /* First set to sleep with PLL clock */
    ICM42688_WriteReg(0, ICM42688_REG_PWR_MGMT0, 0x01);
    HAL_Delay(1);
    g_imu_init_status = 5; /* Clock configured */

    /* Configure Gyro: ±2000 dps, 1 kHz ODR */
    /* GYRO_CONFIG0: FS_SEL=0 (±2000 dps), ODR=6 (1 kHz) */
    data = (0 << 5) | (ICM42688_ODR_1KHZ << 0);
    ICM42688_WriteReg(0, ICM42688_REG_GYRO_CONFIG0, data);
    g_imu_init_status = 6; /* Gyro configured */

    /* Configure Accel: ±16 g, 1 kHz ODR */
    /* ACCEL_CONFIG0: FS_SEL=0 (±16 g), ODR=6 (1 kHz) */
    data = (0 << 5) | (ICM42688_ODR_1KHZ << 0);
    ICM42688_WriteReg(0, ICM42688_REG_ACCEL_CONFIG0, data);
    g_imu_init_status = 7; /* Accel configured */

    /* Configure UI filters (Bank 1) */
    /* GYRO_UI_FILT_ORD: 2nd order filter */
    ICM42688_WriteReg(1, ICM42688_REG_GYRO_UI_FILT_ORD, 0x02);
    /* ACCEL_UI_FILT_ORD: 2nd order filter */
    ICM42688_WriteReg(1, ICM42688_REG_ACCEL_UI_FILT_ORD, 0x02);
    g_imu_init_status = 8; /* Filters configured */

    /* Enable Gyro LN mode and Accel LN mode (Bank 0) */
    /* PWR_MGMT0: GYRO_MODE=11 (LN), ACCEL_MODE=11 (LN), CLKSEL=01 */
    data = (ICM42688_GYRO_MODE_LN << 2) | (ICM42688_ACCEL_MODE_LN << 0) | (0x01 << 4);
    ICM42688_WriteReg(0, ICM42688_REG_PWR_MGMT0, data);
    HAL_Delay(50); /* Wait for sensors to stabilize */
    g_imu_init_status = 9; /* Power mode set */

    /* Configure INT_ASYNC_RESET (Bank 0, 0x64 bit 4 = 0) */
    /* This is important per datasheet Use Notes */
    data = ICM42688_ReadReg(0, ICM42688_REG_MCLK_RDY);
    data &= ~(1 << 4);
    ICM42688_WriteReg(0, ICM42688_REG_MCLK_RDY, data);
    g_imu_init_status = 10; /* INT_ASYNC_RESET configured */

    icm42688_handle.is_initialized = 1;
    g_imu_init_status = 100; /* Initialization complete */

    return 1;
}

/* ============================================================================
 *                          CONFIGURATION FUNCTIONS
 * ============================================================================ */

void ICM42688_SetGyroFS(ICM42688_GyroFS_t fs)
{
    uint8_t data;

    icm42688_handle.gyro_fs = fs;

    /* Calculate sensitivity based on full scale */
    switch (fs) {
        case ICM42688_GYRO_FS_2000DPS:
            icm42688_handle.gyro_sensitivity = 16.4f;  /* 2000 / 32768 * 2^8 */
            break;
        case ICM42688_GYRO_FS_1000DPS:
            icm42688_handle.gyro_sensitivity = 32.8f;
            break;
        case ICM42688_GYRO_FS_500DPS:
            icm42688_handle.gyro_sensitivity = 65.6f;
            break;
        case ICM42688_GYRO_FS_250DPS:
            icm42688_handle.gyro_sensitivity = 131.2f;
            break;
        case ICM42688_GYRO_FS_125DPS:
            icm42688_handle.gyro_sensitivity = 262.4f;
            break;
        case ICM42688_GYRO_FS_62_5DPS:
            icm42688_handle.gyro_sensitivity = 524.8f;
            break;
        case ICM42688_GYRO_FS_31_25DPS:
            icm42688_handle.gyro_sensitivity = 1049.6f;
            break;
        case ICM42688_GYRO_FS_15_625DPS:
            icm42688_handle.gyro_sensitivity = 2099.2f;
            break;
    }

    /* Read current ODR and update FS */
    data = ICM42688_ReadReg(0, ICM42688_REG_GYRO_CONFIG0);
    data = (data & 0x1F) | (fs << 5);
    ICM42688_WriteReg(0, ICM42688_REG_GYRO_CONFIG0, data);
}

void ICM42688_SetAccelFS(ICM42688_AccelFS_t fs)
{
    uint8_t data;

    icm42688_handle.accel_fs = fs;

    /* Calculate sensitivity based on full scale */
    switch (fs) {
        case ICM42688_ACCEL_FS_16G:
            icm42688_handle.accel_sensitivity = 0.488f;  /* 16 / 32768 * 1000 mg/g */
            break;
        case ICM42688_ACCEL_FS_8G:
            icm42688_handle.accel_sensitivity = 0.244f;
            break;
        case ICM42688_ACCEL_FS_4G:
            icm42688_handle.accel_sensitivity = 0.122f;
            break;
        case ICM42688_ACCEL_FS_2G:
            icm42688_handle.accel_sensitivity = 0.061f;
            break;
    }

    /* Read current ODR and update FS */
    data = ICM42688_ReadReg(0, ICM42688_REG_ACCEL_CONFIG0);
    data = (data & 0x1F) | (fs << 5);
    ICM42688_WriteReg(0, ICM42688_REG_ACCEL_CONFIG0, data);
}

void ICM42688_SetGyroODR(ICM42688_ODR_t odr)
{
    uint8_t data;

    data = ICM42688_ReadReg(0, ICM42688_REG_GYRO_CONFIG0);
    data = (data & 0xE0) | (odr << 0);
    ICM42688_WriteReg(0, ICM42688_REG_GYRO_CONFIG0, data);
}

void ICM42688_SetAccelODR(ICM42688_ODR_t odr)
{
    uint8_t data;

    data = ICM42688_ReadReg(0, ICM42688_REG_ACCEL_CONFIG0);
    data = (data & 0xE0) | (odr << 0);
    ICM42688_WriteReg(0, ICM42688_REG_ACCEL_CONFIG0, data);
}

void ICM42688_SetPowerMode(ICM42688_GyroMode_t gyro_mode, ICM42688_AccelMode_t accel_mode)
{
    uint8_t data;

    /* PWR_MGMT0: GYRO_MODE[1:0], ACCEL_MODE[1:0], CLKSEL[1:0] */
    data = (gyro_mode << 2) | (accel_mode << 0) | (0x01 << 4);
    ICM42688_WriteReg(0, ICM42688_REG_PWR_MGMT0, data);
}

/* ============================================================================
 *                          DATA READING FUNCTIONS
 * ============================================================================ */

void ICM42688_ReadRawData(ICM42688_RawData_t *data)
{
    uint8_t buf[14];

    /* Read temperature and sensor data (14 bytes total) */
    /* TEMP_DATA1~0 (2) + ACCEL_DATA_X1~Z0 (6) + GYRO_DATA_X1~Z0 (6) */
    ICM42688_ReadRegs(0, ICM42688_REG_TEMP_DATA1, buf, 14);

    /* Temperature (16-bit, big-endian) */
    data->temperature = (int16_t)((buf[0] << 8) | buf[1]);

    /* Accelerometer (16-bit, big-endian) */
    data->accel_x = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_y = (int16_t)((buf[4] << 8) | buf[5]);
    data->accel_z = (int16_t)((buf[6] << 8) | buf[7]);

    /* Gyroscope (16-bit, big-endian) */
    data->gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
}

void ICM42688_ReadScaledData(ICM42688_ScaledData_t *data)
{
    ICM42688_RawData_t raw;

    ICM42688_ReadRawData(&raw);

    /* Convert accelerometer data to g */
    data->accel_x_g = (float)raw.accel_x * icm42688_handle.accel_sensitivity / 1000.0f;
    data->accel_y_g = (float)raw.accel_y * icm42688_handle.accel_sensitivity / 1000.0f;
    data->accel_z_g = (float)raw.accel_z * icm42688_handle.accel_sensitivity / 1000.0f;

    /* Convert gyroscope data to deg/s */
    data->gyro_x_dps = (float)raw.gyro_x / icm42688_handle.gyro_sensitivity;
    data->gyro_y_dps = (float)raw.gyro_y / icm42688_handle.gyro_sensitivity;
    data->gyro_z_dps = (float)raw.gyro_z / icm42688_handle.gyro_sensitivity;

    /* Convert temperature to °C */
    /* Temperature (°C) = (TEMP_DATA / 132.48) + 25 */
    data->temperature_c = ((float)raw.temperature / 132.48f) + 25.0f;
}

void ICM42688_Update(void)
{
    ICM42688_ReadScaledData(&icm42688_handle.scaled_data);
    ICM42688_ReadRawData(&icm42688_handle.raw_data);

    /* Update debug variables for Keil Watch Window */
    g_imu_accel_x_g = icm42688_handle.scaled_data.accel_x_g;
    g_imu_accel_y_g = icm42688_handle.scaled_data.accel_y_g;
    g_imu_accel_z_g = icm42688_handle.scaled_data.accel_z_g;
    g_imu_gyro_x_dps = icm42688_handle.scaled_data.gyro_x_dps;
    g_imu_gyro_y_dps = icm42688_handle.scaled_data.gyro_y_dps;
    g_imu_gyro_z_dps = icm42688_handle.scaled_data.gyro_z_dps;
    g_imu_temperature_c = icm42688_handle.scaled_data.temperature_c;
    g_imu_accel_x_raw = icm42688_handle.raw_data.accel_x;
    g_imu_accel_y_raw = icm42688_handle.raw_data.accel_y;
    g_imu_accel_z_raw = icm42688_handle.raw_data.accel_z;
    g_imu_gyro_x_raw = icm42688_handle.raw_data.gyro_x;
    g_imu_gyro_y_raw = icm42688_handle.raw_data.gyro_y;
    g_imu_gyro_z_raw = icm42688_handle.raw_data.gyro_z;
}

/* ============================================================================
 *                          UTILITY FUNCTIONS
 * ============================================================================ */

float ICM42688_GetTemperature(void)
{
    int16_t temp_raw;
    uint8_t buf[2];

    ICM42688_ReadRegs(0, ICM42688_REG_TEMP_DATA1, buf, 2);
    temp_raw = (int16_t)((buf[0] << 8) | buf[1]);

    return ((float)temp_raw / 132.48f) + 25.0f;
}

uint8_t ICM42688_CheckWhoAmI(void)
{
    return ICM42688_ReadReg(0, ICM42688_REG_WHO_AM_I);
}
