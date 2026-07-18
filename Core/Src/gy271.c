#include "gy271.h"
#include "delay.h"
#include "i2c.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef enum
{
    GY271_CHIP_NONE = 0,
    GY271_CHIP_QMC5883L,
    GY271_CHIP_HMC5883L
} GY271_ChipType;

static uint8_t gy271_addr = GY271_QMC5883L_ADDR;
static GY271_ChipType gy271_chip = GY271_CHIP_NONE;
static float gy271_sensitivity = 3000.0f; // LSB/Gauss

volatile int16_t debug_gy271_raw_x;
volatile int16_t debug_gy271_raw_y;
volatile int16_t debug_gy271_raw_z;
volatile uint8_t debug_gy271_status;
volatile uint8_t debug_gy271_chip;

// 3 ham sau la 3 cong cu giao tiep i2c
static GY271_StatusTypeDef GY271_WriteReg(uint8_t reg, uint8_t data)
{
    if (i2c_mem_write(gy271_addr, reg, &data, 1) != I2C_OK)
    {
        return GY271_ERROR;
    }

    return GY271_OK;
}

static GY271_StatusTypeDef GY271_ReadReg(uint8_t reg, uint8_t *data)
{
    if (i2c_mem_read(gy271_addr, reg, data, 1) != I2C_OK)
    {
        return GY271_ERROR;
    }

    return GY271_OK;
}

static GY271_StatusTypeDef GY271_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (i2c_mem_read(gy271_addr, reg, data, len) != I2C_OK)
    {
        return GY271_ERROR;
    }

    return GY271_OK;
}

static GY271_StatusTypeDef GY271_CheckID(void)
{
    uint8_t id[3];

    gy271_addr = GY271_QMC5883L_ADDR;
    if (GY271_ReadReg(GY271_QMC_REG_CHIP_ID, &id[0]) == GY271_OK)
    {
        gy271_chip = GY271_CHIP_QMC5883L;
        gy271_sensitivity = 3000.0f; // QMC5883L range 8G
        debug_gy271_chip = (uint8_t)gy271_chip;
        return GY271_OK;
    }

    gy271_addr = GY271_HMC5883L_ADDR;
    if (GY271_ReadRegs(GY271_HMC_REG_ID_A, id, 3) == GY271_OK)
    {
        if ((id[0] == 'H') && (id[1] == '4') && (id[2] == '3'))
        {
            gy271_chip = GY271_CHIP_HMC5883L;
            gy271_sensitivity = 1090.0f; // HMC5883L gain 1.3G
            debug_gy271_chip = (uint8_t)gy271_chip;
            return GY271_OK;
        }
    }

    gy271_chip = GY271_CHIP_NONE;
    debug_gy271_chip = (uint8_t)gy271_chip;
    return GY271_ERROR;
}

static GY271_StatusTypeDef GY271_Reset(void)
{
    if (gy271_chip == GY271_CHIP_QMC5883L)
    {
        if (GY271_WriteReg(GY271_QMC_REG_CTRL2, 0x80) != GY271_OK) // soft reset
        {
            return GY271_ERROR;
        }

        delay_ms(10);
    }

    return GY271_OK;
}

static GY271_StatusTypeDef GY271_SetConfig(void)
{
    if (gy271_chip == GY271_CHIP_QMC5883L)
    {
        if (GY271_WriteReg(GY271_QMC_REG_SET_RESET, 0x01) != GY271_OK)
        {
            return GY271_ERROR;
        }

        if (GY271_WriteReg(GY271_QMC_REG_CTRL2, 0x00) != GY271_OK)
        {
            return GY271_ERROR;
        }

        if (GY271_WriteReg(GY271_QMC_REG_CTRL1, 0x1D) != GY271_OK) // OSR=512, RNG=8G, ODR=200Hz, continuous
        {
            return GY271_ERROR;
        }
    }
    else if (gy271_chip == GY271_CHIP_HMC5883L)
    {
        if (GY271_WriteReg(GY271_HMC_REG_CONFIG_A, 0x70) != GY271_OK) // 8 mau trung binh, 15Hz, normal
        {
            return GY271_ERROR;
        }

        if (GY271_WriteReg(GY271_HMC_REG_CONFIG_B, 0x20) != GY271_OK) // gain 1.3G
        {
            return GY271_ERROR;
        }

        if (GY271_WriteReg(GY271_HMC_REG_MODE, 0x00) != GY271_OK) // continuous
        {
            return GY271_ERROR;
        }
    }
    else
    {
        return GY271_ERROR;
    }

    return GY271_OK;
}

static GY271_StatusTypeDef GY271_WaitDataReady(void)
{
    uint8_t status = 0;

    for (uint16_t i = 0; i < 1000; i++)
    {
        if (gy271_chip == GY271_CHIP_QMC5883L)
        {
            if (GY271_ReadReg(GY271_QMC_REG_STATUS, &status) != GY271_OK)
            {
                return GY271_ERROR;
            }

            debug_gy271_status = status;
            if ((status & GY271_QMC_STATUS_DRDY) != 0)
            {
                return GY271_OK;
            }
        }
        else if (gy271_chip == GY271_CHIP_HMC5883L)
        {
            if (GY271_ReadReg(GY271_HMC_REG_STATUS, &status) != GY271_OK)
            {
                return GY271_ERROR;
            }

            debug_gy271_status = status;
            if ((status & GY271_HMC_STATUS_RDY) != 0)
            {
                return GY271_OK;
            }
        }
        else
        {
            return GY271_ERROR;
        }

        delay_ms(1);
    }

    return GY271_TIMEOUT;
}

static void GY271_ConvertRawToMagnetic(const GY271_RawData *rawData, GY271_MagneticData *magData)
{
    magData->x = (float)rawData->x / gy271_sensitivity;
    magData->y = (float)rawData->y / gy271_sensitivity;
    magData->z = (float)rawData->z / gy271_sensitivity;
}

static void GY271_CalculateHeading(const GY271_MagneticData *magData, float *heading)
{
    *heading = atan2f(magData->y, magData->x) * 180.0f / (float)M_PI;
    if (*heading < 0.0f)
    {
        *heading += 360.0f;
    }
}

GY271_StatusTypeDef GY271_Init(void)
{
    i2c_init();

    if (GY271_CheckID() != GY271_OK)
    {
        return GY271_ERROR;
    }

    if (GY271_Reset() != GY271_OK)
    {
        return GY271_ERROR;
    }

    if (GY271_SetConfig() != GY271_OK)
    {
        return GY271_ERROR;
    }

    delay_ms(20);
    return GY271_OK;
}

GY271_StatusTypeDef GY271_ReadRawData(GY271_RawData *rawData)
{
    uint8_t raw[6];

    if (rawData == 0)
    {
        return GY271_ERROR;
    }

    if (GY271_WaitDataReady() != GY271_OK)
    {
        return GY271_TIMEOUT;
    }

    if (gy271_chip == GY271_CHIP_QMC5883L)
    {
        if (GY271_ReadRegs(GY271_QMC_REG_X_LSB, raw, 6) != GY271_OK)
        {
            return GY271_ERROR;
        }

        rawData->x = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
        rawData->y = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
        rawData->z = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
    }
    else if (gy271_chip == GY271_CHIP_HMC5883L)
    {
        if (GY271_ReadRegs(GY271_HMC_REG_X_MSB, raw, 6) != GY271_OK)
        {
            return GY271_ERROR;
        }

        rawData->x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
        rawData->z = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
        rawData->y = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    }
    else
    {
        return GY271_ERROR;
    }

    debug_gy271_raw_x = rawData->x;
    debug_gy271_raw_y = rawData->y;
    debug_gy271_raw_z = rawData->z;
    return GY271_OK;
}

GY271_StatusTypeDef GY271_ReadMagneticField(GY271_MagneticData *magData)
{
    GY271_RawData rawData;
    GY271_StatusTypeDef status;

    if (magData == 0)
    {
        return GY271_ERROR;
    }

    status = GY271_ReadRawData(&rawData);
    if (status != GY271_OK)
    {
        return status;
    }

    GY271_ConvertRawToMagnetic(&rawData, magData);
    return GY271_OK;
}

GY271_StatusTypeDef GY271_ReadHeading(float *heading)
{
    GY271_MagneticData magData;
    GY271_StatusTypeDef status;

    if (heading == 0)
    {
        return GY271_ERROR;
    }

    status = GY271_ReadMagneticField(&magData);
    if (status != GY271_OK)
    {
        return status;
    }

    GY271_CalculateHeading(&magData, heading);

    return GY271_OK;
}

GY271_StatusTypeDef GY271_ReadData(GY271_RawData *rawData, GY271_MagneticData *magData, float *heading)
{
    GY271_StatusTypeDef status;

    if ((rawData == 0) || (magData == 0) || (heading == 0))
    {
        return GY271_ERROR;
    }

    status = GY271_ReadRawData(rawData);
    if (status != GY271_OK)
    {
        return status;
    }

    GY271_ConvertRawToMagnetic(rawData, magData);
    GY271_CalculateHeading(magData, heading);
    return GY271_OK;
}
