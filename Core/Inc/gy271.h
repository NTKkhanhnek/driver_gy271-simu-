#ifndef INC_GY271_H_
#define INC_GY271_H_

#include <stdint.h> // thu vien chuan, chua cac kieu du lieu

#define GY271_QMC5883L_ADDR     0x0D // dia chi i2c QMC5883L, i2c.c tu dich trai dia chi 7-bit
#define GY271_HMC5883L_ADDR     0x1E // dia chi i2c HMC5883L

#define GY271_QMC_REG_X_LSB     0x00
#define GY271_QMC_REG_STATUS    0x06
#define GY271_QMC_REG_CTRL1     0x09
#define GY271_QMC_REG_CTRL2     0x0A
#define GY271_QMC_REG_SET_RESET 0x0B
#define GY271_QMC_REG_CHIP_ID   0x0D

#define GY271_HMC_REG_CONFIG_A  0x00
#define GY271_HMC_REG_CONFIG_B  0x01
#define GY271_HMC_REG_MODE      0x02
#define GY271_HMC_REG_X_MSB     0x03
#define GY271_HMC_REG_STATUS    0x09
#define GY271_HMC_REG_ID_A      0x0A

#define GY271_QMC_STATUS_DRDY   0x01
#define GY271_HMC_STATUS_RDY    0x01

typedef enum
{
    GY271_OK = 0,
    GY271_ERROR,
    GY271_BUSY,
    GY271_TIMEOUT
} GY271_StatusTypeDef;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} GY271_RawData;

typedef struct
{
    float x;
    float y;
    float z;
} GY271_MagneticData;

GY271_StatusTypeDef GY271_Init(void);
GY271_StatusTypeDef GY271_ReadRawData(GY271_RawData *rawData);
GY271_StatusTypeDef GY271_ReadMagneticField(GY271_MagneticData *magData);
GY271_StatusTypeDef GY271_ReadHeading(float *heading);
GY271_StatusTypeDef GY271_ReadData(GY271_RawData *rawData, GY271_MagneticData *magData, float *heading);

#endif /* INC_GY271_H_ */
