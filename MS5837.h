/*
 * MS5837.h
 *
 *  Created on: 14 Haz 2018
 *      Author: Ekin Basar Komur
 */

#ifndef MS5837_H_
#define MS5837_H_
#include "stm32f4xx_hal.h"

//Definitions:
#define MS5837_ADDR               0x76

#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

//Models:
#define MS5837_30BA               0x00
#define MS5837_02BA				  0x01


//Function Prototypes:

uint8_t init();
uint8_t crc4(uint16_t n_prom[]);
void Read();
void calculate();
void setModel(uint8_t model);
void setFluidDensity(float density);
float pressure(float conversion);
float temperature();
float depth();
float altitude();

int8_t I2C_read8(uint8_t addr);
int16_t I2C_read16(uint8_t addr);
int32_t I2C_read32(uint8_t addr);
void I2C_send(uint8_t addr);

#endif /* MS5837_H_ */
