/*
 * MS5837.c
 *
 *  Created on: 14 Haz 2018
 *      Author: Ekin Basar Komur
 */
#include "MS5837.h"
extern I2C_HandleTypeDef hi2c1;

uint8_t _model = MS5837_30BA;
float fluidDensity = 1029;
const float Pa = 100.0f;
const float bar = 0.001f;
const float mbar = 1.0f;
float conversion = 1.0f;

int32_t TEMP;
int32_t P;
uint16_t C[8];
uint32_t D1;
uint32_t D2;


uint8_t init() {
	I2C_send(MS5837_RESET);
	HAL_Delay(10);

	for ( uint8_t i = 0 ; i < 7 ; i++ ) {
		C[i] = I2C_read16(MS5837_PROM_READ+i*2);
		HAL_Delay(20);
	}

	uint8_t crcRead = C[0] >> 12;
	uint8_t crcCalculated = crc4(C);

	if ( crcCalculated == crcRead ) {
		return 1;

	}
	return 0;
}

uint8_t crc4(uint16_t n_prom[]) {
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0 ; i < 16; i++ ) {
		if ( i%2 == 1 ) {
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		} else {
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
			if ( n_rem & 0x8000 ) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}

void Read() {
	I2C_send(MS5837_CONVERT_D1_8192);
	HAL_Delay(20);

	D1 = I2C_read32(MS5837_ADC_READ);
	D1 = D1>>8;
	HAL_Delay(20);
	I2C_send(MS5837_CONVERT_D2_8192);
	HAL_Delay(20);

	D2 = I2C_read32(MS5837_ADC_READ);
	D2 = D2>>8;

	calculate();
}

void calculate() {

	int32_t dT = 0;
	int64_t SENS = 0;
	int64_t OFF = 0;
	int32_t SENSi = 0;
	int32_t OFFi = 0;
	int32_t Ti = 0;
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;

	dT = D2-(uint32_t)C[5]*256l;
	if (_model) {
		SENS = (int64_t)C[1]*65536l+((int64_t)C[3]*dT)/128l;
		OFF = (int64_t)C[2]*131072l+((int64_t)C[4]*dT)/64l;
		P = (D1*SENS/(2097152l)-OFF)/(32768l);
	} else {
		SENS = (int64_t)C[1]*32768l+((int64_t)C[3]*dT)/256l;
		OFF = (int64_t)C[2]*65536l+((int64_t)C[4]*dT)/128l;
		P = (D1*SENS/(2097152l)-OFF)/(8192l);
	}

	TEMP = 2000l+(int64_t)dT*C[6]/8388608LL;

	if (_model) {
		if((TEMP/100)<20){
			Ti = (11*(int64_t)dT*(int64_t)dT)/(34359738368LL);
			OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
			SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
		}
	} else {
		if((TEMP/100)<20){
			Ti = (3*(int64_t)dT*(int64_t)dT)/(8589934592LL);
			OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
			SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
			if((TEMP/100)<-15){
				OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
				SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
			}
		}
		else if((TEMP/100)>=20){
			Ti = 2*(dT*dT)/(137438953472LL);
			OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
			SENSi = 0;
		}
	}

	OFF2 = OFF-OFFi;
	SENS2 = SENS-SENSi;

	if (_model) {
		TEMP = (TEMP-Ti);
		P = (((D1*SENS2)/2097152l-OFF2)/32768l)/100;
	} else {
		TEMP = (TEMP-Ti);
		P = (((D1*SENS2)/2097152l-OFF2)/8192l)/10;
	}
	TEMP = TEMP/100.0f;
}

void setModel(uint8_t model) {
	_model = model;
}

void setFluidDensity(float density) {
	fluidDensity = density;
}

float pressure(float conversion) {
	return P*conversion;
}

float temperature() {
	return TEMP/100.0f;
}

float depth() {
	return (pressure(Pa)-101300)/(fluidDensity*9.80665);
}

float altitude() {
	return (1-pow((pressure()/1013.25),.190284))*145366.45*.3048;
}

void I2C_send(uint8_t addr) {
	HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR<<1, &addr, 1, 100 );
}

int8_t I2C_read8(uint8_t addr){
	uint8_t data=0;
	HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR<<1, &addr, 1, 100 );
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1,MS5837_ADDR<<1,&data , 1, 100);
	return data;
}

int16_t I2C_read16(uint8_t addr){
	uint8_t dataArr[2] = {0,0};
	HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR<<1, &addr, 1, 100 );
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1,MS5837_ADDR<<1,dataArr, 2, 100);
	uint16_t data = (dataArr[0] << 8) | dataArr[1];
	return data;
}

int32_t I2C_read32(uint8_t addr){
	uint8_t dataArr[4] = {0,0,0,0};
	HAL_I2C_Master_Transmit(&hi2c1, MS5837_ADDR<<1, &addr, 1, 100 );
	HAL_Delay(20);
	HAL_I2C_Master_Receive(&hi2c1,MS5837_ADDR<<1,dataArr, 4, 100);
	uint32_t data = (dataArr[0] << 24) | (dataArr[1] << 16) | (dataArr[2] << 8) | dataArr[3];
	return data;
}
