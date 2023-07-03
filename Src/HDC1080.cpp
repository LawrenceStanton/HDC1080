/**
 ******************************************************************************
 * @file           : HDC1080.cpp
 * @brief          : Source file for HDC1080 Driver
 * @author			: Lawrence Stanton
 ******************************************************************************
 * @attention
 *
 * © LD Stanton 2022 - 2023
 *
 * This file and its content are the copyright property of the author. All
 * rights are reserved. No warranty is given. No liability is assumed.
 * Confidential unless licensed otherwise. If licensed, refer to the
 * accompanying file "LICENCE" for license details.
 *
 ******************************************************************************
 */

#include "HDC1080.hpp"

/**
 * @note The below values are given a wide margin above the maximum conversion time specifications
 * 		 to allow for a imprecise delay timer method.
 *
 * @note As of current in this source, the conversion time does not consider the resolution of the measurement.
 * 		 It is possible to reduce the delay time by defining shorter conversion times for lower resolution measuremnts
 * 		 and amending the measurement register getter methods to use these shorter delay times.
 */
#define HDC1080_CONVERSION_TIME_TEMPERATURE 15u
#define HDC1080_CONVERSION_TIME_HUMIDITY	15u
#define HDC1080_CONVERSION_TIME_DUAL		20u

using Register		= HDC1080::I2C::Register;
using MemoryAddress = HDC1080::I2C::MemoryAddress;

HDC1080::HDC1080(
	HDC1080::I2C					 &i2c,
	HDC1080::AcqModeConfig			  acqMode,
	HDC1080::TempResolutionConfig	  tRes,
	HDC1080::HumidityResolutionConfig hRes,
	HDC1080::HeaterConfig			  heater
)
	: i2c(i2c) {
	setConfig(acqMode, tRes, hRes, heater);
}

float HDC1080::getTemperature(Register temperatureRegister) {
	return (float)temperatureRegister * (165.0 / 65536.0) - 40.0;
}

float HDC1080::getTemperature() {
	auto temperatureRegister = getTemperatureRegister();

	if (temperatureRegister.has_value())
		return getTemperature(temperatureRegister.value());

	return -40.0;
}

float HDC1080::getHumidity(Register humidityRegister) {
	return (float)humidityRegister * (25.0 / 16384.0);
}

float HDC1080::getHumidity() {
	auto humidityRegister = getHumidityRegister();

	if (humidityRegister.has_value())
		return getHumidity(humidityRegister.value());

	return 0.0;
}

std::optional<Register> HDC1080::getTemperatureRegister() {
	return getMeasurementRegister(HDC1080_TEMPERATURE_ADDR, HDC1080_CONVERSION_TIME_TEMPERATURE);
}

std::optional<Register> HDC1080::getHumidityRegister() {
	return getMeasurementRegister(HDC1080_HUMIDITY_ADDR, HDC1080_CONVERSION_TIME_HUMIDITY);
}

std::optional<Register> HDC1080::setConfig(
	AcqModeConfig			 acqMode,
	TempResolutionConfig	 tRes,
	HumidityResolutionConfig hRes,
	HeaterConfig			 heater
) {
	Register reg = static_cast<Register>(acqMode) //
				 | static_cast<Register>(tRes)	  //
				 | static_cast<Register>(hRes)	  //
				 | static_cast<Register>(heater);

	return this->i2c.write(HDC1080_CONFIG_ADDR, reg);
}

std::optional<uint64_t> HDC1080::getSerialID() {
	Register serialID[3];
	for (unsigned int i = 0; i < 3; i++) {
		auto transmission = this->i2c.read(HDC1080_SERIAL_ID_ADDR + i);
		if (transmission.has_value())
			serialID[i] = transmission.value();
		else
			return std::nullopt;
	}
	const uint64_t serialIdValue = (((uint64_t)serialID[0] & 0xFFFF) << 25) // SERIAL_ID[40:25], register bits [15:0]
								 | (((uint64_t)serialID[1] & 0xFFFF) << 9)	// SERIAL_ID[24:9], register bits [15:0]
								 | (((uint64_t)serialID[2] & 0xFF80) >> 7); // SERIAL_ID[8:0], register bits [15:7]

	return serialIdValue;
}

/**
 * @brief Special routine for fetching a memory register that enforces a required measurement conversion time.
 *
 * @param memAddr	Starting memory address to fetch. Use HDC1080_TEMPERATURE_ADDR for dual measurement.
 * @param waitTime	Required conversion time delay between addressing the memory and fetching data. (ms)
 * @param pData		Pointer to memory allocated for received data.
 * @param n			Number of measurements to retrieve (is only ever 1 (for single) or 2 (for dual))
 *
 * @note This method does not check the validity of memAddr and reading a reserved address is forbidden.
 * @note In the event of communication failure, writes 0x0000u to all scoped values of *pData.
 */
std::optional<HDC1080::I2C::Register>
HDC1080::getMeasurementRegister(HDC1080::I2C::MemoryAddress memAddr, uint32_t waitTime) {
	if (!this->i2c.transmit(static_cast<uint8_t>(memAddr)))
		return {};

	this->i2c.delay(waitTime);

	std::optional<uint8_t> transmissionData[2];
	for (auto &&data : transmissionData) {
		data = this->i2c.receive();
		if (!data)
			return {};
	}

	return static_cast<HDC1080::I2C::Register>(transmissionData[0].value() << 8 | transmissionData[1].value());
}

/*************************************************************************************************************************
 * 					STM32 Hardware Abstraction Layer Specific I2C and Delay Implementations
 * For convenience, the below methods are methods are suitable implementations of the I2C communication and delay
 * methods for STM32-based systems using the ST Microelectronics STM32 Hardware Abstraction Layer (HAL) for a single
 * HDC1080.
 *************************************************************************************************************************/
// #define HDC1080_USE_STM32_HAL_METHODS
// #ifdef HDC1080_USE_STM32_HAL_METHODS

// #include "stm32g0xx_hal.h" // STM32 HAL driver to be included. Adjust depending on the STM32 platform.

// extern I2C_HandleTypeDef *hdc1080_hi2c; // Pointer to the HAL I2C handle for the I2C interface that the HDC1080
// is on.
// 										// Define this variable in main.cpp.

// HDC1080::Status HDC1080::I2C_MemRead(uint8_t ID, uint8_t I2C_Addr, uint8_t memAddr, uint8_t *pData, uint16_t
// size) { 	(void)ID; // Unused 	HAL_StatusTypeDef halResult = HAL_I2C_Mem_Read(hdc1080_hi2c, (I2C_Addr << 1),
// memAddr, 1, pData, size, 100); 	return (halResult == HAL_OK) ? HDC1080::Status::OK : HDC1080::Status::FAIL_I2C;
// }

// HDC1080::Status HDC1080::I2C_MemWrite(uint8_t ID, uint8_t I2C_Addr, uint8_t memAddr, uint8_t *pData, uint16_t
// size) { 	(void)ID; // Unused 	HAL_StatusTypeDef halResult = HAL_I2C_Mem_Write(hdc1080_hi2c, (I2C_Addr << 1),
// memAddr, 1, pData, size, 100); 	return (halResult == HAL_OK) ? HDC1080::Status::OK : HDC1080::Status::FAIL_I2C;
// }

// HDC1080::Status HDC1080::I2C_Transmit(uint8_t ID, uint8_t I2C_Addr, uint8_t *pData, uint16_t size) {
// 	(void)ID; // Unused
// 	HAL_StatusTypeDef halResult = HAL_I2C_Master_Transmit(hdc1080_hi2c, (I2C_Addr << 1), pData, size, 100);
// 	return (halResult == HAL_OK) ? HDC1080::Status::OK : HDC1080::Status::FAIL_I2C;
// }

// HDC1080::Status HDC1080::I2C_Receive(uint8_t ID, uint8_t I2C_Addr, uint8_t *pData, uint16_t size) {
// 	(void)ID; // Unused
// 	HAL_StatusTypeDef halResult = HAL_I2C_Master_Receive(hdc1080_hi2c, (I2C_Addr << 1), pData, size, 100);
// 	return (halResult == HAL_OK) ? HDC1080::Status::OK : HDC1080::Status::FAIL_I2C;
// }

// void HDC1080::Delay(uint16_t ms) {
// 	HAL_Delay(ms);
// }

// #endif /* HDC1080_USE_STM32_HAL_METHODS */

/*** END OF FILE ***/
