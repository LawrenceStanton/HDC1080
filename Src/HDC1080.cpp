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

#define HDC1080_CONFIG_RESET 0x8000u
/**
 * @}
 */

/**
 * @note The below values are given a wide margin above the maximum conversion time specifications
 * 		 to allow for a nonprecise delay timer method.
 *
 * @note As of current in this source, the conversion time does not consider the resolution of the measurement.
 * 		 It is possible to reduce the delay time by defining shorter conversion times for lower resolution measuremnts
 * 		 and amending the measurement register getter methods to use these shorter delay times.
 */
#define HDC1080_CONVERSION_TIME_TEMPERATURE 15u
#define HDC1080_CONVERSION_TIME_HUMIDITY	15u
#define HDC1080_CONVERSION_TIME_DUAL		20u

/**
 * @brief Construct a new HDC1080::HDC1080 object given all configuration settings.
 *
 * @param acqMode	The Mode of Aquisition (single or dual measurements).
 * @param tRes		The resolution of the temperature measurement (11 or 14 bit).
 * @param hRes		The resolution of the humidity measurement (8, 11, or 14 bit).
 * @param heater	The heater setting (on or off).
 * @param ID		User defined identifier (for systems with multiple HDC1080s)
 */
HDC1080::HDC1080(
	HDC1080::AcqModeConfig			  acqMode = HDC1080::AcqModeConfig::DUAL,
	HDC1080::TempResolutionConfig	  tRes	  = HDC1080::TempResolutionConfig::A_14BIT,
	HDC1080::HumidityResolutionConfig hRes	  = HDC1080::HumidityResolutionConfig::A_14BIT,
	HDC1080::HeaterConfig			  heater  = HDC1080::HeaterConfig::OFF,
	uint16_t						  ID	  = 0x0000u
) {

	this->ID = ID;
	setConfig(acqMode, tRes, hRes, heater);
}

/**
 * @brief Triggers a software reset of the HDC1080 by setting the respective bit in the configuration register.
 */
void HDC1080::softReset() {
	setRegister(HDC1080_CONFIG_ADDR, HDC1080_CONFIG_RESET);
}

#ifdef HDC1080_IMPLEMENT_FLOAT_VARIABLES

/**
 * @brief Updates the register and real class copies of the temperature measurement.
 *
 * @return float The updated real temperature measurement.
 *
 * @note In the event of tReg = 0x0000u, which might indicate a failure, this results in T = -40degC.
 */
float HDC1080::getTemperature() {
	getTemperatureRegister();
	updateTemperature();
	return this->T;
}

/**
 * @brief Updates the register and real class copies of the humidity measurement.
 *
 * @return float The updated real humidity measurement.
 *
 * @note In the event of hReg = 0x0000u, which might indicate a failure, this results in H = 0.0.
 */
float HDC1080::getHumidity() {
	getHumidityRegister();
	updateHumidity();
	return this->H;
}

/**
 * @brief Updates both measurement register and real class copies of temperature and humidity.
 *
 * @note In the event of hReg = 0x0000u, which might indicate a failure, this results in H = 0.0.
 */
void HDC1080::getTemperatureHumidity() {
	getTemperatureHumidityRegisters();
	updateTemperature();
	updateHumidity();
}

/**
 * @brief Updates the real temperature class copy based on the current value of the temperature register class copy.
 *
 * @note In the event of tReg = 0x0000u, which might indicate a failure, this results in T = -40degC.
 * @note In the event of hReg = 0x0000u, which might indicate a failure, this results in H = 0.0.
 */
void HDC1080::updateTemperature() {
	this->T = (float)this->tReg * (165.0 / 65536.0) - 40.0; // T = tReg[15:00] * (165degC / 2^16) - 40degC
}

/**
 * @brief Updates the real humidity class copy based on the curent value of the humidity register class copy.
 *
 * @note In the event of hReg = 0x0000u, which might indicate a failure, this results in H = 0.0.
 */
void HDC1080::updateHumidity() {
	this->H = (float)this->hReg * (25.0 / 16384.0); // H = hReg[15:00] * (100 / 2^16)
}
#endif /* HDC1080_IMPLEMENT_FLOAT_VARIABLES */

/**
 * @brief Gets the temperature register and updates the class copy.
 *
 * @return uint16_t The fetched value of the temperature register.
 *
 * @note This method does not update the real temperature class copy. Use getTemperature() for this.
 */
uint16_t HDC1080::getTemperatureRegister() {
	uint16_t reg;
	getMeasurementRegisters(HDC1080_TEMPERATURE_ADDR, HDC1080_CONVERSION_TIME_TEMPERATURE, &reg, 1);
	this->tReg = reg;
	return reg;
}

/**
 * @brief Gets the humidity register and updates the class copy.
 *
 * @return uint16_t The fetched value of the humidity register.
 *
 * @note This method does not update the real humidity class copy. Use getHumidity() for this.
 */
uint16_t HDC1080::getHumidityRegister() {
	uint16_t reg;
	getMeasurementRegisters(HDC1080_HUMIDITY_ADDR, HDC1080_CONVERSION_TIME_HUMIDITY, &reg, 1);
	this->hReg = reg;
	return reg;
}

/**
 * @brief Gets both the temperature and humidity measurement registers and updates the class copies.
 *
 * @note This method does not update the real measurements class copies. Use getTemperatureHumidity() for this.
 */
void HDC1080::getTemperatureHumidityRegisters() {
	uint16_t regs[2];
	getMeasurementRegisters(HDC1080_TEMPERATURE_ADDR, HDC1080_CONVERSION_TIME_DUAL, regs, 2);
	this->tReg = regs[0];
	this->hReg = regs[1];
}

/**
 * @brief Sets the HDC1080 Configuration Register given all programmable settings.
 *
 * @param acqMode	The Mode of Aquisition (single or dual measurements).
 * @param tRes		The resolution of the temperature measurement (11 or 14 bit).
 * @param hRes		The resolution of the humidity measurement (8, 11, or 14 bit).
 * @param heater	The heater setting (on or off).
 *
 * @note The heater should only be turned on if necessary for saturated conditions. Refer to the datasheet §8.3.3.
 * @note If only a single measurement is desired, set the other measurement's resolution to any valid value.
 */
void HDC1080::setConfig(
	AcqModeConfig			 acqMode,
	TempResolutionConfig	 tRes,
	HumidityResolutionConfig hRes,
	HeaterConfig			 heater
) {
	auto reg =
		(static_cast<uint16_t>(acqMode) | static_cast<uint16_t>(tRes) | static_cast<uint16_t>(hRes)
		 | static_cast<uint16_t>(heater));

	setRegister(HDC1080_CONFIG_ADDR, reg);
}

/**
 * @brief Fetches the Serial ID of the HDC1080.
 *
 * @return uint64_t The Serial ID (40bit value, right aligned).
 */
uint64_t HDC1080::getSerialID() {
	uint16_t serialID[3];
	for (unsigned int i = 0; i < 3; i++)
		serialID[i] = getRegister(HDC1080_SERIAL_ID_ADDR + i);
	return (((uint64_t)serialID[0] << 24) | ((uint64_t)serialID[1] << 8) | ((uint64_t)serialID[2] >> 8));
}

/**
 * @brief Gets a specific 2-byte register from the HDC1080.
 *
 * @param memAddr	The address if the register to fetch.
 * @return uint16_t The register data.
 *
 * @note This method does not check the validity of memAddr and reading a reserved address is forbidden.
 * @note In the event of communication failure, returns 0x0000u.
 */
uint16_t HDC1080::getRegister(uint8_t memAddr) {
	uint16_t data;
	auto	 hdcResult = I2C_MemRead(this->ID, HDC1080_I2C_ADDR, memAddr, (uint8_t *)&data, HDC1080_MEM_SIZE);
	return (hdcResult == HDC1080::Status::OK) ? __HDC1080_UINT16_REVERSE_CAST(data) : 0x0000u;
}

/**
 * @brief Sets a specific 2-byte register of the HDC1080.
 *
 * @param memAddr	Memory address.
 * @param data		Data to be written.
 *
 * @note This method does not check the validity of memAddr and writing to a reserved address is forbidden.
 */
void HDC1080::setRegister(uint8_t memAddr, uint16_t data) {
	data = __HDC1080_UINT16_REVERSE_CAST(data);
	I2C_MemWrite(this->ID, HDC1080_I2C_ADDR, memAddr, (uint8_t *)&data, sizeof(data));
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
void HDC1080::getMeasurementRegisters(uint8_t memAddr, uint16_t waitTime, uint16_t *pData, uint8_t n) {
	if (I2C_Transmit(this->ID, HDC1080_I2C_ADDR, &memAddr, 1) == HDC1080::Status::OK) {
		Delay(waitTime);

		auto hdcResult = I2C_Receive(this->ID, HDC1080_I2C_ADDR, (uint8_t *)pData, n * 2);
		if (hdcResult == HDC1080::Status::OK) {
			for (int i = 0; i < n; i++)
				pData[i] = __HDC1080_UINT16_REVERSE_CAST(pData[i]);
		} else
			for (int i = 0; i < n; i++)
				pData[i] = 0x0000u;
	} else
		for (int i = 0; i < n; i++)
			pData[i] = 0x0000u;
}

/*************************************************************************************************************************
 * 					STM32 Hardware Abstraction Layer Specific I2C and Delay Implementations
 * For convenience, the below methods are methods are suitable implementations of the I2C communication and delay
 * methods for STM32-based systems using the ST Microelectronics STM32 Hardware Abstraction Layer (HAL) for a single
 * HDC1080.
 *************************************************************************************************************************/
#define HDC1080_USE_STM32_HAL_METHODS
#ifdef HDC1080_USE_STM32_HAL_METHODS

#include "stm32g0xx_hal.h" // STM32 HAL driver to be included. Adjust depending on the STM32 platform.

extern I2C_HandleTypeDef *hdc1080_hi2c; // Pointer to the HAL I2C handle for the I2C interface that the HDC1080 is on.
										// Define this variable in main.cpp.

HDC1080::Status HDC1080::I2C_MemRead(uint8_t ID, uint8_t I2C_Addr, uint8_t memAddr, uint8_t *pData, uint16_t size) {
	(void)ID; // Unused
	HAL_StatusTypeDef halResult = HAL_I2C_Mem_Read(hdc1080_hi2c, (I2C_Addr << 1), memAddr, 1, pData, size, 100);
	return (halResult == HAL_OK) ? HDC1080::Status::OK : HDC1080::Status::FAIL_I2C;
}

HDC1080::Status HDC1080::I2C_MemWrite(uint8_t ID, uint8_t I2C_Addr, uint8_t memAddr, uint8_t *pData, uint16_t size) {
	(void)ID; // Unused
	HAL_StatusTypeDef halResult = HAL_I2C_Mem_Write(hdc1080_hi2c, (I2C_Addr << 1), memAddr, 1, pData, size, 100);
	return (halResult == HAL_OK) ? HDC1080::Status::OK : HDC1080::Status::FAIL_I2C;
}

HDC1080::Status HDC1080::I2C_Transmit(uint8_t ID, uint8_t I2C_Addr, uint8_t *pData, uint16_t size) {
	(void)ID; // Unused
	HAL_StatusTypeDef halResult = HAL_I2C_Master_Transmit(hdc1080_hi2c, (I2C_Addr << 1), pData, size, 100);
	return (halResult == HAL_OK) ? HDC1080::Status::OK : HDC1080::Status::FAIL_I2C;
}

HDC1080::Status HDC1080::I2C_Receive(uint8_t ID, uint8_t I2C_Addr, uint8_t *pData, uint16_t size) {
	(void)ID; // Unused
	HAL_StatusTypeDef halResult = HAL_I2C_Master_Receive(hdc1080_hi2c, (I2C_Addr << 1), pData, size, 100);
	return (halResult == HAL_OK) ? HDC1080::Status::OK : HDC1080::Status::FAIL_I2C;
}

void HDC1080::Delay(uint16_t ms) {
	HAL_Delay(ms);
}

#endif /* HDC1080_USE_STM32_HAL_METHODS */

/*** END OF FILE ***/
