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
 * 		 It is possible to reduce the delay time by defining shorter conversion times for lower resolution measurements
 * 		 and amending the measurement register getter methods to use these shorter delay times.
 */
#define HDC1080_CONVERSION_TIME_TEMPERATURE 15u
#define HDC1080_CONVERSION_TIME_HUMIDITY	15u
#define HDC1080_CONVERSION_TIME_DUAL		20u

using Register		= HDC1080::I2C::Register;
using MemoryAddress = HDC1080::I2C::MemoryAddress;

inline static Register setBits(Register reg, Register bits) {
	return (reg | bits);
}

inline static Register clearBits(Register reg, Register bits) {
	return (reg & ~bits);
}

/**
 * @brief Writes the given bits to the given register, masking the bits to be written with the given mask.
 *
 * @param reg The register to write to.
 * @param bits The bits to write to the register.
 * @param mask The mask to apply to the bits to be written.
 * @return Register
 */
inline static Register writeBits(Register reg, Register bits, Register mask) {
	return (reg & ~mask) | (bits & mask);
}

HDC1080::HDC1080(HDC1080::I2C *i2c) : i2c(i2c) {}

float HDC1080::getTemperature(Register temperatureRegister) {
	return (float)temperatureRegister * (165.0 / 65536.0) - 40.0;
}

float HDC1080::getTemperature() {
	auto temperatureRegister = getTemperatureRegister();

	if (temperatureRegister.has_value()) return getTemperature(temperatureRegister.value());

	return -40.0;
}

float HDC1080::getHumidity(Register humidityRegister) {
	return (float)humidityRegister * (25.0 / 16384.0);
}

float HDC1080::getHumidity() {
	auto humidityRegister = getHumidityRegister();

	if (humidityRegister.has_value()) return getHumidity(humidityRegister.value());

	return 0.0;
}

std::optional<Register> HDC1080::getTemperatureRegister() {
	return getMeasurementRegister(HDC1080_TEMPERATURE_ADDR, HDC1080_CONVERSION_TIME_TEMPERATURE);
}

std::optional<Register> HDC1080::getHumidityRegister() {
	return getMeasurementRegister(HDC1080_HUMIDITY_ADDR, HDC1080_CONVERSION_TIME_HUMIDITY);
}

std::optional<uint64_t> HDC1080::getSerialID() {
	Register serialID[3];
	for (unsigned int i = 0; i < 3; i++) {
		auto transmission = this->i2c->read(HDC1080_SERIAL_ID_ADDR + i);
		if (transmission.has_value()) serialID[i] = transmission.value();
		else return std::nullopt;
	}
	const uint64_t serialIdValue = (((uint64_t)serialID[0] & 0xFFFF) << 25) // SERIAL_ID[40:25], register bits [15:0]
								 | (((uint64_t)serialID[1] & 0xFFFF) << 9)	// SERIAL_ID[24:9], register bits [15:0]
								 | (((uint64_t)serialID[2] & 0xFF80) >> 7); // SERIAL_ID[8:0], register bits [15:7]

	return serialIdValue;
}

std::optional<HDC1080::Battery> HDC1080::getBatteryStatus() {
	auto transmission = this->i2c->read(HDC1080_CONFIG_ADDR);

	if (transmission.has_value())
		return static_cast<HDC1080::Battery>(!(transmission.value() & HDC1080_CONFIG_BATTERY_STATUS_MASK));
	else return std::nullopt;
}

std::optional<HDC1080::I2C::Register>
HDC1080::getMeasurementRegister(HDC1080::I2C::MemoryAddress memAddr, uint32_t waitTime) {
	if (!this->i2c->transmit(static_cast<uint8_t>(memAddr))) return {};

	this->i2c->delay(waitTime);

	std::optional<uint8_t> transmissionData[2];
	for (auto &&data : transmissionData) {
		data = this->i2c->receive();
		if (!data) return {};
	}

	return static_cast<HDC1080::I2C::Register>(transmissionData[0].value() << 8 | transmissionData[1].value());
}

struct Config {
	HDC1080::AcquisitionMode	   acqMode;
	HDC1080::TemperatureResolution tRes;
	HDC1080::HumidityResolution	   hRes;
	HDC1080::Heater				   heater;

#ifdef HDC1080_GTEST_TESTING
	bool operator==(const Config &other) const = default;
#endif
};

static Register constructConfigRegister(Config config) {
	return static_cast<Register>(config.acqMode) //
		 | static_cast<Register>(config.tRes)	 //
		 | static_cast<Register>(config.hRes)	 //
		 | static_cast<Register>(config.heater);
}

static Config decodeConfigRegister(Register configRegister) {
	return Config{
		static_cast<HDC1080::AcquisitionMode>(configRegister & HDC1080_CONFIG_ACQUISITION_MODE_MASK),
		static_cast<HDC1080::TemperatureResolution>(configRegister & HDC1080_CONFIG_TEMPERATURE_RESOLUTION_MASK),
		static_cast<HDC1080::HumidityResolution>(configRegister & HDC1080_CONFIG_HUMIDITY_RESOLUTION_MASK),
		static_cast<HDC1080::Heater>(configRegister & HDC1080_CONFIG_HEATER_MASK)};
}

std::optional<Register> HDC1080::setConfig(
	std::optional<AcquisitionMode>		 acqMode,
	std::optional<TemperatureResolution> tRes,
	std::optional<HumidityResolution>	 hRes,
	std::optional<Heater>				 heater
) {
	if (!acqMode && !tRes && !hRes && !heater) return {};

	// Check for all options being supplied
	if (acqMode && tRes && hRes && heater) {
		Config config{acqMode.value(), tRes.value(), hRes.value(), heater.value()};

		auto newConfigRegister = constructConfigRegister(config);
		return this->i2c->write(HDC1080_CONFIG_ADDR, newConfigRegister);
	} else {
		auto currentConfigRegister = this->i2c->read(HDC1080_CONFIG_ADDR);

		if (!currentConfigRegister) return {};

		auto config = decodeConfigRegister(currentConfigRegister.value());

		if (acqMode) config.acqMode = acqMode.value();
		if (tRes) config.tRes = tRes.value();
		if (hRes) config.hRes = hRes.value();
		if (heater) config.heater = heater.value();

		auto newConfigRegister = constructConfigRegister(config);

		if (newConfigRegister == currentConfigRegister.value()) return newConfigRegister;

		return this->i2c->write(HDC1080_CONFIG_ADDR, newConfigRegister);
	}
}

std::optional<Register> HDC1080::setAcquisitionMode(AcquisitionMode acqMode) {
	return setConfig(acqMode, std::nullopt, std::nullopt, std::nullopt);
}

std::optional<Register> HDC1080::setTemperatureResolution(TemperatureResolution tRes) {
	return setConfig(std::nullopt, tRes, std::nullopt, std::nullopt);
}

std::optional<Register> HDC1080::setHumidityResolution(HumidityResolution hRes) {
	return setConfig(std::nullopt, std::nullopt, hRes, std::nullopt);
}

std::optional<Register> HDC1080::setHeater(Heater heater) {
	return setConfig(std::nullopt, std::nullopt, std::nullopt, heater);
}

std::optional<Register> HDC1080::softReset(void) {
	return this->i2c->write(HDC1080_CONFIG_ADDR, HDC1080_CONFIG_RESET_MASK);
}
