/**
 ******************************************************************************
 * @file           : HDC1080.cpp
 * @brief          : Source for HDC1080 Driver
 * @author		   : Lawrence Stanton
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

using namespace units::literals;

#define HDC1080_MEM_SIZE 2u

/* Register Addresses */
#define HDC1080_TEMPERATURE_ADDR	 0x00u
#define HDC1080_HUMIDITY_ADDR		 0x01u
#define HDC1080_CONFIG_ADDR			 0x02u
#define HDC1080_SERIAL_ID_ADDR		 0xFBu
#define HDC1080_MANUFACTURER_ID_ADDR 0xFEu
#define HDC1080_DEVICE_ID_ADDR		 0xFFu

/* Register Address Sizes (in bytes) */
#define HDC1080_TEMPERATURE_ADDR_SIZE	  2u
#define HDC1080_HUMIDITY_ADDR_SIZE		  2u
#define HDC1080_CONFIG_ADDR_SIZE		  2u
#define HDC1080_SERIAL_ID_ADDR_SIZE		  6u
#define HDC1080_MANUFACTURER_ID_ADDR_SIZE 2u
#define HDC1080_DEVICE_ID_ADDR_SIZE		  2u

/* Power-on Reset Values */
// PORV = Power-On Reset Value
#define HDC1080_TEMPERATURE_ADDR_PORV	  0x0000u
#define HDC1080_HUMIDITY_ADDR_PORV		  0x0000u
#define HDC1080_CONFIG_ADDR_PORV		  0x1000u
#define HDC1080_MANUFACTURER_ID_ADDR_PORV 0x5449u
#define HDC1080_DEVICE_ID_ADDR_PORV		  0x1050u

/* Configuration Register Bit Masks */
#define HDC1080_CONFIG_RESET_MASK				   0x8000u
#define HDC1080_CONFIG_HEATER_MASK				   0x2000u
#define HDC1080_CONFIG_ACQUISITION_MODE_MASK	   0x1000u
#define HDC1080_CONFIG_BATTERY_STATUS_MASK		   0x0800u
#define HDC1080_CONFIG_TEMPERATURE_RESOLUTION_MASK 0x0400u
#define HDC1080_CONFIG_HUMIDITY_RESOLUTION_MASK	   0x0300u

/**
 * @note The below values are given a wide margin above the maximum conversion time
 * specifications to allow for a imprecise delay timer method.
 *
 * @note As of current in this source, the conversion time does not consider the
 * resolution of the measurement. It is possible to reduce the delay time by defining
 * shorter conversion times for lower resolution measurements and amending the measurement
 * register getter methods to use these shorter delay times.
 */
#define HDC1080_CONVERSION_TIME_TEMPERATURE 15ms
#define HDC1080_CONVERSION_TIME_HUMIDITY	15ms
#define HDC1080_CONVERSION_TIME_DUAL		20ms

using Register		= HDC1080::I2C::Register;
using MemoryAddress = HDC1080::I2C::MemoryAddress;

HDC1080::HDC1080(I2C &i2c) : i2c{i2c} {}

std::optional<Register> HDC1080::getTemperatureRegister(void) const {
	return getMeasurementRegister(
		HDC1080_TEMPERATURE_ADDR,
		HDC1080_CONVERSION_TIME_TEMPERATURE
	);
}

std::optional<Register> HDC1080::getHumidityRegister(void) const {
	return getMeasurementRegister(
		HDC1080_HUMIDITY_ADDR,
		HDC1080_CONVERSION_TIME_HUMIDITY
	);
}

std::optional<Register> HDC1080::getDeviceID(void) const {
	return this->i2c.read(HDC1080_DEVICE_ID_ADDR);
}

std::optional<uint16_t> HDC1080::getManufacturerID(void) const {
	return this->i2c.read(HDC1080_MANUFACTURER_ID_ADDR);
}

std::optional<uint64_t> HDC1080::getSerialID(void) const {
	Register serialID[3];
	for (unsigned int i = 0; i < 3; i++) {
		auto transmission = this->i2c.read(HDC1080_SERIAL_ID_ADDR + i);
		if (transmission.has_value()) serialID[i] = transmission.value();
		else return std::nullopt;
	}
	const uint64_t serialIdValue =
		(((uint64_t)serialID[0] & 0xFFFF) << 25) // SERIAL_ID[40:25], register bits [15:0]
		| (((uint64_t)serialID[1] & 0xFFFF) << 9) // SERIAL_ID[24:9], register bits [15:0]
		| (((uint64_t)serialID[2] & 0xFF80) >> 7); // SERIAL_ID[8:0], register bits [15:7]

	return serialIdValue;
}

std::optional<HDC1080::Battery> HDC1080::getBatteryStatus(void) const {
	auto transmission = this->i2c.read(HDC1080_CONFIG_ADDR);

	if (transmission.has_value())
		return static_cast<HDC1080::Battery>(
			!(transmission.value() & HDC1080_CONFIG_BATTERY_STATUS_MASK)
		);
	else return std::nullopt;
}

std::optional<HDC1080::I2C::Register> HDC1080::getMeasurementRegister(
	HDC1080::I2C::MemoryAddress memAddr,
	Duration					waitTime
) const {
	if (!this->i2c.transmit(static_cast<uint8_t>(memAddr))) return {};

	this->i2c.delay(waitTime);

	std::optional<uint8_t> transmissionData[2];
	for (auto &&data : transmissionData) {
		data = this->i2c.receive();
		if (!data) return {};
	}

	return static_cast<HDC1080::I2C::Register>(
		transmissionData[0].value() << 8 | transmissionData[1].value()
	);
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
		static_cast<HDC1080::AcquisitionMode>(
			configRegister & HDC1080_CONFIG_ACQUISITION_MODE_MASK
		),
		static_cast<HDC1080::TemperatureResolution>(
			configRegister & HDC1080_CONFIG_TEMPERATURE_RESOLUTION_MASK
		),
		static_cast<HDC1080::HumidityResolution>(
			configRegister & HDC1080_CONFIG_HUMIDITY_RESOLUTION_MASK
		),
		static_cast<HDC1080::Heater>(configRegister & HDC1080_CONFIG_HEATER_MASK),
	};
}

std::optional<Register> HDC1080::setConfig(
	std::optional<AcquisitionMode>		 acqMode,
	std::optional<TemperatureResolution> tRes,
	std::optional<HumidityResolution>	 hRes,
	std::optional<Heater>				 heater
) const {
	if (!acqMode && !tRes && !hRes && !heater) return {};

	// Check for all options being supplied
	if (acqMode && tRes && hRes && heater) {
		Config config{acqMode.value(), tRes.value(), hRes.value(), heater.value()};

		auto newConfigRegister = constructConfigRegister(config);
		return this->i2c.write(HDC1080_CONFIG_ADDR, newConfigRegister);
	} else {
		auto currentConfigRegister = this->i2c.read(HDC1080_CONFIG_ADDR);

		if (!currentConfigRegister) return {};

		auto config = decodeConfigRegister(currentConfigRegister.value());

		if (acqMode) config.acqMode = acqMode.value();
		if (tRes) config.tRes = tRes.value();
		if (hRes) config.hRes = hRes.value();
		if (heater) config.heater = heater.value();

		auto newConfigRegister = constructConfigRegister(config);

		if (newConfigRegister == currentConfigRegister.value()) return newConfigRegister;

		return this->i2c.write(HDC1080_CONFIG_ADDR, newConfigRegister);
	}
}

std::optional<Register> HDC1080::softReset(void) const {
	return this->i2c.write(HDC1080_CONFIG_ADDR, HDC1080_CONFIG_RESET_MASK);
}

/* Measurement Conversion Methods */
/**
 * @brief Get the temperature measurement, given the register value.
 *
 * @return The temperature measurement in degrees Celsius.
 */
static Celsius constexpr convertTemperature(Register temperatureRegister) {
	return Celsius(
		temperatureRegister * (165.0f / 65536.0f) - 40.0f
	); // Refer to HDC1080 datasheet for formula
}

/**
 * @brief Get the humidity measurement, given the register value.
 *
 * @return The percent relative humidity.
 */
static RelativeHumidity convertHumidity(Register humidityRegister) {
	return RelativeHumidity(
		humidityRegister * (25.0f / 16384.0f) * 100
	); // Refer to HDC1080 datasheet for formula
}

Celsius HDC1080_X::getTemperature(void) const {
	auto temperatureRegister = getTemperatureRegister();

	if (temperatureRegister.has_value())
		return convertTemperature(temperatureRegister.value());

	return -40.0_degC;
}

RelativeHumidity HDC1080_X::getHumidity(void) const {
	auto humidityRegister = getHumidityRegister();

	if (humidityRegister.has_value()) return convertHumidity(humidityRegister.value());

	return 0.0;
}

std::optional<Register> HDC1080_X::setAcquisitionMode(AcquisitionMode acqMode) const {
	return setConfig(acqMode, std::nullopt, std::nullopt, std::nullopt);
}

std::optional<Register> HDC1080_X::setTemperatureResolution(TemperatureResolution tRes
) const {
	return setConfig(std::nullopt, tRes, std::nullopt, std::nullopt);
}

std::optional<Register> HDC1080_X::setHumidityResolution(HumidityResolution hRes) const {
	return setConfig(std::nullopt, std::nullopt, hRes, std::nullopt);
}

std::optional<Register> HDC1080_X::setHeater(Heater heater) const {
	return setConfig(std::nullopt, std::nullopt, std::nullopt, heater);
}
