/**
 ******************************************************************************
 * @file           : HDC1080.hpp
 * @brief          : Header file for HDC1080.cpp
 * @author			: Lawrence Stanton
 ******************************************************************************
 * @attention
 *
 * © LD Stanton 2022
 *
 * This file and its content are the copyright property of the author. All
 * rights are reserved. No warranty is given. No liability is assumed.
 * Confidential unless licensed otherwise. If licensed, refer to the
 * accompanying file "LICENCE" for license details.
 *
 ******************************************************************************
 */
#pragma once

#include <cstdint>
#include <optional>

#ifdef HDC1080_GTEST_TESTING
#include "gtest/gtest.h"
#endif

#define HDC1080_I2C_ADDR 0x40u
#define HDC1080_MEM_SIZE 2u

/* uint16_t Addresses */
#define HDC1080_TEMPERATURE_ADDR	 0x00u
#define HDC1080_HUMIDITY_ADDR		 0x01u
#define HDC1080_CONFIG_ADDR			 0x02u
#define HDC1080_SERIAL_ID_ADDR		 0xFBu
#define HDC1080_MANUFACTURER_ID_ADDR 0xFEu
#define HDC1080_DEVICE_ID_ADDR		 0xFFu

/* uint16_t Address Sizes (in bytes) */
#define HDC1080_TEMPERATURE_ADDR_SIZE	  2u
#define HDC1080_HUMIDITY_ADDR_SIZE		  2u
#define HDC1080_CONFIG_ADDR_SIZE		  2u
#define HDC1080_SERIAL_ID_ADDR_SIZE		  6u
#define HDC1080_MANUFACTURER_ID_ADDR_SIZE 2u
#define HDC1080_DEVICE_ID_ADDR_SIZE		  2u

/* Power-on Reset Values */
#define HDC1080_TEMPERATURE_ADDR_PORV	  0x0000u
#define HDC1080_HUMIDITY_ADDR_PORV		  0x0000u
#define HDC1080_CONFIG_ADDR_PORV		  0x1000u
#define HDC1080_MANUFACTURER_ID_ADDR_PORV 0x5449u
#define HDC1080_DEVICE_ID_ADDR_PORV		  0x1050u

#define HDC1080_CONFIG_RESET 0x8000u

class HDC1080 {
public:
	/**
	 * @brief I2C interface for the HDC1080.
	 *
	 * @details
	 * This interface is used to abstract the I2C communication from the HDC1080 class.
	 * A concrete implementation is then aggregated by the HDC1080 class.
	 * The concrete implementation of this interface should be provided by the user.
	 * This also allows for the HDC1080 class to be tested without the need for a real I2C bus.
	 * The concrete implementation's lifetime must be managed by the user and must outlive the HDC1080 class.
	 * It is recommended that the concrete implementation be a static object.
	 */
	class I2C {
	public:
		typedef uint8_t	 MemoryAddress; // Every HDC1080 memory address is 8 bits wide.
		typedef uint16_t Register;		// Every HDC1080 register is 16 bits wide.

		/**
		 * @brief Read a register from the HDC1080.
		 *
		 * @param memoryAddress The HDC1080 internal memory address of the register to read.
		 * @return std::optional<Register> The value of the register if the read was successful.
		 */
		virtual std::optional<Register> read(MemoryAddress memoryAddress)				  = 0;
		/**
		 * @brief Write a register to the HDC1080.
		 *
		 * @param memoryAddress The HDC1080 internal memory address of the register to write.
		 * @param data The data to write to the register.
		 * @return std::optional<Register> The value written to the register if the write was successful.
		 */
		virtual std::optional<Register> write(MemoryAddress memoryAddress, Register data) = 0;

		/**
		 * @brief Transmit a byte of data to the HDC1080.
		 *
		 * @param data The byte of data to transmit.
		 * @return std::optional<uint8_t> The byte of data transmitted to the HDC1080 if successful.
		 *
		 * @note This function is normally used to get measurement data from the HDC1080, where some measurement
		 * delays are necessary, and therefore this driver will assume implementation of the I2C protocol.
		 */
		virtual std::optional<uint8_t> transmit(uint8_t data) = 0;
		/**
		 * @brief Receive a byte of data from the HDC1080.
		 *
		 * @return std::optional<uint8_t> The byte of data received from the HDC1080 if successful.
		 *
		 * @note This function is normally used to get measurement data from the HDC1080, where some measurement
		 * delays are necessary, and therefore this driver will assume implementation of the I2C protocol.
		 */
		virtual std::optional<uint8_t> receive()			  = 0;

		/**
		 * @brief Delay for a given number of milliseconds.
		 *
		 * @note This need not be a precise delay.
		 *
		 * @param ms The number of milliseconds to delay for.
		 */
		virtual void delay(uint32_t ms) = 0;
	};

	using MemoryAddress = I2C::MemoryAddress;
	using Register		= I2C::Register;

	enum class AcqModeConfig : uint16_t {
		SINGLE = 0x0000u,
		DUAL   = 0x0100u,
	};

	enum class TempResolutionConfig : uint16_t {
		A_14BIT = 0x0000u,
		A_11BIT = 0x0040u,
	};

	enum class HumidityResolutionConfig : uint16_t {
		A_14BIT = 0x0000u,
		A_11BIT = 0x0010u,
		A_8BIT	= 0x0020u,
	};

	enum class HeaterConfig : uint16_t {
		OFF = 0x2000u,
		ON	= 0x0000u,
	};

protected:
	I2C &i2c;

public:
	/**
	 * @brief Construct a new HDC1080::HDC1080 object given all configuration settings.
	 *
	 * @param acqMode	The Mode of Acquisition (single or dual measurements).
	 * @param tRes		The resolution of the temperature measurement (11 or 14 bit).
	 * @param hRes		The resolution of the humidity measurement (8, 11, or 14 bit).
	 * @param heater	The heater setting (on or off).
	 */
	HDC1080(
		I2C						&i2c,												   // I2C Driver
		AcqModeConfig			 acqMode = HDC1080::AcqModeConfig::DUAL,			   // Acquisition Mode
		TempResolutionConfig	 tRes	 = HDC1080::TempResolutionConfig::A_14BIT,	   // Temperature Resolution
		HumidityResolutionConfig hRes	 = HDC1080::HumidityResolutionConfig::A_14BIT, // Humidity Resolution
		HeaterConfig			 heater	 = HDC1080::HeaterConfig::OFF				   // Heater On/Off
	);

	// inline ~HDC1080() { softReset(); }

	/**
	 * @brief Triggers a software reset of the HDC1080 by setting the respective bit in the configuration register.
	 */
	// inline void softReset() { setuint16_t(HDC1080_CONFIG_ADDR, HDC1080_CONFIG_RESET); };

	/* Floating Point Measurement Conversion Methods */
	/**
	 * @brief Updates the uint16_t and real class copies of the temperature measurement.
	 *
	 * @return float The updated real temperature measurement.
	 */
	static float getTemperature(Register temperatureRegister);

	/**
	 * @brief Get the Temperature
	 *
	 * @return float The updated real temperature measurement.
	 *
	 * @note In the event of failure, this results in T = -40.0.
	 */
	float getTemperature();

	/**
	 * @brief Updates the uint16_t and real class copies of the humidity measurement.
	 *
	 * @return float The updated real humidity measurement.
	 *
	 * @note In the event of hReg = 0x0000u, which might indicate a failure, this results in H = 0.0.
	 */
	static float getHumidity(Register humidityRegister);

	/**
	 * @brief Get the Humidity
	 *
	 * @return float The updated real humidity measurement.
	 *
	 * @note In the event of failure, this results in H = 0.0.
	 */
	float getHumidity();

	/* Other Device Information Get Methods */
	/**
	 * @brief Get the Device ID of the HDC1080.
	 *
	 * @return std::optional<Register> 0x1050 if successful.
	 */
	inline std::optional<Register> getDeviceID() { return this->i2c.read(HDC1080_DEVICE_ID_ADDR); }

	/**
	 * @brief Get the Manufacturer ID of the HDC1080.
	 *
	 * @return std::optional<uint16_t> 0x5449 if successful.
	 */
	inline std::optional<uint16_t> getManufacturerID() { return this->i2c.read(HDC1080_MANUFACTURER_ID_ADDR); }

	/**
	 * @brief Get the Serial ID of the HDC1080.
	 *
	 * @return std::optional<uint64_t> The 41-bit serial ID if successful.
	 *
	 * @note The serial ID is a 41-bit number by the register map, but the HDC1080 datasheet states "40-bit".
	 * 		 This driver will assume the register map is correct.
	 * @note Unused bits are set to 0.
	 */
	std::optional<uint64_t> getSerialID();

	// std::optional<bool> getBatteryStatus(); // TODO

private:
	/**
	 * @brief Get the Temperature Register
	 *
	 * @return std::optional<Register> The fetched value of the temperature register if successful.
	 */
	std::optional<Register> getTemperatureRegister();
	/**
	 * @brief Get the Humidity Register
	 *
	 * @return std::optional<Register> The fetched value of the humidity register if successful.
	 */
	std::optional<Register> getHumidityRegister();

	/* 	TODO: Individual Device Setting Setter Methods
	void setAcquisitionMode(AcqModeConfig acqMode);
	void setTemperatureResolution(TempResolutionConfig tRes);
	void setHumidityResolution(HumidityResolutionConfig hRes);
	void setHeater(HeaterConfig heater);
	*/

	/**
	 * @brief Sets the HDC1080 Configuration Register given all programmable settings.
	 *
	 * @param acqMode	The Mode of Acquisition (single or dual measurements).
	 * @param tRes		The resolution of the temperature measurement (11 or 14 bit).
	 * @param hRes		The resolution of the humidity measurement (8, 11, or 14 bit).
	 * @param heater	The heater setting (on or off).
	 *
	 * @note The heater should only be turned on if necessary for saturated conditions. Refer to the datasheet §8.3.3.
	 * @note If only a single measurement is desired, the other measurement's resolution may be set to any valid value.
	 */
	std::optional<Register> setConfig(
		AcqModeConfig			 acqMode, //
		TempResolutionConfig	 tRes,
		HumidityResolutionConfig hRes,
		HeaterConfig			 heater
	);

	std::optional<Register> getMeasurementRegister(MemoryAddress memAddr, uint32_t waitTime = 0);

/* Registration for Private Member Testing Purposes Only */
#ifdef HDC1080_GTEST_TESTING
	friend class HDC1080_Test;

	FRIEND_TEST(HDC1080_Test, getTemperatureRegisterNormallyReturnsValue);
	FRIEND_TEST(HDC1080_Test, getTemperatureRegisterReturnsEmptyOptionalWhenI2CFails);

	FRIEND_TEST(HDC1080_Test, getHumidityRegisterNormallyReturnsValue);
	FRIEND_TEST(HDC1080_Test, getHumidityRegisterReturnsEmptyOptionalWhenI2CReceiveFails);

	FRIEND_TEST(HDC1080_Test, getMeasurementRegisterNormallyReturnsValue);
	FRIEND_TEST(HDC1080_Test, getMeasurementRegisterReturnsEmptyOptionalWhenI2CReceiveFails);
#endif
};

/*** END OF FILE ***/
