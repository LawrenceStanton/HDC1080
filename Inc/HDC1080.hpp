/**
 ******************************************************************************
 * @file           : HDC1080.hpp
 * @brief          : HDC1080 Digital Humidity and Temperature Sensor Driver
 * @author		   : Lawrence Stanton
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

#define HDC1080_I2C_ADDR 0x40u

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
		virtual void delay(uint32_t ms) const = 0;

		virtual ~I2C() = default;
	};

	using MemoryAddress = I2C::MemoryAddress;
	using Register		= I2C::Register;

	// Configuration Register Enumerations
	enum class AcquisitionMode : Register {
		SINGLE = 0x0000u,
		DUAL   = 0x1000u, // Temperature is measured first, then humidity.
	};

	enum class TemperatureResolution : Register {
		A_14BIT = 0x0000u,
		A_11BIT = 0x0400u,
	};

	enum class HumidityResolution : Register {
		A_14BIT = 0x0000u,
		A_11BIT = 0x0100u,
		A_8BIT	= 0x0200u,
	};

	enum class Heater : Register {
		ON	= 0x2000u,
		OFF = 0x0000u,
	};

	enum class Battery : bool {
		LOW	 = false,
		HIGH = true,
	};

protected:
	I2C *i2c;

public:
	/**
	 * @brief Construct a new HDC1080::HDC1080 object given all configuration settings.
	 *
	 * @param acqMode	The Mode of Acquisition (single or dual measurements).
	 * @param tRes		The resolution of the temperature measurement (11 or 14 bit).
	 * @param hRes		The resolution of the humidity measurement (8, 11, or 14 bit).
	 * @param heater	The heater setting (on or off).
	 */
	HDC1080(I2C *i2c);

	~HDC1080() = default;

	/**
	 * @brief Get the Temperature from the HDC1080.
	 *
	 * @return float The temperature measurement in degrees Celsius.
	 *
	 * @note In the event of I2C failure, this results in T = -40.0.
	 */
	float getTemperature() const;

	/**
	 * @brief Get the Humidity from the HDC1080.
	 *
	 * @return float The humidity measurement in percent relative humidity.
	 *
	 * @note In the event of I2C failure, this results in H = 0.0.
	 */
	float getHumidity() const;

	/* Other Device Information Get Methods */
	/**
	 * @brief Get the Device ID of the HDC1080.
	 *
	 * @return std::optional<Register> 0x1050 if successful.
	 */
	std::optional<Register> getDeviceID() const;

	/**
	 * @brief Get the Manufacturer ID of the HDC1080.
	 *
	 * @return std::optional<uint16_t> 0x5449 if successful.
	 */
	std::optional<uint16_t> getManufacturerID() const;

	/**
	 * @brief Get the Serial ID of the HDC1080.
	 *
	 * @return std::optional<uint64_t> The 41-bit serial ID if successful.
	 *
	 * @note The serial ID is a 41-bit number by the register map, but the HDC1080 datasheet states "40-bit".
	 * 		 This driver will assume the register map is correct.
	 * @note Unused bits are set to 0.
	 */
	std::optional<uint64_t> getSerialID() const;

	/**
	 * @brief Get the Battery Status of the HDC1080.
	 *
	 * @return std::optional<bool> True if the supply voltage greater than 2V8, false if less than 2V8.
	 */
	std::optional<Battery> getBatteryStatus() const;

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
	 * @note All arguments are optional. Unspecified arguments will remain unchanged. If all arguments are specified,
	 *    	 the config register is not read before being written.
	 * @note If no arguments are specified the function short circuits and returns an empty optional.
	 */
	std::optional<Register> setConfig(
		std::optional<AcquisitionMode>		 acqMode, //
		std::optional<TemperatureResolution> tRes,
		std::optional<HumidityResolution>	 hRes,
		std::optional<Heater>				 heater
	) const;

	/**
	 * @brief Single Configuration Register Set Methods.
	 *
	 * @param configParam The desired configuration parameter to set.
	 * @return std::optional<Register> The written value of the configuration register if successful.
	 */
	std::optional<Register> setAcquisitionMode(AcquisitionMode acqMode) const;
	std::optional<Register> setTemperatureResolution(TemperatureResolution tRes) const;
	std::optional<Register> setHumidityResolution(HumidityResolution hRes) const;
	std::optional<Register> setHeater(Heater heater) const;

	/**
	 * @brief Performs a software reset of the HDC1080.
	 *
	 * @return std::optional<Register> The written value of the configuration register if successful.
	 * @note The written value shall always be 0x8000.
	 */
	std::optional<Register> softReset(void) const;

private:
	/**
	 * @brief Get the Temperature Register
	 *
	 * @return std::optional<Register> The fetched value of the temperature register if successful.
	 */
	inline std::optional<Register> getTemperatureRegister() const;

	/**
	 * @brief Get the Humidity Register
	 *
	 * @return std::optional<Register> The fetched value of the humidity register if successful.
	 */
	inline std::optional<Register> getHumidityRegister() const;

	/**
	 * @brief Get a Measurement Register from the HDC1080.
	 *
	 * @param memAddr The memory address of the register to fetch.
	 * @param waitTime The measurement conversion delay time in milliseconds.
	 * @return std::optional<Register> The fetched value of the measurement register if successful.
	 */
	std::optional<Register> getMeasurementRegister(MemoryAddress memAddr, uint32_t waitTime = 0) const;

/* Registration for Private Member Testing Purposes Only */
#ifdef HDC1080_GTEST_TESTING
	friend class HDC1080_Test;

	FRIEND_TEST(HDC1080_Test, constructorAssignsI2cInterfacePointer);

	FRIEND_TEST(HDC1080_Test, getTemperatureRegisterNormallyReturnsValue);
	FRIEND_TEST(HDC1080_Test, getTemperatureRegisterReturnsEmptyOptionalWhenI2CFails);

	FRIEND_TEST(HDC1080_Test, getHumidityRegisterNormallyReturnsValue);
	FRIEND_TEST(HDC1080_Test, getHumidityRegisterReturnsEmptyOptionalWhenI2CReceiveFails);

	FRIEND_TEST(HDC1080_Test, getMeasurementRegisterNormallyReturnsValue);
	FRIEND_TEST(HDC1080_Test, getMeasurementRegisterReturnsEmptyOptionalWhenI2CReceiveFails);
#endif
};

/*** END OF FILE ***/
