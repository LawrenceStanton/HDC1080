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

#include <stdint.h>

#define HDC1080_I2C_ADDR 0x40u
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
#define HDC1080_TEMPERATURE_ADDR_PORV	  0x0000u
#define HDC1080_HUMIDITY_ADDR_PORV		  0x0000u
#define HDC1080_CONFIG_ADDR_PORV		  0x1000u
#define HDC1080_MANUFACTURER_ID_ADDR_PORV 0x5449u
#define HDC1080_DEVICE_ID_ADDR_PORV		  0x1050u

/**
 * @note Define this constant in order to implement variables and methods for computing the floating point measurement
 * values.
 */
#define HDC1080_IMPLEMENT_FLOAT_VARIABLES

/**
 * @note 	Due to the endianness of a system and the use of variable types in this driver, it may or may not
 * 			be necessary to swap the first and last bytes of a 2-byte value when typecasting from 8-bit value
 * 			to a 16-bit. The 8-bit values are primarily used for I2C transactions, which are 1-byte sized.
 */
#define HDC1080_LITTLE_ENDIAN // Define this variable if the system is set to little endian format. Else comment out.
#ifdef HDC1080_LITTLE_ENDIAN
#define __HDC1080_UINT16_REVERSE_CAST(__REG__) (((__REG__ << 8) & 0xFF00) | ((__REG__ >> 8) & 0x00FF))
#else
#define __HDC1080_UINT16_REVERSE_CAST(__REG__) __REG__
#endif /* HDC1080_LITTLE_ENDIAN */

/**
 * @note 	Due to the endianness of a system and the use of variable types in this driver, it may or may not
 * 			be necessary to swap the first and last bytes of a 2-byte value when typecasting from 8-bit value
 * 			to a 16-bit. The 8-bit values are primarily used for I2C transactions, which are 1-byte sized.
 */
class HDC1080 {
public:
	/* Measurement Register Class Copies */
	volatile uint16_t tReg = 0x0000;
	volatile uint16_t hReg = 0x0000;

#ifdef HDC1080_IMPLEMENT_FLOAT_VARIABLES
	/* Real Measurement Class Copies */
	volatile float T = -273.0;
	volatile float H = 0.0;
#endif /* HDC1080_IMPLEMENT_FLOAT_VARIABLES */

	/* Configuration Setting Enumerations */
	enum class AcqModeConfig : uint16_t;
	enum class HeaterConfig : uint16_t;
	enum class TempResolutionConfig : uint16_t;
	enum class HumidityResolutionConfig : uint16_t;
	enum class Status : int;

	/* Constructors and Destructors */
	HDC1080(
		AcqModeConfig			 acqMode,
		TempResolutionConfig	 tRes,
		HumidityResolutionConfig hRes,
		HeaterConfig			 heater,
		uint16_t				 ID
	);

	inline ~HDC1080() { softReset(); }

	void softReset();

#ifdef HDC1080_IMPLEMENT_FLOAT_VARIABLES
	/* Real Measurement Get Methods */
	float getTemperature();
	float getHumidity();
	void  getTemperatureHumidity();
#endif /* HDC1080_IMPLEMENT_FLOAT_VARIABLES */

	/* Measurement Register Get Methods */
	uint16_t getTemperatureRegister();
	uint16_t getHumidityRegister();
	void	 getTemperatureHumidityRegisters();

	/* Other Device Information Get Methods */
	inline uint16_t getDeviceID() { return getRegister(HDC1080_DEVICE_ID_ADDR); }
	inline uint16_t getManufacturerID() { return getRegister(HDC1080_MANUFACTURER_ID_ADDR); }
	uint64_t		getSerialID();
	// bool	 getBatteryStatus();	TODO

	/* 	TODO: Individual Device Setting Setter Methods

	void setAcquisitionMode(AcqModeConfig acqMode);
	void setTemperatureResolution(TempResolutionConfig tRes);
	void setHumidityResolution(HumidityResolutionConfig hRes);
	void setHeater(HeaterConfig heater);

	*/

private:
	uint8_t ID; // User-defined identifier. To be used by host application to identify device in multi-HDC1080 system.

#ifdef HDC1080_IMPLEMENT_FLOAT_VARIABLES
	/* Real Measurement Updating Methods */
	void updateTemperature();
	void updateHumidity();
#endif /* HDC1080_IMPLEMENT_FLOAT_VARIABLES */

	/* Configuration Register Getters and Setters */
	void
	setConfig(AcqModeConfig acqMode, TempResolutionConfig tRes, HumidityResolutionConfig hRes, HeaterConfig heater);
	inline void		setConfig(uint16_t config) { setRegister(HDC1080_CONFIG_ADDR, config); }
	inline uint16_t getConfig() { return getRegister(HDC1080_CONFIG_ADDR); }

	/* General Purpose Register Getters and Setters */
	uint16_t getRegister(uint8_t memAddr);
	void	 setRegister(uint8_t memAddr, uint16_t data);
	void	 getMeasurementRegisters(uint8_t memAddr, uint16_t waitTime, uint16_t *pData, uint8_t n);

	/* System-level I2C communication methods. To be implemented by host application. */

	/**
	 * @brief Read a set amount of data via the I2C interface, given the device I2C and internal memory addresses.
	 *
	 * @param ID		The user-defined identifier of the HDC1080 class making this call.
	 * @param i2cAddr	The I2C address. This driver will only ever call this with HDC1080_I2C_ADDR (0x40).
	 * @param memAddr	The starting internal memory address to read from.
	 * @param pData		Pointer to memory allocated for saving data read.
	 * @param size		The size of pData, which is the amount of data to read over I2C.
	 * @return HDC1080::Status HDC1080::Status::OK if success, else HDC1080::Status::FAIL_I2C.
	 *
	 * @note This method must be implemented by host application and based on the specific host system infrastructure.
	 *
	 * @note This mehtod is primarily used when reading general information but not measurements (due to measurement
	 * conversion time).
	 *
	 * @note The HDC1080 will auto-increment the address pointer when reading multiple registers in one transaction.
	 * 		 There is no need to manually increment the memory address pointer.
	 * @note ID should be used to determine which physical I2C interface to use if there are multiple HDC1080s.
	 * 		 Else this parameter is unused.
	 */
	static HDC1080::Status I2C_MemRead(uint8_t ID, uint8_t i2cAddr, uint8_t memAddr, uint8_t *pData, uint16_t size);

	/**
	 * @brief Write a set amount of data via the I2C interface, given the device I2C and internal memory addresses.
	 *
	 * @param ID		The user-defined identifier of the HDC1080 class making this call.
	 * @param i2cAddr	The I2C address. This driver will only ever call this with HDC1080_I2C_ADDR (0x40).
	 * @param memAddr	The starting internal memory address to write to.
	 * @param pData		Pointer to data to be written.
	 * @param size		The size of pData, which is the amount of data to written over I2C.
	 * @return HDC1080::Status HDC1080::Status::OK if success, else HDC1080::Status::FAIL_I2C.
	 *
	 * @note This method must be implemented by host application and based on the specific host system infrastructure.
	 *
	 * @note This method is primarily used when writing general information.
	 *
	 * @note The HDC1080 will auto-increment the address pointer when writing multiple registers in one transaction.
	 * 		 There is no need to manually increment the memory address pointer.
	 * @note ID should be used to determine which physical I2C interface to use if there are multiple HDC1080s.
	 * 		 Else this parameter is unused.
	 */
	static HDC1080::Status I2C_MemWrite(uint8_t ID, uint8_t i2cAddr, uint8_t memAddr, uint8_t *pData, uint16_t size);

	/**
	 * @brief Send data onto the I2C bus, given the I2C address of the device to send to.
	 *
	 * @param ID		The user-defined identifier of the HDC1080 class making this call.
	 * @param i2cAddr	The I2C address. This driver will only ever call this with HDC1080_I2C_ADDR (0x40).
	 * @param pData		Pointer to data to be sent.
	 * @param size		Size of pData, which is the amount of data to send.
	 * @return HDC1080::Status HDC1080::Status::OK if success, else HDC1080::Status::FAIL_I2C.
	 *
	 * @note This method must be implemented by host application and based on the specific host system infrastructure.
	 *
	 * @note This methods is primarily used when fetching measurements, which require separation between memory
	 * addressing and data transaction in order to insert a measurement conversion time delay.
	 *
	 * @note The HDC1080 will auto-increment the address pointer when writing multiple registers in one transaction.
	 * 		 There is no need to manually increment the memory address pointer.
	 * @note ID should be used to determine which physical I2C interface to use if there are multiple HDC1080s.
	 * 		 Else this parameter is unused.
	 */
	static HDC1080::Status I2C_Transmit(uint8_t ID, uint8_t i2cAddr, uint8_t *pData, uint16_t size);

	/**
	 * @brief Receive data from the I2C bus, given the I2C address of the device that will send data.
	 *
	 * @param ID		The user-defined identifier of the HDC1080 class making this call.
	 * @param i2cAddr	The I2C address. This driver will only ever call this with HDC1080_I2C_ADDR (0x40).
	 * @param pData		Pointer to memory to save received data to.
	 * @param size		Size of pData, which is the amount of data to receive.
	 * @return HDC1080::Status HDC1080::Status::OK if success, else HDC1080::Status::FAIL_I2C.
	 *
	 * @note This method must be implemented by host application and based on the specific host system infrastructure.
	 *
	 * @note This methods is primarily used when fetching measurements, which require separation between memory
	 * addressing and data transaction in order to insert a measurement conversion time delay.
	 *
	 * @note The HDC1080 will auto-increment the address pointer when writing multiple registers in one transaction.
	 * 		 There is no need to manually increment the memory address pointer.
	 * @note ID should be used to determine which physical I2C interface to use if there are multiple HDC1080s.
	 * 		 Else this parameter is unused.
	 */
	static HDC1080::Status I2C_Receive(uint8_t ID, uint8_t i2cAddr, uint8_t *pData, uint16_t size);

	/**
	 * @brief Delay the program execution by a set amount of time
	 *
	 * @param ms The delay time in milliseconds
	 *
	 * @note This method must be implemented by host application and based on the specific host system infrastructure.
	 *
	 * @note This need not be a precise delay.
	 */
	static inline void Delay(uint16_t ms);
};

/**
 * @brief Heater configuration setting enumerable.
 */
enum class HDC1080::HeaterConfig : uint16_t { OFF = 0x2000u, ON = 0x0000u };

/**
 * @brief Aquisition mode setting enumerable.
 */
enum class HDC1080::AcqModeConfig : uint16_t { SINGLE = 0x0000u, DUAL = 0x0100u };

/**
 * @brief Temperature resolution setting enumerable.
 */
enum class HDC1080::TempResolutionConfig : uint16_t { A_14BIT = 0x0000u, A_11BIT = 0x0040u };

/**
 * @brief Humidity resolution setting enumberable.
 */
enum class HDC1080::HumidityResolutionConfig : uint16_t { A_14BIT = 0x0000u, A_11BIT = 0x0010u, A_8BIT = 0x0020u };

/**
 * @brief Internal method status return type / error code enumerable.
 */
enum class HDC1080::Status : int { OK = 0, FAIL_I2C = -1, FAIL_INVALID_PARAMETER = -2 };

/*** END OF FILE ***/
