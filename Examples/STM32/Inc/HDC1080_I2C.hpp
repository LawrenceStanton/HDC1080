/**
 ******************************************************************************
 * @file			: HDC1080_I2C.hpp
 * @brief			: I2C interface for the HDC1080.
 * @author			: Lawrence Stanton
 ******************************************************************************
 * @attention
 *
 * © LD Stanton 2023
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

#include "HDC1080.hpp"
#include "stm32g0xx_hal.h" // <-- Change this to the appropriate HAL header for your MCU product line.

class I2C_HDC1080 : public HDC1080::I2C {
	I2C_HandleTypeDef *hi2c;

public:
	virtual std::optional<Register> read(MemoryAddress address) final;
	virtual std::optional<Register> write(MemoryAddress address, Register data) final;
	virtual std::optional<uint8_t>	transmit(uint8_t data) final;
	virtual std::optional<uint8_t>	receive() final;

	virtual void delay(uint32_t ms) final;

	/**
	 * @brief Construct a new I2C object
	 *
	 * @param hi2c The I2C handle to use for communication.
	 * @note The I2C handle must be initialized before passing it to this constructor.
	 */
	I2C_HDC1080(I2C_HandleTypeDef *hi2c) : hi2c(hi2c) {}
};

/*** END OF FILE ***/
