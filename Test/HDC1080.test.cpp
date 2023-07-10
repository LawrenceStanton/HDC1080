#include "gmock/gmock.h"
#include "gtest/gtest.h"

#define HDC1080_GTEST_TESTING

#include "HDC1080.hpp"

#include "../Src/HDC1080.cpp" // For static helper functions.

using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Eq;
using ::testing::Return;
using ::testing::ReturnArg;

using MemoryAddress = HDC1080::I2C::MemoryAddress;
using Register		= HDC1080::I2C::Register;

// Tests of Static Functions

TEST(HDC1080_TestStatic, getTemperatureStaticImplementsExpectedEquation) {
	EXPECT_FLOAT_EQ(HDC1080::getTemperature(0x0000u), -40.0);
	EXPECT_FLOAT_EQ(HDC1080::getTemperature(0x1234u), -28.26751);
	EXPECT_FLOAT_EQ(HDC1080::getTemperature(0xFFFFu), 124.99748);
}

TEST(HDC1080_TestStatic, getHumidityStaticImplementsExpectedEquation) {
	EXPECT_FLOAT_EQ(HDC1080::getHumidity(0x0000u), 0.0);
	EXPECT_FLOAT_EQ(HDC1080::getHumidity(0x1234u), 7.1105957);
	EXPECT_FLOAT_EQ(HDC1080::getHumidity(0xFFFFu), 99.99847);
}

TEST(HDC1080_TestStatic, constructConfigRegisterRandomChecks) {
	EXPECT_EQ(
		constructConfigRegister(Config{
			HDC1080::AcquisitionMode::DUAL,
			HDC1080::TemperatureResolution::A_11BIT,
			HDC1080::HumidityResolution::A_11BIT,
			HDC1080::Heater::ON}),
		0b0011'0101'0000'0000u
	);
	EXPECT_EQ(
		constructConfigRegister(Config{
			HDC1080::AcquisitionMode::SINGLE,
			HDC1080::TemperatureResolution::A_14BIT,
			HDC1080::HumidityResolution::A_14BIT,
			HDC1080::Heater::OFF}),
		0b0000'0000'0000'0000u
	);
	EXPECT_EQ(
		constructConfigRegister(Config{
			HDC1080::AcquisitionMode::SINGLE,
			HDC1080::TemperatureResolution::A_14BIT,
			HDC1080::HumidityResolution::A_8BIT,
			HDC1080::Heater::ON}),
		0b0010'0010'0000'0000u
	);
}

TEST(HDC1080_TestStatic, decodeConfigRegisterRandomChecks) {
	EXPECT_EQ(
		decodeConfigRegister(0b0011'0101'0000'0000u),
		(Config{
			HDC1080::AcquisitionMode::DUAL,
			HDC1080::TemperatureResolution::A_11BIT,
			HDC1080::HumidityResolution::A_11BIT,
			HDC1080::Heater::ON})
	);
	EXPECT_EQ(
		decodeConfigRegister(0b0000'0000'0000'0000u),
		(Config{
			HDC1080::AcquisitionMode::SINGLE,
			HDC1080::TemperatureResolution::A_14BIT,
			HDC1080::HumidityResolution::A_14BIT,
			HDC1080::Heater::OFF})
	);
	EXPECT_EQ(
		decodeConfigRegister(0b0010'0010'0000'0000u),
		(Config{
			HDC1080::AcquisitionMode::SINGLE,
			HDC1080::TemperatureResolution::A_14BIT,
			HDC1080::HumidityResolution::A_8BIT,
			HDC1080::Heater::ON})
	);
}

class MockedI2C : public HDC1080::I2C {
public:
	MOCK_METHOD(std::optional<Register>, read, (MemoryAddress addr), (final));
	MOCK_METHOD(std::optional<Register>, write, (MemoryAddress addr, Register data), (final));
	MOCK_METHOD(std::optional<uint8_t>, transmit, (uint8_t data), (final));
	MOCK_METHOD(std::optional<uint8_t>, receive, (), (final));

	void delay(uint32_t ms) final {}
};

class HDC1080_Test : public ::testing::Test {
public:
	MockedI2C i2c{};
	HDC1080	  hdc1080{&this->i2c};

	inline void disableI2C() {
		EXPECT_CALL(this->i2c, transmit(_)).WillRepeatedly(Return(std::nullopt));
		EXPECT_CALL(this->i2c, receive()).WillRepeatedly(Return(std::nullopt));
	}
};

TEST_F(HDC1080_Test, constructorAssignsI2cInterfacePointer) {
	ASSERT_EQ(this->hdc1080.i2c, &this->i2c);
}

TEST_F(HDC1080_Test, getTemperatureRegisterNormallyReturnsValue) {
	EXPECT_CALL(this->i2c, transmit(_)).WillRepeatedly(ReturnArg<0>());
	EXPECT_CALL(this->i2c, receive()).WillOnce(Return(0x12u)).WillOnce(Return(0x34u));

	EXPECT_EQ(this->hdc1080.getTemperatureRegister().value(), 0x1234u);
}

TEST_F(HDC1080_Test, getTemperatureReturnsMinusFortyWhenGetTemperatureRegisterFails) {
	disableI2C();

	EXPECT_FLOAT_EQ(this->hdc1080.getTemperature(), -40.0);
}

TEST_F(HDC1080_Test, getTemperatureRegisterReturnsEmptyOptionalWhenI2CFails) {
	disableI2C();

	EXPECT_EQ(this->hdc1080.getTemperatureRegister(), std::nullopt);
}

TEST_F(HDC1080_Test, getTemperatureReturnsUpdatedValue) {
	EXPECT_CALL(this->i2c, transmit(_)).WillRepeatedly(ReturnArg<0>());
	EXPECT_CALL(this->i2c, receive()).WillOnce(Return(0xABu)).WillOnce(Return(0xCDu));

	EXPECT_FLOAT_EQ(this->hdc1080.getTemperature(), HDC1080::getTemperature(0xABCDu));
}

TEST_F(HDC1080_Test, getHumidityRegisterNormallyReturnsValue) {
	EXPECT_CALL(this->i2c, transmit(_)).WillRepeatedly(ReturnArg<0>());
	EXPECT_CALL(this->i2c, receive()).WillOnce(Return(0x12u)).WillOnce(Return(0x34u));

	EXPECT_EQ(this->hdc1080.getHumidityRegister().value(), 0x1234u);
}

TEST_F(HDC1080_Test, getHumidityReturnsZeroWhenI2CReceiveFails) {
	disableI2C();

	EXPECT_FLOAT_EQ(this->hdc1080.getHumidity(), 0.0);
}

TEST_F(HDC1080_Test, getHumidityReturnsUpdatedValue) {
	EXPECT_CALL(this->i2c, transmit(_)).WillRepeatedly(ReturnArg<0>());
	EXPECT_CALL(this->i2c, receive()).WillOnce(Return(0xABu)).WillOnce(Return(0xCDu));

	EXPECT_FLOAT_EQ(this->hdc1080.getHumidity(), HDC1080::getHumidity(0xABCDu));
}

TEST_F(HDC1080_Test, getHumidityRegisterReturnsEmptyOptionalWhenI2CReceiveFails) {
	disableI2C();

	EXPECT_EQ(this->hdc1080.getHumidityRegister(), std::nullopt);
}

TEST_F(HDC1080_Test, getMeasurementRegisterNormallyReturnsValue) {
	EXPECT_CALL(this->i2c, transmit(_)).WillRepeatedly(ReturnArg<0>());
	EXPECT_CALL(this->i2c, receive()).WillOnce(Return(0xABu)).WillOnce(Return(0xCDu));

	EXPECT_EQ(this->hdc1080.getMeasurementRegister(0x00u).value(), 0xABCDu);
}

TEST_F(HDC1080_Test, getMeasurementRegisterReturnsEmptyOptionalWhenI2CReceiveFails) {
	disableI2C();
	EXPECT_EQ(this->hdc1080.getMeasurementRegister(0x00u), std::nullopt);
}

TEST_F(HDC1080_Test, getDeviceIDNormallyReturnsValue) {
	const MemoryAddress deviceIDRegister = 0xFFu;
	const Register		deviceID		 = 0x1050u;
	EXPECT_CALL(this->i2c, read(Eq(deviceIDRegister))).WillOnce(Return(deviceID));
	EXPECT_EQ(this->hdc1080.getDeviceID().value(), deviceID);
}

TEST_F(HDC1080_Test, getDeviceIDReturnsEmptyOptionalWhenI2CReadFails) {
	disableI2C();

	EXPECT_EQ(this->hdc1080.getDeviceID(), std::nullopt);
}

TEST_F(HDC1080_Test, getManufacturerIDNormallyReturnsValue) {
	const MemoryAddress manufacturerIDRegister = 0xFEu;
	const Register		manufacturerID		   = 0x5449u;
	EXPECT_CALL(this->i2c, read(Eq(manufacturerIDRegister))).WillOnce(Return(manufacturerID));
	EXPECT_EQ(this->hdc1080.getManufacturerID().value(), manufacturerID);
}

TEST_F(HDC1080_Test, getManufacturerIDReturnsEmptyOptionalWhenI2CReadFails) {
	disableI2C();

	EXPECT_EQ(this->hdc1080.getManufacturerID(), std::nullopt);
}

TEST_F(HDC1080_Test, getSerialIDNormallyReturnsValue) {
	const MemoryAddress serialIDRegisters[] = {0xFBu, 0xFCu, 0xFDu};
	const Register		serialIDValues[]	= {0xFFFFu, 0xFFFFu, 0xFF80u}; // 1 in every bit that 41-bit Serial ID has
	// Note: In reality the serial ID can be any 41-bit value, but we're only testing that the bits are read correctly.

	for (uint8_t i = 0; i < 3; i++) {
		EXPECT_CALL(this->i2c, read(Eq(serialIDRegisters[i]))).WillOnce(Return(serialIDValues[i]));
	}

	EXPECT_EQ(this->hdc1080.getSerialID().value(), 0x1FFFFFFFFFFu);
}

TEST_F(HDC1080_Test, getSerialIDReturnsEmptyOptionalWhenI2CReadFails) {
	disableI2C();

	EXPECT_EQ(this->hdc1080.getSerialID(), std::nullopt);
}

TEST_F(HDC1080_Test, getBatteryStatusNormallyReturnsValue) {
	const MemoryAddress configRegister		   = 0x02u;
	const Register		configValueBatteryHigh = 0x1000u;
	const Register		configValueBatteryLow  = 0x1800u;

	EXPECT_CALL(this->i2c, read(Eq(configRegister)))
		.WillOnce(Return(configValueBatteryHigh))
		.WillOnce(Return(configValueBatteryLow));

	EXPECT_EQ(this->hdc1080.getBatteryStatus(), HDC1080::Battery::HIGH);
	EXPECT_EQ(this->hdc1080.getBatteryStatus(), HDC1080::Battery::LOW);
}

TEST_F(HDC1080_Test, getBatteryStatusReturnsEmptyOptionalWhenI2CReadFails) {
	disableI2C();
	EXPECT_EQ(this->hdc1080.getBatteryStatus(), std::nullopt);
}

TEST_F(HDC1080_Test, setConfigNormallyWritesValueWithoutReadingIfGivenAllArguments) {
	// Note: Constructing the register value to write is tested in the constructConfigRegisterRandomChecks test.

	const MemoryAddress configRegister			   = 0x02u;
	const Register		configInitialValue		   = 0x1000u;
	const Register		configExpectedWrittenValue = 0b0011'0101'0000'0000u;

	EXPECT_CALL(this->i2c, read(_)).Times(0); // No reads should be performed
	EXPECT_CALL(this->i2c, write(Eq(configRegister), _)).Times(1);

	auto write = this->hdc1080.setConfig(
		HDC1080::AcquisitionMode::DUAL,
		HDC1080::TemperatureResolution::A_11BIT,
		HDC1080::HumidityResolution::A_11BIT,
		HDC1080::Heater::ON
	);
	EXPECT_EQ(write.value(), configExpectedWrittenValue);
}

TEST_F(HDC1080_Test, setConfigReturnsEmptyOptionalWhenGivenNoArguments) {
	EXPECT_EQ(this->hdc1080.setConfig({}, {}, {}, {}), std::nullopt);
}

TEST_F(HDC1080_Test, setConfigNormallyReadsCurrentConfigAndWritesBackNewValues) {
	const MemoryAddress configRegister			   = 0x02u;
	const Register		configInitialValue		   = 0x1100u; // Humidity 11-bit
	const Register		configExpectedWrittenValue = 0b0000'0101'0000'0000u;

	EXPECT_CALL(this->i2c, read(Eq(configRegister))).WillOnce(Return(configInitialValue));
	EXPECT_CALL(this->i2c, write(Eq(configRegister), _)).Times(1);

	auto write = this->hdc1080.setConfig(
		{}, //
		HDC1080::TemperatureResolution::A_11BIT,
		{},
		{}
	);
	EXPECT_EQ(write.value(), configExpectedWrittenValue);
}

TEST_F(HDC1080_Test, setConfigShortCircuitsWhenConfigUnchangedAndReturnsCurrentConfigRegisterValue) {
	const MemoryAddress configRegister	   = 0x02u;
	const Register		configInitialValue = 0x3500u; // Humidity 11-bit, Temperature 11-bit, Heater ON

	EXPECT_CALL(this->i2c, read(Eq(configRegister))).WillOnce(Return(configInitialValue));
	EXPECT_CALL(this->i2c, write).Times(0); // Check that function short circuits without writing.

	auto write = this->hdc1080.setConfig(
		{}, //
		HDC1080::TemperatureResolution::A_11BIT,
		HDC1080::HumidityResolution::A_11BIT,
		HDC1080::Heater::ON
	);
	EXPECT_EQ(write.value(), configInitialValue);
}

TEST_F(HDC1080_Test, setConfigReturnsEmptyOptionalWhenI2CWriteFails) {
	disableI2C();

	EXPECT_EQ(
		this->hdc1080.setConfig(
			HDC1080::AcquisitionMode::DUAL,
			HDC1080::TemperatureResolution::A_11BIT,
			HDC1080::HumidityResolution::A_11BIT,
			HDC1080::Heater::OFF
		),
		std::nullopt
	);
}

TEST_F(HDC1080_Test, setAcquisitionModeNormallySetsValue) {
	// Mock changing the config value to SINGLE mode and then back to DUAL mode.
	const Register configInitialValueDualAcquisitionMode   = 0x1000u;
	const Register configInitialValueSingleAcquisitionMode = 0x0000u;

	EXPECT_CALL(this->i2c, read)
		.WillOnce(Return(configInitialValueDualAcquisitionMode))
		.WillOnce(Return(configInitialValueSingleAcquisitionMode));
	EXPECT_CALL(this->i2c, write).WillRepeatedly(ReturnArg<1>());

	EXPECT_EQ(
		this->hdc1080.setAcquisitionMode(HDC1080::AcquisitionMode::SINGLE).value(),
		configInitialValueSingleAcquisitionMode
	);

	EXPECT_EQ(
		this->hdc1080.setAcquisitionMode(HDC1080::AcquisitionMode::DUAL).value(),
		configInitialValueDualAcquisitionMode
	);
}

TEST_F(HDC1080_Test, setAcquisitionModeReturnsEmptyOptionalWhenI2CWriteFails) {
	disableI2C();
	EXPECT_EQ(this->hdc1080.setAcquisitionMode(HDC1080::AcquisitionMode::DUAL), std::nullopt);
}

TEST_F(HDC1080_Test, setTemperatureResolutionNormallySetsValue) {
	// Mock changing the config value to 14-bit resolution and then back to 11-bit resolution.
	const Register configInitialValue14BitResolution = 0x1000u;
	const Register configInitialValue11BitResolution = 0x1400u;

	EXPECT_CALL(this->i2c, read)
		.WillOnce(Return(configInitialValue11BitResolution))
		.WillOnce(Return(configInitialValue14BitResolution));
	EXPECT_CALL(this->i2c, write).WillRepeatedly(ReturnArg<1>());

	EXPECT_EQ(
		this->hdc1080.setTemperatureResolution(HDC1080::TemperatureResolution::A_11BIT).value(),
		configInitialValue11BitResolution
	);

	EXPECT_EQ(
		this->hdc1080.setTemperatureResolution(HDC1080::TemperatureResolution::A_14BIT).value(),
		configInitialValue14BitResolution
	);
}

TEST_F(HDC1080_Test, setTemperatureResolutionReturnsEmptyOptionalWhenI2CWriteFails) {
	disableI2C();
	EXPECT_EQ(this->hdc1080.setTemperatureResolution(HDC1080::TemperatureResolution::A_11BIT), std::nullopt);
}

TEST_F(HDC1080_Test, setHumidityResolutionNormallySetsValue) {
	// Mock changing the config value to 11-bit resolution, then 8-bit resolution, then 14-bit resolution.
	const Register configInitialValue14BitResolution = 0x1000u;
	const Register configInitialValue11BitResolution = 0x1100u;
	const Register configInitialValue8BitResolution	 = 0x1200u;

	EXPECT_CALL(this->i2c, read)
		.WillOnce(Return(configInitialValue11BitResolution))
		.WillOnce(Return(configInitialValue8BitResolution))
		.WillOnce(Return(configInitialValue14BitResolution));
	EXPECT_CALL(this->i2c, write).WillRepeatedly(ReturnArg<1>());

	EXPECT_EQ(
		this->hdc1080.setHumidityResolution(HDC1080::HumidityResolution::A_11BIT).value(),
		configInitialValue11BitResolution
	);
	EXPECT_EQ(
		this->hdc1080.setHumidityResolution(HDC1080::HumidityResolution::A_8BIT).value(),
		configInitialValue8BitResolution
	);
	EXPECT_EQ(
		this->hdc1080.setHumidityResolution(HDC1080::HumidityResolution::A_14BIT).value(),
		configInitialValue14BitResolution
	);
}

TEST_F(HDC1080_Test, setHumidityResolutionReturnsEmptyOptionalWhenI2CWriteFails) {
	disableI2C();
	EXPECT_EQ(this->hdc1080.setHumidityResolution(HDC1080::HumidityResolution::A_11BIT), std::nullopt);
}

TEST_F(HDC1080_Test, setHeaterNormallySetsValue) {
	// Mock changing the config value to ON mode and then back to OFF mode.
	const Register configInitialValueHeaterOn  = 0x3000u;
	const Register configInitialValueHeaterOff = 0x1000u;

	EXPECT_CALL(this->i2c, read)
		.WillOnce(Return(configInitialValueHeaterOn))
		.WillOnce(Return(configInitialValueHeaterOff));
	EXPECT_CALL(this->i2c, write).WillRepeatedly(ReturnArg<1>());

	EXPECT_EQ(this->hdc1080.setHeater(HDC1080::Heater::ON).value(), configInitialValueHeaterOn);
	EXPECT_EQ(this->hdc1080.setHeater(HDC1080::Heater::OFF).value(), configInitialValueHeaterOff);
}

TEST_F(HDC1080_Test, setHeaterReturnsEmptyOptionalWhenI2CWriteFails) {
	disableI2C();
	EXPECT_EQ(this->hdc1080.setHeater(HDC1080::Heater::ON), std::nullopt);
}
