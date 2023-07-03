#include "gmock/gmock.h"
#include "gtest/gtest.h"

#define HDC1080_GTEST_TESTING

#include "HDC1080.hpp"

using ::testing::_;
using ::testing::AnyNumber;
using ::testing::Eq;
using ::testing::Return;

using MemoryAddress = HDC1080::I2C::MemoryAddress;
using Register		= HDC1080::I2C::Register;
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
	HDC1080	  hdc1080{dynamic_cast<HDC1080::I2C &>(this->i2c)};

	inline void disableI2C() {
		EXPECT_CALL(this->i2c, transmit(_)).WillRepeatedly(Return(std::nullopt));
		EXPECT_CALL(this->i2c, receive()).WillRepeatedly(Return(std::nullopt));
	}
};

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

TEST_F(HDC1080_Test, getTemperatureRegisterNormallyReturnsValue) {
	EXPECT_CALL(this->i2c, transmit(_)).Times(AnyNumber()).WillRepeatedly(Return(0x00u));
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
	EXPECT_CALL(this->i2c, transmit(_)).Times(AnyNumber()).WillRepeatedly(Return(0x00));
	EXPECT_CALL(this->i2c, receive()).WillOnce(Return(0xABu)).WillOnce(Return(0xCDu));

	EXPECT_FLOAT_EQ(this->hdc1080.getTemperature(), HDC1080::getTemperature(0xABCDu));
}

TEST_F(HDC1080_Test, getHumidityRegisterNormallyReturnsValue) {
	EXPECT_EQ(this->hdc1080.getHumidityRegister().value(), 0x5656u);
}

TEST_F(HDC1080_Test, getHumidityReturnsZeroWhenI2CReceiveFails) {
	disableI2C();
	EXPECT_FLOAT_EQ(this->hdc1080.getHumidity(), 0.0);
}

TEST_F(HDC1080_Test, getHumidityReturnsUpdatedValue) {
	EXPECT_CALL(this->i2c, transmit(_)).WillRepeatedly(Return(0x00));
	EXPECT_CALL(this->i2c, receive()).WillOnce(Return(0xABu)).WillOnce(Return(0xCDu));
	EXPECT_FLOAT_EQ(this->hdc1080.getHumidity(), HDC1080::getHumidity(0xABCDu));
}

TEST_F(HDC1080_Test, getHumidityRegisterReturnsEmptyOptionalWhenI2CReceiveFails) {
	disableI2C();

	EXPECT_EQ(this->hdc1080.getHumidityRegister(), std::nullopt);
}

TEST_F(HDC1080_Test, getMeasurementRegisterNormallyReturnsValue) {
	EXPECT_CALL(this->i2c, transmit(_)).WillRepeatedly(Return(0x00));
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
