/*
 * APDS9960_RGBY_Ball_Ident_Driver.hpp
 *
 *  Created on: May 27, 2023
 *      Author: parch
 */

#ifndef SRC_APDS9960_RGBY_BALL_IDENT_DRIVER_HPP_
#define SRC_APDS9960_RGBY_BALL_IDENT_DRIVER_HPP_

class APDS9960_ColorSense{
private:

	uint8_t proximity[2];
	uint8_t red[2];
	uint8_t green[2];
	uint8_t blue[2];
	I2C_HandleTypeDef i2c;

	//This is the device address. It needs to be bitshift to the right to accommodate the read/write bit that occurs during I2C.
	const uint8_t APDS9960_ADDR = 0x39 << 1;

	//These are what are being written to registers to enable different part of the sensor.
	uint8_t ColorProxEnabled_byte = 0b00000111;     //Color and Proximity are enabled. Sensor is also enabled. Write to Enable Register
	uint8_t ColorEnabled_byte = 0b00000011;         //Color only is enabled. Sensor is also enabled. Write to Enable Register
	uint8_t ColorSensitivity_byte = 0b00000010;          //sensitivity set to x4 for Color Sensitivity. Max value is 3. Write to Control Register 1

	//These are Register Addresses for important Registers we interact with
	const uint8_t APDS9960_ENABLE = 0x80;
	const uint8_t APDS9960_ControlRegister = 0x8F;
	const uint8_t RedColorLSBRegister = 0x96;
	const uint8_t RedColorMSBRegister = 0x97;
	const uint8_t GreenColorLSBRegister = 0x98;
	const uint8_t GreenColorMSBRegister = 0x99;
	const uint8_t BlueColorLSBRegister = 0x9A;
	const uint8_t BlueColorMSBRegister = 0x9B;
	const uint8_t ProximityDataRegister = 0x9C;

public:
	APDS9960_ColorSense(I2C_HandleTypeDef i2c);
	char getColor();





};




#endif /* SRC_APDS9960_RGBY_BALL_IDENT_DRIVER_HPP_ */
