/*
 * APDS9960RGBYBallIdentDriver.cpp
 *
 *  Created on: May 27, 2023
 *      Author: parch
 */

#include <stdio.h>
#include "main.h"
#include <cmath>
#include "APDS9960_RGBY_Ball_Ident_Driver.hpp"

APDS9960_ColorSense::APDS9960_ColorSense(I2C_HandleTypeDef i2c){
	this->i2c = i2c;
	HAL_I2C_Mem_Write(&i2c, APDS9960_ADDR, APDS9960_ENABLE, 1, &ColorProxEnabled_byte, 1, 1000); //enable sensor
	HAL_I2C_Mem_Write(&i2c, APDS9960_ADDR, APDS9960_ControlRegister, 1, &ColorSensitivity_byte, 1, 1000); //make sensor more sensitive

}
char APDS9960_ColorSense::getColor(){
	HAL_I2C_Mem_Read(&i2c, APDS9960_ADDR, ProximityDataRegister, 1, proximity, 1, 1000);
	HAL_I2C_Mem_Read(&i2c, APDS9960_ADDR, RedColorLSBRegister, 1, red, 2, 1000);
	HAL_I2C_Mem_Read(&i2c, APDS9960_ADDR, GreenColorLSBRegister, 1, green, 2, 1000);
	HAL_I2C_Mem_Read(&i2c, APDS9960_ADDR, BlueColorLSBRegister, 1, blue, 2, 1000);
		if(proximity[0] > 100) {
			if(blue[0] > red[0] && blue[0] > green[0]) {
				return 'b';
			} else if (red[0] > 4 && abs(red[0] - green[0]) <= 2){
				return 'y';
			} else if (green[0] > red[0]) {
				return 'g';
			} else {
				return 'r';
			}
		}

		return 'n';
}

