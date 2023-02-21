/*
 * fserial.h
 *
 *  Created on: Mar 26, 2021
 *      Author: Administrator
 */

#ifndef INC_FSERIAL_H_
#define INC_FSERIAL_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#elif defined(bcm2835)
#include "source/rpi_bcm2835.h"
#include "source/bcm2835_spi.h"
#include "source/bcm2835_stream.h"
#endif

#include "arduio.h"
#include "SERIAL_SWITCH.h"
#include "TMC_platforms.h"

class Stream{
public:
	Stream() {}
	~Stream() {}

public:
	virtual void active() = 0;
	virtual size_t write(const uint8_t data) = 0;
	virtual int16_t read() = 0;
	virtual int available() = 0;
	virtual void delay(uint8_t mscnt) = 0;
	virtual uint32_t millis() = 0;
};

class SW_SPIClass {
	public:
		SW_SPIClass(uint16_t sw_mosi_pin, uint16_t sw_miso_pin, uint16_t sw_sck_pin);
		void init();
		void begin() {};
		void switchCSpin(bool state);
		uint8_t transfer(uint8_t ulVal);
		uint16_t transfer16(uint16_t data);
		void endTransaction() {};
	private:
		const uint16_t	mosi_pin,
						miso_pin,
						sck_pin;

		#if defined(ARDUINO_ARCH_AVR)
			fastio_bm mosi_bm,
					miso_bm,
					sck_bm;
			fastio_reg mosi_register,
							 miso_register,
							 sck_register;
		#endif
};

#endif /* INC_FSERIAL_H_ */
