// Cardiaccs I2C library. Forked from i2cdevlib.

/* ============================================
I2C device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef I2CDEV_HPP
#define I2CDEV_HPP

#include <stdint.h>
#include <utility>
#include <math.h>
#include <Arduino.h>
#include "FT.h"


class I2C
{
public:

    I2C();

    // Terminal debug constant (uncomment to enable)
    static constexpr bool ftDebug = false;
    static constexpr int defaultReadTimeout = 1000; // ms
    static constexpr int defaultWriteTimeout = defaultReadTimeout;
    static constexpr int buffer_length = 32;

    static int write8(uint8_t addr, uint8_t reg, uint8_t length, const uint8_t data[]);
    static int write8(uint8_t addr, uint8_t reg, uint8_t data); 
    static int write16(uint8_t addr, uint8_t reg, uint16_t data); 
    static int read8(uint8_t addr, uint8_t reg, uint8_t &data); 
    static int read8(uint8_t addr, uint8_t reg, uint8_t length, uint8_t data[]); 
    static int read8(uint8_t addr, uint8_t length, uint8_t data[]); 
	  static int read8db(uint8_t addr, uint8_t reg, uint8_t length, uint8_t data[]); 
    static int read16(uint8_t addr, uint8_t reg, uint16_t &data); 
    static int read16(uint8_t addr, uint8_t reg, uint8_t length, uint16_t data[]);

    static int read8Bit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t &data);
    static int read16Bit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint16_t &data);
    static int read8Bits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t &data); 
    static int read16Bits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint16_t& data);
    static int write8Bit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t data); 
    static int write16Bit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint16_t data);
    static int write8Bits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data);
    static int write16Bits(uint8_t addr, uint8_t reg, uint8_t bitStart, uint8_t length, uint16_t data);

    static bool timeover(int timeout, int64_t t1);
    static uint16_t bytesToWord(uint8_t a, uint8_t b);
    static std::pair<uint8_t, uint8_t> wordToBytes(uint16_t data);
    static uint16_t bitsValues(uint16_t val, uint8_t start, uint8_t length);

    // Default timeout value for read operations. Set this to 0 to disable timeout detection.
    static int readTimeout;
    static int writeTimeout;

private:
    static uint16_t bitsValuesWrite(uint16_t val, uint16_t w, uint8_t start, uint8_t length);

};

#endif // I2C_HPP
