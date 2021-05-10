// Cardiaccs I2C library. Forked from i2cdevlib.

/* ============================================
I2C device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

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

#ifndef BMA355_HPP
#define BMA355_HPP

#include <array>
#include <stdint.h>
#include <utility>
#include <vector>
#include <Arduino.h>

#include "I2C.h"

class BMA355
{

public:

    BMA355();
    BMA355(uint8_t address);

    static constexpr uint8_t MAX_FRAMES = 32;
    static constexpr uint8_t ADDRESS_00 = 0x18;
    static constexpr uint8_t ADDRESS_01 = 0x19;
    static constexpr uint8_t DEFAULT_ADDRESS = ADDRESS_01;

    static constexpr uint8_t RA_CHIP_ID = 0x00;

    static constexpr uint8_t RA_X_AXIS_LSB = 0x02;
    static constexpr uint8_t RA_X_AXIS_MSB = 0x03;
    static constexpr uint8_t RA_Y_AXIS_LSB = 0x04;
    static constexpr uint8_t RA_Y_AXIS_MSB = 0x05;
    static constexpr uint8_t RA_Z_AXIS_LSB = 0x06;
    static constexpr uint8_t RA_Z_AXIS_MSB = 0x07;
    static constexpr uint8_t RA_TEMP_RD = 0x08;

    static constexpr uint8_t RA_INT_STATUS_0 = 0x09;
    static constexpr uint8_t RA_INT_STATUS_1 = 0x0a;
    static constexpr uint8_t RA_INT_STATUS_2 = 0x0b;
    static constexpr uint8_t RA_INT_STATUS_3 = 0x0c;

    static constexpr uint8_t RA_FIFO_STATUS = 0x0e;

    static constexpr uint8_t RA_RANGE = 0x0f;
    static constexpr uint8_t RA_RANGE_BIT = 3;
    static constexpr uint8_t RA_RANGE_LENGTH = 4;

    static constexpr uint8_t RA_BWIDTH = 0x10;
    static constexpr uint8_t RA_BWIDTH_BIT = 4;
    static constexpr uint8_t RA_BWIDTH_LENGTH = 5;

    static constexpr uint8_t RA_PWR_MODE = 0x11;
    static constexpr uint8_t RA_LOW_PWR_CONF = 0x12;
    static constexpr uint8_t RA_ACCD_HBW = 0x13;

    static constexpr uint8_t RA_SOFT_RESET = 0x14;

    static constexpr uint8_t RA_INT_EN_0 = 0x16;
    static constexpr uint8_t RA_INT_EN_1 = 0x17;
    static constexpr uint8_t RA_INT_EN_2 = 0x18;
    static constexpr uint8_t RA_INT_MAP_0 = 0x19;
    static constexpr uint8_t RA_INT_MAP_1 = 0x1a;
    static constexpr uint8_t RA_INT_MAP_2 = 0x1b;
    static constexpr uint8_t RA_INT_SRC = 0x1e;
    static constexpr uint8_t RA_INT_OUT_CTRL = 0x20;
    static constexpr uint8_t RA_INT_RST_LATCH = 0x21;

    static constexpr uint8_t RA_LG_DURATION = 0x22;
    static constexpr uint8_t RA_LG_THRESHOLD = 0x23;
    static constexpr uint8_t RA_HYSTERESIS = 0x24;
    static constexpr uint8_t RA_HG_DURATION = 0x25;
    static constexpr uint8_t RA_HG_THRESHOLD = 0x26;

    static constexpr uint8_t RA_SLOPE_DURATION = 0x27;
    static constexpr uint8_t RA_MOTION_THRS = 0x28;
    static constexpr uint8_t RA_SLONOMO_THRS = 0x29;
    static constexpr uint8_t RA_TAP_DURATION = 0x2a;
    static constexpr uint8_t RA_TAP_THRS = 0x2b;

    static constexpr uint8_t PMU_SELF_TEST = 0x32;
    static constexpr uint8_t PMU_SELF_TEST_AXIS_DISABLED = 0b00;
    static constexpr uint8_t PMU_SELF_TEST_AXIS_X = 0b01;
    static constexpr uint8_t PMU_SELF_TEST_AXIS_Y = 0b10;
    static constexpr uint8_t PMU_SELF_TEST_AXIS_Z = 0b11;
    static constexpr uint8_t PMU_SELF_TEST_SIGN_NEGATIVE = 0b000;
    static constexpr uint8_t PMU_SELF_TEST_SIGN_POSITIVE = 0b100;
    static constexpr uint8_t PMU_SELF_TEST_AMP_LOW = 0b00000;
    static constexpr uint8_t PMU_SELF_TEST_AMP_HIGH = 0b10000;

    static constexpr uint8_t RA_BGW_SPI3_WDT = 0x34;

    static constexpr uint8_t RA_OFC_CTRL = 0x36;
    static constexpr uint8_t RA_OFC_SETTING = 0x37;
    static constexpr uint8_t RA_OFFSET_X = 0x38;
    static constexpr uint8_t RA_OFFSET_Y = 0x39;
    static constexpr uint8_t RA_OFFSET_Z = 0x3a;
    static constexpr uint8_t RA_TRIM_GP1 = 0x3c;
    static constexpr uint8_t RA_FIFO_CONF_1 = 0x3e;
    static constexpr uint8_t RA_FIFO_DATA = 0x3f;

    static constexpr uint8_t X_AXIS_LSB_BIT = 7;
    static constexpr uint8_t X_AXIS_LSB_LENGTH = 4;
    static constexpr uint8_t X_NEW_DATA_BIT = 0;

    static constexpr uint8_t Y_AXIS_LSB_BIT = 7;
    static constexpr uint8_t Y_AXIS_LSB_LENGTH = 4;
    static constexpr uint8_t Y_NEW_DATA_BIT = 0;

    static constexpr uint8_t Z_AXIS_LSB_BIT = 7;
    static constexpr uint8_t Z_AXIS_LSB_LENGTH = 4;
    static constexpr uint8_t Z_NEW_DATA_BIT = 0;

    static constexpr uint8_t FLAT_INT_BIT = 7;
    static constexpr uint8_t ORIENT_INT_BIT = 6;
    static constexpr uint8_t S_TAP_INT_BIT = 5;
    static constexpr uint8_t D_TAP_INT_BIT = 4;
    static constexpr uint8_t SLO_NOT_MOT_INT_BIT = 3;
    static constexpr uint8_t SLOPE_INT_BIT = 2;
    static constexpr uint8_t HIGH_INT_BIT = 1;
    static constexpr uint8_t LOW_INT_BIT = 0;

    static constexpr uint8_t DATA_INT_BIT = 7;
    static constexpr uint8_t FIFO_WM_INT_BIT = 6;
    static constexpr uint8_t FIFO_FULL_INT_BIT = 5;

    static constexpr uint8_t TAP_SIGN_INT_BIT = 7;
    static constexpr uint8_t TAP_FIRST_Z_INT_BIT = 6;
    static constexpr uint8_t TAP_FIRST_Y_INT_BIT = 5;
    static constexpr uint8_t TAP_FIRST_X_INT_BIT = 4;
    static constexpr uint8_t SLOPE_SIGN_INT_BIT = 3;
    static constexpr uint8_t SLOPE_FIRST_Z_INT_BIT = 2;
    static constexpr uint8_t SLOPE_FIRST_Y_INT_BIT = 1;
    static constexpr uint8_t SLOPE_FIRST_X_INT_BIT = 0;

    static constexpr uint8_t ORIENT_INT_LENGTH = 3;
    static constexpr uint8_t HIGH_G_INT_BIT = 3;
    static constexpr uint8_t HIGH_G_INT_LENGTH = 4;

    static constexpr uint8_t FIFO_OVERRUN_BIT = 7;
    static constexpr uint8_t FIFO_FRAME_CNTR_BIT = 6;
    static constexpr uint8_t FIFO_FRAME_CNTR_LEN = 7;

    static constexpr uint8_t DATA_HIGH_BW_BIT = 7;
    static constexpr uint8_t SHADOW_DIS_BIT = 6;

    static constexpr uint8_t FIFO_MODE_BIT = 7;
    static constexpr uint8_t FIFO_MODE_LEN = 2;

    static constexpr uint8_t FIFO_DATA_SEL_BIT = 1;
    static constexpr uint8_t FIFO_DATA_SEL_LEN = 2;

    static constexpr uint8_t WDT_ENABLE_BIT = 1;

    // range and bandwidth
    enum class Range
    {
        r2g = 3,
        r4g = 5,
        r8g = 8,
        r16g = 12,
    };

    //TODO: put some more of these constants into enum class

    //static constexpr uint8_t R_2G = 0b0011;
    //static constexpr uint8_t R_4G = 0b0101;
    //static constexpr uint8_t R_8G = 0b1000;
    //static constexpr uint8_t R_16G = 0b1100;

    static constexpr uint8_t BW_8HZ = 0b01000;
    static constexpr uint8_t BW_15HZ = 0b01001;
    static constexpr uint8_t BW_31HZ = 0b01010;
    static constexpr uint8_t BW_62HZ = 0b01011;
    static constexpr uint8_t BW_125HZ = 0b01100;
    static constexpr uint8_t BW_250HZ = 0b01101;
    static constexpr uint8_t BW_500HZ = 0b01110;
    static constexpr uint8_t BW_1000HZ = 0b01111;

    static constexpr uint8_t SOFT_RESET = 0xb6;

    static constexpr uint8_t FIFO_BYPASS = 0b00;
    static constexpr uint8_t FIFO_FIFO = 0b01;
    static constexpr uint8_t FIFO_STREAM = 0b10;

    static constexpr uint8_t FIFO_SEL_XYZ = 0b00;
    static constexpr uint8_t FIFO_SEL_X = 0b01;
    static constexpr uint8_t FIFO_SEL_Y = 0b10;
    static constexpr uint8_t FIFO_SEL_Z = 0b11;

    static constexpr double SCALE_2G = 0.00098;
    static constexpr double SCALE_4G = 0.00195;
    static constexpr double SCALE_8G = 0.00391;
    static constexpr double SCALE_16G = 0.00781;
/*
    map<int, Range> gMap = {
        { 2, Range::r2g },
        { 4, Range::r4g },
        { 8, Range::r8g },
        { 16, Range::r16g },
    };

    map<int, double> scaleMap = {
        { 2, SCALE_2G },
        { 4, SCALE_4G },
        { 8, SCALE_8G },
        { 16, SCALE_16G },
    };
*/
  
    // Parse raw BMA data. Calculate double value from raw int data.
    double gToScale(int g) const;
    int getScale(double &scale);

    int init();
    int detect();

    int testConnection() const;

    // AXIS registers
    std::tuple<int, std::array<int16_t, 3>> getAccelerationSelfTest() const;
    static int16_t toTwosComplement(uint8_t msb, uint8_t lsb);
    int getAcceleration(uint8_t axis) const;

    // FIFO register
    int getFifoFrameCount() const;
    int getData(std::vector<std::array<int16_t, 3>> &data);
    int getAcc(int16_t *data);

    int getFifoMode() const;
    int setFifoMode(uint8_t mode) const;

    int getFifoDataSelect() const;
    int setFifoDataSelect(uint8_t mode) const;

    int setHighBW(uint8_t bandwidth) const;

    // RANGE / BANDWIDTH registers
    int softReset() const;
    Range gToRange(uint8_t g) const;
    int getRange() const;
    int rangeToG(uint8_t range) const;
    int getG() const;
    int setRange(uint8_t range) const;
    int getBandwidth() const;
    int setBandwidth(uint8_t bandwidth) const;

    int setTestMode(uint8_t data) const;
    int getTestMode() const;

private:
    uint8_t addr;
    std::tuple<int, int16_t> getAccelerationSelfTestAxis(uint8_t axis, uint8_t ax) const;

    static int16_t twosComplement12bitToInt(int16_t res);
    int done();

    static constexpr bool readBacks{ true };
    uint8_t range;

};

#endif // BMA355_HPP
