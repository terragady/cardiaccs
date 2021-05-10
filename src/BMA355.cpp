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

#include "BMA355.h"
#include "FT.h"

#include <bitset>

/** Default constructor, uses default I2C address.
 * @see BMA150_DEFAULT_ADDRESS
 */
BMA355::BMA355() {
    addr = DEFAULT_ADDRESS;
    range = 8;
}

/** Specific address constructor.
 * @param address I2C address
 * @see BMA150_DEFAULT_ADDRESS
 * @see BMA150_ADDRESS_00
 */
BMA355::BMA355(uint8_t address) {
    addr = address;
}

/*
double BMA355::gToScale(int g) const
{
    if (!scaleMap.count(g))
        return 0;

    return scaleMap[g];
}

int BMA355::getScale(double &scale)
{
    const int g = getG();
    if (g <= 0)
        return g;
    scale = gToScale(g);
    return scale >= 0 ? 1 : int(FT::Status::accScaleError);
}
*/

/** Power on and prepare for general usage. This sets the full scale range of the sensor, as well as the bandwidth
 */
int BMA355::init()
{
    int res;
    std::array<int16_t, 3> v{ };
    std::vector<std::array<int16_t, 3>> data;

    //Serial.print("Initializing BMA355...");

    if ((res = softReset()) < 0)
        Serial.println( "Error: BMA355 soft reset failed");
    else if ((res = testConnection()) < 0)
        Serial.println("Error: BMA355 test connection failed");
    else if ((res = setRange(range)) < 0) // default is pm 8 G
        Serial.println( "Error: BMA355 set range failed");
    //else if ((res = setBandwidth(BMA355::BW_500HZ)) < 0) // set to max if filter is desired
    //    Serial.println( "Error: BMA355 set bandwith failed");
    else if ((res = setHighBW(1)) < 0) // unfiltered data
        Serial.println( "Error: BMA355 set  high BW failed");
    else if ((res = setFifoMode(BMA355::FIFO_BYPASS)) < 0) // use bypass 
        {Serial.print( "Error: BMA355 set FIFO mode failed. Error code: "); Serial.println(res);}
    else if ((res = setFifoDataSelect(BMA355::FIFO_SEL_XYZ)) < 0)
        {Serial.print( "Error: BMA355 set FIFO data select failed. Error code: "); Serial.println(res);}
    //else if (std::tie(res, v) = getAccelerationSelfTest(); res < 0)
    //    Serial.println( "BMA355 error during self test");
    //else if ((res = setTestMode(BMA355::PMU_SELF_TEST_AXIS_DISABLED)) < 0)
    //   Serial.println( "BMA355 reset self test failed");
    else if ((res = getData(data)) < 0)
        Serial.println( "Error: BMA355 read fifo failed");
    else
    {
		//Serial.println("Done");
        return 1;
    }

    return res;
}

int BMA355::detect()
{
    uint8_t data = 0;

    if (const int res = I2C::read8(addr, BMA355::RA_CHIP_ID, data); res != 1)
        return res;

    if (data != 0b11101010) // 234 
        return int(FT::Status::invalidDeviceId);

    return data;
}

// Verify the I2C connection. Make sure the device is connected and responds as expected.
int BMA355::testConnection() const
{
    int res;

    uint8_t original = 0;
    if ((res = I2C::read8(addr, BMA355::RA_LG_THRESHOLD, original)) != 1)
        return res;

    const uint8_t data = 0x34;
    uint8_t test = UINT8_MAX; // qrand() % UINT8_MAX;

    if ((res = I2C::write8(addr, BMA355::RA_LG_THRESHOLD, data)) != 1)
        return res;

    if ((res = I2C::read8(addr, BMA355::RA_LG_THRESHOLD, test)) != 1)
        return res;

    if ((res = I2C::write8(addr, BMA355::RA_LG_THRESHOLD, original)) != 1)
        return res;

    if (test != data)
        res = int(FT::Status::testConnectionFailed);

    return res;
}

/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see BMA355::RA_X_AXIS_LSB
 */
int BMA355::getAcceleration(uint8_t axis) const
{
    std::array<uint8_t, 2> data{ };

    if (const int res = I2C::read8(addr, axis, uint8_t(data.size()), data.data()); res < 0)
        return res;
    const int r = twosComplement12bitToInt(toTwosComplement(data.at(1), data.at(0)));
    return r;
}

int BMA355::getFifoFrameCount() const
{
    uint8_t data = 0;

    if (const int res = I2C::read8Bits(addr, BMA355::RA_FIFO_STATUS, BMA355::FIFO_FRAME_CNTR_BIT, BMA355::FIFO_FRAME_CNTR_LEN, data); res != 1)
        return res;
    return data;
}

int BMA355::getAcc(int16_t *data)
{

    int n = getFifoFrameCount();
    if (n < 0) return n;

    //wait until at least one frame is available with timeout 
    const int32_t t = millis();
    while (n < 1)
    {
        n = getFifoFrameCount();
        if (n < 0)
            return n;
        const int32_t dt = millis() - t;
        if (dt > 1000)
            return int(FT::Status::timeout);
    }
    
    const uint8_t fifoBytesToRead = 3 * 2;
    //std::vector<uint8_t> buf(fifoBytesToRead, 0);
    uint8_t buf[6];

    const int bytesRead = I2C::read8(addr, BMA355::RA_FIFO_DATA, fifoBytesToRead, buf);

    if (bytesRead != fifoBytesToRead)
        return int(FT::Status::fifoFailure);

    for (int i = 0; i < 3; ++i)
        data[i] = twosComplement12bitToInt(toTwosComplement(buf[(2 * i) + 1], buf[(2 * i)]));

    return 0;
}

int BMA355::getData(std::vector<std::array<int16_t, 3>> &data)
// this function is too slow for some reason
{
    int n = getFifoFrameCount();
    if (n < 0) return n;

    //wait until at least one frame is available with timeout 
    const int32_t t = millis();
    while (n < 1)
    {
        n = getFifoFrameCount();
        if (n < 0)
            return n;
        const int32_t dt = millis() - t;
        if (dt > 1000)
            return int(FT::Status::timeout);
    }

    // Check that the framecount does not exceed max frames,
    // which will happen if this byte is set to 0xff in case of
    // sensor disconnection
    if (n > MAX_FRAMES)
    {
        //            qInfo().noquote() << "Frame count exceeds 32:" << QString::number(n);
        return int(FT::Status::tooManyFrames);
    }

    const uint8_t fifoBytesToRead = n * 3 * 2;
    std::vector<uint8_t> buf(fifoBytesToRead, 0);

    const int bytesRead = I2C::read8(addr, BMA355::RA_FIFO_DATA, fifoBytesToRead, buf.data());

    if (bytesRead != fifoBytesToRead)
        return int(FT::Status::fifoFailure);

    data.resize(n);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < 3; ++i)
            data[j][i] = twosComplement12bitToInt(toTwosComplement(buf.at((3 * j + i) * 2 + 1), buf.at((3 * j + i) * 2)));

    return n;
}


/** Get FIFO mode
 * @return Current FIFO mode setting
 * 00b = BYPASS
 * 01b = FIFO
 * 10b = STREAM
 * @see BMA355::RA_FIFO_CONF_1
 * @see BMA355::FIFO_MODE_BIT
 */
int BMA355::getFifoMode() const
{
    uint8_t data = 0;

    if (const int res = I2C::read8Bits(addr, BMA355::RA_FIFO_CONF_1, BMA355::FIFO_MODE_BIT, BMA355::FIFO_MODE_LEN, data); res != 1)
        return res;
    return data;
}

int BMA355::setFifoMode(uint8_t mode) const
{
    int res;
    if ((res = I2C::write8Bits(addr, BMA355::RA_FIFO_CONF_1, BMA355::FIFO_MODE_BIT, BMA355::FIFO_MODE_LEN, mode)) < 0)
        return res;

    if (readBacks && (res = getFifoMode()) != mode)
        res = int(FT::Status::writeToDeviceRegisterNotAcknowledged);
    return res;
}

/** Get FIFO data select mode
 * 00b = X+Y+Z
 * 01b = X only
 * 10b = Y only
 * 11b = Z only
 */
int BMA355::getFifoDataSelect() const
{
    uint8_t data = 0;

    int res;
    if ((res = I2C::read8Bits(addr, BMA355::RA_FIFO_CONF_1, BMA355::FIFO_DATA_SEL_BIT, BMA355::FIFO_DATA_SEL_LEN, data)) != 1)
        return res;
    return data;
}

int BMA355::setFifoDataSelect(uint8_t mode) const
{
    int res = I2C::write8Bits(addr, BMA355::RA_FIFO_CONF_1, BMA355::FIFO_DATA_SEL_BIT, BMA355::FIFO_DATA_SEL_LEN, mode);
    if (res <= 0)
        return res;

    if (readBacks && (res = getFifoDataSelect()) != mode)
        res = int(FT::Status::writeToDeviceRegisterNotAcknowledged);

    return res;
}


// RANGE / BANDWIDTH registers

/** Get Sensor Full Range
 * @return Current Sensor Full Scale Range
 * 3  = +/- 2G
 * 5  = +/- 4G
 * 8  = +/- 8G
 * 12 = +/- 16G
 * @see BMA355::RA_RANGE_BWIDTH
 * @see BMA355::RA_RANGE_BIT
 * @see BMA355::RA_RANGE_LENGTH
 */
int BMA355::getRange() const
{
    uint8_t data = 0;

    if (const int res = I2C::read8Bits(addr, BMA355::RA_RANGE, RA_RANGE_BIT, RA_RANGE_LENGTH, data); res < 0)
        return res;
    return data;
}
/*
int BMA355::rangeToG(uint8_t _range) const
{
    foreach(const int key, gMap.keys())
        if (gMap[key] == Range(_range))
            return key;

    return int(FT::Status::accScaleError);
}

int BMA355::getG() const
{
    const int res = getRange();
    if (res < 0)
        return res;
    return rangeToG(res);
}
*/

int BMA355::softReset() const
{
    const int res = I2C::write8(addr, BMA355::RA_SOFT_RESET, 0xB6);
    delay(2); // Wake-Up Time 1, see BMA355 datasheet
    return res;
}

/** Set Sensor Full Range
* @param g New full-scale range value
* @see getRange()
* @see BMA355::RA_RANGE_BWIDTH
* @see BMA355::RANGE_BIT
* @see BMA355::RANGE_LENGTH
*/
/*
BMA355::Range BMA355::gToRange(uint8_t g) const
{
    return gMap.count(g) ? gMap[g] : Range(FT::Status::accScaleError);
}
*/
int BMA355::setRange(uint8_t _range) const
{
    //_range = uint8_t(gToRange(_range));
    int res = I2C::write8(addr, BMA355::RA_RANGE, _range);
    if (res < 0)
        return res;
    if (readBacks && (res = getRange()) != _range)
        res = int(FT::Status::writeToDeviceRegisterNotAcknowledged);
    return res;
}


/** Get digital filter bandwidth.
 * The bandwidth parameter is used to setup the digital filtering of ADC output data to obtain
 * the desired bandwidth.
 * @return Current Sensor Bandwidth
 * 0 = 25Hz
 * 1 = 50Hz
 * 2 = 100Hz
 * 3 = 190Hz
 * 4 = 375Hz
 * 5 = 750Hz
 * 6 = 1500Hz
 * @see BMA355::RA_BWIDTH
 * @see BMA355::RA_BWIDTH_BIT
 * @see BMA355::RA_BWIDTH_LENGTH
 */
int BMA355::getBandwidth() const
{
    uint8_t data = 0;

    if (const int res = I2C::read8Bits(addr, BMA355::RA_BWIDTH, RA_BWIDTH_BIT, RA_BWIDTH_LENGTH, data); res != 1)
        return res;

    return data;
}

/** Set Sensor Full Range
 * @param bandwidth New bandwidth value
 * @see getBandwidth()
 * @see BMA355::RA_RANGE_BWIDTH
 * @see BMA355::RANGE_BIT
 * @see BMA355::RANGE_LENGTH
 */
int BMA355::setBandwidth(uint8_t bandwidth) const
{
    int res;
    if ((res = I2C::write8(addr, BMA355::RA_BWIDTH, bandwidth)) < 0)
        return res;
    if (readBacks && (res = getBandwidth()) != bandwidth)
        res = int(FT::Status::writeToDeviceRegisterNotAcknowledged);
    return res;
}

/** Set Sensor High bandwidth
 * @param bandwidth 1 or 0
 * @see BMA355::RA_ACCD_HBW
 * @see BMA355::RANGE_BIT
 * @see BMA355::RANGE_LENGTH
 */
int BMA355::setHighBW(uint8_t bandwidth) const
{
    int res;
    if ((res = I2C::write8Bit(addr, BMA355::RA_ACCD_HBW, 7, bandwidth)) < 0)
        return res;
    //if (readBacks && (res = getBandwidth()) != bandwidth)
    //    res = int(FT::Status::writeToDeviceRegisterNotAcknowledged);
    return res;
}



int BMA355::setTestMode(uint8_t data) const
{
    int res;
    if ((res = I2C::write8(addr, BMA355::PMU_SELF_TEST, data) != 1))
        return res;
    if (readBacks && (res = getTestMode()) != data)
        res = int(FT::Status::writeToDeviceRegisterNotAcknowledged);
    return res;
}

int BMA355::getTestMode() const
{
    uint8_t data = 0;
    if (const int res = I2C::read8(addr, BMA355::PMU_SELF_TEST, data); res != 1)
        return res;
    return data;
}
/*
std::tuple<int, int16_t> BMA355::getAccelerationSelfTestAxis(uint8_t axis, uint8_t ax) const
{
    int res;
    if ((res = setTestMode(axis | BMA355::PMU_SELF_TEST_SIGN_POSITIVE | BMA355::PMU_SELF_TEST_AMP_HIGH)) < 0)
        return { res, 0 };
    QThread::msleep(50); // delay specified in BMA355 datasheet
    const int p = getAcceleration(ax);
    if ((res = setTestMode(axis | BMA355::PMU_SELF_TEST_SIGN_NEGATIVE | BMA355::PMU_SELF_TEST_AMP_HIGH)) < 0)
        return { res, 0 };
    QThread::msleep(50);
    const int n = getAcceleration(ax);
    return { res, n - p };
}

std::tuple<int, std::array<int16_t, 3>> BMA355::getAccelerationSelfTest() const
{
    int res;
    std::array<int16_t, 3> v{ };

    if (std::tie(res, v.at(0)) = getAccelerationSelfTestAxis(BMA355::PMU_SELF_TEST_AXIS_X, BMA355::RA_X_AXIS_LSB); res < 0)
        return { res, {} };

    if (std::tie(res, v.at(1)) = getAccelerationSelfTestAxis(BMA355::PMU_SELF_TEST_AXIS_Y, BMA355::RA_Y_AXIS_LSB); res < 0)
        return { res, {} };

    if (std::tie(res, v.at(2)) = getAccelerationSelfTestAxis(BMA355::PMU_SELF_TEST_AXIS_Z, BMA355::RA_Z_AXIS_LSB); res < 0)
        return { res, {} };

    return { res, v };
}
*/
int16_t BMA355::toTwosComplement(uint8_t msb, uint8_t lsb)
{
    const int16_t res = ((int16_t(msb) << 8) | lsb) >> 4;
    return res;
}

int16_t BMA355::twosComplement12bitToInt(int16_t res)
{
    int16_t foo;
    if (res > 2047) foo = -4096 + res;
    else foo = res;
    return foo + 2048;
}

int BMA355::done()
{
    return 1;
}
